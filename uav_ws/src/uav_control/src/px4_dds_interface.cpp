#include "uav_control/px4_dds_interface.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace uav_control
{
namespace
{
using Status = px4_msgs::msg::VehicleStatus;
using Command = px4_msgs::msg::VehicleCommand;
using Ack = px4_msgs::msg::VehicleCommandAck;

double elapsed(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

std::string mode_name(uint8_t mode)
{
  switch (mode) {
    case Status::NAVIGATION_STATE_OFFBOARD: return "OFFBOARD";
    case Status::NAVIGATION_STATE_AUTO_LAND: return "AUTO.LAND";
    case Status::NAVIGATION_STATE_AUTO_LOITER: return "AUTO.LOITER";
    case Status::NAVIGATION_STATE_AUTO_RTL: return "AUTO.RTL";
    case Status::NAVIGATION_STATE_AUTO_MISSION: return "AUTO.MISSION";
    case Status::NAVIGATION_STATE_AUTO_TAKEOFF: return "AUTO.TAKEOFF";
    case Status::NAVIGATION_STATE_MANUAL: return "MANUAL";
    case Status::NAVIGATION_STATE_ALTCTL: return "ALTCTL";
    case Status::NAVIGATION_STATE_POSCTL: return "POSCTL";
    default: return "PX4_NAV_" + std::to_string(mode);
  }
}
}  // namespace

Px4DdsInterface::Px4DdsInterface(const rclcpp::NodeOptions & options)
: Node("uav_control", options)
{
  read_only_ = declare_parameter<bool>("read_only", true);
  allow_local_vision_heading_ = declare_parameter<bool>("allow_local_vision_heading", false);
  estimator_timeout_ = declare_parameter<double>("estimator_timeout_s", 2.0);
  rate_ = declare_parameter<double>("publish_rate_hz", 20.0);
  state_timeout_ = declare_parameter<double>("state_timeout_s", 1.0);
  // LandDetector publishes at 1 Hz when unchanged; allow scheduling jitter / one missed sample.
  land_timeout_ = declare_parameter<double>("landed_state_timeout_s", 2.5);
  timesync_timeout_ = declare_parameter<double>("timesync_timeout_s", 3.0);
  position_timeout_ = declare_parameter<double>("position_timeout_s", 0.5);
  target_timeout_ = declare_parameter<double>("target_timeout_s", 1.0);
  command_timeout_ = declare_parameter<double>("command_timeout_s", 3.0);
  prestream_s_ = declare_parameter<double>("prestream_s", 1.1);
  handover_timeout_ = declare_parameter<double>("land_handover_timeout_s", 3.0);
  for (double value : {rate_, state_timeout_, position_timeout_, target_timeout_,
    command_timeout_, prestream_s_, handover_timeout_, timesync_timeout_, estimator_timeout_, land_timeout_})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("DDS rates and timeouts must be finite and positive");
    }
  }
  if (rate_ <= 2.0 || rate_ > 100.0 || prestream_s_ <= 1.0 ||
    get_parameter("use_sim_time").as_bool())
  {
    throw std::invalid_argument("DDS requires 2 < rate <= 100, prestream > 1 s and system time");
  }
  const auto id = [this](const char * name, int value, int maximum) {
      const int result = declare_parameter<int>(name, value);
      if (result < 1 || result > maximum) {
        throw std::invalid_argument(std::string(name) + " is out of range");
      }
      return result;
    };
  target_system_ = id("target_system", 1, 255);
  target_component_ = id("target_component", 1, 255);
  source_system_ = id("source_system", 1, 255);
  source_component_ = id("source_component", 191, 65535);
  const auto prefix = declare_parameter<std::string>("fmu_prefix", "/fmu");
  if (prefix.empty() || prefix.front() != '/' || prefix.back() == '/') {
    throw std::invalid_argument("fmu_prefix must be absolute without a trailing slash");
  }
  const auto topic = [&prefix](const char * path, uint32_t version = 0) {
      return prefix + "/" + path + (version ? "_v" + std::to_string(version) : "");
    };
  const auto qos = rclcpp::SensorDataQoS();
  status_sub_ = create_subscription<Status>(
    topic("out/vehicle_status", Status::MESSAGE_VERSION), qos,
    [this](Status::SharedPtr msg) {
      if (msg->system_id == target_system_ && msg->component_id == target_component_ &&
        receive(status_stream_, msg->timestamp, state_timeout_)) {status_ = *msg;}
    });
  position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    topic("out/vehicle_local_position", px4_msgs::msg::VehicleLocalPosition::MESSAGE_VERSION),
    qos, [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
      const bool reset = position_stream_.timestamp &&
        (msg->xy_reset_counter != position_.xy_reset_counter ||
        msg->z_reset_counter != position_.z_reset_counter ||
        msg->heading_reset_counter != position_.heading_reset_counter ||
        msg->ref_timestamp != position_.ref_timestamp);
      if (receive(position_stream_, msg->timestamp, position_timeout_)) {
        position_ = *msg;
        if (reset) {invalidate("local position origin/heading reset", true);}
      }
    });
  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
    topic("out/vehicle_attitude", px4_msgs::msg::VehicleAttitude::MESSAGE_VERSION), qos,
    [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
      const bool reset = attitude_stream_.timestamp &&
        msg->quat_reset_counter != attitude_.quat_reset_counter;
      if (receive(attitude_stream_, msg->timestamp, position_timeout_)) {
        attitude_ = *msg;
        if (reset) {invalidate("attitude estimator reset", true);}
      }
    });
  land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
    topic("out/vehicle_land_detected", px4_msgs::msg::VehicleLandDetected::MESSAGE_VERSION),
    qos, [this](px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
      if (receive(land_stream_, msg->timestamp, land_timeout_)) {land_ = *msg;}
    });
  timesync_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
    topic("out/timesync_status"), qos, [this](px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
      if (msg->source_protocol == px4_msgs::msg::TimesyncStatus::SOURCE_PROTOCOL_DDS &&
        receive(timesync_stream_, msg->timestamp, timesync_timeout_)) {timesync_ = *msg;}
    });
  ack_sub_ = create_subscription<Ack>(
    topic("out/vehicle_command_ack", Ack::MESSAGE_VERSION), qos,
    std::bind(&Px4DdsInterface::ack_callback, this, std::placeholders::_1));
  estimator_sub_ = create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
    topic("out/estimator_status_flags"), qos,
    [this](px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg) {
      if (receive(estimator_stream_, msg->timestamp, estimator_timeout_)) {
        estimator_flags_ = *msg;
      }
    });
  state_pub_ = create_publisher<uav_interfaces::msg::VehicleState>("/uav_control/state", 10);
  reset_service_ = create_service<std_srvs::srv::Trigger>("/uav_control/reset",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = state_valid() && on_ground() &&
        status_.arming_state == Status::ARMING_STATE_DISARMED && !pending_ &&
        status_.nav_state != Status::NAVIGATION_STATE_OFFBOARD;
      if (response->success) {
        const bool was_invalid = origin_invalid_;
        origin_invalid_ = false;
        response->success = position_valid();
        if (!response->success) {origin_invalid_ = was_invalid;}
      }
      if (response->success) {
        safety_latched_ = false;
        safety_reason_.clear();
        safety_land_needed_ = false;
        streaming_ = false;
        offboard_active_ = false;
        land_requested_ = false;
        target_stamp_ = 0;
        target_epoch_ = timestamp_us();
        last_request_id_ = 0;
        command_id_ = 0;
        command_completed_ = command_accepted_ = false;
        command_message_.clear();
      }
      response->message = response->success ? "ready; require a new target/task" :
        "require fresh valid ground/disarmed state, positioning and no pending command";
    });
  if (!read_only_) {
    heartbeat_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      topic("in/offboard_control_mode"), 10);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      topic("in/trajectory_setpoint", px4_msgs::msg::TrajectorySetpoint::MESSAGE_VERSION), 10);
    command_pub_ = create_publisher<Command>(
      topic("in/vehicle_command", Command::MESSAGE_VERSION), 10);
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/uav_control/target", 10,
      std::bind(&Px4DdsInterface::target_callback, this, std::placeholders::_1));
    command_service_ = create_service<FlightCommand>("/uav_control/command",
      std::bind(&Px4DdsInterface::command_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  }
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_)),
    std::bind(&Px4DdsInterface::tick, this));
  RCLCPP_INFO(get_logger(), "PX4 v1.17 DDS %s: %.1f Hz; prefix=%s; source=%u/%u",
    read_only_ ? "READ ONLY" : "CONTROL", rate_, prefix.c_str(), source_system_, source_component_);
}

uint64_t Px4DdsInterface::timestamp_us() const
{
  // PX4 UXRCE_DDS_SYNCT=1 translates wire timestamps to/from Agent system time.
  // Do not apply TimesyncStatus.estimated_offset a second time.
  return static_cast<uint64_t>(now().nanoseconds() / 1000);
}

bool Px4DdsInterface::timestamp_valid(uint64_t stamp, double timeout) const
{
  if (stamp == 0 || stamp > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
    return false;
  }
  const double age = (static_cast<int64_t>(timestamp_us()) - static_cast<int64_t>(stamp)) / 1e6;
  return age >= -0.1 && age <= timeout;
}

bool Px4DdsInterface::receive(Stream & stream, uint64_t stamp, double timeout)
{
  if (stamp <= stream.timestamp || !timestamp_valid(stamp, timeout)) {return false;}
  stream = {stamp, Clock::now()};
  return true;
}

bool Px4DdsInterface::fresh(const Stream & stream, double timeout) const
{
  return timestamp_valid(stream.timestamp, timeout) && elapsed(stream.received) <= timeout;
}

bool Px4DdsInterface::state_valid() const
{
  return fresh(status_stream_, state_timeout_) && fresh(timesync_stream_, timesync_timeout_) &&
    timestamp_valid(timesync_.remote_timestamp, timesync_timeout_) &&
    (status_.arming_state == Status::ARMING_STATE_ARMED ||
    status_.arming_state == Status::ARMING_STATE_DISARMED);
}

bool Px4DdsInterface::position_valid() const
{
  return position_coordinates_valid() && heading_valid();
}

bool Px4DdsInterface::position_coordinates_valid() const
{
  return !origin_invalid_ && state_valid() && fresh(position_stream_, position_timeout_) &&
    timestamp_valid(position_.timestamp_sample, position_timeout_) &&
    position_.xy_valid && position_.z_valid &&
    std::isfinite(position_.x) && std::isfinite(position_.y) && std::isfinite(position_.z);
}

bool Px4DdsInterface::heading_valid() const
{
  double norm2 = 0.0;
  for (auto value : attitude_.q) {norm2 += static_cast<double>(value) * value;}
  // PX4 deliberately leaves yaw_align=false when fusing arbitrary-heading LOCAL_FRD
  // vision. Accept that local heading only by explicit deployment opt-in and fresh
  // FCU fusion status. Never relabel the vision input as north-aligned NED.
  const bool local_vision_heading = allow_local_vision_heading_ &&
    fresh(estimator_stream_, estimator_timeout_) &&
    timestamp_valid(estimator_flags_.timestamp_sample, estimator_timeout_) &&
    estimator_flags_.cs_tilt_align && !estimator_flags_.cs_yaw_align &&
    estimator_flags_.cs_ev_yaw && !estimator_flags_.cs_ev_yaw_fault &&
    !estimator_flags_.fs_bad_hdg && !estimator_flags_.reject_yaw;
  return !origin_invalid_ && state_valid() && fresh(position_stream_, position_timeout_) &&
    fresh(attitude_stream_, position_timeout_) &&
    timestamp_valid(position_.timestamp_sample, position_timeout_) &&
    timestamp_valid(attitude_.timestamp_sample, position_timeout_) &&
    (position_.heading_good_for_control || local_vision_heading) &&
    std::isfinite(norm2) && norm2 > 0.5 && norm2 < 1.5;
}

bool Px4DdsInterface::velocity_valid() const
{
  return !origin_invalid_ && state_valid() && fresh(position_stream_, position_timeout_) &&
    timestamp_valid(position_.timestamp_sample, position_timeout_) &&
    position_.v_xy_valid && position_.v_z_valid &&
    std::isfinite(position_.vx) && std::isfinite(position_.vy) &&
    std::isfinite(position_.vz);
}

bool Px4DdsInterface::estimator_health_valid() const
{
  return !origin_invalid_ && state_valid() && fresh(position_stream_, position_timeout_) &&
    timestamp_valid(position_.timestamp_sample, position_timeout_);
}

bool Px4DdsInterface::estimator_healthy() const
{
  return estimator_health_valid() && position_.xy_valid && position_.z_valid &&
    position_.v_xy_valid && position_.v_z_valid && heading_valid() &&
    !position_.dead_reckoning && std::isfinite(position_.x) &&
    std::isfinite(position_.y) && std::isfinite(position_.z) &&
    std::isfinite(position_.vx) && std::isfinite(position_.vy) &&
    std::isfinite(position_.vz);
}

bool Px4DdsInterface::on_ground() const
{
  return fresh(land_stream_, land_timeout_) && land_.landed;
}

void Px4DdsInterface::invalidate(const std::string & reason, bool origin_reset)
{
  if (!safety_latched_) {
    RCLCPP_WARN(get_logger(), "DDS control invalidated: %s; ground reset required", reason.c_str());
    safety_started_ = Clock::now();
    safety_reason_ = reason;
  }
  safety_latched_ = true;
  origin_invalid_ = origin_invalid_ || origin_reset;
  target_stamp_ = 0;
}

void Px4DdsInterface::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (safety_latched_ || !position_valid() ||
    msg->header.frame_id != "map" || msg->header.stamp.sec < 0 ||
    msg->header.stamp.nanosec >= 1000000000u) {return;}
  const uint64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds() / 1000;
  const auto & p = msg->pose.position;
  const auto & q = msg->pose.orientation;
  const double norm2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  const double max = std::numeric_limits<float>::max();
  if (stamp <= target_stamp_ || stamp <= target_epoch_ || !timestamp_valid(stamp, target_timeout_) ||
    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    std::abs(p.x) > max || std::abs(p.y) > max || std::abs(p.z) > max ||
    !std::isfinite(norm2) || std::abs(norm2 - 1.0) > 0.01) {return;}
  target_ = *msg;
  target_stamp_ = stamp;
  target_rx_ = Clock::now();
}

void Px4DdsInterface::command_callback(
  const std::shared_ptr<FlightCommand::Request> request,
  std::shared_ptr<FlightCommand::Response> response)
{
  if (request->command_id == 0 || request->command_id <= last_request_id_) {
    response->message = "require a new monotonically increasing nonzero command_id";
  } else if (!state_valid() || pending_) {
    response->message = "FCU state/time sync invalid or another command pending";
  } else if (request->command == FlightCommand::Request::DISARM && !on_ground()) {
    response->message = "DISARM requires fresh landed confirmation";
  } else if (request->command == FlightCommand::Request::LAND && !position_valid()) {
    response->message = "LAND requires fresh valid positioning";
  } else if (request->command == FlightCommand::Request::LAND &&
    (!offboard_active_ || status_.nav_state != Status::NAVIGATION_STATE_OFFBOARD))
  {
    response->message = "LAND requires control ownership in OFFBOARD; do not override mode takeover";
  } else if ((request->command == FlightCommand::Request::SET_OFFBOARD ||
    request->command == FlightCommand::Request::ARM) &&
    (safety_latched_ || land_requested_ || !position_valid() || !velocity_valid() ||
    !estimator_healthy() || status_.failsafe ||
    (!status_.pre_flight_checks_pass &&
    status_.arming_state == Status::ARMING_STATE_DISARMED) || !streaming_ ||
    !timestamp_valid(target_stamp_, target_timeout_) || elapsed(target_rx_) > target_timeout_ ||
    elapsed(stream_started_) < prestream_s_ ||
    (request->command == FlightCommand::Request::ARM &&
    status_.nav_state != Status::NAVIGATION_STATE_OFFBOARD)))
  {
    response->message = "require fresh target/position, prestream and OFFBOARD before ARM";
  } else {
    response->dispatched = send_command(request->command_id, request->command);
    const uint32_t opcode = request->command == FlightCommand::Request::SET_OFFBOARD ?
      Command::VEHICLE_CMD_DO_SET_MODE :
      (request->command == FlightCommand::Request::LAND ? Command::VEHICLE_CMD_NAV_LAND :
      Command::VEHICLE_CMD_COMPONENT_ARM_DISARM);
    response->message = response->dispatched ?
      "sent to PX4; wait for ACK and actual FCU state" :
      (timed_out_commands_.count(opcode) ?
      "PX4 opcode quarantined after timeout/unknown result; restart with a new source_component required" :
      "PX4 command could not be sent");
    if (response->dispatched) {last_request_id_ = request->command_id;}
  }
}

bool Px4DdsInterface::send_command(uint32_t id, uint8_t code)
{
  if (read_only_ || pending_ || !state_valid()) {return false;}
  if (code == FlightCommand::Request::LAND &&
    (!offboard_active_ || status_.nav_state != Status::NAVIGATION_STATE_OFFBOARD))
  {
    return false;
  }
  Command msg;
  msg.timestamp = timestamp_us();
  msg.target_system = target_system_;
  msg.target_component = target_component_;
  msg.source_system = source_system_;
  msg.source_component = source_component_;
  msg.from_external = true;
  if (code == FlightCommand::Request::SET_OFFBOARD) {
    msg.command = Command::VEHICLE_CMD_DO_SET_MODE;
    msg.param1 = 1.0F;  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    msg.param2 = 6.0F;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
  } else if (code == FlightCommand::Request::ARM || code == FlightCommand::Request::DISARM) {
    msg.command = Command::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.param1 = code == FlightCommand::Request::ARM ? 1.0F : 0.0F;
  } else if (code == FlightCommand::Request::LAND) {
    msg.command = Command::VEHICLE_CMD_NAV_LAND;
    msg.param4 = msg.param7 = std::numeric_limits<float>::quiet_NaN();
    msg.param5 = msg.param6 = std::numeric_limits<double>::quiet_NaN();
  } else {return false;}
  if (timed_out_commands_.count(msg.command)) {return false;}
  command_id_ = id;
  pending_command_code_ = code;
  pending_start_nav_state_ = status_.nav_state;
  pending_px4_command_ = msg.command;
  command_stamp_ = msg.timestamp;
  command_started_ = Clock::now();
  pending_ = true;
  command_completed_ = false;
  command_accepted_ = false;
  command_message_ = "waiting for PX4 ACK";
  if (code == FlightCommand::Request::LAND) {
    land_requested_ = true;
    safety_started_ = Clock::now();
  }
  command_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "DDS command id=%u opcode=%u sent", id, msg.command);
  return true;
}

void Px4DdsInterface::ack_callback(const Ack::SharedPtr msg)
{
  // ACK target routes to our command SOURCE; from_external is false for FCU-generated ACK.
  if (!pending_ || msg->command != pending_px4_command_ || msg->from_external ||
    msg->target_system != source_system_ || msg->target_component != source_component_ ||
    msg->timestamp < command_stamp_ || !timestamp_valid(msg->timestamp, command_timeout_) ||
    elapsed(command_started_) > command_timeout_) {return;}
  if (msg->result == Ack::VEHICLE_CMD_RESULT_IN_PROGRESS) {
    command_message_ = "PX4 command in progress";
    return;  // Intermediate ACK does not extend the finite deadline.
  }
  finish_command(msg->result == Ack::VEHICLE_CMD_RESULT_ACCEPTED,
    "PX4 ACK result=" + std::to_string(msg->result));
}

void Px4DdsInterface::finish_command(bool accepted, const std::string & message)
{
  pending_ = false;
  command_completed_ = true;
  command_accepted_ = accepted;
  command_message_ = message;
  RCLCPP_INFO(get_logger(), "DDS command id=%u accepted=%s: %s", command_id_,
    accepted ? "true" : "false", message.c_str());
}

void Px4DdsInterface::tick()
{
  if (pending_ && elapsed(command_started_) > command_timeout_) {
    timed_out_commands_.insert(pending_px4_command_);
    finish_command(false, "PX4 ACK timeout; opcode blocked until node restart with a new source_component");
  }
  if (!read_only_) {
    const bool valid_position = position_valid();
    const bool valid_velocity = velocity_valid();
    const bool healthy_estimator = estimator_healthy();
    const bool valid_target = timestamp_valid(target_stamp_, target_timeout_) &&
      elapsed(target_rx_) <= target_timeout_;
    if (pending_) {
      const bool actual_state_confirmed = state_valid() && (
        (pending_command_code_ == FlightCommand::Request::SET_OFFBOARD &&
        status_.nav_state == Status::NAVIGATION_STATE_OFFBOARD) ||
        (pending_command_code_ == FlightCommand::Request::ARM &&
        status_.arming_state == Status::ARMING_STATE_ARMED) ||
        (pending_command_code_ == FlightCommand::Request::DISARM &&
        status_.arming_state == Status::ARMING_STATE_DISARMED && on_ground()) ||
        (pending_command_code_ == FlightCommand::Request::LAND &&
        status_.nav_state == Status::NAVIGATION_STATE_AUTO_LAND));
      bool invalidated = !state_valid();
      if (!actual_state_confirmed && !invalidated) {
        if (pending_command_code_ == FlightCommand::Request::SET_OFFBOARD ||
          pending_command_code_ == FlightCommand::Request::ARM)
        {
          invalidated = !valid_position || !valid_velocity || !healthy_estimator ||
            status_.failsafe || !valid_target || !streaming_ ||
            (pending_command_code_ == FlightCommand::Request::ARM &&
            status_.nav_state != Status::NAVIGATION_STATE_OFFBOARD) ||
            (pending_command_code_ == FlightCommand::Request::SET_OFFBOARD &&
            status_.nav_state != pending_start_nav_state_ &&
            status_.nav_state != Status::NAVIGATION_STATE_OFFBOARD) ||
            (!status_.pre_flight_checks_pass &&
            status_.arming_state == Status::ARMING_STATE_DISARMED);
        } else if (pending_command_code_ == FlightCommand::Request::LAND) {
          // Once LAND is sent, loss of navigation health must not cancel its handover path.
          invalidated = status_.nav_state != Status::NAVIGATION_STATE_OFFBOARD &&
            status_.nav_state != Status::NAVIGATION_STATE_AUTO_LAND;
        } else if (pending_command_code_ == FlightCommand::Request::DISARM) {
          invalidated = !on_ground();
        }
      }
      if (!actual_state_confirmed && invalidated && pending_) {
        timed_out_commands_.insert(pending_px4_command_);
        finish_command(false,
          "PX4 command invalidated after dispatch; actual result unknown, delayed ACK ignored");
      }
    }
    if (state_valid() && status_.failsafe && !safety_latched_)
    {
      safety_land_needed_ = status_.arming_state == Status::ARMING_STATE_ARMED && valid_position;
      invalidate("PX4 reports an active failsafe");
    }
    if (streaming_ && (!state_valid() || !valid_position || !valid_velocity ||
      !healthy_estimator || !valid_target ||
      (!status_.pre_flight_checks_pass &&
      status_.arming_state == Status::ARMING_STATE_DISARMED)))
    {
      // Target loss can hand over to Land only while localization/link are still valid.
      safety_land_needed_ = state_valid() && valid_position;
      const std::string reason = !valid_target ? "target timeout" :
        (!valid_position ? "position/heading estimate stale or invalid" :
        ((!valid_velocity || !healthy_estimator) ? "navigation estimate health lost" :
        ((!status_.pre_flight_checks_pass &&
        status_.arming_state == Status::ARMING_STATE_DISARMED) ?
        "PX4 pre-flight checks lost while disarmed" :
        (state_valid() && status_.failsafe ? "PX4 reports an active failsafe" :
        "FCU/time sync stale"))));
      invalidate(reason);
      if (pending_ && (!state_valid() || !valid_position)) {
        timed_out_commands_.insert(pending_px4_command_);
        finish_command(false, "FCU/position lost; opcode quarantined, delayed ACK ignored");
      }
    }
    if (streaming_ && state_valid()) {
      if (status_.nav_state == Status::NAVIGATION_STATE_OFFBOARD) {
        offboard_active_ = true;
      } else if (offboard_active_ &&
        status_.nav_state != Status::NAVIGATION_STATE_AUTO_LAND)
      {
        offboard_active_ = false;
        invalidate("FCU left OFFBOARD: " + mode_name(status_.nav_state));
      }
    }
    if (safety_land_needed_ && state_valid() && valid_position && !pending_ &&
      !land_requested_ && offboard_active_ &&
      status_.nav_state == Status::NAVIGATION_STATE_OFFBOARD &&
      status_.arming_state == Status::ARMING_STATE_ARMED && !on_ground())
    {
      send_command(0, FlightCommand::Request::LAND);
      safety_land_needed_ = false;
    }
    if (state_valid() && status_.nav_state == Status::NAVIGATION_STATE_AUTO_LAND) {
      if (streaming_) {invalidate("AUTO.LAND handover confirmed");}
      streaming_ = false;
    } else if (land_requested_ && elapsed(safety_started_) > handover_timeout_) {
      invalidate("AUTO.LAND handover timeout");
      streaming_ = false;
    } else if (!safety_latched_ && !land_requested_ && valid_position && valid_velocity &&
      healthy_estimator && !status_.failsafe && valid_target)
    {
      if (!streaming_) {stream_started_ = Clock::now();}
      streaming_ = true;
    } else if (safety_latched_) {
      streaming_ = false;
    }
    // During LAND handover, only a continuously fresh target may maintain the stream.
    if (streaming_ && state_valid() && valid_position && valid_target) {
      const auto stamp = timestamp_us();
      px4_msgs::msg::TrajectorySetpoint setpoint;
      setpoint.timestamp = stamp;
      const auto & p = target_.pose.position;
      setpoint.position = dds_frames::position({
        static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
      const auto & q = target_.pose.orientation;
      const double yaw_enu = std::atan2(2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z));
      setpoint.yaw = dds_frames::yaw(yaw_enu);
      const float nan = std::numeric_limits<float>::quiet_NaN();
      setpoint.velocity.fill(nan);
      setpoint.acceleration.fill(nan);
      setpoint.jerk.fill(nan);
      setpoint.yawspeed = nan;
      setpoint_pub_->publish(setpoint);
      px4_msgs::msg::OffboardControlMode heartbeat;
      heartbeat.timestamp = stamp;
      heartbeat.position = true;
      heartbeat_pub_->publish(heartbeat);
    }
  }
  publish_state();
}

void Px4DdsInterface::publish_state()
{
  uav_interfaces::msg::VehicleState state;
  state.state_valid = state_valid();
  state.connected = state.state_valid;
  state.armed = status_.arming_state == Status::ARMING_STATE_ARMED;
  state.mode = mode_name(status_.nav_state);
  state.position_valid = position_coordinates_valid();
  state.heading_valid = heading_valid();
  state.velocity_valid = velocity_valid();
  state.local_velocity.header.frame_id = "map";
  if (timestamp_valid(position_.timestamp_sample, position_timeout_)) {
    state.local_velocity.header.stamp = rclcpp::Time(
      static_cast<int64_t>(position_.timestamp_sample * 1000));
  }
  state.local_velocity.vector.x = position_.vy;
  state.local_velocity.vector.y = position_.vx;
  state.local_velocity.vector.z = -position_.vz;
  state.estimator_health_valid = estimator_health_valid();
  state.estimator_healthy = estimator_healthy();
  state.failsafe_state_valid = state.state_valid;
  state.failsafe_active = status_.failsafe;
  state.preflight_check_state_valid = state.state_valid;
  state.preflight_checks_passed = status_.pre_flight_checks_pass;
  state.control_inhibited = read_only_ || safety_latched_;
  state.control_message = read_only_ ? "read-only backend" : safety_reason_;
  state.local_pose.header.frame_id = "map";
  if (timestamp_valid(position_.timestamp_sample, position_timeout_)) {
    state.local_pose.header.stamp = rclcpp::Time(
      static_cast<int64_t>(position_.timestamp_sample * 1000));
  }
  state.local_pose.pose.position.x = position_.y;
  state.local_pose.pose.position.y = position_.x;
  state.local_pose.pose.position.z = -position_.z;
  if (state.heading_valid) {
    const auto q = dds_frames::attitude(attitude_.q);
    state.local_pose.pose.orientation.w = q[0];
    state.local_pose.pose.orientation.x = q[1];
    state.local_pose.pose.orientation.y = q[2];
    state.local_pose.pose.orientation.z = q[3];
  }
  state.landed_state_valid = state.state_valid && fresh(land_stream_, land_timeout_);
  state.landed_state = state.landed_state_valid ?
    (land_.landed ? state.LANDED_ON_GROUND : state.LANDED_IN_AIR) : state.LANDED_UNKNOWN;
  state.last_command_id = command_id_;
  state.command_completed = command_completed_;
  state.command_accepted = command_accepted_;
  state.command_message = command_message_;
  state_pub_->publish(state);
}
}  // namespace uav_control
