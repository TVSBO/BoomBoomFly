#include "uav_control/mavros_interface.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

namespace uav_control
{

MavrosInterface::MavrosInterface(const rclcpp::NodeOptions & options)
: Node("uav_control", options)
{
  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;
  read_only_ = declare_parameter<bool>("read_only", true, read_only_descriptor);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
  state_timeout_s_ = declare_parameter<double>("state_timeout_s", 1.0);
  position_timeout_s_ = declare_parameter<double>("position_timeout_s", 0.5);
  target_timeout_s_ = declare_parameter<double>("target_timeout_s", 1.0);
  land_handover_timeout_s_ = declare_parameter<double>("land_handover_timeout_s", 3.0);
  command_queue_timeout_s_ = declare_parameter<double>("command_queue_timeout_s", 3.0);
  command_timeout_s_ = declare_parameter<double>("command_timeout_s", 3.0);
  for (const auto value : {publish_rate_hz_, state_timeout_s_, position_timeout_s_,
      target_timeout_s_, land_handover_timeout_s_, command_queue_timeout_s_, command_timeout_s_})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Control rates and timeout parameters must be positive");
    }
  }
  if (publish_rate_hz_ <= 2.0 || publish_rate_hz_ > 100.0) {
    throw std::invalid_argument("Control requires 2 < publish_rate_hz <= 100");
  }

  state_sub_ = create_subscription<mavros_msgs::msg::State>("/mavros/state", 10,
    std::bind(&MavrosInterface::state_callback, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
    std::bind(&MavrosInterface::pose_callback, this, std::placeholders::_1));
  velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(),
    std::bind(&MavrosInterface::velocity_callback, this, std::placeholders::_1));
  extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
    "/mavros/extended_state", 10,
    std::bind(&MavrosInterface::extended_state_callback, this, std::placeholders::_1));
  target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/uav_control/target", 10,
    std::bind(&MavrosInterface::target_callback, this, std::placeholders::_1));
  setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/setpoint_position/local", 10);
  state_pub_ = create_publisher<uav_interfaces::msg::VehicleState>("/uav_control/state", 10);
  set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
  command_service_ = create_service<FlightCommand>("/uav_control/command",
    std::bind(&MavrosInterface::command_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_)),
    std::bind(&MavrosInterface::control_tick, this));
  RCLCPP_INFO(get_logger(), "MAVROS control ready: %.1f Hz; read_only=%s; target /uav_control/target",
    publish_rate_hz_, read_only_ ? "true" : "false");
}

bool MavrosInterface::timestamp_fresh(
  const builtin_interfaces::msg::Time & stamp, double timeout) const
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
    (stamp.sec == 0 && stamp.nanosec == 0)) {return false;}
  const double age = (now().nanoseconds() - rclcpp::Time(stamp).nanoseconds()) / 1e9;
  return age >= -0.1 && age <= timeout;
}

void MavrosInterface::state_callback(const mavros_msgs::msg::State::SharedPtr msg)
{
  if (!timestamp_fresh(msg->header.stamp, state_timeout_s_)) {return;}
  if (!have_state_ || msg->connected != fcu_state_.connected ||
    msg->mode != fcu_state_.mode || msg->armed != fcu_state_.armed)
  {
    RCLCPP_INFO(get_logger(), "FCU %s, mode=%s, armed=%s",
      msg->connected ? "CONNECTED" : "DISCONNECTED", msg->mode.c_str(),
      msg->armed ? "true" : "false");
  }
  fcu_state_ = *msg;
  state_rx_ = Clock::now();
  have_state_ = true;
}

void MavrosInterface::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg->header.frame_id != "map" ||
    !timestamp_fresh(msg->header.stamp, position_timeout_s_)) {return;}
  local_pose_ = *msg;
  pose_rx_ = Clock::now();
  have_pose_ = true;
}

void MavrosInterface::velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const auto & velocity = msg->twist.linear;
  if (!timestamp_fresh(msg->header.stamp, position_timeout_s_)) {return;}
  const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (stamp <= velocity_stamp_ns_ ||
    (msg->header.frame_id != "map" && msg->header.frame_id != "base_link") ||
    !std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z))
  {
    return;
  }
  // MAVROS publishes this topic's linear vector in local ENU despite its base_link header.
  local_velocity_.header = msg->header;
  local_velocity_.header.frame_id = "map";
  local_velocity_.vector.x = velocity.x;
  local_velocity_.vector.y = velocity.y;
  local_velocity_.vector.z = velocity.z;
  velocity_stamp_ns_ = stamp;
  velocity_rx_ = Clock::now();
  have_velocity_ = true;
}

void MavrosInterface::extended_state_callback(
  const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
  if (!timestamp_fresh(msg->header.stamp, state_timeout_s_)) {return;}
  extended_state_ = *msg;
  extended_state_rx_ = Clock::now();
  have_extended_state_ = true;
}

void MavrosInterface::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto & p = msg->pose.position;
  const auto & q = msg->pose.orientation;
  const double norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (read_only_ || safety_latched_ || msg->header.frame_id != "map" ||
    !timestamp_fresh(msg->header.stamp, target_timeout_s_) ||
    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(norm2) || std::abs(norm2 - 1.0) > 0.01)
  {
    return;
  }
  const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (stamp <= target_stamp_ns_) {return;}
  target_ = *msg;
  target_stamp_ns_ = stamp;
  target_rx_ = Clock::now();
  have_target_ = true;
}

std::string MavrosInterface::command_block_reason(uint8_t command) const
{
  if (read_only_) {return "read-only backend";}
  const auto time = Clock::now();
  if (!state_fresh(time) || !fcu_state_.connected) {return "FCU state disconnected or stale";}
  if (command == FlightCommand::Request::DISARM) {
    return landed_state_fresh(time) &&
           extended_state_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND ?
           "" : "DISARM requires fresh landed confirmation";
  }
  if (command == FlightCommand::Request::LAND) {
    return offboard_active_ && fcu_state_.mode == "OFFBOARD" &&
           position_fresh(time) ? "" : "LAND requires valid positioning and ownership in OFFBOARD";
  }
  if (command != FlightCommand::Request::SET_OFFBOARD && command != FlightCommand::Request::ARM) {
    return "unsupported command";
  }
  if (safety_latched_ || !position_fresh(time) || !velocity_fresh(time) ||
    !target_fresh(time) ||
    stream_started_ == Clock::time_point{} ||
    std::chrono::duration<double>(time - stream_started_).count() <= 1.0)
  {
    return "require fresh target/position and prestream; control must not be inhibited";
  }
  if (command == FlightCommand::Request::ARM && fcu_state_.mode != "OFFBOARD") {
    return "ARM requires OFFBOARD";
  }
  return "";
}

void MavrosInterface::command_callback(
  const std::shared_ptr<FlightCommand::Request> request,
  std::shared_ptr<FlightCommand::Response> response)
{
  if (request->command_id == 0 || command_pending_) {
    response->message = command_pending_ ? "another MAVROS command is pending" :
      "command_id 0 is reserved";
    return;
  }
  response->message = command_block_reason(request->command);
  if (!response->message.empty()) {return;}
  response->dispatched = send_command(request->command_id, request->command);
  response->message = response->dispatched ?
    (queued_command_ ? "queued while waiting for MAVROS service" : "forwarded to MAVROS service") :
    "command could not be forwarded";
}

bool MavrosInterface::command_service_ready(uint8_t command) const
{
  if (command == FlightCommand::Request::SET_OFFBOARD) {return set_mode_client_->service_is_ready();}
  if (command == FlightCommand::Request::ARM || command == FlightCommand::Request::DISARM) {
    return arming_client_->service_is_ready();
  }
  return command == FlightCommand::Request::LAND && land_client_->service_is_ready();
}

bool MavrosInterface::send_command(uint32_t command_id, uint8_t command)
{
  if (command_pending_ || !command_block_reason(command).empty()) {return false;}
  last_command_id_ = command_id;
  pending_command_code_ = command;
  pending_start_mode_ = fcu_state_.mode;
  command_started_ = Clock::now();
  command_pending_ = true;
  command_completed_ = command_accepted_ = false;
  const auto generation = ++command_generation_;
  if (!command_service_ready(command)) {
    queued_command_ = true;
    command_message_ = "waiting for MAVROS service";
    return true;
  }
  command_message_ = "waiting for MAVROS response";
  if (command == FlightCommand::Request::SET_OFFBOARD) {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "OFFBOARD";
    pending_request_id_ = set_mode_client_->async_send_request(request,
      [this, generation](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        try {
          const auto result = future.get();
          finish_command(generation, result->mode_sent,
            result->mode_sent ? "MAVROS accepted OFFBOARD request" : "MAVROS rejected OFFBOARD request");
        } catch (const std::exception & error) {finish_command(generation, false, error.what());}
      }).request_id;
  } else if (command == FlightCommand::Request::ARM || command == FlightCommand::Request::DISARM) {
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = command == FlightCommand::Request::ARM;
    pending_request_id_ = arming_client_->async_send_request(request,
      [this, generation](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        try {
          const auto result = future.get();
          finish_command(generation, result->success,
            result->success ? "MAVROS accepted arming request" : "MAVROS rejected arming request");
        } catch (const std::exception & error) {finish_command(generation, false, error.what());}
      }).request_id;
  } else {
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    pending_request_id_ = land_client_->async_send_request(request,
      [this, generation](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
        try {
          const auto result = future.get();
          finish_command(generation, result->success,
            result->success ? "MAVROS accepted land request" : "MAVROS rejected land request");
        } catch (const std::exception & error) {finish_command(generation, false, error.what());}
      }).request_id;
  }
  return true;
}

void MavrosInterface::finish_command(
  uint64_t generation, bool accepted, const std::string & message)
{
  if (!command_pending_ || generation != command_generation_) {return;}
  command_pending_ = queued_command_ = false;
  pending_request_id_ = -1;
  command_completed_ = true;
  command_accepted_ = accepted;
  command_message_ = message;
  ++command_generation_;
  RCLCPP_INFO(get_logger(), "MAVROS command id=%u completed: accepted=%s (%s)",
    last_command_id_, accepted ? "true" : "false", message.c_str());
}

void MavrosInterface::stop_pending_command(const std::string & reason)
{
  if (!command_pending_) {return;}
  if (pending_request_id_ >= 0) {
    if (pending_command_code_ == FlightCommand::Request::SET_OFFBOARD) {
      set_mode_client_->remove_pending_request(pending_request_id_);
    } else if (pending_command_code_ == FlightCommand::Request::LAND) {
      land_client_->remove_pending_request(pending_request_id_);
    } else {
      arming_client_->remove_pending_request(pending_request_id_);
    }
  }
  // Removing a ROS request does not undo a command already sent to the FCU.
  finish_command(command_generation_, false, reason);
}

bool MavrosInterface::state_fresh(const Clock::time_point time) const
{
  return have_state_ && timestamp_fresh(fcu_state_.header.stamp, state_timeout_s_) &&
    std::chrono::duration<double>(time - state_rx_).count() <= state_timeout_s_;
}

bool MavrosInterface::pose_fresh(const Clock::time_point time) const
{
  return have_pose_ && timestamp_fresh(local_pose_.header.stamp, position_timeout_s_) &&
    std::chrono::duration<double>(time - pose_rx_).count() <= position_timeout_s_;
}

bool MavrosInterface::position_coordinates_fresh(const Clock::time_point time) const
{
  if (!pose_fresh(time)) {return false;}
  const auto & p = local_pose_.pose.position;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool MavrosInterface::heading_fresh(const Clock::time_point time) const
{
  if (!pose_fresh(time)) {return false;}
  const auto & q = local_pose_.pose.orientation;
  const double norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return std::isfinite(norm2) && norm2 > 0.5 && norm2 < 1.5;
}

bool MavrosInterface::position_fresh(const Clock::time_point time) const
{
  return position_coordinates_fresh(time) && heading_fresh(time);
}

bool MavrosInterface::velocity_fresh(const Clock::time_point time) const
{
  return have_velocity_ && timestamp_fresh(local_velocity_.header.stamp, position_timeout_s_) &&
    std::chrono::duration<double>(time - velocity_rx_).count() <= position_timeout_s_;
}

bool MavrosInterface::landed_state_fresh(const Clock::time_point time) const
{
  return have_extended_state_ && timestamp_fresh(extended_state_.header.stamp, state_timeout_s_) &&
    extended_state_.landed_state != mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED &&
    std::chrono::duration<double>(time - extended_state_rx_).count() <= state_timeout_s_;
}

bool MavrosInterface::target_fresh(const Clock::time_point time) const
{
  return have_target_ && timestamp_fresh(target_.header.stamp, target_timeout_s_) &&
    std::chrono::duration<double>(time - target_rx_).count() <= target_timeout_s_;
}

bool MavrosInterface::airborne_or_unknown(const Clock::time_point time) const
{
  return !landed_state_fresh(time) ||
    extended_state_.landed_state != mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
}

void MavrosInterface::publish_state(const Clock::time_point time)
{
  uav_interfaces::msg::VehicleState state;
  state.state_valid = state_fresh(time);
  state.connected = fcu_state_.connected;
  state.armed = fcu_state_.armed;
  state.mode = fcu_state_.mode;
  state.position_valid = position_coordinates_fresh(time);
  state.heading_valid = heading_fresh(time);
  state.velocity_valid = velocity_fresh(time);
  state.local_velocity = local_velocity_;
  state.local_pose = local_pose_;
  // These MAVROS inputs do not expose estimator, failsafe, or preflight state.
  // Their validity flags intentionally remain false rather than implying health.
  state.landed_state_valid = landed_state_fresh(time);
  state.landed_state = have_extended_state_ ? extended_state_.landed_state : 0;
  state.control_inhibited = read_only_ || safety_latched_;
  state.control_message = read_only_ ? "read-only backend" : safety_reason_;
  state.last_command_id = last_command_id_;
  state.command_completed = command_completed_;
  state.command_accepted = command_accepted_;
  state.command_message = command_message_;
  state_pub_->publish(state);
}

void MavrosInterface::control_tick()
{
  const auto time = Clock::now();
  const bool connected = state_fresh(time) && fcu_state_.connected;
  const bool position = position_fresh(time);
  const bool velocity = velocity_fresh(time);
  const bool target = target_fresh(time);
  if (connected && have_target_ && fcu_state_.mode == "OFFBOARD" && !safety_latched_) {
    offboard_active_ = true;
  }
  const bool pending_state_confirmed = command_pending_ && !queued_command_ && connected &&
    ((pending_command_code_ == FlightCommand::Request::SET_OFFBOARD &&
    fcu_state_.mode == "OFFBOARD") ||
    (pending_command_code_ == FlightCommand::Request::ARM && fcu_state_.armed) ||
    (pending_command_code_ == FlightCommand::Request::LAND &&
    (fcu_state_.mode == "AUTO.LAND" || (landed_state_fresh(time) &&
    extended_state_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND))) ||
    (pending_command_code_ == FlightCommand::Request::DISARM && !fcu_state_.armed &&
    landed_state_fresh(time) &&
    extended_state_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND));
  if (connected && offboard_active_ && fcu_state_.mode != "OFFBOARD" &&
    fcu_state_.mode != "AUTO.LAND")
  {
    offboard_active_ = false;
    safety_latched_ = true;
    safety_reason_ = "FCU left OFFBOARD: " + fcu_state_.mode;
    if (!pending_state_confirmed) {stop_pending_command(safety_reason_);}
  }
  if (command_pending_) {
    const double age = std::chrono::duration<double>(time - command_started_).count();
    if (queued_command_) {
      const auto reason = command_block_reason(pending_command_code_);
      if (!reason.empty()) {
        stop_pending_command("queued command canceled: " + reason);
      } else if (age >= command_queue_timeout_s_) {
        stop_pending_command("MAVROS service readiness timeout");
      } else if (command_service_ready(pending_command_code_)) {
        const auto id = last_command_id_;
        const auto code = pending_command_code_;
        command_pending_ = queued_command_ = false;
        send_command(id, code);
      }
    } else {
      if (!pending_state_confirmed) {
        auto reason = command_block_reason(pending_command_code_);
        if (pending_command_code_ == FlightCommand::Request::SET_OFFBOARD &&
          fcu_state_.mode != pending_start_mode_)
        {
          reason = "FCU mode changed before OFFBOARD confirmation";
        } else if (pending_command_code_ == FlightCommand::Request::ARM &&
          fcu_state_.mode != "OFFBOARD")
        {
          reason = "OFFBOARD ownership lost before ARM confirmation";
        }
        if (!reason.empty()) {
          stop_pending_command("in-flight command invalidated; FCU result unknown: " + reason);
        }
      }
      if (command_pending_ && age >= command_timeout_s_) {
        stop_pending_command("MAVROS response timeout; actual FCU result unknown");
        if (!safety_latched_) {safety_started_ = time;}
        safety_latched_ = true;
        safety_reason_ = "MAVROS response timeout; actual FCU result unknown";
      }
    }
  }
  const bool airborne = fcu_state_.armed && airborne_or_unknown(time);
  if (!safety_latched_ && offboard_active_ && airborne &&
    (!connected || !position || !velocity || (fcu_state_.mode == "OFFBOARD" && !target)))
  {
    safety_latched_ = true;
    safety_started_ = time;
    safety_reason_ = !connected ? "FCU state stale/disconnected" :
      (!position ? "position stale/invalid" : (!velocity ? "velocity stale/invalid" : "target timeout"));
    if (position) {target_ = local_pose_; target_.header.frame_id = "map";}
    RCLCPP_ERROR(get_logger(), "Control inhibited: %s", safety_reason_.c_str());
  }
  if (!safety_latched_) {
    if (!read_only_ && connected && position && velocity && target && fcu_state_.mode != "AUTO.LAND") {
      if (stream_started_ == Clock::time_point{}) {stream_started_ = time;}
      auto setpoint = target_;
      setpoint.header.stamp = now();
      setpoint_pub_->publish(setpoint);
    } else {
      stream_started_ = Clock::time_point{};
    }
  } else if (!read_only_ && offboard_active_) {
    const double age = std::chrono::duration<double>(time - safety_started_).count();
    if (connected && position && fcu_state_.mode == "OFFBOARD" && have_target_ &&
      age <= land_handover_timeout_s_)
    {
      auto setpoint = target_;
      setpoint.header.stamp = now();
      setpoint_pub_->publish(setpoint);
    }
    if (!safety_land_sent_ && airborne && !command_pending_ &&
      send_command(0, FlightCommand::Request::LAND))
    {
      safety_land_sent_ = true;
    }
  }
  publish_state(time);
}

}  // namespace uav_control
