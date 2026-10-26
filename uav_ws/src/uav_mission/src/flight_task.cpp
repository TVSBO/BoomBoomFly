#include "uav_mission/flight_task.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uav_mission
{
namespace
{
double pose_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

FlightTask::FlightTask(const FlightTaskConfig & config)
: config_(config), stage_started_(Clock::now()), stage_name_("WAIT_READY")
{
  if (config_.mission_profile != "hover" && config_.mission_profile != "square" &&
    config_.mission_profile != "grid_s") {
    throw std::invalid_argument("mission_profile must be hover, square or grid_s");
  }
  for (double value : {config_.grid_spacing_m, config_.grid_speed_m_s,
      config_.grid_acceleration_m_s2, config_.grid_position_tolerance_m,
      config_.grid_arrival_speed_m_s, config_.grid_settle_duration_s,
      config_.grid_leg_timeout_s, config_.grid_boundary_margin_m,
      config_.grid_max_altitude_error_m})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Grid parameters must be finite and positive");
    }
  }
  if (config_.grid_round_trips < 1 ||
    config_.grid_position_tolerance_m >= config_.grid_spacing_m ||
    grid_leg_duration_s() + config_.grid_settle_duration_s >= config_.grid_leg_timeout_s)
  {
    throw std::invalid_argument("Grid needs positive rounds, tolerance below spacing and enough leg time");
  }
  for (double value : {config_.square_side_m, config_.square_position_tolerance_m,
      config_.square_max_speed_m_s, config_.square_settle_duration_s,
      config_.square_leg_timeout_s, config_.square_boundary_margin_m,
      config_.square_max_altitude_error_m})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Square mission parameters must be finite and positive");
    }
  }
  if (config_.square_position_tolerance_m >= config_.square_side_m ||
    config_.square_settle_duration_s >= config_.square_leg_timeout_s)
  {
    throw std::invalid_argument("Square tolerance must be smaller than the side, and settle time smaller than leg timeout");
  }
}

void FlightTask::enter(Stage stage, Clock::time_point now_time)
{
  stage_ = stage;
  stage_started_ = now_time;
  // Preserve an in-flight landing/disarm action while safety handling takes over.
  const bool safety_command = pending_command_ ==
    uav_interfaces::srv::FlightCommand::Request::LAND || pending_command_ ==
    uav_interfaces::srv::FlightCommand::Request::DISARM;
  if (stage != Stage::SAFETY_LAND || !safety_command)
  {
    pending_command_id_ = 0;
    pending_command_ = 0;
    command_dispatched_ = false;
    command_dispatched_ok_ = false;
  }
  switch (stage) {
    case Stage::WAIT_READY: stage_name_ = "WAIT_READY"; break;
    case Stage::WAIT_ARM: stage_name_ = "WAIT_ARM"; break;
    case Stage::PREPARE_OFFBOARD: stage_name_ = "PREPARE_OFFBOARD"; break;
    case Stage::REQUEST_OFFBOARD: stage_name_ = "REQUEST_OFFBOARD"; break;
    case Stage::TAKEOFF:
      stage_name_ = "TAKEOFF";
      if (takeoff_started_ == Clock::time_point{}) {
        takeoff_started_ = now_time;
      }
      break;
    case Stage::STABILIZE: stage_name_ = "STABILIZE"; break;
    case Stage::HOVER:
      stage_name_ = "HOVER";
      hover_measurements_valid_ = true;
      max_horizontal_error_m_ = 0.0;
      max_vertical_error_m_ = 0.0;
      max_yaw_error_rad_ = 0.0;
      break;
    case Stage::SQUARE_LEFT: stage_name_ = "SQUARE_LEFT"; break;
    case Stage::SQUARE_FORWARD: stage_name_ = "SQUARE_FORWARD"; break;
    case Stage::SQUARE_RIGHT: stage_name_ = "SQUARE_RIGHT"; break;
    case Stage::SQUARE_BACKWARD: stage_name_ = "SQUARE_BACKWARD"; break;
    case Stage::GRID_S_LEG: stage_name_ = "GRID_S_LEG"; break;
    case Stage::REQUEST_LAND:
      stage_name_ = "REQUEST_LAND";
      landing_started_ = now_time;
      break;
    case Stage::REQUEST_DISARM: stage_name_ = "REQUEST_DISARM"; break;
    case Stage::LANDING: stage_name_ = "LANDING"; break;
    case Stage::SAFETY_LAND: stage_name_ = "SAFETY_LAND"; break;
    case Stage::FINISHED: stage_name_ = "FINISHED"; break;
  }
}

void FlightTask::begin_safety(Clock::time_point now_time)
{
  if (hover_result_ == TaskResult::PENDING) {hover_result_ = TaskResult::FAILED;}
  if (is_square_stage()) {square_result_ = TaskResult::FAILED;}
  if (stage_ == Stage::GRID_S_LEG) {grid_result_ = TaskResult::FAILED;}
  if (stage_ == Stage::SAFETY_LAND) {return;}
  safety_retry_after_ = now_time;
  if (last_state_fresh_ && last_state_.position_valid && last_state_.heading_valid) {
    const auto & p = last_state_.local_pose.pose.position;
    const double yaw = pose_yaw(last_state_.local_pose.pose.orientation);
    set_target(p.x, p.y, p.z, yaw);
  }
  enter(Stage::SAFETY_LAND, now_time);
}

void FlightTask::begin_failure(const std::string & reason, Clock::time_point now_time)
{
  if (failure_reason_.empty()) {
    failure_reason_ = reason;
  }
  if (flight_may_be_armed_) {
    begin_safety(now_time);
  } else {
    control_released_ = last_state_fresh_ && last_state_.armed;
    finish(false, false);
  }
}

void FlightTask::finish(bool passed, bool canceled)
{
  if (!passed && hover_result_ == TaskResult::PENDING) {hover_result_ = TaskResult::FAILED;}
  if (!passed && is_square_stage()) {square_result_ = TaskResult::FAILED;}
  if (!passed && stage_ == Stage::GRID_S_LEG) {grid_result_ = TaskResult::FAILED;}
  passed_ = passed;
  canceled_ = canceled;
  finished_ = true;
  enter(Stage::FINISHED, Clock::now());
}

void FlightTask::request_command(uint8_t command, Clock::time_point now_time)
{
  pending_command_id_ = next_command_id_++;
  pending_command_ = command;
  command_dispatched_ = false;
  command_dispatched_ok_ = false;
  command_started_ = now_time;
}

bool FlightTask::status_valid(
  const uav_interfaces::msg::VehicleState * state, bool state_stream_fresh) const
{
  return state && state_stream_fresh && state->state_valid && state->connected &&
    state->landed_state_valid;
}

bool FlightTask::flight_state_valid(
  const uav_interfaces::msg::VehicleState * state, bool state_stream_fresh) const
{
  return status_valid(state, state_stream_fresh) && state->position_valid &&
    state->heading_valid && state->velocity_valid &&
    (!state->estimator_health_valid || state->estimator_healthy) &&
    (!state->failsafe_state_valid || !state->failsafe_active);
}

bool FlightTask::command_accepted(const uav_interfaces::msg::VehicleState * state) const
{
  return command_dispatched_ && command_dispatched_ok_ && state &&
    state->last_command_id == pending_command_id_ && state->command_completed &&
    state->command_accepted;
}

bool FlightTask::command_rejected(const uav_interfaces::msg::VehicleState * state) const
{
  return command_dispatched_ && command_dispatched_ok_ && state &&
    state->last_command_id == pending_command_id_ && state->command_completed &&
    !state->command_accepted;
}

double FlightTask::horizontal_error(const uav_interfaces::msg::VehicleState & state) const
{
  const auto & p = state.local_pose.pose.position;
  return std::hypot(p.x - target_.pose.position.x, p.y - target_.pose.position.y);
}

double FlightTask::vertical_error(const uav_interfaces::msg::VehicleState & state) const
{
  return std::abs(state.local_pose.pose.position.z - target_z_);
}

double FlightTask::yaw_error(const uav_interfaces::msg::VehicleState & state) const
{
  return std::abs(wrap_angle(pose_yaw(state.local_pose.pose.orientation) - home_yaw_));
}

void FlightTask::update_errors(const uav_interfaces::msg::VehicleState & state)
{
  max_horizontal_error_m_ = std::max(max_horizontal_error_m_, horizontal_error(state));
  max_vertical_error_m_ = std::max(max_vertical_error_m_, vertical_error(state));
  max_yaw_error_rad_ = std::max(max_yaw_error_rad_, yaw_error(state));
}

void FlightTask::set_target(double x, double y, double z, double yaw)
{
  target_.header.frame_id = "map";
  target_.pose.position.x = x;
  target_.pose.position.y = y;
  target_.pose.position.z = z;
  target_.pose.orientation.x = 0.0;
  target_.pose.orientation.y = 0.0;
  target_.pose.orientation.z = std::sin(yaw / 2.0);
  target_.pose.orientation.w = std::cos(yaw / 2.0);
  target_valid_ = true;
}

bool FlightTask::is_square_stage() const
{
  return stage_ == Stage::SQUARE_LEFT || stage_ == Stage::SQUARE_FORWARD ||
    stage_ == Stage::SQUARE_RIGHT || stage_ == Stage::SQUARE_BACKWARD;
}

void FlightTask::start_square_leg(Stage stage, Clock::time_point now_time)
{
  // Adapted from dofarifa/Unmanned-Aerial-Vehicle, f4ad78e83a03627eae62552dcf1d7eddc16fbf2e,
  // src/uav_controller/src/waypoint_flight_node.cpp::set_square_target.
  // The original NED offsets are expressed here in the existing z-up map frame.
  // Freeze axes at takeoff: left -> forward -> right -> backward, with fixed yaw.
  const double forward_x = config_.square_side_m * std::cos(home_yaw_);
  const double forward_y = config_.square_side_m * std::sin(home_yaw_);
  const double left_x = -forward_y;
  const double left_y = forward_x;
  double x = home_x_;
  double y = home_y_;
  if (stage == Stage::SQUARE_LEFT || stage == Stage::SQUARE_FORWARD) {
    x += left_x;
    y += left_y;
  }
  if (stage == Stage::SQUARE_FORWARD || stage == Stage::SQUARE_RIGHT) {
    x += forward_x;
    y += forward_y;
  }
  set_target(x, y, target_z_, home_yaw_);
  have_stable_since_ = false;
  enter(stage, now_time);
}

std::array<double, 2> FlightTask::grid_point(unsigned index) const
{
  // Forward/left coordinates: lower-right start, eight S-path steps.
  static constexpr std::array<std::array<int, 2>, 9> points{{
    {{0, 0}}, {{0, 1}}, {{0, 2}}, {{1, 2}}, {{1, 1}},
    {{1, 0}}, {{2, 0}}, {{2, 1}}, {{2, 2}}}};
  const double f = points.at(index)[0] * config_.grid_spacing_m;
  const double l = points.at(index)[1] * config_.grid_spacing_m;
  return {home_x_ + f * std::cos(home_yaw_) - l * std::sin(home_yaw_),
    home_y_ + f * std::sin(home_yaw_) + l * std::cos(home_yaw_)};
}

double FlightTask::grid_leg_duration_s() const
{
  // Quintic s(u)=10u^3-15u^4+6u^5: max s'=1.875, max |s''|=10/sqrt(3).
  return std::max(1.875 * config_.grid_spacing_m / config_.grid_speed_m_s,
    std::sqrt((10.0 / std::sqrt(3.0)) * config_.grid_spacing_m /
    config_.grid_acceleration_m_s2));
}

void FlightTask::start_grid_leg(Clock::time_point now_time)
{
  const unsigned start = grid_leg_in_round_ < 8 ? grid_leg_in_round_ : 16 - grid_leg_in_round_;
  const auto p = grid_point(start);
  set_target(p[0], p[1], target_z_, home_yaw_);
  have_stable_since_ = false;
  enter(Stage::GRID_S_LEG, now_time);
  stage_name_ = std::string(grid_leg_in_round_ < 8 ? "GRID_S_OUT_" : "GRID_S_RETURN_") +
    std::to_string(grid_leg_in_round_ % 8 + 1);
}

FlightTask::Output FlightTask::output(Clock::time_point)
{
  Output result;
  result.hover_result = hover_result_;
  result.square_result = square_result_;
  result.grid_result = grid_result_;
  result.publish_target = target_valid_ && !finished_ && stage_ != Stage::SAFETY_LAND &&
    stage_ != Stage::WAIT_READY && stage_ != Stage::WAIT_ARM;
  result.target = target_;
  result.finished = finished_;
  result.passed = passed_;
  result.canceled = canceled_;
  result.control_released = control_released_;
  return result;
}

FlightTask::Output FlightTask::update(
  const uav_interfaces::msg::VehicleState * state,
  bool state_stream_fresh, Clock::time_point now_time, bool vio_health_ok,
  const std::string & vio_health_reason)
{
  // Require a fresh observed edge, never a cached armed state after a data gap.
  const bool arm_observation_valid = status_valid(state, state_stream_fresh);
  const bool arm_rising_edge = arm_observation_valid && have_arm_observation_ &&
    !previous_armed_ && state->armed;
  have_arm_observation_ = arm_observation_valid;
  if (arm_observation_valid) {
    previous_armed_ = state->armed;
  }
  if (state && state_stream_fresh) {
    last_state_ = *state;
    last_state_fresh_ = true;
    if (control_acquired_ && state->armed) {
      flight_may_be_armed_ = true;
    }
    if (state->position_valid) {
      final_horizontal_displacement_m_ = std::hypot(
        state->local_pose.pose.position.x - home_x_, state->local_pose.pose.position.y - home_y_);
      final_vertical_displacement_m_ =
        state->local_pose.pose.position.z - home_z_;
      if (target_valid_) {
        last_horizontal_target_error_m_ = horizontal_error(*state);
        last_vertical_target_error_m_ = vertical_error(*state);
        have_position_measurement_ = true;
      }
    }
  } else {
    last_state_fresh_ = false;
  }

  if (!finished_ && control_acquired_ && status_valid(state, state_stream_fresh) &&
    state->mode != "OFFBOARD" && state->mode != "AUTO.LAND")
  {
    failure_reason_ = "飞控控制权已交出，停止自动命令: " + state->mode;
    control_released_ = true;
    finish(false, cancel_requested_);
  }

  if (cancel_requested_ && stage_ != Stage::SAFETY_LAND && stage_ != Stage::FINISHED) {
    if (flight_may_be_armed_) {
      canceled_ = true;
      failure_reason_ = "任务收到取消请求";
      begin_safety(now_time);
    } else {
      finish(false, true);
    }
  }
  if (finished_) {
    return output(now_time);
  }

  if (stage_ == Stage::WAIT_READY) {
    if (status_valid(state, state_stream_fresh) && state->armed) {
      begin_failure("任务启动时已解锁；请先上锁，再启动任务并用遥控器解锁", now_time);
      return output(now_time);
    }
    if (flight_state_valid(state, state_stream_fresh) &&
      (!config_.require_vio_health || vio_health_ok) && !state->control_inhibited && !state->armed &&
      (!state->preflight_check_state_valid || state->preflight_checks_passed) &&
      state->landed_state == uav_interfaces::msg::VehicleState::LANDED_ON_GROUND)
    {
      enter(Stage::WAIT_ARM, now_time);
    } else if (std::chrono::duration<double>(now_time - stage_started_).count() >=
      config_.ready_timeout_s)
    {
      begin_failure("等待飞控连接、有效新鲜定位和地面状态超时", now_time);
    }
    return output(now_time);
  }

  if (stage_ == Stage::WAIT_ARM) {
    const bool ready = flight_state_valid(state, state_stream_fresh) &&
      (!config_.require_vio_health || vio_health_ok) && !state->control_inhibited &&
      (!state->preflight_check_state_valid || state->preflight_checks_passed) &&
      state->landed_state == uav_interfaces::msg::VehicleState::LANDED_ON_GROUND;
    if (arm_observation_valid && state->armed) {
      if (!arm_rising_edge || !ready) {
        begin_failure("解锁时任务未就绪或未观察到新鲜解锁变化；请上锁后重新启动任务", now_time);
      } else {
        // The pilot owns arming. Capture the origin at the observed arm edge.
        const auto & pose = state->local_pose.pose;
        home_x_ = pose.position.x;
        home_y_ = pose.position.y;
        home_z_ = pose.position.z;
        home_yaw_ = pose_yaw(pose.orientation);
        target_z_ = home_z_ + config_.takeoff_height_m;
        arm_mode_ = state->mode;
        set_target(home_x_, home_y_, home_z_, home_yaw_);
        enter(Stage::PREPARE_OFFBOARD, now_time);
      }
    } else if (!ready) {
      enter(Stage::WAIT_READY, now_time);
    }
    // Waiting for the pilot has no timeout and sends no target or flight command.
    return output(now_time);
  }

  if (stage_ == Stage::SAFETY_LAND) {
    Output result = output(now_time);
    if (!status_valid(state, state_stream_fresh)) {
      result.publish_target = false;
      return result;
    }

    const bool on_ground =
      state->landed_state == uav_interfaces::msg::VehicleState::LANDED_ON_GROUND;
    const double pending_age = std::chrono::duration<double>(now_time - command_started_).count();
    const bool command_timed_out = pending_age >= config_.command_timeout_s;
    const auto clear_pending = [this]() {
        pending_command_id_ = 0;
        pending_command_ = 0;
        command_dispatched_ = false;
        command_dispatched_ok_ = false;
      };
    if (pending_command_id_ != 0) {
      if (pending_command_ == uav_interfaces::srv::FlightCommand::Request::LAND) {
        const bool actual_handover = state->mode == "AUTO.LAND" || on_ground;
        if (command_rejected(state)) {
          failure_reason_ += "; 安全降落命令被拒绝: " + state->command_message;
          safety_command_failed_ = true;
          clear_pending();
        } else if (command_accepted(state) && actual_handover) {
          clear_pending();
        } else if (command_timed_out) {
          failure_reason_ += "; 安全降落命令最终确认或 AUTO.LAND 模式确认超时";
          safety_command_failed_ = true;
          clear_pending();
        }
      } else if (pending_command_ == uav_interfaces::srv::FlightCommand::Request::DISARM) {
        if (command_accepted(state) && !state->armed) {
          clear_pending();
        } else if (command_rejected(state) || command_timed_out) {
          failure_reason_ += command_rejected(state) ?
            "; 地面 DISARM 命令未确认: " + state->command_message :
            "; 地面 DISARM 最终确认超时";
          clear_pending();
          safety_retry_after_ = now_time + std::chrono::seconds(1);
        }
      }
    }
    if (pending_command_id_ == 0 && on_ground && !state->armed) {
      finish(false, canceled_);
      return output(now_time);
    }

    if (on_ground) {
      result.publish_target = false;
      if (pending_command_id_ == 0 && !safety_disarm_blocked_ &&
        now_time >= safety_retry_after_)
      {
        request_command(uav_interfaces::srv::FlightCommand::Request::DISARM, now_time);
        result.send_command = true;
        result.command_id = pending_command_id_;
        result.command = pending_command_;
      }
      return result;
    }

    const bool auto_land = state->mode == "AUTO.LAND";
    result.publish_target = !auto_land && !safety_command_failed_ && !state->control_inhibited &&
      state->position_valid && state->heading_valid && state->mode == "OFFBOARD";
    if (state->position_valid && state->heading_valid) {
      const auto & p = state->local_pose.pose.position;
      set_target(p.x, p.y, p.z, pose_yaw(state->local_pose.pose.orientation));
      result.target = target_;
    }
    if (auto_land) {
      return result;
    }

    // The backend owns target-loss landing once it reports control inhibition.
    if (!safety_command_failed_ && !state->control_inhibited &&
      pending_command_id_ == 0 && state->position_valid && state->heading_valid &&
      state->mode == "OFFBOARD" &&
      now_time >= safety_retry_after_)
    {
      request_command(uav_interfaces::srv::FlightCommand::Request::LAND, now_time);
      result.send_command = true;
      result.command_id = pending_command_id_;
      result.command = pending_command_;
    }
    return result;
  }

  if (!status_valid(state, state_stream_fresh)) {
    const char * reason = !state || !state_stream_fresh ? "统一飞控状态消息过期或缺失" :
      !state->state_valid ? "飞控状态或时间同步无效、过期" :
      !state->connected ? "飞控连接断开" : "着陆状态未知、过期";
    begin_failure(reason, now_time);
    return output(now_time);
  }

  const bool confirmed_landing = stage_ == Stage::LANDING || stage_ == Stage::REQUEST_DISARM ||
    (stage_ == Stage::REQUEST_LAND && state->mode == "AUTO.LAND");
  if (!confirmed_landing && !flight_state_valid(state, state_stream_fresh)) {
    begin_failure("定位、航向、速度或导航估计健康状态无效", now_time);
    return output(now_time);
  }
  if (!confirmed_landing && config_.require_vio_health && !vio_health_ok) {
    begin_failure("VIO bridge health lost: " + (vio_health_reason.empty() ?
      std::string("diagnostic is stale or unhealthy") : vio_health_reason), now_time);
    return output(now_time);
  }
  if (!confirmed_landing && state->failsafe_state_valid && state->failsafe_active) {
    begin_failure("飞控报告 failsafe", now_time);
    return output(now_time);
  }

  const bool land_stage = stage_ == Stage::REQUEST_LAND || stage_ == Stage::LANDING ||
    stage_ == Stage::REQUEST_DISARM;
  if (state->control_inhibited && !land_stage) {
    begin_failure("控制后端禁止继续任务: " + state->control_message, now_time);
    return output(now_time);
  }
  const bool preparing = stage_ == Stage::PREPARE_OFFBOARD || stage_ == Stage::REQUEST_OFFBOARD;
  if (preparing && state->mode != "OFFBOARD" &&
    (state->mode != arm_mode_ ||
    state->landed_state != uav_interfaces::msg::VehicleState::LANDED_ON_GROUND))
  {
    failure_reason_ = "切入 Offboard 前遥控模式改变或已人工起飞，停止自动任务";
    control_released_ = true;
    finish(false, false);
    return output(now_time);
  }
  if (land_stage) {
    if (stage_ == Stage::REQUEST_LAND && state->mode != "OFFBOARD" &&
      state->mode != "AUTO.LAND")
    {
      begin_failure("降落请求前 Offboard 模式意外改变为 " + state->mode, now_time);
      return output(now_time);
    }
    if (stage_ != Stage::REQUEST_LAND && state->mode != "AUTO.LAND" &&
      state->landed_state != uav_interfaces::msg::VehicleState::LANDED_ON_GROUND)
    {
      begin_failure("降落期间飞控模式离开 AUTO.LAND", now_time);
      return output(now_time);
    }
  } else if (stage_ != Stage::PREPARE_OFFBOARD && stage_ != Stage::REQUEST_OFFBOARD &&
    state->mode != "OFFBOARD") {
    begin_failure("Offboard 模式意外改变为 " + state->mode, now_time);
    return output(now_time);
  }

  if ((preparing || stage_ == Stage::TAKEOFF || stage_ == Stage::STABILIZE || stage_ == Stage::HOVER ||
    is_square_stage() || stage_ == Stage::GRID_S_LEG) &&
    !state->armed)
  {
    begin_failure("遥控器或飞控已上锁，停止自动任务", now_time);
    return output(now_time);
  }

  Output result = output(now_time);
  const auto elapsed = std::chrono::duration<double>(now_time - stage_started_).count();
  switch (stage_) {
    case Stage::PREPARE_OFFBOARD:
      if (!flight_state_valid(state, state_stream_fresh)) {
        begin_failure("Offboard 准备期间状态或定位失效", now_time);
      } else if (elapsed >= config_.prestream_s) {
        enter(Stage::REQUEST_OFFBOARD, now_time);
      }
      break;

    case Stage::REQUEST_OFFBOARD:
      if (pending_command_id_ == 0) {
        // Once mode switching is requested, its actual outcome may arrive late.
        // Failure/cancel must use the existing ground-disarm/landing cleanup.
        flight_may_be_armed_ = true;
        request_command(uav_interfaces::srv::FlightCommand::Request::SET_OFFBOARD, now_time);
        result.send_command = true;
        result.command_id = pending_command_id_;
        result.command = pending_command_;
      } else if (command_rejected(state)) {
        begin_failure("MAVROS/PX4 拒绝 Offboard 请求: " + state->command_message, now_time);
      } else if (command_accepted(state) && state->mode == "OFFBOARD") {
        control_acquired_ = true;
        set_target(home_x_, home_y_, target_z_, home_yaw_);
        result.target = target_;
        enter(Stage::TAKEOFF, now_time);
      } else if (std::chrono::duration<double>(now_time - command_started_).count() >=
        config_.command_timeout_s)
      {
        begin_failure("Offboard 命令接受或模式确认超时", now_time);
      }
      break;

    case Stage::TAKEOFF:
      if (std::chrono::duration<double>(now_time - takeoff_started_).count() >=
        config_.takeoff_timeout_s)
      {
        begin_failure("起飞到目标高度超时", now_time);
      } else if (state->landed_state == uav_interfaces::msg::VehicleState::LANDED_IN_AIR &&
        horizontal_error(*state) <= config_.horizontal_tolerance_m &&
        vertical_error(*state) <= config_.vertical_tolerance_m)
      {
        stable_since_ = now_time;
        have_stable_since_ = true;
        enter(Stage::STABILIZE, now_time);
      }
      break;

    case Stage::STABILIZE:
      update_errors(*state);
      if (horizontal_error(*state) > config_.horizontal_tolerance_m ||
        vertical_error(*state) > config_.vertical_tolerance_m)
      {
        have_stable_since_ = false;
        enter(Stage::TAKEOFF, now_time);
      } else if (!have_stable_since_) {
        stable_since_ = now_time;
        have_stable_since_ = true;
      } else if (std::chrono::duration<double>(now_time - stable_since_).count() >=
        config_.stable_duration_s)
      {
        enter(Stage::HOVER, now_time);
      }
      if (std::chrono::duration<double>(now_time - takeoff_started_).count() >=
        config_.takeoff_timeout_s)
      {
        begin_failure("起飞后连续稳定超时", now_time);
      }
      break;

    case Stage::HOVER:
      update_errors(*state);
      if (horizontal_error(*state) > config_.horizontal_tolerance_m ||
        vertical_error(*state) > config_.vertical_tolerance_m)
      {
        begin_failure("悬停期间位置误差超限", now_time);
      } else if (state->landed_state != uav_interfaces::msg::VehicleState::LANDED_IN_AIR) {
        begin_failure("悬停期间未确认在空中，禁止进入后续任务", now_time);
      } else if (elapsed >= config_.hover_duration_s) {
        hover_result_ = TaskResult::PASSED;
        if (config_.mission_profile == "square") {
          start_square_leg(Stage::SQUARE_LEFT, now_time);
        } else if (config_.mission_profile == "grid_s") {
          start_grid_leg(now_time);
        } else {
          enter(Stage::REQUEST_LAND, now_time);
        }
      }
      break;

    case Stage::SQUARE_LEFT:
    case Stage::SQUARE_FORWARD:
    case Stage::SQUARE_RIGHT:
    case Stage::SQUARE_BACKWARD:
    {
      const auto & p = state->local_pose.pose.position;
      const auto & v = state->local_velocity.vector;
      const double radius = std::hypot(p.x - home_x_, p.y - home_y_);
      const double max_radius = std::sqrt(2.0) * config_.square_side_m +
        config_.square_boundary_margin_m;
      if (!std::isfinite(radius) || !std::isfinite(p.z) || radius > max_radius ||
        vertical_error(*state) > config_.square_max_altitude_error_m)
      {
        begin_failure("Square mission position/altitude envelope exceeded", now_time);
      } else if (elapsed >= config_.square_leg_timeout_s) {
        begin_failure("Square leg timeout: " + stage_name_, now_time);
      } else {
        const double distance = std::hypot(horizontal_error(*state), vertical_error(*state));
        const double speed = std::hypot(std::hypot(v.x, v.y), v.z);
        if (!std::isfinite(distance) || !std::isfinite(speed) ||
          distance > config_.square_position_tolerance_m || speed > config_.square_max_speed_m_s)
        {
          have_stable_since_ = false;
        } else if (!have_stable_since_) {
          stable_since_ = now_time;
          have_stable_since_ = true;
        } else if (std::chrono::duration<double>(now_time - stable_since_).count() >=
          config_.square_settle_duration_s)
        {
          ++completed_square_legs_;
          if (stage_ == Stage::SQUARE_LEFT) {
            start_square_leg(Stage::SQUARE_FORWARD, now_time);
          } else if (stage_ == Stage::SQUARE_FORWARD) {
            start_square_leg(Stage::SQUARE_RIGHT, now_time);
          } else if (stage_ == Stage::SQUARE_RIGHT) {
            start_square_leg(Stage::SQUARE_BACKWARD, now_time);
          } else {
            square_result_ = TaskResult::PASSED;
            enter(Stage::REQUEST_LAND, now_time);
          }
        }
      }
      break;
    }

    case Stage::GRID_S_LEG:
    {
      const auto & p = state->local_pose.pose.position;
      const auto & v = state->local_velocity.vector;
      const double dx = p.x - home_x_, dy = p.y - home_y_;
      const double forward = dx * std::cos(home_yaw_) + dy * std::sin(home_yaw_);
      const double left = -dx * std::sin(home_yaw_) + dy * std::cos(home_yaw_);
      const double margin = config_.grid_boundary_margin_m;
      const double extent = 2.0 * config_.grid_spacing_m;
      if (!std::isfinite(forward) || !std::isfinite(left) || !std::isfinite(p.z) ||
        forward < -margin || forward > extent + margin || left < -margin || left > extent + margin ||
        std::abs(p.z - target_z_) > config_.grid_max_altitude_error_m ||
        state->landed_state != uav_interfaces::msg::VehicleState::LANDED_IN_AIR)
      {
        begin_failure("Grid position/altitude envelope exceeded or not airborne", now_time);
      } else if (elapsed >= config_.grid_leg_timeout_s) {
        begin_failure("Grid leg timeout: " + stage_name_, now_time);
      } else {
        const unsigned start = grid_leg_in_round_ < 8 ? grid_leg_in_round_ : 16 - grid_leg_in_round_;
        const unsigned end = grid_leg_in_round_ < 8 ? start + 1 : start - 1;
        const auto a = grid_point(start), b = grid_point(end);
        const double duration = grid_leg_duration_s();
        const double u = std::clamp(elapsed / duration, 0.0, 1.0);
        const double blend = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
        set_target(a[0] + (b[0] - a[0]) * blend,
          a[1] + (b[1] - a[1]) * blend, target_z_, home_yaw_);
        const double error = std::hypot(std::hypot(p.x - b[0], p.y - b[1]), p.z - target_z_);
        const double speed = std::hypot(std::hypot(v.x, v.y), v.z);
        if (elapsed < duration || !std::isfinite(error) || !std::isfinite(speed) ||
          error > config_.grid_position_tolerance_m || speed > config_.grid_arrival_speed_m_s)
        {
          have_stable_since_ = false;
        } else if (!have_stable_since_) {
          stable_since_ = now_time;
          have_stable_since_ = true;
        } else if (std::chrono::duration<double>(now_time - stable_since_).count() >=
          config_.grid_settle_duration_s)
        {
          ++completed_grid_legs_;
          if (++grid_leg_in_round_ == 16) {
            ++completed_grid_round_trips_;
            grid_leg_in_round_ = 0;
          }
          if (completed_grid_round_trips_ == static_cast<unsigned>(config_.grid_round_trips)) {
            grid_result_ = TaskResult::PASSED;
            enter(Stage::REQUEST_LAND, now_time);
          } else {
            start_grid_leg(now_time);
          }
        }
      }
      break;
    }

    case Stage::REQUEST_LAND:
      if (pending_command_id_ == 0) {
        request_command(uav_interfaces::srv::FlightCommand::Request::LAND, now_time);
        result.send_command = true;
        result.command_id = pending_command_id_;
        result.command = pending_command_;
      } else if (command_rejected(state)) {
        begin_failure("MAVROS/PX4 拒绝降落请求: " + state->command_message, now_time);
      } else if (command_accepted(state) && state->mode == "AUTO.LAND") {
        enter(Stage::LANDING, now_time);
      } else if (std::chrono::duration<double>(now_time - command_started_).count() >=
        config_.command_timeout_s)
      {
        begin_failure("降落命令服务接受或 AUTO.LAND 模式确认超时", now_time);
      }
      break;

    case Stage::REQUEST_DISARM:
      if (state->landed_state != uav_interfaces::msg::VehicleState::LANDED_ON_GROUND) {
        begin_failure("解锁状态确认期间地面着陆状态丢失", now_time);
      } else if (pending_command_id_ == 0) {
        if (!state->armed) {
          finish(true, false);
        } else {
          request_command(uav_interfaces::srv::FlightCommand::Request::DISARM, now_time);
          result.send_command = true;
          result.command_id = pending_command_id_;
          result.command = pending_command_;
        }
      } else if (command_rejected(state)) {
        begin_failure("MAVROS/PX4 拒绝上锁请求: " + state->command_message, now_time);
      } else if (command_accepted(state) && !state->armed) {
        finish(true, false);
      } else if (std::chrono::duration<double>(now_time - command_started_).count() >=
        config_.command_timeout_s)
      {
        begin_failure("上锁状态确认超时", now_time);
      }
      break;

    case Stage::LANDING:
      if (state->landed_state == uav_interfaces::msg::VehicleState::LANDED_ON_GROUND) {
        result.publish_target = false;
        if (!state->armed) {
          finish(true, false);
        } else {
          enter(Stage::REQUEST_DISARM, now_time);
        }
      } else if (std::chrono::duration<double>(now_time - landing_started_).count() >=
        config_.landing_timeout_s)
      {
        begin_failure("等待飞控报告着陆超时", now_time);
      }
      result.publish_target = state->mode != "AUTO.LAND";
      break;

    case Stage::WAIT_READY:
    case Stage::WAIT_ARM:
    case Stage::SAFETY_LAND:
    case Stage::FINISHED:
      break;
  }

  result.hover_result = hover_result_;
  result.square_result = square_result_;
  result.grid_result = grid_result_;
  result.finished = finished_;
  result.passed = passed_;
  result.canceled = canceled_;
  result.target = target_;
  if (stage_ == Stage::SAFETY_LAND) {
    result = output(now_time);
    result.publish_target = status_valid(state, state_stream_fresh) && state->position_valid &&
      state->heading_valid && !safety_command_failed_ && !state->control_inhibited &&
      state->mode == "OFFBOARD";
  }
  if (finished_) {
    result.publish_target = false;
  }
  return result;
}

void FlightTask::command_dispatched(
  uint32_t command_id, bool dispatched, const std::string & message, Clock::time_point now_time)
{
  if (command_id != pending_command_id_) {
    return;
  }
  command_dispatched_ = true;
  command_dispatched_ok_ = dispatched;
  if (!dispatched) {
    const auto command = pending_command_;
    const bool quarantined = message.find("quarantined") != std::string::npos;
    pending_command_id_ = 0;
    pending_command_ = 0;
    command_dispatched_ = false;
    command_dispatched_ok_ = false;
    if (stage_ == Stage::SAFETY_LAND) {
      if (command == uav_interfaces::srv::FlightCommand::Request::LAND) {
        safety_command_failed_ = quarantined || std::chrono::duration<double>(
          now_time - stage_started_).count() >= config_.command_timeout_s;
      } else if (command == uav_interfaces::srv::FlightCommand::Request::DISARM && quarantined) {
        safety_disarm_blocked_ = true;
        failure_reason_ += "; 后端隔离了ARM/DISARM共用的PX4 opcode，需核对飞控并重启DDS后端";
      }
      safety_retry_after_ = now_time + std::chrono::seconds(1);
    } else {
      begin_failure("控制命令未能转发: " + message, now_time);
    }
  }
}

void FlightTask::cancel(Clock::time_point now_time)
{
  if (!finished_ && !cancel_requested_) {
    cancel_requested_ = true;
    if (flight_may_be_armed_) {
      canceled_ = true;
      failure_reason_ = "任务收到取消请求";
      begin_safety(now_time);
    } else {
      control_released_ = last_state_fresh_ && last_state_.armed;
      failure_reason_ = "任务取消；尚未请求 Offboard，控制权保留在遥控器";
      finish(false, true);
    }
  }
}

const std::string & FlightTask::stage_name() const
{
  return stage_name_;
}

const std::string & FlightTask::result_reason() const
{
  return failure_reason_;
}

double FlightTask::max_horizontal_error_m() const { return max_horizontal_error_m_; }
double FlightTask::max_vertical_error_m() const { return max_vertical_error_m_; }
double FlightTask::max_yaw_error_rad() const { return max_yaw_error_rad_; }
double FlightTask::final_horizontal_displacement_m() const
{
  return final_horizontal_displacement_m_;
}
double FlightTask::final_vertical_displacement_m() const { return final_vertical_displacement_m_; }
bool FlightTask::have_hover_measurements() const { return hover_measurements_valid_; }
double FlightTask::last_horizontal_target_error_m() const
{
  return last_horizontal_target_error_m_;
}
double FlightTask::last_vertical_target_error_m() const { return last_vertical_target_error_m_; }
bool FlightTask::have_position_measurement() const { return have_position_measurement_; }
unsigned FlightTask::completed_square_legs() const { return completed_square_legs_; }

}  // namespace uav_mission
