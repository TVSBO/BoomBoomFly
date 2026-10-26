#pragma once

#include <chrono>
#include <array>
#include <cstdint>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/srv/flight_command.hpp>

namespace uav_mission
{

struct FlightTaskConfig
{
  std::string mission_profile{"hover"};
  double grid_spacing_m{0.4};
  int grid_round_trips{1};
  double grid_speed_m_s{0.15};
  double grid_acceleration_m_s2{0.2};
  double grid_position_tolerance_m{0.08};
  double grid_arrival_speed_m_s{0.15};
  double grid_settle_duration_s{0.5};
  double grid_leg_timeout_s{15.0};
  double grid_boundary_margin_m{0.25};
  double grid_max_altitude_error_m{0.25};
  double square_side_m{0.5};
  double square_position_tolerance_m{0.08};
  double square_max_speed_m_s{0.15};
  double square_settle_duration_s{0.5};
  double square_leg_timeout_s{10.0};
  double square_boundary_margin_m{0.25};
  double square_max_altitude_error_m{0.25};
  double takeoff_height_m{1.0};
  double horizontal_tolerance_m{0.2};
  double vertical_tolerance_m{0.15};
  double stable_duration_s{2.0};
  double hover_duration_s{10.0};
  double prestream_s{1.2};
  double ready_timeout_s{30.0};
  double command_timeout_s{7.0};
  double takeoff_timeout_s{30.0};
  double landing_timeout_s{45.0};
  bool require_vio_health{false};
};

class FlightTask
{
public:
  using Clock = std::chrono::steady_clock;

  // Shared subtask result, distinct from final landing / mission success.
  enum class TaskResult : uint8_t { PENDING = 0, PASSED = 1, FAILED = 2 };

  struct Output
  {
    TaskResult hover_result{TaskResult::PENDING};
    TaskResult square_result{TaskResult::PENDING};
    TaskResult grid_result{TaskResult::PENDING};
    bool publish_target{false};
    geometry_msgs::msg::PoseStamped target;
    bool send_command{false};
    uint32_t command_id{0};
    uint8_t command{0};
    bool finished{false};
    bool passed{false};
    bool canceled{false};
    bool control_released{false};
  };

  explicit FlightTask(const FlightTaskConfig & config);
  Output update(
    const uav_interfaces::msg::VehicleState * state,
    bool state_stream_fresh, Clock::time_point now, bool vio_health_ok = true,
    const std::string & vio_health_reason = {});
  void command_dispatched(
    uint32_t command_id, bool dispatched, const std::string & message, Clock::time_point now);
  void cancel(Clock::time_point now);

  const std::string & stage_name() const;
  const std::string & result_reason() const;
  double max_horizontal_error_m() const;
  double max_vertical_error_m() const;
  double max_yaw_error_rad() const;
  double final_horizontal_displacement_m() const;
  double final_vertical_displacement_m() const;
  bool have_hover_measurements() const;
  double last_horizontal_target_error_m() const;
  double last_vertical_target_error_m() const;
  bool have_position_measurement() const;
  unsigned completed_square_legs() const;
  TaskResult hover_result() const { return hover_result_; }
  TaskResult square_result() const { return square_result_; }
  TaskResult grid_result() const { return grid_result_; }
  unsigned completed_grid_legs() const { return completed_grid_legs_; }
  unsigned completed_grid_round_trips() const { return completed_grid_round_trips_; }

private:
  enum class Stage
  {
    WAIT_READY,
    WAIT_ARM,
    PREPARE_OFFBOARD,
    REQUEST_OFFBOARD,
    TAKEOFF,
    STABILIZE,
    HOVER,
    SQUARE_LEFT,
    SQUARE_FORWARD,
    SQUARE_RIGHT,
    SQUARE_BACKWARD,
    GRID_S_LEG,
    REQUEST_LAND,
    REQUEST_DISARM,
    LANDING,
    SAFETY_LAND,
    FINISHED
  };

  void enter(Stage stage, Clock::time_point now);
  void begin_failure(const std::string & reason, Clock::time_point now);
  void begin_safety(Clock::time_point now);
  void finish(bool passed, bool canceled);
  void request_command(uint8_t command, Clock::time_point now);
  bool command_accepted(const uav_interfaces::msg::VehicleState * state) const;
  bool command_rejected(const uav_interfaces::msg::VehicleState * state) const;
  bool status_valid(
    const uav_interfaces::msg::VehicleState * state, bool state_stream_fresh) const;
  bool flight_state_valid(
    const uav_interfaces::msg::VehicleState * state, bool state_stream_fresh) const;
  void update_errors(const uav_interfaces::msg::VehicleState & state);
  void set_target(double x, double y, double z, double yaw);
  void start_square_leg(Stage stage, Clock::time_point now);
  bool is_square_stage() const;
  std::array<double, 2> grid_point(unsigned index) const;
  double grid_leg_duration_s() const;
  void start_grid_leg(Clock::time_point now);
  double horizontal_error(const uav_interfaces::msg::VehicleState & state) const;
  double vertical_error(const uav_interfaces::msg::VehicleState & state) const;
  double yaw_error(const uav_interfaces::msg::VehicleState & state) const;
  Output output(Clock::time_point now);

  FlightTaskConfig config_;
  TaskResult hover_result_{TaskResult::PENDING};
  TaskResult square_result_{TaskResult::PENDING};
  TaskResult grid_result_{TaskResult::PENDING};
  unsigned grid_leg_in_round_{0};
  unsigned completed_grid_legs_{0};
  unsigned completed_grid_round_trips_{0};
  Stage stage_{Stage::WAIT_READY};
  Clock::time_point stage_started_{};
  Clock::time_point command_started_{};
  Clock::time_point takeoff_started_{};
  Clock::time_point landing_started_{};
  Clock::time_point stable_since_{};
  Clock::time_point safety_retry_after_{};
  bool have_stable_since_{false};
  bool flight_may_be_armed_{false};
  bool control_acquired_{false};
  bool control_released_{false};
  bool have_arm_observation_{false};
  bool previous_armed_{false};
  std::string arm_mode_;
  bool safety_command_failed_{false};
  bool safety_disarm_blocked_{false};
  bool last_state_fresh_{false};
  uav_interfaces::msg::VehicleState last_state_;
  bool hover_measurements_valid_{false};
  bool have_position_measurement_{false};
  bool target_valid_{false};
  bool command_dispatched_{false};
  bool command_dispatched_ok_{false};
  bool cancel_requested_{false};
  bool finished_{false};
  bool passed_{false};
  bool canceled_{false};
  uint32_t next_command_id_{1};
  uint32_t pending_command_id_{0};
  uint8_t pending_command_{0};
  std::string failure_reason_;
  std::string stage_name_;
  geometry_msgs::msg::PoseStamped target_;
  double home_x_{0.0};
  double home_y_{0.0};
  double home_z_{0.0};
  double home_yaw_{0.0};
  double target_z_{0.0};
  double max_horizontal_error_m_{0.0};
  double max_vertical_error_m_{0.0};
  double max_yaw_error_rad_{0.0};
  double final_horizontal_displacement_m_{0.0};
  double final_vertical_displacement_m_{0.0};
  double last_horizontal_target_error_m_{0.0};
  double last_vertical_target_error_m_{0.0};
  unsigned completed_square_legs_{0};
};

}  // namespace uav_mission
