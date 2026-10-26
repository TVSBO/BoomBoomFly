#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/srv/flight_command.hpp>

namespace uav_control
{

class MavrosInterface : public rclcpp::Node
{
public:
  explicit MavrosInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Clock = std::chrono::steady_clock;
  using FlightCommand = uav_interfaces::srv::FlightCommand;

  void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void extended_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void command_callback(
    const std::shared_ptr<FlightCommand::Request> request,
    std::shared_ptr<FlightCommand::Response> response);
  void control_tick();
  void publish_state(const Clock::time_point now);
  bool command_service_ready(uint8_t command) const;
  bool send_command(uint32_t command_id, uint8_t command);
  void finish_command(uint64_t generation, bool accepted, const std::string & message);
  void stop_pending_command(const std::string & reason);
  std::string command_block_reason(uint8_t command) const;
  bool timestamp_fresh(const builtin_interfaces::msg::Time & stamp, double timeout) const;

  bool state_fresh(const Clock::time_point now) const;
  bool pose_fresh(const Clock::time_point now) const;
  bool position_coordinates_fresh(const Clock::time_point now) const;
  bool heading_fresh(const Clock::time_point now) const;
  bool position_fresh(const Clock::time_point now) const;
  bool velocity_fresh(const Clock::time_point now) const;
  bool landed_state_fresh(const Clock::time_point now) const;
  bool target_fresh(const Clock::time_point now) const;
  bool airborne_or_unknown(const Clock::time_point now) const;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<uav_interfaces::msg::VehicleState>::SharedPtr state_pub_;
  rclcpp::Service<FlightCommand>::SharedPtr command_service_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::State fcu_state_;
  mavros_msgs::msg::ExtendedState extended_state_;
  geometry_msgs::msg::PoseStamped local_pose_;
  geometry_msgs::msg::Vector3Stamped local_velocity_;
  geometry_msgs::msg::PoseStamped target_;
  Clock::time_point state_rx_{};
  Clock::time_point pose_rx_{};
  Clock::time_point velocity_rx_{};
  Clock::time_point extended_state_rx_{};
  Clock::time_point target_rx_{};
  Clock::time_point safety_started_{};
  Clock::time_point command_started_{};
  Clock::time_point stream_started_{};

  double publish_rate_hz_{};
  double state_timeout_s_{};
  double position_timeout_s_{};
  double target_timeout_s_{};
  double land_handover_timeout_s_{};
  double command_queue_timeout_s_{};
  double command_timeout_s_{};
  bool have_state_{false};
  bool have_pose_{false};
  bool have_velocity_{false};
  bool have_extended_state_{false};
  bool have_target_{false};
  bool read_only_{true};
  bool command_pending_{false};
  bool queued_command_{false};
  uint8_t pending_command_code_{0};
  int64_t pending_request_id_{-1};
  uint64_t command_generation_{0};
  int64_t target_stamp_ns_{0};
  int64_t velocity_stamp_ns_{0};
  bool safety_latched_{false};
  bool safety_land_sent_{false};
  bool offboard_active_{false};
  std::string safety_reason_;
  std::string pending_start_mode_;
  uint32_t last_command_id_{0};
  bool command_completed_{false};
  bool command_accepted_{false};
  std::string command_message_;
};

}  // namespace uav_control
