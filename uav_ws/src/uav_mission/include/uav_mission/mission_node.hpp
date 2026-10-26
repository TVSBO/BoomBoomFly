#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/srv/flight_command.hpp>

#include "uav_mission/flight_task.hpp"
#include "uav_mission/vio_health.hpp"

namespace uav_mission
{

class MissionNode : public rclcpp::Node
{
public:
  MissionNode();

private:
  using Clock = FlightTask::Clock;
  void state_callback(const uav_interfaces::msg::VehicleState::SharedPtr msg);
  void vio_diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void cancel_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void timer_callback();
  void publish_diagnostics(bool state_stream_fresh, bool vio_health_fresh);
  void report_result(const FlightTask::Output & output);

  std::unique_ptr<FlightTask> task_;
  std::optional<uav_interfaces::msg::VehicleState> latest_state_;
  Clock::time_point state_rx_{};
  double state_timeout_s_{};
  double vio_health_timeout_s_{};
  bool require_vio_health_{false};
  std::string mission_profile_;
  bool vio_health_ok_{false};
  VioHealthSample vio_health_state_;
  int64_t vio_stamp_ns_{0};
  Clock::time_point vio_rx_{};
  uint32_t last_dispatched_command_id_{0};
  bool last_dispatch_known_{false};
  bool last_command_dispatched_{false};
  std::string last_dispatch_message_;
  bool result_reported_{false};
  std::string last_logged_stage_;
  FlightTask::TaskResult last_grid_result_{FlightTask::TaskResult::PENDING};
  FlightTask::TaskResult last_hover_result_{FlightTask::TaskResult::PENDING};
  FlightTask::TaskResult last_square_result_{FlightTask::TaskResult::PENDING};

  rclcpp::Subscription<uav_interfaces::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr vio_diagnostics_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Client<uav_interfaces::srv::FlightCommand>::SharedPtr command_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_mission
