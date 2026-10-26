#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <uav_interfaces/msg/vehicle_state.hpp>
#include <uav_interfaces/srv/flight_command.hpp>

namespace uav_control
{

namespace dds_frames
{
constexpr double pi = 3.14159265358979323846;

// Both transforms are their own inverse. World z-up/z-down and body FLU/FRD.
// With LOCAL_FRD vision, map is a local z-up frame; its axes do not imply true north.
inline std::array<float, 3> position(const std::array<float, 3> & p)
{
  return {p[1], p[0], -p[2]};
}

inline double yaw(double angle)
{
  return std::remainder(pi / 2.0 - angle, 2.0 * pi);
}

// Hamilton quaternion, wxyz: q_ENU_FLU = q_ENU_NED * q_NED_FRD * q_FRD_FLU.
inline std::array<double, 4> attitude(const std::array<float, 4> & q)
{
  const double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  const double scale = std::sqrt(0.5) / norm;
  return {scale * (q[0] + q[3]), scale * (q[1] + q[2]),
    scale * (q[1] - q[2]), scale * (q[0] - q[3])};
}
}  // namespace dds_frames

class Px4DdsInterface : public rclcpp::Node
{
public:
  explicit Px4DdsInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Clock = std::chrono::steady_clock;
  using FlightCommand = uav_interfaces::srv::FlightCommand;
  struct Stream
  {
    uint64_t timestamp{0};
    Clock::time_point received{};
  };

  uint64_t timestamp_us() const;
  bool timestamp_valid(uint64_t timestamp, double timeout) const;
  bool receive(Stream & stream, uint64_t timestamp, double timeout);
  bool fresh(const Stream & stream, double timeout) const;
  bool state_valid() const;
  bool position_valid() const;
  bool position_coordinates_valid() const;
  bool heading_valid() const;
  bool velocity_valid() const;
  bool estimator_health_valid() const;
  bool estimator_healthy() const;
  bool on_ground() const;
  void invalidate(const std::string & reason, bool origin_reset = false);
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void command_callback(
    const std::shared_ptr<FlightCommand::Request> request,
    std::shared_ptr<FlightCommand::Response> response);
  bool send_command(uint32_t id, uint8_t code);
  void ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  void finish_command(bool accepted, const std::string & message);
  void tick();
  void publish_state();

  bool read_only_, allow_local_vision_heading_;
  double rate_, state_timeout_, land_timeout_, position_timeout_, target_timeout_, command_timeout_;
  double prestream_s_, handover_timeout_, timesync_timeout_, estimator_timeout_;
  uint8_t target_system_, target_component_, source_system_;
  uint16_t source_component_;
  px4_msgs::msg::VehicleStatus status_;
  px4_msgs::msg::VehicleLocalPosition position_;
  px4_msgs::msg::VehicleAttitude attitude_;
  px4_msgs::msg::VehicleLandDetected land_;
  px4_msgs::msg::TimesyncStatus timesync_;
  px4_msgs::msg::EstimatorStatusFlags estimator_flags_;
  Stream status_stream_, position_stream_, attitude_stream_, land_stream_, timesync_stream_;
  Stream estimator_stream_;
  geometry_msgs::msg::PoseStamped target_;
  uint64_t target_stamp_{0}, target_epoch_{0};
  Clock::time_point target_rx_{}, stream_started_{}, safety_started_{}, command_started_{};
  bool streaming_{false}, safety_latched_{false}, land_requested_{false};
  bool offboard_active_{false};
  std::string safety_reason_;
  bool origin_invalid_{false}, safety_land_needed_{false};
  bool pending_{false};
  uint32_t last_request_id_{0}, command_id_{0}, pending_px4_command_{0};
  uint8_t pending_command_code_{0}, pending_start_nav_state_{0};
  uint64_t command_stamp_{0};
  bool command_completed_{false}, command_accepted_{false};
  std::string command_message_;
  // ACK has no application request ID. Never retry a timed-out opcode in this process.
  std::set<uint32_t> timed_out_commands_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr estimator_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<uav_interfaces::msg::VehicleState>::SharedPtr state_pub_;
  rclcpp::Service<FlightCommand>::SharedPtr command_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_control
