/**
 * @file imu_bridge.cpp
 * @brief Converts PX4 SensorCombined samples from FRD to ROS FLU.
 */

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
std::string fmu_topic(const std::string & prefix, const char * suffix)
{
  std::string root = prefix;
  while (!root.empty() && root.back() == '/') {
    root.pop_back();
  }
  if (root.empty()) {
    throw std::invalid_argument("fmu_prefix must name a topic root");
  }
  return root + suffix;
}
}  // namespace

class ImuBridge : public rclcpp::Node
{
public:
  ImuBridge()
  : Node("imu_bridge")
  {
    declare_read_only<std::string>("fmu_prefix", "/fmu");
    declare_read_only<int64_t>("moving_average_samples", 1);
    moving_average_samples_ = get_parameter("moving_average_samples").as_int();
    if (moving_average_samples_ < 1 || moving_average_samples_ > 9 ||
      moving_average_samples_ % 2 == 0)
    {
      throw std::invalid_argument(
              "moving_average_samples must be an odd integer from 1 to 9");
    }

    const auto prefix = get_parameter("fmu_prefix").as_string();
    const auto sensor_topic = fmu_topic(prefix, "/out/sensor_combined");
    sub_sensor_ = create_subscription<px4_msgs::msg::SensorCombined>(
      sensor_topic, rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::SensorCombined::ConstSharedPtr msg) {on_sensor(*msg);});
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(
      "/px4/imu", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "IMU bridge: %s -> /px4/imu (FRD to FLU, %d sample(s))",
      sensor_topic.c_str(), moving_average_samples_);
  }

private:
  template<typename ValueT>
  void declare_read_only(const std::string & name, const ValueT & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    declare_parameter<ValueT>(name, value, descriptor);
  }

  void reject_sample(const char * reason)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Dropping SensorCombined sample: %s", reason);
  }

  struct Sample
  {
    uint64_t timestamp_us{0};
    std::array<double, 3> gyro{};
    std::array<double, 3> acceleration{};
  };

  void on_sensor(const px4_msgs::msg::SensorCombined & msg)
  {
    if (msg.timestamp == 0 ||
      msg.timestamp > static_cast<uint64_t>(std::numeric_limits<int64_t>::max() / 1000) ||
      msg.timestamp <= last_timestamp_us_)
    {
      reject_sample("missing, unrepresentable, repeated, or backward gyro timestamp");
      return;
    }
    if (msg.accelerometer_timestamp_relative ==
      px4_msgs::msg::SensorCombined::RELATIVE_TIMESTAMP_INVALID)
    {
      reject_sample("invalid accelerometer timestamp relation");
      return;
    }
    if (msg.gyro_clipping != 0 || msg.accelerometer_clipping != 0) {
      reject_sample("sensor clipping reported");
      return;
    }

    Sample sample{};
    sample.timestamp_us = msg.timestamp;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      sample.gyro[axis] = msg.gyro_rad[axis];
      sample.acceleration[axis] = msg.accelerometer_m_s2[axis];
      if (!std::isfinite(sample.gyro[axis]) || !std::isfinite(sample.acceleration[axis])) {
        reject_sample("non-finite gyro or accelerometer sample");
        return;
      }
    }
    last_timestamp_us_ = msg.timestamp;

    samples_.push_back(sample);
    while (samples_.size() > static_cast<std::size_t>(moving_average_samples_)) {
      samples_.pop_front();
    }
    if (samples_.size() < static_cast<std::size_t>(moving_average_samples_)) {
      return;
    }

    std::array<double, 3> gyro{};
    std::array<double, 3> acceleration{};
    for (const auto & buffered : samples_) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        gyro[axis] += buffered.gyro[axis];
        acceleration[axis] += buffered.acceleration[axis];
      }
    }

    const double sample_count = static_cast<double>(moving_average_samples_);
    sensor_msgs::msg::Imu imu{};
    const auto & center = samples_[samples_.size() / 2];
    imu.header.stamp = rclcpp::Time(
      static_cast<int64_t>(center.timestamp_us * 1000), RCL_SYSTEM_TIME);
    imu.header.frame_id = "imu_link";
    imu.angular_velocity.x = gyro[0] / sample_count;
    imu.angular_velocity.y = -gyro[1] / sample_count;
    imu.angular_velocity.z = -gyro[2] / sample_count;
    imu.linear_acceleration.x = acceleration[0] / sample_count;
    imu.linear_acceleration.y = -acceleration[1] / sample_count;
    imu.linear_acceleration.z = -acceleration[2] / sample_count;

    // OpenVINS supplies its IMU noise from the Kalibr configuration.
    imu.orientation_covariance[0] = -1.0;
    pub_imu_->publish(imu);
  }

  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_sensor_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  int moving_average_samples_{1};
  uint64_t last_timestamp_us_{0};
  std::deque<Sample> samples_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuBridge>());
  rclcpp::shutdown();
  return 0;
}
