/**
 * @file odom_bridge.cpp
 * @brief Gates and adapts OpenVINS odometry for MAVROS or PX4 DDS.
 */

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "uav_bridge/vio_safety_gate.hpp"

namespace
{
using Matrix3d = Eigen::Matrix3d;
using Matrix6d = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
using Vector3d = Eigen::Vector3d;

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double seconds_between(int64_t later_ns, int64_t earlier_ns)
{
  return static_cast<double>(later_ns - earlier_ns) * 1.0e-9;
}

bool stamp_to_ns(const builtin_interfaces::msg::Time & stamp, int64_t & result)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {
    return false;
  }
  result = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  return result > 0;
}

float to_px4_float32(double value)
{
  const double limit = std::numeric_limits<float>::max();
  if (!std::isfinite(value) || value < -limit || value > limit) {
    throw std::invalid_argument("odometry value exceeds PX4 float32 range");
  }
  const float narrowed = static_cast<float>(value);
  if (!std::isfinite(narrowed)) {
    throw std::invalid_argument("odometry value became non-finite in PX4 float32");
  }
  return narrowed;
}

float to_px4_variance(double value)
{
  const float narrowed = to_px4_float32(value);
  if (narrowed <= 0.0F) {
    throw std::invalid_argument("PX4 odometry variance must remain positive in float32");
  }
  return narrowed;
}

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

Matrix3d skew(const Vector3d & value)
{
  Matrix3d result;
  result << 0.0, -value.z(), value.y(),
    value.z(), 0.0, -value.x(),
    -value.y(), value.x(), 0.0;
  return result;
}

bool valid_covariance(const Matrix6d & covariance)
{
  if (!covariance.allFinite() ||
    !covariance.isApprox(covariance.transpose(), 1.0e-7) ||
    covariance.diagonal().minCoeff() < 0.0 || covariance.trace() <= 0.0)
  {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(covariance, Eigen::EigenvaluesOnly);
  return solver.info() == Eigen::Success &&
         solver.eigenvalues().minCoeff() >= -1.0e-9 * std::max(1.0, covariance.norm());
}

bool largest_eigenvalue(const Matrix3d & covariance, double & value)
{
  if (!covariance.allFinite() || !covariance.isApprox(covariance.transpose(), 1.0e-7)) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Matrix3d> solver(covariance, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success) {
    return false;
  }
  value = solver.eigenvalues().maxCoeff();
  return std::isfinite(value);
}

std::string bool_string(bool value)
{
  return value ? "true" : "false";
}

class OutputBackend
{
public:
  virtual ~OutputBackend() = default;
  virtual void publish(const nav_msgs::msg::Odometry & odometry, uint8_t reset_counter) = 0;
};

class MavrosBackend final : public OutputBackend
{
public:
  MavrosBackend(
    rclcpp::Node & node, const std::string & output_frame_id,
    const std::string & output_child_frame_id)
  : publisher_(node.create_publisher<nav_msgs::msg::Odometry>(
        "/mavros/odometry/out", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile())),
    static_tf_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(&node))
  {
    const double pi = std::acos(-1.0);
    const Eigen::Quaterniond world_rotation(
      Eigen::AngleAxisd(pi / 2.0, Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pi, Vector3d::UnitX()));
    const Eigen::Quaterniond body_rotation(Eigen::AngleAxisd(pi, Vector3d::UnitX()));
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (const auto & entry : std::vector<std::pair<std::string, Eigen::Quaterniond>>{
      {output_frame_id, world_rotation}, {output_child_frame_id, body_rotation}})
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = node.now();
      transform.header.frame_id = entry.first;
      transform.child_frame_id = entry.first == output_frame_id ?
        output_frame_id + "_ned" : output_child_frame_id + "_frd";
      transform.transform.rotation.x = entry.second.x();
      transform.transform.rotation.y = entry.second.y();
      transform.transform.rotation.z = entry.second.z();
      transform.transform.rotation.w = entry.second.w();
      transforms.push_back(std::move(transform));
    }
    static_tf_->sendTransform(transforms);
  }

  void publish(const nav_msgs::msg::Odometry & odometry, uint8_t) override
  {
    publisher_->publish(odometry);
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};

class DdsBackend final : public OutputBackend
{
public:
  DdsBackend(rclcpp::Node & node, const std::string & topic)
  : publisher_(node.create_publisher<px4_msgs::msg::VehicleOdometry>(
        topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()))
  {
  }

  void publish(const nav_msgs::msg::Odometry & odometry, uint8_t reset_counter) override
  {
    int64_t sample_ns = 0;
    if (!stamp_to_ns(odometry.header.stamp, sample_ns)) {
      throw std::invalid_argument("invalid adapted odometry timestamp");
    }

    static const Matrix3d r_ned_enu = []() {
        Matrix3d rotation;
        rotation << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
        return rotation;
      }();
    static const Matrix3d r_frd_flu = Matrix3d(
      (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0).finished());

    const Vector3d position_enu(
      odometry.pose.pose.position.x,
      odometry.pose.pose.position.y,
      odometry.pose.pose.position.z);
    const Vector3d position_ned = r_ned_enu * position_enu;
    const Eigen::Quaterniond q_enu_flu(
      odometry.pose.pose.orientation.w,
      odometry.pose.pose.orientation.x,
      odometry.pose.pose.orientation.y,
      odometry.pose.pose.orientation.z);
    const Eigen::Matrix3d r_enu_flu = q_enu_flu.toRotationMatrix();
    Eigen::Quaterniond q_ned_frd(r_ned_enu * r_enu_flu * r_frd_flu);
    q_ned_frd.normalize();

    Eigen::Map<const Matrix6d> pose_covariance(odometry.pose.covariance.data());
    Eigen::Map<const Matrix6d> twist_covariance(odometry.twist.covariance.data());
    const Matrix3d position_covariance_ned =
      r_ned_enu * pose_covariance.block<3, 3>(0, 0) * r_ned_enu.transpose();
    const Matrix3d attitude_covariance_body_flu =
      r_enu_flu.transpose() * pose_covariance.block<3, 3>(3, 3) * r_enu_flu;
    const Matrix3d attitude_covariance_body_frd =
      r_frd_flu * attitude_covariance_body_flu * r_frd_flu.transpose();
    const Matrix3d velocity_covariance_body_frd =
      r_frd_flu * twist_covariance.block<3, 3>(0, 0) * r_frd_flu.transpose();
    const Vector3d velocity_frd = r_frd_flu * Vector3d(
      odometry.twist.twist.linear.x,
      odometry.twist.twist.linear.y,
      odometry.twist.twist.linear.z);
    const Vector3d angular_velocity_frd = r_frd_flu * Vector3d(
      odometry.twist.twist.angular.x,
      odometry.twist.twist.angular.y,
      odometry.twist.twist.angular.z);

    px4_msgs::msg::VehicleOdometry message{};
    // Keep the source sample stamp as microseconds with no second offset.
    // timestamp=0 leaves the receive-time field to PX4-side handling. This
    // conversion is source-code semantics, not proof of a deployed clock match.
    message.timestamp = 0;
    message.timestamp_sample = static_cast<uint64_t>(sample_ns / 1000);
    message.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    message.position = {
      to_px4_float32(position_ned.x()), to_px4_float32(position_ned.y()),
      to_px4_float32(position_ned.z())};
    message.q = {
      to_px4_float32(q_ned_frd.w()), to_px4_float32(q_ned_frd.x()),
      to_px4_float32(q_ned_frd.y()), to_px4_float32(q_ned_frd.z())};
    message.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    message.velocity = {
      to_px4_float32(velocity_frd.x()), to_px4_float32(velocity_frd.y()),
      to_px4_float32(velocity_frd.z())};
    message.angular_velocity = {
      to_px4_float32(angular_velocity_frd.x()), to_px4_float32(angular_velocity_frd.y()),
      to_px4_float32(angular_velocity_frd.z())};
    message.position_variance = {
      to_px4_variance(position_covariance_ned(0, 0)),
      to_px4_variance(position_covariance_ned(1, 1)),
      to_px4_variance(position_covariance_ned(2, 2))};
    message.orientation_variance = {
      to_px4_variance(attitude_covariance_body_frd(0, 0)),
      to_px4_variance(attitude_covariance_body_frd(1, 1)),
      to_px4_variance(attitude_covariance_body_frd(2, 2))};
    message.velocity_variance = {
      to_px4_variance(velocity_covariance_body_frd(0, 0)),
      to_px4_variance(velocity_covariance_body_frd(1, 1)),
      to_px4_variance(velocity_covariance_body_frd(2, 2))};
    message.reset_counter = reset_counter;
    message.quality = 0;  // The health contract provides no PX4 quality score.
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
};
}  // namespace

class OdomBridgeNode : public rclcpp::Node
{
public:
  OdomBridgeNode()
  : Node("odom_bridge"), kinematic_gate_(3)
  {
    declare_parameters();
    read_and_validate_parameters();
    if (backend_name_ == "mavros") {
      backend_ = std::make_unique<MavrosBackend>(
        *this, output_frame_id_, output_child_frame_id_);
    } else if (backend_name_ == "dds") {
      backend_ = std::make_unique<DdsBackend>(
        *this, fmu_topic(fmu_prefix_, "/in/vehicle_visual_odometry"));
    } else {
      throw std::invalid_argument(
              "backend must be injected as 'mavros' or 'dds' from uav_control.backend");
    }

    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, input_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_odometry(*message);});
    sub_visual_health_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      visual_health_topic_, input_qos,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        on_visual_health(*message);
      });
    pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/uav_bridge/diagnostics",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {watchdog();});
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});

    RCLCPP_INFO(get_logger(), "Odom bridge backend=%s enabled=%s input=%s health=%s",
      backend_name_.c_str(), bool_string(enabled_).c_str(), input_topic_.c_str(),
      visual_health_topic_.c_str());
  }

private:
  template<typename ValueT>
  void declare_read_only(const std::string & name, const ValueT & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    declare_parameter<ValueT>(name, value, descriptor);
  }

  void declare_parameters()
  {
    declare_read_only<std::string>("backend", "");
    declare_read_only<bool>("enabled", false);
    declare_read_only<std::string>("fmu_prefix", "/fmu");
    declare_read_only<std::string>("input_topic", "/ov_msckf/odomimu");
    declare_read_only<std::string>("visual_health_topic", "/ov_msckf/visual_health");
    declare_read_only<std::string>("input_frame_id", "global");
    declare_read_only<std::string>("input_child_frame_id", "imu");
    declare_read_only<std::string>("output_frame_id", "vio_odom");
    declare_read_only<std::string>("output_child_frame_id", "vio_body");
    declare_read_only<std::vector<double>>(
      "rotation_body_imu_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    declare_read_only<std::vector<double>>(
      "position_body_in_imu_m", std::vector<double>{0.0, 0.0, 0.0});
    declare_read_only<double>("world_yaw_rad", 0.0);
    declare_read_only<double>("publish_rate_hz", 40.0);
    declare_read_only<double>("max_sample_age_s", 0.15);
    declare_read_only<double>("max_future_s", 0.01);
    declare_read_only<double>("visual_timeout_s", 0.3);
    declare_read_only<double>("input_timeout_s", 0.2);
    declare_read_only<double>("max_position_step_m", 1.0);
    declare_read_only<double>("max_attitude_step_rad", 0.75);
    declare_read_only<double>("max_position_variance_m2", 0.16);
    declare_read_only<double>("max_attitude_variance_rad2", 0.36);
    declare_read_only<double>("max_velocity_variance_m2_s2", 1.44);
    declare_read_only<double>("max_angular_velocity_variance_rad2_s2", 4.0);
    declare_read_only<double>("max_position_prediction_error_m", 0.12);
    declare_read_only<double>("max_vio_speed_m_s", 0.0);
  }

  void read_and_validate_parameters()
  {
    backend_name_ = get_parameter("backend").as_string();
    enabled_ = get_parameter("enabled").as_bool();
    fmu_prefix_ = get_parameter("fmu_prefix").as_string();
    input_topic_ = get_parameter("input_topic").as_string();
    visual_health_topic_ = get_parameter("visual_health_topic").as_string();
    input_frame_id_ = get_parameter("input_frame_id").as_string();
    input_child_frame_id_ = get_parameter("input_child_frame_id").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    output_child_frame_id_ = get_parameter("output_child_frame_id").as_string();
    const auto rotation = get_parameter("rotation_body_imu_xyzw").as_double_array();
    const auto lever = get_parameter("position_body_in_imu_m").as_double_array();
    if (rotation.size() != 4 || lever.size() != 3) {
      throw std::invalid_argument("body/IMU transform parameters have invalid length");
    }
    rotation_body_imu_ = Eigen::Quaterniond(rotation[3], rotation[0], rotation[1], rotation[2]);
    position_body_in_imu_ = Vector3d(lever[0], lever[1], lever[2]);
    world_yaw_rad_ = get_parameter("world_yaw_rad").as_double();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    max_sample_age_s_ = get_parameter("max_sample_age_s").as_double();
    max_future_s_ = get_parameter("max_future_s").as_double();
    visual_timeout_s_ = get_parameter("visual_timeout_s").as_double();
    input_timeout_s_ = get_parameter("input_timeout_s").as_double();
    max_position_step_m_ = get_parameter("max_position_step_m").as_double();
    max_attitude_step_rad_ = get_parameter("max_attitude_step_rad").as_double();
    max_position_variance_m2_ = get_parameter("max_position_variance_m2").as_double();
    max_attitude_variance_rad2_ = get_parameter("max_attitude_variance_rad2").as_double();
    max_velocity_variance_m2_s2_ = get_parameter("max_velocity_variance_m2_s2").as_double();
    max_angular_velocity_variance_rad2_s2_ =
      get_parameter("max_angular_velocity_variance_rad2_s2").as_double();
    max_position_prediction_error_m_ =
      get_parameter("max_position_prediction_error_m").as_double();
    max_vio_speed_m_s_ = get_parameter("max_vio_speed_m_s").as_double();

    for (const auto & frame : {input_frame_id_, input_child_frame_id_,
        output_frame_id_, output_child_frame_id_})
    {
      if (frame.empty() || frame.front() == '/') {
        throw std::invalid_argument("frame ids must be nonempty and relative");
      }
    }
    if (input_topic_.empty() || visual_health_topic_.empty() ||
      output_frame_id_ == output_child_frame_id_ ||
      !std::isfinite(rotation_body_imu_.norm()) ||
      std::abs(rotation_body_imu_.norm() - 1.0) > 1.0e-3 ||
      !position_body_in_imu_.allFinite())
    {
      throw std::invalid_argument("invalid topics or body/IMU transform");
    }
    rotation_body_imu_.normalize();

    const std::array<double, 11> positive_limits{
      publish_rate_hz_, max_sample_age_s_, visual_timeout_s_, input_timeout_s_,
      max_position_step_m_, max_attitude_step_rad_, max_position_variance_m2_,
      max_attitude_variance_rad2_, max_velocity_variance_m2_s2_,
      max_angular_velocity_variance_rad2_s2_, max_position_prediction_error_m_};
    if (!std::all_of(positive_limits.begin(), positive_limits.end(),
        [](double value) {return std::isfinite(value) && value > 0.0;} ) ||
      !std::isfinite(max_future_s_) || max_future_s_ < 0.0 ||
      !std::isfinite(world_yaw_rad_) ||
      !uav_bridge::valid_vio_speed_limit(max_vio_speed_m_s_))
    {
      throw std::invalid_argument("invalid odometry bridge limits");
    }
    min_output_period_ns_ = static_cast<int64_t>(1.0e9 / publish_rate_hz_);
  }

  bool check_ros_clock(int64_t now_ros_ns)
  {
    if (have_ros_clock_ && now_ros_ns < last_ros_now_ns_) {
      if (has_output_) {
        enter_pause("ROS clock moved backwards");
      } else {
        reset_recovery();
      }
      have_visual_stamp_ = false;
      last_visual_stamp_ns_ = 0;
      have_seen_odometry_stamp_ = false;
      last_seen_odometry_stamp_ns_ = 0;
      have_valid_odometry_ = false;
      last_valid_odometry_stamp_ns_ = 0;
      last_valid_odometry_receive_steady_ns_ = 0;
      kinematic_gate_.reset_baseline();
    }
    have_ros_clock_ = true;
    last_ros_now_ns_ = now_ros_ns;
    return true;
  }

  bool visual_fresh(int64_t now_ros_ns, int64_t now_steady_ns) const
  {
    if (!have_visual_stamp_) {
      return false;
    }
    const double ros_age = seconds_between(now_ros_ns, last_visual_stamp_ns_);
    const double steady_age = seconds_between(now_steady_ns, last_visual_receive_steady_ns_);
    return ros_age >= -max_future_s_ && ros_age <= visual_timeout_s_ &&
           steady_age >= 0.0 && steady_age <= visual_timeout_s_;
  }

  bool input_fresh(int64_t now_ros_ns, int64_t now_steady_ns) const
  {
    if (!have_valid_odometry_) {
      return false;
    }
    const double ros_age = seconds_between(now_ros_ns, last_valid_odometry_stamp_ns_);
    const double steady_age = seconds_between(now_steady_ns, last_valid_odometry_receive_steady_ns_);
    return ros_age >= -max_future_s_ && ros_age <= max_sample_age_s_ &&
           steady_age >= 0.0 && steady_age <= input_timeout_s_;
  }

  void refresh_watchdog(int64_t now_ros_ns, int64_t now_steady_ns)
  {
    check_ros_clock(now_ros_ns);
    if (!has_output_) {
      return;
    }
    if (!visual_fresh(now_ros_ns, now_steady_ns)) {
      if (!paused_) {
        enter_pause("visual update expired");
      } else if (recovery_counter_.visual_count() > 0 ||
        recovery_counter_.odometry_count() > 0)
      {
        reset_recovery();
      }
    }
    if (!input_fresh(now_ros_ns, now_steady_ns)) {
      if (!paused_) {
        enter_pause("odometry stream expired");
      } else if (recovery_counter_.odometry_count() > 0) {
        reset_recovery();
      }
    }
  }

  void reset_recovery()
  {
    recovery_counter_.reset();
    kinematic_gate_.reset();
  }

  void enter_pause(const std::string & reason)
  {
    const bool first_pause = !paused_;
    paused_ = true;
    pause_reason_ = reason;
    last_rejection_ = reason;
    if (first_pause) {
      reset_recovery();
      RCLCPP_ERROR(get_logger(), "VIO forwarding paused: %s", reason.c_str());
    }
  }

  void reject_health(const std::string & reason, bool invalidate_visual)
  {
    last_rejection_ = reason;
    if (invalidate_visual) {
      have_visual_stamp_ = false;
    }
    if (has_output_) {
      enter_pause(reason);
    }
    if (paused_) {
      reset_recovery();
    }
  }

  void on_visual_health(const diagnostic_msgs::msg::DiagnosticArray & health)
  {
    const int64_t now_ros_ns = get_clock()->now().nanoseconds();
    const int64_t now_steady_ns = steady_now_ns();
    refresh_watchdog(now_ros_ns, now_steady_ns);

    if (health.header.frame_id != input_frame_id_ || health.status.size() != 1) {
      reject_health("invalid visual health frame or status count", true);
      return;
    }
    const auto & status = health.status.front();
    std::string version;
    std::string initialized;
    std::string counter_text;
    for (const auto & value : status.values) {
      std::string * destination = nullptr;
      if (value.key == "contract_version") {destination = &version;}
      if (value.key == "initialized") {destination = &initialized;}
      if (value.key == "reset_counter") {destination = &counter_text;}
      if (destination != nullptr) {
        if (!destination->empty() || value.value.empty()) {
          reject_health("duplicate or empty visual health field", true);
          return;
        }
        *destination = value.value;
      }
    }

    uint64_t generation = 0;
    const auto parsed = std::from_chars(
      counter_text.data(), counter_text.data() + counter_text.size(), generation);
    if (status.name != "openvins/visual_fusion" || status.hardware_id.empty() ||
      version != "1" || (initialized != "true" && initialized != "false") ||
      counter_text.empty() || parsed.ec != std::errc{} ||
      parsed.ptr != counter_text.data() + counter_text.size())
    {
      reject_health("invalid visual health contract", true);
      return;
    }

    const bool session_changed = !visual_session_id_.empty() &&
      (status.hardware_id != visual_session_id_ || generation != visual_generation_);
    if (session_changed) {
      ++dds_reset_counter_;
      if (has_output_ || paused_) {
        enter_pause("VIO session or initialization changed");
      }
      have_visual_stamp_ = false;
      last_visual_stamp_ns_ = 0;
      have_seen_odometry_stamp_ = false;
      last_seen_odometry_stamp_ns_ = 0;
      have_valid_odometry_ = false;
      last_valid_odometry_stamp_ns_ = 0;
      last_valid_odometry_receive_steady_ns_ = 0;
      reset_recovery();
    }
    visual_session_id_ = status.hardware_id;
    visual_generation_ = generation;

    if (initialized != "true" || generation == 0) {
      reject_health("VIO is not initialized", true);
      return;
    }
    if (status.level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
      reject_health("VIO reports no accepted visual update", true);
      return;
    }

    int64_t stamp_ns = 0;
    if (!stamp_to_ns(health.header.stamp, stamp_ns)) {
      reject_health("invalid visual health timestamp", true);
      return;
    }
    if (last_visual_stamp_ns_ != 0 && stamp_ns == last_visual_stamp_ns_) {
      last_rejection_ = "duplicate visual timestamp";
      return;
    }
    if (last_visual_stamp_ns_ != 0 && stamp_ns < last_visual_stamp_ns_) {
      if (has_output_) {
        enter_pause("visual timestamp moved backwards");
      } else if (paused_) {
        reset_recovery();
      }
      have_visual_stamp_ = false;
      last_visual_stamp_ns_ = 0;
      reset_recovery();
      last_rejection_ = "visual timestamp moved backwards";
      return;
    }
    const double age = seconds_between(now_ros_ns, stamp_ns);
    if (age > max_sample_age_s_ || age < -max_future_s_) {
      reject_health(age > max_sample_age_s_ ? "stale visual timestamp" : "future visual timestamp", true);
      return;
    }

    last_visual_stamp_ns_ = stamp_ns;
    last_visual_receive_steady_ns_ = now_steady_ns;
    have_visual_stamp_ = true;
    if (paused_) {
      recovery_counter_.observe_visual(stamp_ns);
    }
  }

  bool adapt_odometry(
    const nav_msgs::msg::Odometry & input, int64_t now_ros_ns, int64_t now_steady_ns,
    nav_msgs::msg::Odometry & output, bool & ready_to_publish, bool & recovered,
    std::string & reason)
  {
    ready_to_publish = false;
    recovered = false;
    refresh_watchdog(now_ros_ns, now_steady_ns);

    int64_t sample_ns = 0;
    if (!stamp_to_ns(input.header.stamp, sample_ns)) {
      reason = "invalid odometry timestamp";
      return false;
    }
    if (have_seen_odometry_stamp_ && sample_ns == last_seen_odometry_stamp_ns_) {
      reason = "duplicate odometry timestamp";
      return false;
    }
    if (input.header.frame_id != input_frame_id_ ||
      input.child_frame_id != input_child_frame_id_)
    {
      reason = "unexpected odometry frames";
      return false;
    }
    const double age = seconds_between(now_ros_ns, sample_ns);
    if (age > max_sample_age_s_ || age < -max_future_s_) {
      reason = age > max_sample_age_s_ ? "stale odometry timestamp" : "future odometry timestamp";
      return false;
    }
    if (have_seen_odometry_stamp_ && sample_ns < last_seen_odometry_stamp_ns_) {
      if (has_output_) {
        enter_pause("odometry timestamp moved backwards");
      } else if (paused_) {
        reset_recovery();
      }
      have_seen_odometry_stamp_ = false;
      last_seen_odometry_stamp_ns_ = 0;
      have_valid_odometry_ = false;
      last_valid_odometry_stamp_ns_ = 0;
      last_valid_odometry_receive_steady_ns_ = 0;
      kinematic_gate_.reset_baseline();
      reason = "odometry timestamp moved backwards";
      return false;
    }
    have_seen_odometry_stamp_ = true;
    last_seen_odometry_stamp_ns_ = sample_ns;

    const bool is_fresh_visual = visual_fresh(now_ros_ns, now_steady_ns);
    if (!is_fresh_visual && paused_ &&
      (recovery_counter_.visual_count() > 0 ||
      recovery_counter_.odometry_count() > 0))
    {
      reset_recovery();
    }

    const auto & pose = input.pose.pose;
    const auto & twist = input.twist.twist;
    const Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    const Vector3d velocity_body(twist.linear.x, twist.linear.y, twist.linear.z);
    const Vector3d angular_velocity(twist.angular.x, twist.angular.y, twist.angular.z);
    Eigen::Quaterniond attitude(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    const Matrix6d pose_covariance =
      Eigen::Map<const Matrix6d>(input.pose.covariance.data());
    const Matrix6d twist_covariance =
      Eigen::Map<const Matrix6d>(input.twist.covariance.data());
    if (!position.allFinite() || !velocity_body.allFinite() || !angular_velocity.allFinite() ||
      !attitude.coeffs().allFinite() || attitude.norm() < 1.0e-6 ||
      std::abs(attitude.norm() - 1.0) > 1.0e-3 ||
      !valid_covariance(pose_covariance) || !valid_covariance(twist_covariance))
    {
      reason = "invalid odometry state or full covariance";
      return false;
    }
    attitude.normalize();

    const Matrix3d r_wg = Eigen::AngleAxisd(
      world_yaw_rad_, Vector3d::UnitZ()).toRotationMatrix();
    const Matrix3d r_gi = attitude.toRotationMatrix();
    const Matrix3d r_bi = rotation_body_imu_.toRotationMatrix();
    const Vector3d position_out = r_wg *
      (position + r_gi * position_body_in_imu_);
    const Eigen::Quaterniond attitude_out(r_wg * r_gi * r_bi.transpose());
    const Vector3d velocity_out = r_bi *
      (velocity_body + angular_velocity.cross(position_body_in_imu_));
    const Vector3d angular_velocity_out = r_bi * angular_velocity;

    Matrix6d pose_jacobian = Matrix6d::Zero();
    pose_jacobian.block<3, 3>(0, 0) = r_wg;
    pose_jacobian.block<3, 3>(0, 3) =
      -r_wg * r_gi * skew(position_body_in_imu_);
    pose_jacobian.block<3, 3>(3, 3) = r_wg * r_gi;
    Matrix6d transformed_pose_covariance =
      pose_jacobian * pose_covariance * pose_jacobian.transpose();
    Eigen::SelfAdjointEigenSolver<Matrix3d> attitude_solver(
      transformed_pose_covariance.block<3, 3>(3, 3), Eigen::EigenvaluesOnly);
    if (attitude_solver.info() != Eigen::Success) {
      reason = "attitude covariance decomposition failed";
      return false;
    }
    // Keep a conservative diagonal upper bound for the world-angle block until
    // the deployed MAVROS covariance contract is verified.
    const double cross_bound = transformed_pose_covariance.block<3, 3>(0, 3).norm();
    transformed_pose_covariance.block<3, 3>(0, 0).diagonal().array() += cross_bound;
    const double attitude_bound = attitude_solver.eigenvalues().maxCoeff() + cross_bound;
    transformed_pose_covariance.block<3, 3>(3, 3) =
      attitude_bound * Matrix3d::Identity();
    transformed_pose_covariance.block<3, 3>(0, 3).setZero();
    transformed_pose_covariance.block<3, 3>(3, 0).setZero();

    Matrix6d twist_jacobian = Matrix6d::Zero();
    twist_jacobian.block<3, 3>(0, 0) = r_bi;
    twist_jacobian.block<3, 3>(0, 3) = -r_bi * skew(position_body_in_imu_);
    twist_jacobian.block<3, 3>(3, 3) = r_bi;
    const Matrix6d transformed_twist_covariance =
      twist_jacobian * twist_covariance * twist_jacobian.transpose();
    if (!position_out.allFinite() || !attitude_out.coeffs().allFinite() ||
      !velocity_out.allFinite() || !angular_velocity_out.allFinite() ||
      !transformed_pose_covariance.allFinite() || !transformed_twist_covariance.allFinite())
    {
      reason = "non-finite transformed odometry";
      return false;
    }

    double position_eigenvalue = 0.0;
    double attitude_eigenvalue = 0.0;
    double velocity_eigenvalue = 0.0;
    double angular_eigenvalue = 0.0;
    if (!largest_eigenvalue(
        transformed_pose_covariance.block<3, 3>(0, 0), position_eigenvalue) ||
      !largest_eigenvalue(
        transformed_pose_covariance.block<3, 3>(3, 3), attitude_eigenvalue) ||
      !largest_eigenvalue(
        transformed_twist_covariance.block<3, 3>(0, 0), velocity_eigenvalue) ||
      !largest_eigenvalue(
        transformed_twist_covariance.block<3, 3>(3, 3), angular_eigenvalue) ||
      position_eigenvalue <= 0.0 || position_eigenvalue > max_position_variance_m2_ ||
      attitude_eigenvalue <= 0.0 || attitude_eigenvalue > max_attitude_variance_rad2_ ||
      velocity_eigenvalue <= 0.0 || velocity_eigenvalue > max_velocity_variance_m2_s2_ ||
      angular_eigenvalue <= 0.0 ||
      angular_eigenvalue > max_angular_velocity_variance_rad2_s2_)
    {
      reason = "transformed covariance exceeds configured limit";
      if (has_output_) {
        enter_pause(reason);
      }
      return false;
    }

    // The input pose is world-frame; linear velocity is expressed in the IMU frame.
    const Vector3d velocity_world = attitude * velocity_body;
    if (!velocity_world.allFinite()) {
      reason = "invalid body-to-world velocity";
      if (has_output_) {
        enter_pause(reason);
      }
      return false;
    }

    const auto gate_result = kinematic_gate_.update(
      sample_ns, position, attitude, velocity_body, velocity_world, max_vio_speed_m_s_,
      max_position_step_m_, max_attitude_step_rad_, max_position_prediction_error_m_);
    if (gate_result != uav_bridge::VioKinematicGateResult::Accepted) {
      reason = uav_bridge::VioKinematicGate::reason(gate_result);
      if (has_output_ && uav_bridge::VioKinematicGate::requires_pause(gate_result)) {
        enter_pause(reason);
      }
      return false;
    }

    last_valid_odometry_stamp_ns_ = sample_ns;
    last_valid_odometry_receive_steady_ns_ = now_steady_ns;
    have_valid_odometry_ = true;

    output = input;
    output.header.frame_id = output_frame_id_;
    output.child_frame_id = output_child_frame_id_;
    output.pose.pose.position.x = position_out.x();
    output.pose.pose.position.y = position_out.y();
    output.pose.pose.position.z = position_out.z();
    output.pose.pose.orientation.x = attitude_out.x();
    output.pose.pose.orientation.y = attitude_out.y();
    output.pose.pose.orientation.z = attitude_out.z();
    output.pose.pose.orientation.w = attitude_out.w();
    output.twist.twist.linear.x = velocity_out.x();
    output.twist.twist.linear.y = velocity_out.y();
    output.twist.twist.linear.z = velocity_out.z();
    output.twist.twist.angular.x = angular_velocity_out.x();
    output.twist.twist.angular.y = angular_velocity_out.y();
    output.twist.twist.angular.z = angular_velocity_out.z();
    Eigen::Map<Matrix6d>(output.pose.covariance.data()) = transformed_pose_covariance;
    Eigen::Map<Matrix6d>(output.twist.covariance.data()) = transformed_twist_covariance;

    if (paused_) {
      if (recovery_counter_.observe_odometry(sample_ns, is_fresh_visual)) {
        paused_ = false;
        pause_reason_.clear();
        recovered = true;
        ready_to_publish = true;
      }
    } else {
      ready_to_publish = is_fresh_visual;
    }
    return true;
  }

  void on_odometry(const nav_msgs::msg::Odometry & input)
  {
    ++received_count_;
    const int64_t now_ros_ns = get_clock()->now().nanoseconds();
    const int64_t now_steady_ns = steady_now_ns();
    nav_msgs::msg::Odometry output;
    bool ready_to_publish = false;
    bool recovered = false;
    std::string reason;
    const auto process_start = std::chrono::steady_clock::now();
    const bool adapted = adapt_odometry(
      input, now_ros_ns, now_steady_ns, output, ready_to_publish, recovered, reason);
    const double process_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - process_start).count();
    last_process_ms_ = process_ms;
    max_process_ms_ = std::max(max_process_ms_, process_ms);

    if (!adapted) {
      ++rejected_count_;
      last_process_result_ = "rejected: " + reason;
      last_rejection_ = reason;
      if (paused_ && reason != "duplicate odometry timestamp") {
        reset_recovery();
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Odometry rejected: %s (received=%llu adapted=%llu published=%llu rejected=%llu rate_limited=%llu)",
        reason.c_str(), static_cast<unsigned long long>(received_count_),
        static_cast<unsigned long long>(adapted_count_),
        static_cast<unsigned long long>(published_count_),
        static_cast<unsigned long long>(rejected_count_),
        static_cast<unsigned long long>(rate_limited_count_));
      return;
    }

    ++adapted_count_;
    if (!ready_to_publish) {
      last_process_result_ = "adapted; waiting for recovery samples";
      return;
    }
    if (!enabled_) {
      last_process_result_ = "adapted; output disabled";
      return;
    }

    int64_t sample_ns = 0;
    (void)stamp_to_ns(input.header.stamp, sample_ns);
    if (!recovered && has_output_ && sample_ns - last_output_stamp_ns_ < min_output_period_ns_) {
      ++rate_limited_count_;
      last_process_result_ = "rate_limited";
      return;
    }

    try {
      backend_->publish(output, dds_reset_counter_);
      ++published_count_;
      last_output_stamp_ns_ = sample_ns;
      has_output_ = true;
      last_process_result_ = recovered ? "published after recovery" : "published";
      if (recovered) {
        RCLCPP_INFO(get_logger(), "VIO recovered");
      }
    } catch (const std::exception & error) {
      ++rejected_count_;
      last_process_result_ = std::string("publish_failed: ") + error.what();
      if (has_output_) {
        enter_pause(last_process_result_);
      } else {
        last_rejection_ = last_process_result_;
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Odometry backend publish failed: %s", error.what());
    }
  }

  void watchdog()
  {
    refresh_watchdog(get_clock()->now().nanoseconds(), steady_now_ns());
  }

  void append_value(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key, const std::string & value) const
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(std::move(item));
  }

  void publish_diagnostics()
  {
    const auto now = get_clock()->now();
    const int64_t now_ros_ns = now.nanoseconds();
    const int64_t now_steady_ns = steady_now_ns();
    refresh_watchdog(now_ros_ns, now_steady_ns);

    const bool is_visual_fresh = visual_fresh(now_ros_ns, now_steady_ns);
    const bool is_input_fresh = input_fresh(now_ros_ns, now_steady_ns);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now;
    array.header.frame_id = input_frame_id_;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "uav_bridge/odom_bridge";
    status.hardware_id = "uav_bridge";
    if (paused_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = pause_reason_;
    } else if (!enabled_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "output disabled";
    } else if (!is_visual_fresh || !is_input_fresh) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = !is_visual_fresh ? "waiting for fresh visual update" :
        "waiting for fresh odometry";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "VIO stream healthy";
    }
    append_value(status, "contract_version", "1");
    append_value(status, "backend", backend_name_);
    append_value(status, "enabled", bool_string(enabled_));
    append_value(status, "max_vio_speed_m_s", std::to_string(max_vio_speed_m_s_));
    append_value(status, "faulted", bool_string(paused_));
    append_value(status, "paused", bool_string(paused_));
    append_value(status, "pause_reason", pause_reason_);
    append_value(status, "visual_fresh", bool_string(is_visual_fresh));
    append_value(status, "visual_age_s", have_visual_stamp_ ?
      std::to_string(seconds_between(now_ros_ns, last_visual_stamp_ns_)) : "-1");
    append_value(status, "visual_stamp_ns", std::to_string(last_visual_stamp_ns_));
    append_value(status, "input_fresh", bool_string(is_input_fresh));
    append_value(status, "input_age_s", have_valid_odometry_ ?
      std::to_string(seconds_between(now_ros_ns, last_valid_odometry_stamp_ns_)) : "-1");
    append_value(status, "received", std::to_string(received_count_));
    append_value(status, "adapted", std::to_string(adapted_count_));
    append_value(status, "published", std::to_string(published_count_));
    append_value(status, "rejected", std::to_string(rejected_count_));
    append_value(status, "rate_limited", std::to_string(rate_limited_count_));
    append_value(
      status, "recovery_visual_count", std::to_string(recovery_counter_.visual_count()));
    append_value(
      status, "recovery_odom_count", std::to_string(recovery_counter_.odometry_count()));
    append_value(status, "last_process_result", last_process_result_);
    append_value(status, "last_rejection", last_rejection_);
    append_value(status, "last_process_ms", std::to_string(last_process_ms_));
    append_value(status, "max_process_ms", std::to_string(max_process_ms_));
    array.status.push_back(std::move(status));
    pub_diagnostics_->publish(array);
  }

  std::string backend_name_;
  bool enabled_{false};
  std::string fmu_prefix_{"/fmu"};
  std::string input_topic_{"/ov_msckf/odomimu"};
  std::string visual_health_topic_{"/ov_msckf/visual_health"};
  std::string input_frame_id_{"global"};
  std::string input_child_frame_id_{"imu"};
  std::string output_frame_id_{"vio_odom"};
  std::string output_child_frame_id_{"vio_body"};
  Eigen::Quaterniond rotation_body_imu_{Eigen::Quaterniond::Identity()};
  Vector3d position_body_in_imu_{Vector3d::Zero()};
  double world_yaw_rad_{0.0};
  double publish_rate_hz_{40.0};
  double max_sample_age_s_{0.15};
  double max_future_s_{0.01};
  double visual_timeout_s_{0.3};
  double input_timeout_s_{0.2};
  double max_position_step_m_{1.0};
  double max_attitude_step_rad_{0.75};
  double max_position_variance_m2_{0.16};
  double max_attitude_variance_rad2_{0.36};
  double max_velocity_variance_m2_s2_{1.44};
  double max_angular_velocity_variance_rad2_s2_{4.0};
  double max_position_prediction_error_m_{0.12};
  double max_vio_speed_m_s_{0.0};
  int64_t min_output_period_ns_{25000000};
  int64_t last_output_stamp_ns_{0};
  int64_t last_ros_now_ns_{0};
  bool have_ros_clock_{false};
  bool has_output_{false};
  bool paused_{false};
  std::string pause_reason_;
  std::string last_rejection_;
  std::string last_process_result_{"waiting for input"};
  std::string visual_session_id_;
  uint64_t visual_generation_{0};
  uint8_t dds_reset_counter_{0};
  int64_t last_visual_stamp_ns_{0};
  int64_t last_visual_receive_steady_ns_{0};
  bool have_visual_stamp_{false};
  int64_t last_seen_odometry_stamp_ns_{0};
  bool have_seen_odometry_stamp_{false};
  int64_t last_valid_odometry_stamp_ns_{0};
  int64_t last_valid_odometry_receive_steady_ns_{0};
  bool have_valid_odometry_{false};
  uav_bridge::VioKinematicGate kinematic_gate_;
  uav_bridge::VioRecoveryCounter recovery_counter_;
  uint64_t received_count_{0};
  uint64_t adapted_count_{0};
  uint64_t published_count_{0};
  uint64_t rejected_count_{0};
  uint64_t rate_limited_count_{0};
  double last_process_ms_{0.0};
  double max_process_ms_{0.0};
  std::unique_ptr<OutputBackend> backend_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_visual_health_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
