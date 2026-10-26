#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace uav_bridge
{

class VioSafetyGate
{
public:
  explicit VioSafetyGate(std::size_t required_consecutive_violations)
  : required_consecutive_violations_(required_consecutive_violations)
  {
    if (required_consecutive_violations_ == 0) {
      throw std::invalid_argument("required consecutive violations must be positive");
    }
  }

  bool update(bool valid) noexcept
  {
    if (valid) {
      consecutive_violations_ = 0;
      return true;
    }

    if (consecutive_violations_ < required_consecutive_violations_) {
      ++consecutive_violations_;
    }
    return consecutive_violations_ < required_consecutive_violations_;
  }

  void reset() noexcept
  {
    consecutive_violations_ = 0;
  }

private:
  const std::size_t required_consecutive_violations_;
  std::size_t consecutive_violations_{0};
};

inline bool valid_vio_speed_limit(double limit_m_s) noexcept
{
  return std::isfinite(limit_m_s) && limit_m_s >= 0.0;
}

enum class VioKinematicGateResult
{
  Accepted,
  SpeedLimit,
  NonMonotonicSample,
  PositionStep,
  AttitudeStep,
  PredictionError,
  RepeatedPredictionError
};

class VioKinematicGate
{
public:
  explicit VioKinematicGate(std::size_t required_consecutive_prediction_errors)
  : prediction_errors_(required_consecutive_prediction_errors)
  {
  }

  void reset() noexcept
  {
    reset_baseline();
    prediction_errors_.reset();
  }

  void reset_baseline() noexcept
  {
    have_baseline_ = false;
  }

  // The adapter validates finite state, normalized attitude, and configured limits first.
  VioKinematicGateResult update(
    int64_t stamp_ns, const Eigen::Vector3d & position_world,
    const Eigen::Quaterniond & attitude_world_imu,
    const Eigen::Vector3d & velocity_body, const Eigen::Vector3d & velocity_world,
    double max_speed_m_s, double max_position_step_m,
    double max_attitude_step_rad, double max_prediction_error_m) noexcept
  {
    if (max_speed_m_s > 0.0 && velocity_body.norm() > max_speed_m_s) {
      return VioKinematicGateResult::SpeedLimit;
    }

    bool prediction_error = false;
    if (have_baseline_) {
      const double dt = static_cast<double>(stamp_ns - last_stamp_ns_) * 1.0e-9;
      if (dt <= 0.0) {
        return VioKinematicGateResult::NonMonotonicSample;
      }
      const Eigen::Vector3d delta_position = position_world - last_position_world_;
      if (delta_position.norm() > max_position_step_m) {
        return VioKinematicGateResult::PositionStep;
      }
      if (attitude_world_imu.angularDistance(last_attitude_world_imu_) >
        max_attitude_step_rad)
      {
        return VioKinematicGateResult::AttitudeStep;
      }
      const Eigen::Vector3d predicted_delta =
        0.5 * (velocity_world + last_velocity_world_) * dt;
      prediction_error =
        (delta_position - predicted_delta).norm() > max_prediction_error_m;
    }

    if (prediction_error) {
      return prediction_errors_.update(false) ?
             VioKinematicGateResult::PredictionError :
             VioKinematicGateResult::RepeatedPredictionError;
    }

    prediction_errors_.update(true);
    last_stamp_ns_ = stamp_ns;
    last_position_world_ = position_world;
    last_attitude_world_imu_ = attitude_world_imu;
    last_velocity_world_ = velocity_world;
    have_baseline_ = true;
    return VioKinematicGateResult::Accepted;
  }

  static const char * reason(VioKinematicGateResult result) noexcept
  {
    switch (result) {
      case VioKinematicGateResult::Accepted:
        return "";
      case VioKinematicGateResult::SpeedLimit:
        return "VIO speed exceeds configured limit";
      case VioKinematicGateResult::NonMonotonicSample:
        return "non-monotonic raw odometry sample";
      case VioKinematicGateResult::PositionStep:
        return "VIO position discontinuity";
      case VioKinematicGateResult::AttitudeStep:
        return "VIO attitude discontinuity";
      case VioKinematicGateResult::PredictionError:
        return "position/velocity prediction error";
      case VioKinematicGateResult::RepeatedPredictionError:
        return "repeated position/velocity prediction error";
    }
    return "unknown VIO kinematic gate result";
  }

  static bool requires_pause(VioKinematicGateResult result) noexcept
  {
    return result != VioKinematicGateResult::Accepted &&
           result != VioKinematicGateResult::PredictionError;
  }

private:
  VioSafetyGate prediction_errors_;
  int64_t last_stamp_ns_{0};
  Eigen::Vector3d last_position_world_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond last_attitude_world_imu_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d last_velocity_world_{Eigen::Vector3d::Zero()};
  bool have_baseline_{false};
};

class VioRecoveryCounter
{
public:
  void reset() noexcept
  {
    visual_count_ = 0;
    odometry_count_ = 0;
    visual_floor_ns_ = 0;
  }

  void observe_visual(int64_t stamp_ns) noexcept
  {
    if (visual_count_ == 0) {
      visual_floor_ns_ = stamp_ns;
    }
    visual_count_ = std::min<std::size_t>(3, visual_count_ + 1);
  }

  bool observe_odometry(int64_t stamp_ns, bool visual_fresh) noexcept
  {
    if (visual_fresh && visual_count_ >= 3 && stamp_ns >= visual_floor_ns_) {
      odometry_count_ = std::min<std::size_t>(3, odometry_count_ + 1);
    }
    return ready();
  }

  bool ready() const noexcept
  {
    return visual_count_ >= 3 && odometry_count_ >= 3;
  }

  std::size_t visual_count() const noexcept {return visual_count_;}
  std::size_t odometry_count() const noexcept {return odometry_count_;}
  int64_t visual_floor_ns() const noexcept {return visual_floor_ns_;}

private:
  std::size_t visual_count_{0};
  std::size_t odometry_count_{0};
  int64_t visual_floor_ns_{0};
};

}  // namespace uav_bridge
