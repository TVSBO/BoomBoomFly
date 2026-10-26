#pragma once

#include <cmath>
#include <cstdint>

#include <Eigen/Core>

namespace ov_msckf {

enum class ImuInputStatus {
  Accepted,
  InvalidTimestamp,
  NonfiniteMeasurement,
  DuplicateTimestamp,
  TimestampMovedBackwards
};

class ImuInputGuard {
public:
  ImuInputStatus accept(
      int32_t seconds, uint32_t nanoseconds,
      const Eigen::Vector3d &angular_velocity,
      const Eigen::Vector3d &linear_acceleration,
      double &timestamp) {
    if (seconds < 0 || nanoseconds >= 1000000000U)
      return ImuInputStatus::InvalidTimestamp;
    if (!angular_velocity.allFinite() || !linear_acceleration.allFinite())
      return ImuInputStatus::NonfiniteMeasurement;

    const int64_t stamp_ns = static_cast<int64_t>(seconds) * 1000000000LL + nanoseconds;
    if (stamp_ns <= 0)
      return ImuInputStatus::InvalidTimestamp;
    if (last_stamp_ns_ != 0 && stamp_ns == last_stamp_ns_)
      return ImuInputStatus::DuplicateTimestamp;
    if (last_stamp_ns_ != 0 && stamp_ns < last_stamp_ns_)
      return ImuInputStatus::TimestampMovedBackwards;

    timestamp = static_cast<double>(stamp_ns) * 1e-9;
    if (!std::isfinite(timestamp))
      return ImuInputStatus::InvalidTimestamp;
    last_stamp_ns_ = stamp_ns;
    return ImuInputStatus::Accepted;
  }

private:
  int64_t last_stamp_ns_ = 0;
};

inline const char * imu_input_status_message(ImuInputStatus status) {
  switch (status) {
    case ImuInputStatus::Accepted:
      return "accepted";
    case ImuInputStatus::InvalidTimestamp:
      return "invalid IMU timestamp";
    case ImuInputStatus::NonfiniteMeasurement:
      return "nonfinite IMU angular velocity or acceleration";
    case ImuInputStatus::DuplicateTimestamp:
      return "duplicate IMU timestamp";
    case ImuInputStatus::TimestampMovedBackwards:
      return "IMU timestamp moved backwards; rejecting until it exceeds the last accepted sample; restart estimator if the timebase reset";
  }
  return "invalid IMU input";
}

}  // namespace ov_msckf
