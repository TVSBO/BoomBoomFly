#include <gtest/gtest.h>

#include <limits>

#include "state/Propagator.h"
#include "state/State.h"
#include "types/IMU.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "utils/sensor_data.h"
#include "ros/ImuInputGuard.h"

namespace {
TEST(FastPropagation, NonzeroOffsetsUseImuTargetAcrossCacheRebuilds) {
  for (const double offset : {-0.0052691525299192535, 0.02, 0.0}) {
    ov_msckf::StateOptions state_options;
    auto state = std::make_shared<ov_msckf::State>(state_options);
    state->_timestamp = 10.0;
    Eigen::Matrix<double, 1, 1> time_offset;
    time_offset << offset;
    state->_calib_dt_CAMtoIMU->set_value(time_offset);
    Eigen::Matrix<double, 16, 1> value = state->_imu->value();
    value.segment<3>(7) << 2.0, 0.0, 0.0;
    state->_imu->set_value(value);
    ov_msckf::Propagator propagator(ov_msckf::NoiseManager{}, 9.81);
    for (int i = 0; i <= 400; ++i) {
      ov_core::ImuData measurement;
      measurement.timestamp = 9.9 + i * 0.001;
      measurement.wm.setZero();
      measurement.am << 0.0, 0.0, 9.81;
      propagator.feed_imu(measurement);
    }
    Eigen::Matrix<double, 13, 1> predicted;
    Eigen::Matrix<double, 12, 12> covariance;
    for (const double target : {10.10, 10.15}) {
      ASSERT_TRUE(propagator.fast_state_propagate(state, target, predicted, covariance));
      EXPECT_NEAR(predicted(4), 2.0 * (target - 10.0 - offset), 1e-9);
      EXPECT_NEAR(predicted(6), 0.0, 1e-9);
      EXPECT_NEAR(predicted(7), 2.0, 1e-9);
      EXPECT_TRUE(covariance.allFinite());
    }
    state->_timestamp = 10.05;
    value(4) = 3.0;
    state->_imu->set_value(value);
    propagator.invalidate_cache();
    ASSERT_TRUE(propagator.fast_state_propagate(state, 10.16, predicted, covariance));
    EXPECT_NEAR(predicted(4), 3.0 + 2.0 * (10.16 - 10.05 - offset), 1e-9);
    EXPECT_FALSE(propagator.fast_state_propagate(state, 10.16, predicted, covariance));
    EXPECT_FALSE(propagator.fast_state_propagate(state, 10.15, predicted, covariance));
    ASSERT_TRUE(propagator.fast_state_propagate(state, 10.17, predicted, covariance));
    EXPECT_NEAR(predicted(4), 3.0 + 2.0 * (10.17 - 10.05 - offset), 1e-9);
  }
}

TEST(VisualFusion, EmptyFeatureUpdatesDoNotReportFusion) {
  ov_msckf::StateOptions state_options;
  auto state = std::make_shared<ov_msckf::State>(state_options);
  ov_msckf::UpdaterOptions options;
  ov_core::FeatureInitializerOptions initialization;
  ov_msckf::UpdaterMSCKF msckf(options, initialization);
  ov_msckf::UpdaterSLAM slam(options, options, initialization);
  std::vector<std::shared_ptr<ov_core::Feature>> features;
  EXPECT_FALSE(msckf.update(state, features));
  EXPECT_FALSE(slam.update(state, features));
}

TEST(ImuInputGuard, RejectsNonIncreasingSamplesWithoutResettingTheFilterTimeline) {
  ov_msckf::ImuInputGuard guard;
  Eigen::Vector3d angular_velocity(0.1, -0.2, 0.3);
  Eigen::Vector3d linear_acceleration(0.0, 0.0, 9.81);
  double timestamp = 0.0;

  EXPECT_EQ(guard.accept(1, 250000000U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::Accepted);
  EXPECT_DOUBLE_EQ(timestamp, 1.25);
  EXPECT_EQ(guard.accept(1, 300000000U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::Accepted);
  EXPECT_DOUBLE_EQ(timestamp, 1.3);
  EXPECT_EQ(guard.accept(1, 300000000U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::DuplicateTimestamp);
  EXPECT_EQ(guard.accept(1, 200000000U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::TimestampMovedBackwards);
  EXPECT_EQ(guard.accept(1, 200000000U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::TimestampMovedBackwards);
  EXPECT_EQ(guard.accept(1, 350000000U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::Accepted);
  EXPECT_DOUBLE_EQ(timestamp, 1.35);
}

TEST(ImuInputGuard, RejectsInvalidStampsAndNonfiniteMotionWithoutAdvancingTime) {
  ov_msckf::ImuInputGuard guard;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration(0.0, 0.0, 9.81);
  double timestamp = 0.0;

  EXPECT_EQ(guard.accept(0, 0U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::InvalidTimestamp);
  EXPECT_EQ(guard.accept(-1, 0U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::InvalidTimestamp);
  EXPECT_EQ(guard.accept(1, 1000000000U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::InvalidTimestamp);
  angular_velocity.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(guard.accept(1, 0U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::NonfiniteMeasurement);
  angular_velocity.setZero();
  linear_acceleration.z() = std::numeric_limits<double>::infinity();
  EXPECT_EQ(guard.accept(1, 0U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::NonfiniteMeasurement);
  linear_acceleration.z() = 9.81;
  EXPECT_EQ(guard.accept(1, 0U, angular_velocity, linear_acceleration, timestamp),
      ov_msckf::ImuInputStatus::Accepted);
  EXPECT_DOUBLE_EQ(timestamp, 1.0);
}
} // namespace
