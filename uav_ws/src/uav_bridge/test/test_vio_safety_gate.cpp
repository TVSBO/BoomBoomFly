#include "uav_bridge/vio_safety_gate.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

namespace
{
using Eigen::Quaterniond;
using Eigen::Vector3d;
using uav_bridge::VioKinematicGate;
using uav_bridge::VioKinematicGateResult;

void check(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

VioKinematicGateResult sample(
  VioKinematicGate & gate, double time_s, double position_x_m,
  double velocity_x_m_s = 0.0, double max_speed_m_s = 0.0)
{
  const int64_t stamp_ns = static_cast<int64_t>(time_s * 1.0e9);
  const Vector3d position(position_x_m, 0.0, 0.0);
  const Vector3d velocity(velocity_x_m_s, 0.0, 0.0);
  return gate.update(
    stamp_ns, position, Quaterniond::Identity(), velocity, velocity,
    max_speed_m_s, 1.0, 0.75, 0.12);
}

void check_prediction_rejections()
{
  VioKinematicGate spike_gate(3);
  check(sample(spike_gate, 1.0, 0.0) == VioKinematicGateResult::Accepted,
    "spike case did not accept its initial baseline");
  check(sample(spike_gate, 1.1, 0.3) == VioKinematicGateResult::PredictionError,
    "single-frame spike was not rejected");
  check(sample(spike_gate, 1.2, 0.0) == VioKinematicGateResult::Accepted,
    "return from a single-frame spike did not clear the prediction gate");

  VioKinematicGate persistent_gate(3);
  check(sample(persistent_gate, 2.0, 0.0) == VioKinematicGateResult::Accepted,
    "persistent-offset case did not accept its initial baseline");
  check(sample(persistent_gate, 2.1, 0.3) == VioKinematicGateResult::PredictionError,
    "first persistent offset was not rejected");
  check(sample(persistent_gate, 2.2, 0.3) == VioKinematicGateResult::PredictionError,
    "second persistent offset was not compared with the last accepted baseline");
  check(sample(persistent_gate, 2.3, 0.3) == VioKinematicGateResult::RepeatedPredictionError,
    "persistent offset did not reach the consecutive-error limit");

  VioKinematicGate gradual_gate(3);
  check(sample(gradual_gate, 3.0, 0.0) == VioKinematicGateResult::Accepted,
    "gradual-offset case did not accept its initial baseline");
  check(sample(gradual_gate, 3.1, 0.13) == VioKinematicGateResult::PredictionError,
    "first gradual offset was not rejected");
  check(sample(gradual_gate, 3.2, 0.26) == VioKinematicGateResult::PredictionError,
    "second gradual offset did not retain the accepted baseline");
  check(sample(gradual_gate, 3.3, 0.39) == VioKinematicGateResult::RepeatedPredictionError,
    "gradual offset did not reach the consecutive-error limit");
}

void check_speed_gate()
{
  check(uav_bridge::valid_vio_speed_limit(0.0), "zero speed limit must disable the gate");
  check(uav_bridge::valid_vio_speed_limit(2.0), "finite nonnegative speed limit was rejected");
  check(!uav_bridge::valid_vio_speed_limit(-0.1), "negative speed limit was accepted");
  check(!uav_bridge::valid_vio_speed_limit(std::numeric_limits<double>::infinity()),
    "infinite speed limit was accepted");
  check(!uav_bridge::valid_vio_speed_limit(std::numeric_limits<double>::quiet_NaN()),
    "NaN speed limit was accepted");

  VioKinematicGate disabled_gate(3);
  check(sample(disabled_gate, 4.0, 0.0, 5.0, 0.0) == VioKinematicGateResult::Accepted,
    "zero speed limit did not disable speed rejection");

  VioKinematicGate boundary_gate(3);
  check(sample(boundary_gate, 5.0, 0.0, 2.0, 2.0) == VioKinematicGateResult::Accepted,
    "speed exactly at the configured limit was rejected");

  VioKinematicGate reject_gate(3);
  check(sample(reject_gate, 6.0, 0.0, 0.0, 2.0) == VioKinematicGateResult::Accepted,
    "speed-rejection case did not accept its initial baseline");
  check(sample(reject_gate, 6.1, 0.3, 3.0, 2.0) == VioKinematicGateResult::SpeedLimit,
    "over-limit speed was not rejected");
  check(sample(reject_gate, 6.2, 0.1, 2.0, 2.0) == VioKinematicGateResult::Accepted,
    "speed rejection changed the prediction baseline or rejected the exact boundary");
}

void check_recovery_counter()
{
  uav_bridge::VioRecoveryCounter recovery;
  recovery.observe_visual(100);
  recovery.observe_visual(200);
  check(recovery.visual_count() == 2 && recovery.visual_floor_ns() == 100,
    "recovery visual count or first-visual floor is wrong");
  recovery.observe_visual(300);
  check(recovery.visual_count() == 3, "three visual samples did not satisfy the visual half");
  check(!recovery.observe_odometry(99, true) && recovery.odometry_count() == 0,
    "odometry older than the visual floor counted toward recovery");
  check(!recovery.observe_odometry(300, false) && recovery.odometry_count() == 0,
    "odometry counted without fresh visual health");
  check(!recovery.observe_odometry(300, true), "recovery completed after one odometry sample");
  check(!recovery.observe_odometry(301, true), "recovery completed after two odometry samples");
  check(recovery.observe_odometry(302, true), "three fresh odometry samples did not recover");

  recovery.reset();
  recovery.observe_visual(400);
  check(!recovery.observe_odometry(400, true), "recovery resumed without three visual samples");
  check(recovery.visual_count() == 1 && recovery.odometry_count() == 0,
    "reset did not clear recovery sample counts");
}
}  // namespace

int main()
{
  check_prediction_rejections();
  check_speed_gate();
  check_recovery_counter();
  return EXIT_SUCCESS;
}
