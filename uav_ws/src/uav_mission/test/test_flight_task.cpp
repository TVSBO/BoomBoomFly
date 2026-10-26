#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <limits>
#include "uav_mission/flight_task.hpp"

using State = uav_interfaces::msg::VehicleState;
using Command = uav_interfaces::srv::FlightCommand::Request;

class FlightTaskTest : public ::testing::Test
{
protected:
  uav_mission::FlightTaskConfig config;
  uav_mission::FlightTask::Clock::time_point now = uav_mission::FlightTask::Clock::now();
  State state;
  std::unique_ptr<uav_mission::FlightTask> task;

  void SetUp() override
  {
    config.require_vio_health = true;
    task = std::make_unique<uav_mission::FlightTask>(config);
    state.state_valid = state.connected = state.landed_state_valid = true;
    state.position_valid = state.heading_valid = state.velocity_valid = true;
    state.estimator_health_valid = state.estimator_healthy = true;
    state.preflight_check_state_valid = state.preflight_checks_passed = true;
    state.failsafe_state_valid = true;
    state.landed_state = State::LANDED_ON_GROUND;
    state.mode = "AUTO.LOITER";
    state.local_pose.pose.position.x = 2;
    state.local_pose.pose.position.y = 3;
    state.local_pose.pose.position.z = 0.4;
    state.local_pose.pose.orientation.w = 1;
  }

  auto step(double seconds = 0.05, bool fresh = true, bool vio = true)
  {
    now += std::chrono::duration_cast<uav_mission::FlightTask::Clock::duration>(
      std::chrono::duration<double>(seconds));
    const auto out = task->update(&state, fresh, now, vio, vio ? "" : "visual input lost");
    if (out.send_command) {
      EXPECT_NE(out.command, Command::ARM);  // Neither profile may arm the vehicle.
    }
    return out;
  }

  void accept(const uav_mission::FlightTask::Output & out)
  {
    ASSERT_TRUE(out.send_command);
    task->command_dispatched(out.command_id, true, "dispatched", now);
    state.last_command_id = out.command_id;
    state.command_completed = state.command_accepted = true;
  }

  void reach_takeoff()
  {
    step();
    ASSERT_EQ(task->stage_name(), "WAIT_ARM");
    state.armed = true;  // RC arm while still on the ground.
    auto out = step();
    ASSERT_EQ(task->stage_name(), "PREPARE_OFFBOARD");
    EXPECT_FALSE(out.send_command);
    EXPECT_DOUBLE_EQ(out.target.pose.position.z, 0.4);
    step(1.3);
    out = step();
    ASSERT_EQ(out.command, Command::SET_OFFBOARD);
    accept(out);
    state.mode = "OFFBOARD";
    out = step();
    ASSERT_EQ(task->stage_name(), "TAKEOFF");
    EXPECT_DOUBLE_EQ(out.target.pose.position.x, 2);
    EXPECT_DOUBLE_EQ(out.target.pose.position.y, 3);
    EXPECT_DOUBLE_EQ(out.target.pose.position.z, 1.4);
    state.landed_state = State::LANDED_IN_AIR;
  }

  void reach_square()
  {
    config.mission_profile = "square";
    task = std::make_unique<uav_mission::FlightTask>(config);
    reach_takeoff();
    state.local_pose.pose.position.z = 1.4;
    step();
    step(2.1);
    ASSERT_EQ(task->stage_name(), "HOVER");
    step(10.1);
    ASSERT_EQ(task->stage_name(), "SQUARE_LEFT");
  }

  auto arrive_at_corner()
  {
    const auto out = step();
    state.local_pose = out.target;
    state.local_velocity.vector.x = 0;
    state.local_velocity.vector.y = 0;
    state.local_velocity.vector.z = 0;
    step();
    return step(config.square_settle_duration_s + 0.01);
  }

  void reach_grid()
  {
    config.mission_profile = "grid_s";
    task = std::make_unique<uav_mission::FlightTask>(config);
    reach_takeoff();
    state.local_pose.pose.position.z = 1.4;
    step();
    step(2.1);
    ASSERT_EQ(task->stage_name(), "HOVER");
    EXPECT_EQ(step(9.9).grid_result, uav_mission::FlightTask::TaskResult::PENDING);
    step(0.2);
    ASSERT_EQ(task->stage_name(), "GRID_S_OUT_1");
    EXPECT_EQ(task->hover_result(), uav_mission::FlightTask::TaskResult::PASSED);
  }

  auto finish_grid_leg()
  {
    const auto endpoint = step(5.1);  // Default 0.4 m leg takes 5 seconds.
    state.local_pose = endpoint.target;
    state.local_velocity.vector.x = state.local_velocity.vector.y = state.local_velocity.vector.z = 0;
    step();
    return step(config.grid_settle_duration_s + 0.01);
  }
};

TEST_F(FlightTaskTest, CompleteRelativeTakeoffHoverLand)
{
  reach_takeoff();
  state.local_pose.pose.position.z = 1.4;
  step();
  EXPECT_EQ(task->stage_name(), "STABILIZE");
  step(2.1);
  EXPECT_EQ(task->stage_name(), "HOVER");
  step(10.1);
  auto out = step();
  ASSERT_EQ(out.command, Command::LAND);
  accept(out);
  state.mode = "AUTO.LAND";
  step();
  state.armed = false;
  state.landed_state = State::LANDED_ON_GROUND;
  out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.passed);
  EXPECT_FALSE(out.publish_target);
}

TEST_F(FlightTaskTest, ReadOnlyOrInhibitedCannotStart)
{
  state.control_inhibited = true;
  auto out = step();
  EXPECT_EQ(task->stage_name(), "WAIT_READY");
  EXPECT_FALSE(out.publish_target);
  EXPECT_FALSE(out.send_command);
}

TEST_F(FlightTaskTest, PreflightFailureCannotStart)
{
  state.preflight_checks_passed = false;
  EXPECT_FALSE(step().send_command);
  EXPECT_EQ(task->stage_name(), "WAIT_READY");
}

TEST_F(FlightTaskTest, VioRequiredBeforeStart)
{
  EXPECT_FALSE(step(0.05, true, false).publish_target);
  EXPECT_EQ(task->stage_name(), "WAIT_READY");
}

TEST_F(FlightTaskTest, VisualLossRequestsLandWithoutAirborneDisarm)
{
  reach_takeoff();
  step(0.05, true, false);
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  auto out = step(0.05, true, false);
  EXPECT_TRUE(out.send_command);
  EXPECT_EQ(out.command, Command::LAND);
}

TEST_F(FlightTaskTest, HeadingLossStopsTargetStream)
{
  reach_takeoff();
  state.heading_valid = false;
  auto out = step();
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  EXPECT_FALSE(out.publish_target);
  EXPECT_FALSE(step().send_command);
}

TEST_F(FlightTaskTest, StaleFcuStateStopsCommands)
{
  reach_takeoff();
  auto out = step(0.05, false);
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  EXPECT_FALSE(out.publish_target);
  out = step(0.05, false);
  EXPECT_FALSE(out.send_command);
}

TEST_F(FlightTaskTest, SquareTakeoffDistinguishesExpiredLandState)
{
  config.mission_profile = "square";
  task = std::make_unique<uav_mission::FlightTask>(config);
  reach_takeoff();
  // A real land-state timeout must remain a failure, even with live position/status.
  state.landed_state_valid = false;
  auto out = step();
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  EXPECT_EQ(task->result_reason(), "着陆状态未知、过期");
  EXPECT_FALSE(out.publish_target);
  EXPECT_FALSE(out.send_command);
  state.landed_state_valid = true;
  out = step();
  EXPECT_EQ(out.command, Command::LAND);
  EXPECT_NE(out.command, Command::DISARM);
}

TEST_F(FlightTaskTest, PilotModeTakeoverReleasesControl)
{
  reach_takeoff();
  state.mode = "POSCTL";
  auto out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.control_released);
  EXPECT_FALSE(out.publish_target);
  EXPECT_FALSE(out.send_command);
}

TEST_F(FlightTaskTest, AcknowledgementWithoutActualModeDoesNotStartClimb)
{
  step();
  state.armed = true;
  step();
  step(1.3);
  accept(step());
  for (int i = 0; i < 5; ++i) {
    auto out = step();
    EXPECT_FALSE(out.send_command);
    EXPECT_DOUBLE_EQ(out.target.pose.position.z, 0.4);
    EXPECT_EQ(task->stage_name(), "REQUEST_OFFBOARD");
  }
}

TEST_F(FlightTaskTest, BothProfilesWaitForRcArmWithoutTargetsOrCommands)
{
  for (const auto * profile : {"hover", "square"}) {
    config.mission_profile = profile;
    task = std::make_unique<uav_mission::FlightTask>(config);
    EXPECT_FALSE(step().send_command);
    ASSERT_EQ(task->stage_name(), "WAIT_ARM");
    const auto out = step(120);  // Pilot wait is not the readiness timeout.
    EXPECT_EQ(task->stage_name(), "WAIT_ARM");
    EXPECT_FALSE(out.finished);
    EXPECT_FALSE(out.publish_target);
    EXPECT_FALSE(out.send_command);
  }
}

TEST_F(FlightTaskTest, StartingWhileArmedIsRejectedForBothProfiles)
{
  state.armed = true;
  for (const auto * profile : {"hover", "square"}) {
    config.mission_profile = profile;
    task = std::make_unique<uav_mission::FlightTask>(config);
    const auto out = step();
    EXPECT_TRUE(out.finished);
    EXPECT_FALSE(out.passed);
    EXPECT_FALSE(out.publish_target);
    EXPECT_FALSE(out.send_command);
  }
}

TEST_F(FlightTaskTest, RcArmWithUnhealthyVisionNeverStartsAfterRecovery)
{
  step();
  state.armed = true;
  auto out = step(0.05, true, false);
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.control_released);
  EXPECT_FALSE(out.send_command);
  out = step(0.05, true, true);
  EXPECT_TRUE(out.finished);
  EXPECT_FALSE(out.publish_target);
  EXPECT_FALSE(out.send_command);
}

TEST_F(FlightTaskTest, ArmAfterStateGapIsNotAcceptedAsFreshPilotAuthorization)
{
  step();
  step(0.05, false);
  state.armed = true;
  const auto out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_FALSE(out.publish_target);
  EXPECT_FALSE(out.send_command);
}

TEST_F(FlightTaskTest, LossOfReadinessRequiresRecoveryWhileDisarmed)
{
  step();
  step(0.05, true, false);
  EXPECT_EQ(task->stage_name(), "WAIT_READY");
  step();
  EXPECT_EQ(task->stage_name(), "WAIT_ARM");
  state.armed = true;
  step();
  EXPECT_EQ(task->stage_name(), "PREPARE_OFFBOARD");
}

TEST_F(FlightTaskTest, OriginIsCapturedAtRcArmRatherThanTaskLaunch)
{
  step();
  state.local_pose.pose.position.x = 4;
  state.local_pose.pose.position.y = -1;
  state.local_pose.pose.position.z = 0.8;
  state.armed = true;
  auto out = step();
  EXPECT_DOUBLE_EQ(out.target.pose.position.x, 4);
  EXPECT_DOUBLE_EQ(out.target.pose.position.y, -1);
  EXPECT_DOUBLE_EQ(out.target.pose.position.z, 0.8);
  step(1.3);
  accept(step());
  state.mode = "OFFBOARD";
  out = step();
  EXPECT_EQ(task->stage_name(), "TAKEOFF");
  EXPECT_DOUBLE_EQ(out.target.pose.position.x, 4);
  EXPECT_DOUBLE_EQ(out.target.pose.position.y, -1);
  EXPECT_DOUBLE_EQ(out.target.pose.position.z, 1.8);
}

TEST_F(FlightTaskTest, CancelWhileWaitingCannotBeRevivedByRcArm)
{
  step();
  task->cancel(now);
  state.armed = true;
  const auto out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.canceled);
  EXPECT_FALSE(out.publish_target);
  EXPECT_FALSE(out.send_command);
}

TEST_F(FlightTaskTest, CancelBeforeOffboardDispatchLeavesPilotInControl)
{
  step();
  state.armed = true;
  step();
  task->cancel(now);
  const auto out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.control_released);
  EXPECT_FALSE(out.publish_target);
  EXPECT_FALSE(out.send_command);
  EXPECT_EQ(state.mode, "AUTO.LOITER");
  EXPECT_TRUE(state.armed);
}

TEST_F(FlightTaskTest, CancelWithOffboardInFlightOnlyDisarmsOnFreshGround)
{
  step();
  state.armed = true;
  step();
  step(1.3);
  auto out = step();
  accept(out);
  task->cancel(now);
  out = step(0.05, false);
  EXPECT_FALSE(out.send_command);
  EXPECT_FALSE(out.publish_target);
  // Mode-switch ACK/state may arrive after cancel; never climb or send ARM.
  state.mode = "OFFBOARD";
  out = step();
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  ASSERT_TRUE(out.send_command);
  ASSERT_EQ(out.command, Command::DISARM);
  accept(out);
  state.armed = false;
  out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.canceled);
}

TEST_F(FlightTaskTest, RcDisarmDuringPreparationPreventsDelayedTakeoff)
{
  step();
  state.armed = true;
  step();
  state.armed = false;
  auto out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_FALSE(out.send_command);
  state.armed = true;
  out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_FALSE(out.send_command);
  EXPECT_FALSE(out.publish_target);
}

TEST_F(FlightTaskTest, ModeChangeBeforeOffboardDispatchStopsAutomaticTakeover)
{
  step();
  state.armed = true;
  step();
  state.mode = "POSCTL";
  const auto out = step(1.3);
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.control_released);
  EXPECT_FALSE(out.send_command);
  EXPECT_FALSE(out.publish_target);
}

TEST_F(FlightTaskTest, ManualTakeoffBeforeOffboardDispatchStopsTask)
{
  step();
  state.armed = true;
  step();
  state.landed_state = State::LANDED_IN_AIR;
  const auto out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.control_released);
  EXPECT_FALSE(out.send_command);
}

TEST_F(FlightTaskTest, CancelInAirLands)
{
  reach_takeoff();
  task->cancel(now);
  auto out = step();
  EXPECT_TRUE(out.send_command);
  EXPECT_EQ(out.command, Command::LAND);
}

TEST_F(FlightTaskTest, SquareClosesAtHomeWithConfirmedLandingAndDisarm)
{
  reach_square();
  EXPECT_EQ(task->hover_result(), uav_mission::FlightTask::TaskResult::PASSED);
  EXPECT_EQ(task->square_result(), uav_mission::FlightTask::TaskResult::PENDING);
  const std::array<std::array<double, 2>, 4> corners{{
    {{2, 3.5}}, {{2.5, 3.5}}, {{2.5, 3}}, {{2, 3}}}};
  for (unsigned i = 0; i < corners.size(); ++i) {
    const auto out = step();
    ASSERT_TRUE(out.publish_target);
    EXPECT_NEAR(out.target.pose.position.x, corners[i][0], 1e-9);
    EXPECT_NEAR(out.target.pose.position.y, corners[i][1], 1e-9);
    EXPECT_NEAR(out.target.pose.position.z, 1.4, 1e-9);
    EXPECT_DOUBLE_EQ(out.target.pose.orientation.w, 1);
    EXPECT_DOUBLE_EQ(out.target.pose.orientation.z, 0);
    arrive_at_corner();
    EXPECT_EQ(task->completed_square_legs(), i + 1);
  }
  ASSERT_EQ(task->stage_name(), "REQUEST_LAND");
  EXPECT_EQ(task->square_result(), uav_mission::FlightTask::TaskResult::PASSED);
  auto out = step();
  ASSERT_EQ(out.command, Command::LAND);
  accept(out);
  state.mode = "AUTO.LAND";
  step();
  state.landed_state = State::LANDED_ON_GROUND;
  EXPECT_FALSE(step().finished);
  out = step();
  ASSERT_EQ(out.command, Command::DISARM);
  accept(out);
  EXPECT_FALSE(step().finished);  // ACK alone is insufficient.
  state.armed = false;
  out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.passed);
  EXPECT_FALSE(out.publish_target);
}

TEST_F(FlightTaskTest, HoverResultSelectsContinuationOnlyAfterFullSuccessfulHold)
{
  using Result = uav_mission::FlightTask::TaskResult;
  for (const auto * profile : {"hover", "square"}) {
    config.mission_profile = profile;
    task = std::make_unique<uav_mission::FlightTask>(config);
    state.armed = false;
    state.mode = "POSCTL";
    state.landed_state = State::LANDED_ON_GROUND;
    state.local_pose.pose.position.z = 0.4;
    state.command_completed = false;
    reach_takeoff();
    EXPECT_EQ(step().hover_result, Result::PENDING);
    state.local_pose.pose.position.z = 1.4;
    step();
    step(2.1);
    ASSERT_EQ(task->stage_name(), "HOVER");
    EXPECT_EQ(step(9.9).hover_result, Result::PENDING);
    EXPECT_EQ(task->completed_square_legs(), 0u);
    const auto out = step(0.2);
    EXPECT_EQ(out.hover_result, Result::PASSED);
    EXPECT_FALSE(out.finished);  // Subtask success is not final mission success.
    EXPECT_FALSE(out.send_command);
    EXPECT_EQ(task->stage_name(), std::string(profile) == "square" ?
      "SQUARE_LEFT" : "REQUEST_LAND");
  }
}

TEST_F(FlightTaskTest, FailedHoverNeverStartsSquare)
{
  using Result = uav_mission::FlightTask::TaskResult;
  config.mission_profile = "square";
  task = std::make_unique<uav_mission::FlightTask>(config);
  reach_takeoff();
  state.local_pose.pose.position.z = 1.4;
  step();
  step(2.1);
  ASSERT_EQ(task->stage_name(), "HOVER");
  state.local_pose.pose.position.x += 0.3;
  const auto out = step(10.1);  // Failure wins even at the completion deadline.
  EXPECT_EQ(out.hover_result, Result::FAILED);
  EXPECT_EQ(out.square_result, Result::PENDING);
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  EXPECT_EQ(task->completed_square_legs(), 0u);
}

TEST_F(FlightTaskTest, SquareFailurePreservesEarlierHoverSuccess)
{
  using Result = uav_mission::FlightTask::TaskResult;
  reach_square();
  const auto out = step(config.square_leg_timeout_s + 0.1);
  EXPECT_EQ(out.hover_result, Result::PASSED);
  EXPECT_EQ(out.square_result, Result::FAILED);
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
}

TEST_F(FlightTaskTest, GroundHeightEstimateCannotCountAsSuccessfulHover)
{
  config.mission_profile = "square";
  task = std::make_unique<uav_mission::FlightTask>(config);
  reach_takeoff();
  state.landed_state = State::LANDED_ON_GROUND;
  state.local_pose.pose.position.z = 1.4;
  step();
  step(12.2);
  EXPECT_EQ(task->stage_name(), "TAKEOFF");
  EXPECT_EQ(task->hover_result(), uav_mission::FlightTask::TaskResult::PENDING);
  EXPECT_EQ(task->completed_square_legs(), 0u);
}

TEST_F(FlightTaskTest, SquareUsesInitialHeadingAndAnchoredCorners)
{
  const double yaw = std::acos(-1.0) / 2;
  state.local_pose.pose.orientation.w = std::cos(yaw / 2);
  state.local_pose.pose.orientation.z = std::sin(yaw / 2);
  reach_square();
  const std::array<std::array<double, 2>, 4> corners{{
    {{1.5, 3}}, {{1.5, 3.5}}, {{2, 3.5}}, {{2, 3}}}};
  for (unsigned i = 0; i < corners.size(); ++i) {
    const auto out = step();
    EXPECT_NEAR(out.target.pose.position.x, corners[i][0], 1e-9);
    EXPECT_NEAR(out.target.pose.position.y, corners[i][1], 1e-9);
    EXPECT_NEAR(out.target.pose.orientation.z, std::sin(yaw / 2), 1e-9);
    // Finish slightly off-center: later corners must not accumulate this drift.
    state.local_pose = out.target;
    state.local_pose.pose.position.x += 0.03;
    step();
    step(0.51);
  }
  EXPECT_EQ(task->completed_square_legs(), 4u);
  EXPECT_EQ(task->stage_name(), "REQUEST_LAND");
}

TEST_F(FlightTaskTest, SquareRequiresContinuousPositionAndSpeedSettling)
{
  reach_square();
  state.local_pose = step().target;
  state.local_velocity.vector.x = 0.20;
  step(0.7);
  EXPECT_EQ(task->completed_square_legs(), 0u);
  state.local_velocity.vector.x = 0;
  step();
  step(0.3);
  state.local_velocity.vector.z = 0.20;  // Vertical speed also matters.
  step();
  state.local_velocity.vector.z = 0;
  step();
  step(0.3);
  EXPECT_EQ(task->completed_square_legs(), 0u);
  step(0.21);
  EXPECT_EQ(task->completed_square_legs(), 1u);
}

TEST_F(FlightTaskTest, SquarePositionExcursionRestartsSettleTimer)
{
  reach_square();
  state.local_pose = step().target;
  step();
  step(0.3);
  state.local_pose.pose.position.x += 0.10;
  step();
  state.local_pose.pose.position.x -= 0.10;
  step();
  step(0.3);
  EXPECT_EQ(task->completed_square_legs(), 0u);
  step(0.21);
  EXPECT_EQ(task->completed_square_legs(), 1u);
}

TEST_F(FlightTaskTest, SquareUsesThreeDimensionalArrivalTolerance)
{
  reach_square();
  state.local_pose = step().target;
  state.local_pose.pose.position.x += 0.06;
  state.local_pose.pose.position.z += 0.06;
  step();
  step(0.6);
  EXPECT_EQ(task->completed_square_legs(), 0u);  // sqrt(.06^2+.06^2) > .08.
}

TEST_F(FlightTaskTest, SquareDoesNotAdvanceOnNonfiniteSpeed)
{
  reach_square();
  state.local_pose = step().target;
  state.local_velocity.vector.x = std::numeric_limits<double>::quiet_NaN();
  step();
  step(0.6);
  EXPECT_EQ(task->completed_square_legs(), 0u);
}

TEST_F(FlightTaskTest, SquareLegTimeoutRequestsSafetyLanding)
{
  reach_square();
  step(config.square_leg_timeout_s + 0.1);
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  auto out = step();
  EXPECT_TRUE(out.send_command);
  EXPECT_EQ(out.command, Command::LAND);
  EXPECT_EQ(task->completed_square_legs(), 0u);
}

TEST_F(FlightTaskTest, SquareTimeoutAppliesSeparatelyToEachLeg)
{
  reach_square();
  arrive_at_corner();
  EXPECT_EQ(task->stage_name(), "SQUARE_FORWARD");
  step(config.square_leg_timeout_s + 0.1);
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  EXPECT_EQ(task->completed_square_legs(), 1u);
}

TEST_F(FlightTaskTest, SquareHorizontalEnvelopeViolationLands)
{
  reach_square();
  state.local_pose.pose.position.x += 2;
  step();
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  EXPECT_EQ(step().command, Command::LAND);
}

TEST_F(FlightTaskTest, SquareAltitudeEnvelopeViolationLands)
{
  reach_square();
  state.local_pose.pose.position.z += 0.3;
  step();
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
}

TEST_F(FlightTaskTest, SquareVisualLossAndRecoveryDoNotResumeOldRoute)
{
  reach_square();
  step(0.05, true, false);
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  auto out = step();
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  EXPECT_TRUE(out.send_command);
  EXPECT_EQ(out.command, Command::LAND);
  EXPECT_EQ(task->completed_square_legs(), 0u);
}

TEST_F(FlightTaskTest, SquareCancelLandsAndPilotTakeoverStopsCommands)
{
  reach_square();
  task->cancel(now);
  EXPECT_EQ(step().command, Command::LAND);
  state.mode = "POSCTL";
  const auto out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.control_released);
  EXPECT_FALSE(out.send_command);
  EXPECT_FALSE(out.publish_target);
}

TEST_F(FlightTaskTest, InvalidSquareConfigurationIsRejectedBeforeFlight)
{
  config.mission_profile = "typo";
  EXPECT_THROW(uav_mission::FlightTask{config}, std::invalid_argument);
  config.mission_profile = "square";
  config.square_side_m = 0;
  EXPECT_THROW(uav_mission::FlightTask{config}, std::invalid_argument);
  config.square_side_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(uav_mission::FlightTask{config}, std::invalid_argument);
  config.square_side_m = 0.05;
  EXPECT_THROW(uav_mission::FlightTask{config}, std::invalid_argument);
  config.square_side_m = 0.5;
  config.square_settle_duration_s = config.square_leg_timeout_s;
  EXPECT_THROW(uav_mission::FlightTask{config}, std::invalid_argument);
}

TEST_F(FlightTaskTest, GridVisitsEveryPointAndRetracesBeforeLanding)
{
  reach_grid();
  const std::array<std::array<int, 2>, 16> points{{
    {{0,1}}, {{0,2}}, {{1,2}}, {{1,1}}, {{1,0}}, {{2,0}}, {{2,1}}, {{2,2}},
    {{2,1}}, {{2,0}}, {{1,0}}, {{1,1}}, {{1,2}}, {{0,2}}, {{0,1}}, {{0,0}}}};
  for (unsigned i = 0; i < points.size(); ++i) {
    const auto endpoint = step(5.1);
    EXPECT_NEAR(endpoint.target.pose.position.x, 2 + points[i][0] * 0.4, 1e-9);
    EXPECT_NEAR(endpoint.target.pose.position.y, 3 + points[i][1] * 0.4, 1e-9);
    EXPECT_DOUBLE_EQ(endpoint.target.pose.position.z, 1.4);
    EXPECT_EQ(task->completed_grid_legs(), i);
    state.local_pose = endpoint.target;
    step();
    step(0.51);
    EXPECT_EQ(task->completed_grid_legs(), i + 1);
  }
  EXPECT_EQ(task->stage_name(), "REQUEST_LAND");
  EXPECT_EQ(task->completed_grid_round_trips(), 1u);
  EXPECT_EQ(task->grid_result(), uav_mission::FlightTask::TaskResult::PASSED);
  auto out = step();
  ASSERT_EQ(out.command, Command::LAND);
  accept(out);
  state.mode = "AUTO.LAND";
  step();
  state.landed_state = State::LANDED_ON_GROUND;
  state.armed = false;
  out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.passed);
}

TEST_F(FlightTaskTest, GridRepeatsExactlyConfiguredRounds)
{
  config.grid_round_trips = 2;
  reach_grid();
  for (unsigned i = 0; i < 32; ++i) {
    finish_grid_leg();
    EXPECT_EQ(task->completed_grid_legs(), i + 1);
    EXPECT_EQ(task->completed_grid_round_trips(), (i + 1) / 16);
    if (i < 31) {
      EXPECT_NE(task->stage_name(), "REQUEST_LAND");
      EXPECT_EQ(task->grid_result(), uav_mission::FlightTask::TaskResult::PENDING);
    }
  }
  EXPECT_EQ(task->stage_name(), "REQUEST_LAND");
}

TEST_F(FlightTaskTest, GridAnchorsHeadingAndDoesNotAccumulateArrivalErrors)
{
  state.local_pose.pose.orientation.w = std::sqrt(0.5);
  state.local_pose.pose.orientation.z = std::sqrt(0.5);
  reach_grid();
  auto out = step(5.1);
  EXPECT_NEAR(out.target.pose.position.x, 1.6, 1e-9);
  EXPECT_NEAR(out.target.pose.position.y, 3, 1e-9);
  state.local_pose = out.target;
  state.local_pose.pose.position.x += 0.02;
  step();
  step(0.51);
  out = step(5.1);
  EXPECT_NEAR(out.target.pose.position.x, 1.2, 1e-9);
  EXPECT_NEAR(out.target.pose.position.y, 3, 1e-9);
  EXPECT_NEAR(out.target.pose.orientation.z, std::sqrt(0.5), 1e-9);
}

TEST_F(FlightTaskTest, GridReferenceRespectsSpeedAccelerationAndCannotFinishWithoutArrival)
{
  config.grid_speed_m_s = 0.5;
  config.grid_acceleration_m_s2 = 0.1;
  reach_grid();
  double previous = state.local_pose.pose.position.y, previous_velocity = 0;
  for (unsigned i = 0; i < 600; ++i) {
    const auto out = step(0.01);
    const double y = out.target.pose.position.y;
    const double velocity = (y - previous) / 0.01;
    EXPECT_GE(y, previous - 1e-9);
    EXPECT_LE(y, 3.4 + 1e-9);
    EXPECT_LE(std::abs(velocity), config.grid_speed_m_s + 1e-3);
    EXPECT_LE(std::abs(velocity - previous_velocity) / 0.01,
      config.grid_acceleration_m_s2 + 1e-3);
    previous = y;
    previous_velocity = velocity;
  }
  EXPECT_EQ(task->completed_grid_legs(), 0u);
}

TEST_F(FlightTaskTest, GridRequiresFullReferenceAndContinuousSlowArrival)
{
  reach_grid();
  state.local_pose.pose.position.y = 3.4;
  step(0.6);
  EXPECT_EQ(task->completed_grid_legs(), 0u);
  state.local_velocity.vector.y = 0.2;
  step(4.5);
  step(0.6);
  EXPECT_EQ(task->completed_grid_legs(), 0u);
  state.local_velocity.vector.y = 0;
  step();
  step(0.3);
  state.local_pose.pose.position.y = 3.2;
  step();
  state.local_pose.pose.position.y = 3.4;
  step();
  step(0.3);
  EXPECT_EQ(task->completed_grid_legs(), 0u);
  step(0.3);
  EXPECT_EQ(task->completed_grid_legs(), 1u);
}

TEST_F(FlightTaskTest, GridTimeoutFailsAndPreservesHoverSuccess)
{
  reach_grid();
  auto out = step(config.grid_leg_timeout_s + 0.1);
  EXPECT_EQ(out.grid_result, uav_mission::FlightTask::TaskResult::FAILED);
  EXPECT_EQ(out.hover_result, uav_mission::FlightTask::TaskResult::PASSED);
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  EXPECT_EQ(step().command, Command::LAND);
}

TEST_F(FlightTaskTest, GridVioLossNeverResumesOldRoute)
{
  reach_grid();
  const auto out = step(0.05, true, false);
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
  EXPECT_EQ(out.grid_result, uav_mission::FlightTask::TaskResult::FAILED);
  EXPECT_EQ(step().command, Command::LAND);
  EXPECT_EQ(task->completed_grid_legs(), 0u);
}

TEST_F(FlightTaskTest, GridEnvelopeViolationFails)
{
  reach_grid();
  state.local_pose.pose.position.y = 4.1;
  EXPECT_EQ(step().grid_result, uav_mission::FlightTask::TaskResult::FAILED);
  EXPECT_EQ(task->stage_name(), "SAFETY_LAND");
}

TEST_F(FlightTaskTest, GridCancelRequestsLand)
{
  reach_grid();
  task->cancel(now);
  const auto out = step();
  EXPECT_EQ(out.grid_result, uav_mission::FlightTask::TaskResult::FAILED);
  EXPECT_EQ(out.command, Command::LAND);
}

TEST_F(FlightTaskTest, GridPilotTakeoverReleasesControl)
{
  reach_grid();
  state.mode = "POSCTL";
  const auto out = step();
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.control_released);
  EXPECT_FALSE(out.send_command);
  EXPECT_FALSE(out.publish_target);
  EXPECT_EQ(out.grid_result, uav_mission::FlightTask::TaskResult::FAILED);
}

TEST_F(FlightTaskTest, GridRejectsInvalidOrUnreachableConfiguration)
{
  config.mission_profile = "grid_s";
  config.grid_round_trips = 0;
  EXPECT_THROW(uav_mission::FlightTask{config}, std::invalid_argument);
  config.grid_round_trips = 1;
  config.grid_spacing_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(uav_mission::FlightTask{config}, std::invalid_argument);
  config.grid_spacing_m = 0.4;
  config.grid_speed_m_s = 0.01;
  EXPECT_THROW(uav_mission::FlightTask{config}, std::invalid_argument);
}
