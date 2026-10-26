"""Exercise the actual read-only DDS backend with synthetic FCU data in domain 83."""
import math
import os
import subprocess
import time

import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (EstimatorStatusFlags, TimesyncStatus, VehicleAttitude,
                          TrajectorySetpoint, VehicleCommand, VehicleLandDetected,
                          VehicleLocalPosition, VehicleStatus)
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from uav_interfaces.msg import VehicleState
from uav_interfaces.srv import FlightCommand


@pytest.fixture
def rig():
    assert os.environ['ROS_DOMAIN_ID'] == '83', 'Never inject synthetic FCU data in the flight domain'
    rclpy.init()
    node = rclpy.create_node('dds_readiness_test')
    publishers = {}
    types = {'vehicle_status': VehicleStatus, 'vehicle_local_position': VehicleLocalPosition,
             'vehicle_attitude': VehicleAttitude, 'vehicle_land_detected': VehicleLandDetected,
             'timesync_status': TimesyncStatus, 'estimator_status_flags': EstimatorStatusFlags}
    for name, cls in types.items():
        version = getattr(cls, 'MESSAGE_VERSION', 0)
        topic = '/fmu/out/' + name + (f'_v{version}' if version else '')
        publishers[name] = node.create_publisher(cls, topic, qos_profile_sensor_data)
    states = []
    sub = node.create_subscription(VehicleState, '/uav_control/state', states.append, 10)
    process = None
    last_land_send = 0.0

    def start(local=True, read_only=True):
        nonlocal process
        process = subprocess.Popen([os.environ['CONTROL_EXECUTABLE'], '--ros-args',
                                   '-p', 'backend:=dds', '-p', f'read_only:={str(read_only).lower()}',
                                   '-p', f'allow_local_vision_heading:={str(local).lower()}'])

    def feed(duration=0.25, land_period=0.0, omit_land=False, omit_status=False, **changes):
        nonlocal last_land_send
        end = time.monotonic() + duration
        while time.monotonic() < end:
            stamp = time.time_ns() // 1000
            status = VehicleStatus(timestamp=stamp, system_id=1, component_id=1,
                                   arming_state=VehicleStatus.ARMING_STATE_DISARMED,
                                   nav_state=VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
                                   pre_flight_checks_pass=False)
            position = VehicleLocalPosition(timestamp=stamp, timestamp_sample=stamp,
                xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
                x=2.0, y=3.0, z=-4.0, vx=0.1, vy=0.2, vz=-0.3,
                heading_good_for_control=False)
            attitude = VehicleAttitude(timestamp=stamp, timestamp_sample=stamp,
                                       q=[1.0, 0.0, 0.0, 0.0])
            flags = EstimatorStatusFlags(timestamp=stamp, timestamp_sample=stamp,
                cs_tilt_align=True, cs_ev_yaw=True)
            objects = {'vehicle_status': status, 'vehicle_local_position': position,
                       'vehicle_attitude': attitude, 'estimator_status_flags': flags,
                       'vehicle_land_detected': VehicleLandDetected(timestamp=stamp, landed=True),
                       'timesync_status': TimesyncStatus(timestamp=stamp, remote_timestamp=stamp,
                           source_protocol=TimesyncStatus.SOURCE_PROTOCOL_DDS)}
            for path, value in changes.items():
                group, field = path.split('__')
                setattr(objects[group], field, value)
            for name, message in objects.items():
                if name == 'vehicle_land_detected':
                    if omit_land or time.monotonic() - last_land_send < land_period:
                        continue
                    last_land_send = time.monotonic()
                if name == 'vehicle_status' and omit_status:
                    continue
                if not (name == 'estimator_status_flags' and changes.get('estimator_status_flags__timestamp') == 0):
                    publishers[name].publish(message)
            rclpy.spin_once(node, timeout_sec=0.02)
            assert process.poll() is None
        assert states
        return states[-1]

    def reset():
        client = node.create_client(Trigger, '/uav_control/reset')
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        while not future.done():
            feed(0.05)
        result = future.result()
        node.destroy_client(client)
        return result

    yield start, feed, reset, node
    if process is not None:
        process.terminate()
        process.wait(timeout=5)
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()


def test_local_heading_and_guards(rig):
    start, feed, reset, node = rig
    start()
    state = feed(2.0)
    assert state.heading_valid and state.position_valid and state.estimator_healthy
    assert state.control_inhibited and state.control_message == 'read-only backend'
    assert not state.preflight_checks_passed  # The local-heading option cannot bypass FCU preflight.
    p = state.local_pose.pose.position
    assert (p.x, p.y, p.z) == (3.0, 2.0, 4.0)
    q = state.local_pose.pose.orientation
    assert math.isclose(q.w, math.sqrt(0.5), abs_tol=1e-6)
    assert math.isclose(q.z, math.sqrt(0.5), abs_tol=1e-6)
    assert not any(name == '/uav_control/command' for name, _ in node.get_service_names_and_types())

    for field, value in [('cs_ev_yaw', False), ('cs_tilt_align', False),
                         ('cs_ev_yaw_fault', True), ('fs_bad_hdg', True),
                         ('reject_yaw', True), ('cs_yaw_align', True)]:
        assert not feed(**{'estimator_status_flags__' + field: value}).heading_valid, field
        assert feed().heading_valid
    assert not feed(vehicle_attitude__q=[float('nan'), 0.0, 0.0, 0.0]).heading_valid
    assert feed().heading_valid
    # Keep all other FCU streams fresh while fusion status expires.
    assert not feed(2.3, estimator_status_flags__timestamp=0).heading_valid
    assert feed().heading_valid
    assert not feed(vehicle_local_position__dead_reckoning=True).estimator_healthy
    assert feed().estimator_healthy
    assert not feed(vehicle_local_position__heading_reset_counter=1).heading_valid
    assert not feed().heading_valid  # Recovery alone must not clear a frame-reset latch.
    assert reset().success
    assert feed().heading_valid


def test_default_still_requires_global_heading(rig):
    start, feed, _, _ = rig
    start(local=False)
    assert not feed(2.0).heading_valid
    assert feed(vehicle_local_position__heading_good_for_control=True).heading_valid


def test_target_axes_and_preflight_guard_in_isolated_domain(rig):
    start, feed, _, node = rig
    start(read_only=False)
    publisher = node.create_publisher(PoseStamped, '/uav_control/target', 10)
    setpoints, commands = [], []
    version = getattr(TrajectorySetpoint, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    sub = node.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint' + suffix,
                                   setpoints.append, 10)
    version = getattr(VehicleCommand, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    command_sub = node.create_subscription(VehicleCommand, '/fmu/in/vehicle_command' + suffix,
                                           commands.append, 10)
    assert feed(2.0).heading_valid
    target = PoseStamped()
    target.header.frame_id = 'map'
    target.pose.position.x, target.pose.position.y, target.pose.position.z = 1.5, -2.0, 0.7
    target.pose.orientation.w = 1.0
    for _ in range(30):
        target.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(target)
        feed(0.05)
    assert setpoints
    actual = setpoints[-1]
    assert list(actual.position) == pytest.approx([-2.0, 1.5, -0.7])
    assert actual.yaw == pytest.approx(math.pi / 2)
    assert all(math.isnan(v) for v in actual.velocity)
    client = node.create_client(FlightCommand, '/uav_control/command')
    assert client.wait_for_service(timeout_sec=2)
    result = client.call_async(FlightCommand.Request(command_id=1, command=FlightCommand.Request.SET_OFFBOARD))
    rclpy.spin_until_future_complete(node, result, timeout_sec=1)
    assert result.done() and not result.result().dispatched
    assert not commands  # Local heading and prestream cannot bypass the failing preflight.
    node.destroy_subscription(sub)
    node.destroy_subscription(command_sub)
    node.destroy_publisher(publisher)
    node.destroy_client(client)


def test_low_rate_land_status_jitter_and_real_loss(rig):
    start, feed, _, _ = rig
    start()
    assert feed(2.0).landed_state_valid
    assert feed(vehicle_land_detected__landed=False).landed_state == VehicleState.LANDED_IN_AIR
    # An unchanged LandDetector state arrives at ~1 Hz plus scheduling jitter.
    # Inspect throughout multiple periods, including the old >1-second expiry window.
    for _ in range(16):
        state = feed(0.25, land_period=1.1, vehicle_land_detected__landed=False)
        assert state.state_valid and state.landed_state_valid
        assert state.landed_state == VehicleState.LANDED_IN_AIR
    # Losing only the land stream must still expire it, without invalidating FCU status.
    state = feed(2.8, omit_land=True)
    assert state.state_valid and not state.landed_state_valid
    assert state.landed_state == VehicleState.LANDED_UNKNOWN
    assert feed().landed_state_valid
    # The 1-second vehicle-status guard is independent of the land timeout.
    state = feed(1.3, omit_status=True)
    assert not state.state_valid and not state.landed_state_valid
    assert feed().landed_state_valid
    # Fresh receipt of an old timestamp must not keep the landed state valid.
    old_stamp = time.time_ns() // 1000 - 4_000_000
    state = feed(2.8, vehicle_land_detected__timestamp=old_stamp)
    assert state.state_valid and not state.landed_state_valid
    assert feed().landed_state_valid
