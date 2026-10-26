#!/usr/bin/env python3
"""Read-only check of OpenVINS visual updates and the selected odometry bridge."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu

from vio_tools import dds_message_topic, new_sample, observe_sample
from vio_tools import parse_backend_config, parse_bridge_diagnostics
from vio_tools import parse_visual_health, summarize_sample, within_stamp_tolerance


COMMON_TOPICS = {
    '/camera/camera/infra1/image_rect_raw': Image,
    '/camera/camera/infra2/image_rect_raw': Image,
    '/ov_msckf/odomimu': Odometry,
    '/ov_msckf/visual_health': DiagnosticArray,
    '/uav_bridge/diagnostics': DiagnosticArray,
}


def _backend_topics(backend, fmu_prefix):
    topics = dict(COMMON_TOPICS)
    if backend == 'mavros':
        from mavros_msgs.msg import State

        state_topic = '/mavros/state'
        imu_topic = '/mavros/imu/data_raw'
        raw_imu_topic = None
        output_topic = '/mavros/odometry/out'
        topics[state_topic] = State
        topics[imu_topic] = Imu
        topics[output_topic] = Odometry
    else:
        from px4_msgs.msg import SensorCombined, VehicleOdometry, VehicleStatus

        state_topic = dds_message_topic(
            fmu_prefix, 'out/vehicle_status', getattr(VehicleStatus, 'MESSAGE_VERSION', 0))
        imu_topic = '/px4/imu'
        raw_imu_topic = dds_message_topic(
            fmu_prefix, 'out/sensor_combined', getattr(SensorCombined, 'MESSAGE_VERSION', 0))
        output_topic = dds_message_topic(
            fmu_prefix, 'in/vehicle_visual_odometry',
            getattr(VehicleOdometry, 'MESSAGE_VERSION', 0))
        topics[state_topic] = VehicleStatus
        topics[imu_topic] = Imu
        topics[raw_imu_topic] = SensorCombined
        topics[output_topic] = VehicleOdometry
    return topics, state_topic, imu_topic, raw_imu_topic, output_topic


def _observe_backend_stamp(sample, stamp_us):
    stamp_us = int(stamp_us)
    previous = sample.get('last_backend_stamp_us')
    if stamp_us <= 0:
        sample['zero_stamps'] += 1
    elif previous is not None:
        if stamp_us == previous:
            sample['duplicate_stamps'] += 1
        elif stamp_us < previous:
            sample['backwards_stamps'] += 1
        else:
            sample['stamp_advances'] += 1
    sample['last_backend_stamp_us'] = stamp_us


def _backend_config_file(override):
    if override:
        return override
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('uav_bringup')) / 'config' / 'uav.yaml'


def _status_payload(status):
    return {
        'name': status.name,
        'level': int.from_bytes(status.level, 'little')
        if isinstance(status.level, bytes) else int(status.level),
        'message': status.message,
        'hardware_id': status.hardware_id,
        'values': [{'key': value.key, 'value': value.value} for value in status.values],
    }


def _visual_summary(state, now_ros_ns):
    latest = state['latest'] or {}
    stamp_ns = latest.get('stamp_ns', 0)
    return {
        'messages': state['messages'],
        'latest': latest,
        'visual_updates_in_current_session': state['visual_updates'],
        'duplicate_visual_stamps': state['duplicate_stamps'],
        'backwards_visual_stamps': state['backwards_stamps'],
        'session_or_generation_changes': state['identity_changes'],
        'contract_errors': dict(state['contract_errors']),
        'max_visual_update_gap_s': round(max(state['update_gaps_s']), 4)
        if state['update_gaps_s'] else None,
        'visual_update_age_s': round((now_ros_ns - stamp_ns) / 1e9, 4)
        if stamp_ns > 0 else None,
    }


def _bridge_summary(state, diagnostic_sample):
    latest = state['latest'] or {}
    first = state['first_counters'] or {}
    last = state['latest_counters'] or {}
    deltas = {key: last[key] - first[key] for key in last} if first else {}
    return {
        'messages': state['messages'],
        'latest': latest,
        'counter_delta_during_observation': deltas,
        'counter_decreases': dict(state['counter_decreases']),
        'contract_errors': dict(state['contract_errors']),
        'diagnostic_age_s': diagnostic_sample.get('stamp_age_s'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--backend-file', type=Path,
                        help='central uav.yaml (default: installed uav_bringup config)')
    parser.add_argument('--require-output', action='store_true',
                        help='Require bridge publication and locally observed backend output')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 3:
        parser.error('--seconds must be finite and at least 3 (allow DDS discovery and 1 Hz diagnostics)')
    try:
        backend, fmu_prefix = parse_backend_config(
            _backend_config_file(args.backend_file).read_text(encoding='utf-8'))
        topics, state_topic, imu_topic, raw_imu_topic, output_topic = _backend_topics(
            backend, fmu_prefix)
    except (ImportError, OSError, ValueError) as error:
        parser.error(str(error))

    rclpy.init()
    node = rclpy.create_node('check_vio_readonly')
    buffer = None
    _listener = None
    if backend == 'mavros':
        from tf2_ros import Buffer, TransformListener

        buffer = Buffer()
        _listener = TransformListener(buffer, node)
    samples = {topic: new_sample() for topic in topics}
    visual = {
        'messages': 0, 'latest': None, 'identity': None, 'visual_updates': 0,
        'last_visual_stamp_ns': None, 'duplicate_stamps': 0, 'backwards_stamps': 0,
        'identity_changes': 0, 'contract_errors': Counter(), 'update_gaps_s': [],
    }
    bridge = {
        'messages': 0, 'latest': None, 'first_counters': None, 'latest_counters': None,
        'previous_counters': None, 'counter_decreases': Counter(),
        'contract_errors': Counter(),
    }
    odom_receive_by_stamp = {}
    observed_input_output_delays_s = []

    def receive(topic, message):
        receive_steady = time.monotonic()
        receive_ros_ns = node.get_clock().now().nanoseconds
        stamp_ns = None
        stamp_valid = True
        if hasattr(message, 'header'):
            stamp = message.header.stamp
            stamp_valid = stamp.sec >= 0 and 0 <= stamp.nanosec < 1000000000
            stamp_ns = stamp.sec * 1000000000 + stamp.nanosec if stamp_valid else -1
        sample = samples[topic]
        observe_sample(sample, receive_steady, receive_ros_ns, stamp_ns)

        if topic == state_topic and backend == 'mavros':
            connected = bool(message.connected)
            state_valid = bool(getattr(message, 'state_valid', True))
            sample['state'] = {
                'connected': connected,
                'state_valid': state_valid,
                'armed': message.armed,
                'mode': message.mode,
            }
            sample['disconnected_samples'] = sample.get('disconnected_samples', 0) + int(
                not connected or not state_valid)
        elif topic == state_topic:
            stamp_us = int(message.timestamp)
            sample['px4_status_timestamp_us'] = stamp_us
            _observe_backend_stamp(sample, stamp_us)
            if stamp_us > 0:
                sample['backend_stamp_age_s'] = (receive_ros_ns - stamp_us * 1000) / 1e9
        elif isinstance(message, Image):
            sample['image'] = [message.width, message.height, message.encoding]
            sample['frame_id'] = message.header.frame_id
        elif isinstance(message, Odometry):
            sample['position_m'] = [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ]
            sample['frame_id'] = message.header.frame_id
            sample['child_frame_id'] = message.child_frame_id
            if topic == '/ov_msckf/odomimu':
                if message.header.frame_id != 'global' or message.child_frame_id != 'imu':
                    sample['frame_mismatches'] = sample.get('frame_mismatches', 0) + 1
                if stamp_ns is not None and stamp_ns > 0:
                    odom_receive_by_stamp[stamp_ns] = receive_steady
                    if len(odom_receive_by_stamp) > 4096:
                        odom_receive_by_stamp.pop(next(iter(odom_receive_by_stamp)))
            elif topic == output_topic:
                if message.header.frame_id != 'vio_odom' or message.child_frame_id != 'vio_body':
                    sample['frame_mismatches'] = sample.get('frame_mismatches', 0) + 1
                input_received = odom_receive_by_stamp.pop(stamp_ns, None)
                if input_received is not None:
                    observed_input_output_delays_s.append(receive_steady - input_received)
        elif topic == output_topic:
            stamp_us = int(message.timestamp_sample)
            stamp_ns = stamp_us * 1000
            sample['position_m'] = [float(value) for value in message.position]
            sample['dds_pose_frame'] = int(message.pose_frame)
            sample['velocity_m_s'] = [float(value) for value in message.velocity]
            sample['dds_velocity_frame'] = int(message.velocity_frame)
            _observe_backend_stamp(sample, stamp_us)
            if stamp_us > 0:
                sample['backend_stamp_age_s'] = (receive_ros_ns - stamp_ns) / 1e9
                sample.setdefault('backend_stamp_ages_on_receive_s', []).append(
                    sample['backend_stamp_age_s'])
        elif topic == raw_imu_topic:
            sample['px4_timestamp_us'] = int(message.timestamp)
            _observe_backend_stamp(sample, message.timestamp)

        if topic == '/ov_msckf/visual_health':
            visual['messages'] += 1
            report = parse_visual_health(
                message.header.frame_id, stamp_ns or 0,
                [_status_payload(status) for status in message.status], stamp_valid)
            report['age_at_receive_s'] = ((receive_ros_ns - stamp_ns) / 1e9
                                          if stamp_ns is not None and stamp_ns > 0 else None)
            report['within_bridge_timestamp_limits'] = (
                report['visual_update'] and within_stamp_tolerance(stamp_ns, receive_ros_ns))
            if report['visual_update'] and not report['within_bridge_timestamp_limits']:
                report['visual_update'] = False
                report['reason'] = 'visual-health timestamp is stale or future-dated for bridge'
            visual['latest'] = report
            if not report['contract_valid']:
                visual['contract_errors'][report['reason']] += 1
                visual['identity'] = None
                visual['visual_updates'] = 0
                visual['last_visual_stamp_ns'] = None
                visual['update_gaps_s'] = []
                return
            identity = (report['session_id'], report['reset_counter'])
            if visual['identity'] != identity:
                if visual['identity'] is not None:
                    visual['identity_changes'] += 1
                visual['identity'] = identity
                visual['visual_updates'] = 0
                visual['last_visual_stamp_ns'] = None
                visual['update_gaps_s'] = []
            if not report['initialized'] or report['reset_counter'] == 0:
                visual['visual_updates'] = 0
                visual['last_visual_stamp_ns'] = None
                visual['update_gaps_s'] = []
                return
            if not report['fusion_healthy'] or not report['within_bridge_timestamp_limits']:
                visual['visual_updates'] = 0
                visual['last_visual_stamp_ns'] = None
                visual['update_gaps_s'] = []
                return
            if report['visual_update']:
                previous = visual['last_visual_stamp_ns']
                current = report['stamp_ns']
                if previous is None:
                    visual['visual_updates'] = 1
                elif current == previous:
                    visual['duplicate_stamps'] += 1
                elif current < previous:
                    visual['backwards_stamps'] += 1
                    visual['visual_updates'] = 0
                    visual['last_visual_stamp_ns'] = None
                    visual['update_gaps_s'] = []
                    return
                else:
                    visual['visual_updates'] += 1
                    visual['update_gaps_s'].append((current - previous) / 1e9)
                visual['last_visual_stamp_ns'] = current

        elif topic == '/uav_bridge/diagnostics':
            bridge['messages'] += 1
            report = parse_bridge_diagnostics(
                [_status_payload(status) for status in message.status])
            bridge['latest'] = report
            if not report['contract_valid']:
                bridge['contract_errors'][report['reason']] += 1
                return
            counters = {key: report[key] for key in (
                'received', 'adapted', 'published', 'rejected', 'rate_limited')}
            if bridge['first_counters'] is None:
                bridge['first_counters'] = counters
            previous = bridge['previous_counters']
            if previous:
                for key, value in counters.items():
                    if value < previous[key]:
                        bridge['counter_decreases'][key] += 1
            bridge['previous_counters'] = counters
            bridge['latest_counters'] = counters

    subscriptions = [node.create_subscription(
        message_type, topic, lambda message, topic=topic: receive(topic, message),
        qos_profile_sensor_data) for topic, message_type in topics.items()]
    started_steady = time.monotonic()
    deadline = started_steady + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        ended_steady = time.monotonic()
        now_ros_ns = node.get_clock().now().nanoseconds
        results = {}
        for topic, sample in samples.items():
            result = summarize_sample(sample, started_steady, ended_steady, now_ros_ns)
            result['publishers'] = node.count_publishers(topic)
            for key in (
                    'state', 'disconnected_samples', 'image', 'frame_id', 'child_frame_id',
                    'position_m', 'frame_mismatches', 'dds_pose_frame',
                    'dds_velocity_frame', 'velocity_m_s', 'last_backend_stamp_us',
                    'backend_stamp_age_s', 'px4_timestamp_us', 'px4_status_timestamp_us'):
                if key in sample:
                    result[key] = sample[key]
            if sample.get('backend_stamp_ages_on_receive_s'):
                ages = sample['backend_stamp_ages_on_receive_s']
                result['backend_stamp_age_on_receive_s'] = {
                    'min': round(min(ages), 4),
                    'max': round(max(ages), 4),
                }
            results[topic] = result

        transforms = {}
        if backend == 'mavros':
            transforms = {
                f'{parent}->{child}': bool(buffer.can_transform(parent, child, rclpy.time.Time()))
                for parent, child in (
                    ('vio_odom', 'vio_odom_ned'), ('vio_body', 'vio_body_frd'))
            }
        visual_report = _visual_summary(visual, now_ros_ns)
        bridge_report = _bridge_summary(bridge, results['/uav_bridge/diagnostics'])

        def recent(topic, max_gap_s, max_age_s):
            sample = results[topic]
            return (sample.get('count', 0) > 0 and
                    sample.get('last_receive_age_s', float('inf')) <= max_age_s and
                    sample.get('max_receive_gap_s', float('inf')) <= max_gap_s)

        if backend == 'mavros':
            state = results[state_topic].get('state', {})
            state_ok = (
                state.get('connected', False) and state.get('state_valid', False) and
                results[state_topic].get('disconnected_samples', 0) == 0 and
                recent(state_topic, 2.0, 2.0))
        else:
            px4_status = results[state_topic]
            status_age_s = (px4_status.get('backend_stamp_age_s', float('inf')) +
                            px4_status.get('trailing_gap_s', float('inf')))
            state_ok = (
                px4_status.get('count', 0) > 0 and px4_status.get('stamp_advances', 0) > 0 and
                px4_status.get('zero_stamps', 0) == 0 and
                px4_status.get('invalid_stamps', 0) == 0 and
                px4_status.get('duplicate_stamps', 0) == 0 and
                px4_status.get('backwards_stamps', 0) == 0 and
                -0.01 <= status_age_s <= 2.0 and recent(state_topic, 2.0, 2.0))
        inputs_ok = (
            state_ok and
            recent(imu_topic, 0.3, 0.3) and
            (raw_imu_topic is None or recent(raw_imu_topic, 0.3, 0.3)) and
            recent('/camera/camera/infra1/image_rect_raw', 0.5, 0.5) and
            recent('/camera/camera/infra2/image_rect_raw', 0.5, 0.5))

        visual_age_s = visual_report['visual_update_age_s']
        visual_ok = (
            visual_report['latest'].get('fusion_healthy', False) and
            visual_report['latest'].get('within_bridge_timestamp_limits', False) and
            visual_report['visual_updates_in_current_session'] > 1 and
            visual_age_s is not None and -0.01 <= visual_age_s <= 0.3 and
            recent('/ov_msckf/visual_health', 0.3, 0.3))

        odometry = results['/ov_msckf/odomimu']
        odometry_ok = (
            odometry.get('count', 0) > 0 and odometry.get('stamp_advances', 0) > 1 and
            odometry.get('duplicate_stamps', 0) == 0 and
            odometry.get('backwards_stamps', 0) == 0 and
            odometry.get('zero_stamps', 0) == 0 and
            odometry.get('invalid_stamps', 0) == 0 and
            odometry.get('frame_id') == 'global' and odometry.get('child_frame_id') == 'imu' and
            odometry.get('frame_mismatches', 0) == 0 and
            -0.01 <= odometry.get('stamp_age_s', float('inf')) <= 0.15 and
            recent('/ov_msckf/odomimu', 0.2, 0.2))
        transforms_ok = backend != 'mavros' or all(transforms.values())
        vio_ok = visual_ok and odometry_ok and transforms_ok

        output = results[output_topic]
        output_age_s = (output.get('stamp_age_s', float('inf')) if backend == 'mavros'
                        else output.get('backend_stamp_age_s', float('inf')) +
                        output.get('trailing_gap_s', float('inf')))
        output_ok = (
            output.get('count', 0) > 0 and
            output.get('stamp_advances', 0) > 1 and
            output.get('duplicate_stamps', 0) == 0 and
            output.get('backwards_stamps', 0) == 0 and
            output.get('zero_stamps', 0) == 0 and
            output.get('invalid_stamps', 0) == 0 and
            output.get('max_receive_gap_s', float('inf')) <= 0.2 and
            recent(output_topic, 0.2, 0.2) and
            -0.01 <= output_age_s <= 0.15 and
            (backend == 'mavros' and
                output.get('frame_id') == 'vio_odom' and
                output.get('child_frame_id') == 'vio_body' and
                output.get('frame_mismatches', 0) == 0 or
             backend == 'dds' and
                output.get('dds_pose_frame') == 2 and
                output.get('dds_velocity_frame') == 3 and
                len(output.get('position_m', [])) == 3 and
                all(math.isfinite(value) for value in output.get('position_m', [])) and
                len(output.get('velocity_m_s', [])) == 3 and
                all(math.isfinite(value) for value in output.get('velocity_m_s', []))))

        diag = bridge_report['latest']
        counter_delta = bridge_report['counter_delta_during_observation']
        diagnostic_sample = results['/uav_bridge/diagnostics']
        diagnostic_age = bridge_report['diagnostic_age_s']
        diagnostics_fresh = (
            diagnostic_age is not None and diag.get('contract_valid', False) and
            -0.01 <= diagnostic_age <= 2.0 and
            diagnostic_sample.get('zero_stamps', 0) == 0 and
            diagnostic_sample.get('invalid_stamps', 0) == 0 and
            diagnostic_sample.get('duplicate_stamps', 0) == 0 and
            diagnostic_sample.get('backwards_stamps', 0) == 0 and
            not bridge_report['counter_decreases'])
        bridge_ready = (
            diagnostics_fresh and diag.get('enabled', False) and
            diag.get('visual_fresh', False) and diag.get('input_fresh', False) and
            not diag.get('faulted', True) and not diag.get('paused', True) and
            diag.get('level') == 0 and counter_delta.get('published', 0) > 0 and output_ok)
        ready = inputs_ok and vio_ok and (bridge_ready if args.require_output else True)

        delay_values = observed_input_output_delays_s
        delay_summary = None
        if delay_values:
            ordered_delays = sorted(delay_values)
            delay_summary = {
                'samples': len(delay_values),
                'median_s': round(ordered_delays[len(ordered_delays) // 2], 5),
                'p95_s': round(ordered_delays[min(len(ordered_delays) - 1,
                                                  int(len(ordered_delays) * 0.95))], 5),
                'max_s': round(max(delay_values), 5),
                'meaning': 'checker receive-time delta from odomimu to matching bridge output stamp',
            }

        nodes = [f'{namespace.rstrip("/")}/{name}'
                 for name, namespace in node.get_node_names_and_namespaces()
                 if name != node.get_name()]
        print(json.dumps({
            'backend': backend,
            'backend_state_topic': state_topic,
            'backend_output_topic': output_topic,
            'observation_s': round(ended_steady - started_steady, 3),
            'nodes': sorted(nodes),
            'topics': results,
            'visual_health_v1': visual_report,
            'bridge_diagnostics_v1': bridge_report,
            'axis_tf': transforms,
            'observed_input_to_output_delay': delay_summary,
            'inputs_ok': inputs_ok,
            'visual_fusion_ok': visual_ok,
            'vio_ok': vio_ok,
            'bridge_output_observed': output_ok,
            'bridge_ready': bridge_ready,
            'checks_passed': ready,
            'bridge_published_is_not_px4_receipt': True,
            'px4_receipt_and_ekf_fusion': 'not checked by this script',
        }, indent=2))
        return 0 if ready else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
