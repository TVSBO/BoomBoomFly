#!/usr/bin/env python3
"""Read-only, bounded-memory analysis of a recorded VIO and flight-state bag."""

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import re
import sys

from vio_tools import RECORD_TOPICS, parse_backend_config
from vio_tools import parse_bridge_diagnostics, parse_visual_health


SAMPLE_LIMIT = 4096
EVENT_LIMIT = 5000
BRIDGE_DIAGNOSTICS_TOPIC = '/uav_bridge/diagnostics'


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _rounded(value, places=6):
    number = _finite(value)
    return round(number, places) if number is not None else None


def _vector(values, length=3):
    try:
        result = tuple(_finite(values[index]) for index in range(length))
    except (IndexError, TypeError):
        return None
    return result if all(value is not None for value in result) else None


def _distance(left, right):
    return math.dist(left, right)


def rotate_body_velocity_to_world(velocity, quaternion_xyzw):
    """Rotate an Odometry child-frame vector by its pose orientation."""
    vector = _vector(velocity)
    quaternion = _vector(quaternion_xyzw, 4)
    if vector is None or quaternion is None:
        return None
    x, y, z, w = quaternion
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    vx, vy, vz = vector
    # q * (v, 0) * conjugate(q), expanded to avoid another dependency.
    return _vector((
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy
        + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy
        + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy
        + (1 - 2 * (x * x + y * y)) * vz,
    ))


class BoundedValues:
    """Keep exact extrema and a bounded recent window for percentile estimates."""

    def __init__(self, limit=SAMPLE_LIMIT):
        self.values = deque(maxlen=limit)
        self.count = 0
        self.minimum = None
        self.maximum = None

    def add(self, value):
        number = _finite(value)
        if number is None:
            return False
        self.values.append(number)
        self.count += 1
        self.minimum = number if self.minimum is None else min(self.minimum, number)
        self.maximum = number if self.maximum is None else max(self.maximum, number)
        return True

    def summary(self):
        if not self.values:
            return {'count': self.count, 'sample_count': 0, 'min': None, 'max': None,
                    'p50': None, 'p95': None}
        ordered = sorted(self.values)

        def percentile(fraction):
            return ordered[min(len(ordered) - 1, int((len(ordered) - 1) * fraction))]

        return {
            'count': self.count,
            'sample_count': len(ordered),
            'percentiles_use': 'all samples' if self.count <= len(ordered)
            else f'latest {len(ordered)} samples',
            'min': _rounded(self.minimum),
            'max': _rounded(self.maximum),
            'p50': _rounded(percentile(0.50)),
            'p95': _rounded(percentile(0.95)),
        }


class Timeline:
    def __init__(self, limit=EVENT_LIMIT):
        self.limit = limit
        self.events = []
        self.dropped = 0

    def add(self, bag_time_ns, kind, **fields):
        if bag_time_ns is None:
            return
        if len(self.events) >= self.limit:
            self.dropped += 1
            return
        event = {'_bag_time_ns': int(bag_time_ns), 'event': kind}
        event.update(fields)
        self.events.append(event)

    def finish(self, bag_start_ns):
        events = []
        for item in sorted(self.events, key=lambda event: event['_bag_time_ns']):
            event = dict(item)
            timestamp = event.pop('_bag_time_ns')
            event['bag_time_s'] = _rounded((timestamp - bag_start_ns) / 1e9)
            event['bag_record_time_ns'] = timestamp
            events.append(event)
        return {'events': events, 'dropped_event_count': self.dropped,
                'event_limit': self.limit}


class TopicStats:
    def __init__(self, topic, msg_type, collect_intervals=False):
        self.topic = topic
        self.msg_type = msg_type
        self.count = 0
        self.first_ns = None
        self.last_ns = None
        self.previous_ns = None
        self.duplicate_times = 0
        self.backwards_times = 0
        self.intervals = BoundedValues() if collect_intervals else None

    def observe(self, timestamp_ns):
        timestamp_ns = int(timestamp_ns)
        self.count += 1
        if self.first_ns is None:
            self.first_ns = timestamp_ns
        if self.previous_ns is not None:
            delta = timestamp_ns - self.previous_ns
            if delta == 0:
                self.duplicate_times += 1
            elif delta < 0:
                self.backwards_times += 1
            elif self.intervals is not None:
                self.intervals.add(delta / 1e9)
        self.previous_ns = timestamp_ns
        self.last_ns = timestamp_ns

    def summary(self):
        duration = ((self.last_ns - self.first_ns) / 1e9
                    if self.first_ns is not None and self.last_ns > self.first_ns else None)
        result = {
            'type': self.msg_type,
            'count': self.count,
            'bag_record_duration_s': _rounded(duration),
            'bag_record_hz': _rounded((self.count - 1) / duration)
            if duration is not None and self.count > 1 else None,
            'duplicate_bag_record_times': self.duplicate_times,
            'backwards_bag_record_times': self.backwards_times,
        }
        if self.intervals is not None:
            result['bag_record_intervals_s'] = self.intervals.summary()
        return result


class OdomAnalyzer:
    COVARIANCE_DIAGONALS = (
        ('position_x', 0, 'max_position_variance_m2'),
        ('position_y', 7, 'max_position_variance_m2'),
        ('position_z', 14, 'max_position_variance_m2'),
        ('attitude_x', 21, 'max_attitude_variance_rad2'),
        ('attitude_y', 28, 'max_attitude_variance_rad2'),
        ('attitude_z', 35, 'max_attitude_variance_rad2'),
    )
    TWIST_DIAGONALS = (
        ('linear_x', 0, 'max_velocity_variance_m2_s2'),
        ('linear_y', 7, 'max_velocity_variance_m2_s2'),
        ('linear_z', 14, 'max_velocity_variance_m2_s2'),
        ('angular_x', 21, 'max_angular_velocity_variance_rad2_s2'),
        ('angular_y', 28, 'max_angular_velocity_variance_rad2_s2'),
        ('angular_z', 35, 'max_angular_velocity_variance_rad2_s2'),
    )

    def __init__(self, thresholds, world_frame, child_frame, timeline):
        self.thresholds = thresholds
        self.world_frame = world_frame
        self.child_frame = child_frame
        self.timeline = timeline
        self.count = 0
        self.valid_stamps = 0
        self.first_stamp_ns = None
        self.last_stamp_ns = None
        self.previous_stamp_ns = None
        self.duplicate_stamps = 0
        self.backwards_stamps = 0
        self.zero_stamps = 0
        self.invalid_stamps = 0
        self.age = BoundedValues()
        self.source_gaps = BoundedValues()
        self.receive_gaps = BoundedValues()
        self.last_receive_ns = None
        self.position_steps = BoundedValues()
        self.prediction_errors = BoundedValues()
        self.position_count = 0
        self.first_position = None
        self.last_position = None
        self.minimum_position = [None, None, None]
        self.maximum_position = [None, None, None]
        self.max_from_first = 0.0
        self.sum_squared_from_first = 0.0
        self.frame_mismatches = Counter()
        self.world_velocity_count = 0
        self.covariance_missing = Counter()
        self.covariance = {}
        self.previous_raw = None
        self.coordinate_discontinuities = Counter()

    def break_coordinate_continuity(self, reason):
        self.previous_raw = None
        self.coordinate_discontinuities[reason] += 1

    def _observe_covariance(self, label, values, diagonals):
        if values is None or len(values) < 36:
            self.covariance_missing[label] += 1
            return
        for axis, index, threshold_key in diagonals:
            state = self.covariance.setdefault(axis, {
                'sample_count': 0, 'invalid_count': 0, 'over_limit_count': 0,
                'max_diagonal': None,
            })
            value = _finite(values[index])
            if value is None or value < 0.0:
                state['invalid_count'] += 1
                continue
            state['sample_count'] += 1
            state['max_diagonal'] = value if state['max_diagonal'] is None else max(
                state['max_diagonal'], value)
            threshold = self.thresholds.get(threshold_key)
            if threshold is not None and value > threshold:
                state['over_limit_count'] += 1

    def observe(self, record, receive_ns):
        self.count += 1
        receive_ns = int(receive_ns)
        if self.last_receive_ns is not None:
            gap_s = (receive_ns - self.last_receive_ns) / 1e9
            if gap_s >= 0:
                self.receive_gaps.add(gap_s)
            timeout = self.thresholds.get('input_timeout_s')
            if timeout is not None and gap_s > timeout:
                self.timeline.add(
                    self.last_receive_ns + int(timeout * 1e9), 'vio_odom_supply_gap',
                    configured_input_timeout_s=timeout, observed_gap_s=_rounded(gap_s),
                )
        self.last_receive_ns = receive_ns

        stamp_ns = record.get('stamp_ns')
        if stamp_ns is None:
            self.invalid_stamps += 1
        elif stamp_ns <= 0:
            self.zero_stamps += 1
        else:
            self.valid_stamps += 1
            if self.first_stamp_ns is None:
                self.first_stamp_ns = stamp_ns
            self.last_stamp_ns = stamp_ns
            self.age.add((receive_ns - stamp_ns) / 1e9)
            if self.previous_stamp_ns is not None:
                if stamp_ns == self.previous_stamp_ns:
                    self.duplicate_stamps += 1
                elif stamp_ns < self.previous_stamp_ns:
                    self.backwards_stamps += 1
                    self.break_coordinate_continuity('source timestamp moved backwards')
                else:
                    delta_s = (stamp_ns - self.previous_stamp_ns) / 1e9
                    self.source_gaps.add(delta_s)
            self.previous_stamp_ns = stamp_ns

        position = _vector(record.get('position'))
        orientation = _vector(record.get('orientation'), 4)
        velocity_body = _vector(record.get('velocity'))
        position_frame = record.get('frame_id', '')
        child_frame = record.get('child_frame_id', '')
        if position_frame != self.world_frame:
            self.frame_mismatches['pose_frame'] += 1
            position = None
        if child_frame != self.child_frame:
            self.frame_mismatches['child_frame'] += 1

        velocity_world = None
        if (position_frame == self.world_frame and child_frame == self.child_frame and
                orientation is not None and velocity_body is not None):
            velocity_world = rotate_body_velocity_to_world(velocity_body, orientation)
            if velocity_world is not None:
                self.world_velocity_count += 1

        if position is not None:
            self.position_count += 1
            if self.first_position is None:
                self.first_position = position
            self.last_position = position
            for index, value in enumerate(position):
                low = self.minimum_position[index]
                high = self.maximum_position[index]
                self.minimum_position[index] = value if low is None else min(low, value)
                self.maximum_position[index] = value if high is None else max(high, value)
            distance = _distance(position, self.first_position)
            self.max_from_first = max(self.max_from_first, distance)
            self.sum_squared_from_first += distance * distance

        previous = self.previous_raw
        if previous is not None:
            if position is not None and previous['position'] is not None:
                step = _distance(position, previous['position'])
                self.position_steps.add(step)
                threshold = self.thresholds.get('max_position_step_m')
                if threshold is not None and step > threshold:
                    self.timeline.add(
                        receive_ns, 'vio_position_step_over_configured_limit',
                        step_m=_rounded(step), configured_limit_m=threshold,
                    )
            previous_stamp = previous['stamp_ns']
            if (stamp_ns is not None and previous_stamp is not None and
                    stamp_ns > previous_stamp and position is not None and
                    previous['position'] is not None and velocity_world is not None and
                    previous['velocity_world'] is not None):
                delta_s = (stamp_ns - previous_stamp) / 1e9
                predicted = tuple(
                    0.5 * (before + after) * delta_s
                    for before, after in zip(previous['velocity_world'], velocity_world)
                )
                actual = tuple(now - before for now, before in zip(position, previous['position']))
                residual = _distance(actual, predicted)
                self.prediction_errors.add(residual)
                threshold = self.thresholds.get('max_position_prediction_error_m')
                if threshold is not None and residual > threshold:
                    self.timeline.add(
                        receive_ns, 'vio_adjacent_sample_prediction_residual_over_limit',
                        residual_m=_rounded(residual), configured_limit_m=threshold,
                        source_interval_s=_rounded(delta_s),
                    )

        self._observe_covariance('pose', record.get('pose_covariance'), self.COVARIANCE_DIAGONALS)
        self._observe_covariance('twist', record.get('twist_covariance'), self.TWIST_DIAGONALS)
        self.previous_raw = {
            'stamp_ns': stamp_ns,
            'position': position,
            'velocity_world': velocity_world,
        }

    def finish(self, bag_end_ns, stationary):
        timeout = self.thresholds.get('input_timeout_s')
        if (timeout is not None and self.last_receive_ns is not None and
                bag_end_ns - self.last_receive_ns > timeout * 1e9):
            self.timeline.add(
                self.last_receive_ns + int(timeout * 1e9), 'vio_odom_supply_gap',
                configured_input_timeout_s=timeout,
                trailing_gap_s=_rounded((bag_end_ns - self.last_receive_ns) / 1e9),
            )
        duration = ((self.last_stamp_ns - self.first_stamp_ns) / 1e9
                    if self.first_stamp_ns is not None and self.last_stamp_ns is not None
                    and self.last_stamp_ns > self.first_stamp_ns
                    and not self.backwards_stamps else None)
        trajectory = None
        if self.position_count and not self.coordinate_discontinuities:
            trajectory = {
                'label': 'static drift under explicit user assumption' if stationary
                else 'trajectory displacement from first sample; no stationary assumption',
                'first_to_last_displacement_m': _rounded(_distance(
                    self.first_position, self.last_position)),
                'max_distance_from_first_m': _rounded(self.max_from_first),
                'rms_distance_from_first_m': _rounded(math.sqrt(
                    self.sum_squared_from_first / self.position_count)),
                'axis_min_m': [_rounded(value) for value in self.minimum_position],
                'axis_max_m': [_rounded(value) for value in self.maximum_position],
            }
        rate = ((self.valid_stamps - 1) / duration if duration is not None and
                self.valid_stamps > 1 else None)
        return {
            'messages': self.count,
            'valid_positive_source_stamps': self.valid_stamps,
            'source_stamp_hz': _rounded(rate),
            'source_stamp_duration_s': _rounded(duration),
            'source_rate_unavailable_reason': 'Source timestamps moved backwards.'
            if self.backwards_stamps else None,
            'duplicate_source_stamps': self.duplicate_stamps,
            'backwards_source_stamps': self.backwards_stamps,
            'zero_source_stamps': self.zero_stamps,
            'invalid_source_stamps': self.invalid_stamps,
            'source_stamp_gaps_s': self.source_gaps.summary(),
            'bag_record_minus_source_stamp_age_s': {
                **self.age.summary(),
                'meaning': ('bag record time minus message header stamp; meaningful only when '
                            'both clocks share a domain, and not an end-to-end latency'),
            },
            'bag_receive_intervals_s': self.receive_gaps.summary(),
            'position': trajectory,
            'coordinate_discontinuities': dict(self.coordinate_discontinuities),
            'position_unavailable_reason': (
                'Coordinate continuity was reset; whole-bag displacement and stationary '
                'drift cannot be compared across resets.'
                if self.coordinate_discontinuities else None),
            'position_step_m_between_adjacent_raw_samples': self.position_steps.summary(),
            'position_step_limit_m_from_config': self.thresholds.get('max_position_step_m'),
            'world_velocity_integration_residual_m_between_adjacent_raw_samples':
                self.prediction_errors.summary(),
            'prediction_error_limit_m_from_config': self.thresholds.get(
                'max_position_prediction_error_m'),
            'world_velocity_samples': self.world_velocity_count,
            'frame_mismatches': dict(self.frame_mismatches),
            'covariance_diagonal_diagnostics': {
                'axes': {
                    name: {**state, 'max_diagonal': _rounded(state['max_diagonal'])}
                    for name, state in sorted(self.covariance.items())
                },
                'missing_pose_or_twist_covariance_samples': dict(self.covariance_missing),
                'semantics': (
                    'Input diagonals are compared numerically with the configured transformed-'
                    'block limits. These are diagnostic comparisons, not a reproduction of the '
                    'full PSD, transformed eigenvalue, cross-covariance, or lever-arm gate.'),
                'configured_limits': {
                    key: self.thresholds.get(key) for key in (
                        'max_position_variance_m2', 'max_attitude_variance_rad2',
                        'max_velocity_variance_m2_s2',
                        'max_angular_velocity_variance_rad2_s2')
                },
            },
        }


class VisualAnalyzer:
    def __init__(self, timeout_s, timeline, expected_frame_id='global'):
        self.timeout_s = timeout_s
        self.timeline = timeline
        self.expected_frame_id = expected_frame_id
        self.messages = 0
        self.valid_contracts = 0
        self.contract_errors = Counter()
        self.identity = None
        self.previous_health = None
        self.last_stamp_ns = None
        self.last_update_receive_ns = None
        self.duplicate_stamps = 0
        self.backwards_stamps = 0
        self.identity_changes = 0
        self.accepted_updates = 0
        self.gap_reported = False
        self.latest = None

    def observe(self, receive_ns, frame_id, stamp_ns, statuses, stamp_valid):
        self.messages += 1
        report = parse_visual_health(frame_id, stamp_ns, statuses, stamp_valid)
        if report['contract_valid'] and frame_id != self.expected_frame_id:
            report.update(contract_valid=False, fusion_healthy=False, visual_update=False,
                          reason='visual-health frame does not match configured input frame')
        self.latest = report
        if not report['contract_valid']:
            self.contract_errors[report['reason']] += 1
        else:
            self.valid_contracts += 1

        identity = ((report.get('session_id'), report.get('reset_counter'))
                    if report['contract_valid'] else None)
        identity_changed = identity is not None and identity != self.identity
        if identity_changed:
            if self.identity is not None:
                self.identity_changes += 1
            self.timeline.add(
                receive_ns, 'visual_session_or_generation',
                previous=self.identity, current=identity,
            )
            self.identity = identity
            self.last_stamp_ns = None

        new_update = False
        if report['fusion_healthy'] and identity is not None:
            if self.last_stamp_ns is None or stamp_ns > self.last_stamp_ns:
                new_update = True
                self.accepted_updates += 1
                if (self.last_update_receive_ns is not None and self.timeout_s is not None and
                        receive_ns - self.last_update_receive_ns > self.timeout_s * 1e9):
                    self.timeline.add(
                        self.last_update_receive_ns + int(self.timeout_s * 1e9),
                        'visual_advancing_stamp_gap',
                        configured_visual_timeout_s=self.timeout_s,
                        observed_gap_s=_rounded(
                            (receive_ns - self.last_update_receive_ns) / 1e9),
                    )
                self.last_stamp_ns = stamp_ns
                self.last_update_receive_ns = receive_ns
                self.gap_reported = False
            elif stamp_ns == self.last_stamp_ns:
                self.duplicate_stamps += 1
            else:
                self.backwards_stamps += 1

        health = (report['contract_valid'], report['fusion_healthy'], report['reason'], identity)
        if health != self.previous_health:
            self.timeline.add(
                receive_ns, 'visual_health',
                contract_valid=report['contract_valid'],
                fusion_healthy=report['fusion_healthy'],
                reason=report['reason'], session_id=report.get('session_id'),
                reset_counter=report.get('reset_counter'), source_stamp_ns=stamp_ns,
                advancing_visual_stamp=new_update,
            )
            self.previous_health = health

    def finish(self, bag_end_ns):
        if (self.timeout_s is not None and self.last_update_receive_ns is not None and
                bag_end_ns - self.last_update_receive_ns > self.timeout_s * 1e9 and
                not self.gap_reported):
            self.timeline.add(
                self.last_update_receive_ns + int(self.timeout_s * 1e9),
                'visual_advancing_stamp_gap',
                configured_visual_timeout_s=self.timeout_s,
                trailing_gap_s=_rounded((bag_end_ns - self.last_update_receive_ns) / 1e9),
                note='Repeated health heartbeat with the same stamp does not renew this update.',
            )
            self.gap_reported = True
        return {
            'messages': self.messages,
            'valid_contract_messages': self.valid_contracts,
            'accepted_advancing_visual_stamps': self.accepted_updates,
            'duplicate_visual_stamps': self.duplicate_stamps,
            'backwards_visual_stamps': self.backwards_stamps,
            'session_or_generation_changes_after_first': self.identity_changes,
            'contract_errors': dict(self.contract_errors),
            'latest_contract_result': self.latest,
            'same_stamp_heartbeat_semantics':
                'A repeated visual-health stamp is counted as a duplicate and never renews '
                'the advancing-visual-stamp timeout.',
        }


class BridgeAnalyzer:
    def __init__(self, timeline):
        self.timeline = timeline
        self.messages = 0
        self.contract_errors = Counter()
        self.previous = None
        self.latest = None

    def observe(self, receive_ns, statuses):
        self.messages += 1
        report = parse_bridge_diagnostics(statuses)
        self.latest = report
        if not report['contract_valid']:
            if report['reason'] != self.previous:
                self.timeline.add(receive_ns, 'bridge_diagnostics_invalid', reason=report['reason'])
            self.contract_errors[report['reason']] += 1
            self.previous = report['reason']
            return
        fingerprint = tuple(report.get(key) for key in (
            'enabled', 'faulted', 'paused', 'pause_reason', 'last_rejection',
            'last_process_result'))
        if fingerprint != self.previous:
            self.timeline.add(
                receive_ns, 'bridge_state', enabled=report['enabled'],
                faulted=report['faulted'], paused=report['paused'],
                pause_reason=report['pause_reason'],
                last_rejection=report['last_rejection'],
                last_process_result=report['last_process_result'],
            )
            self.previous = fingerprint

    def summary(self):
        return {'messages': self.messages, 'contract_errors': dict(self.contract_errors),
                'latest_contract_result': self.latest}


class StateAnalyzer:
    def __init__(self, backend, timeline):
        self.backend = backend
        self.timeline = timeline
        self.messages = 0
        self.changes = 0
        self.latest = None

    def observe(self, receive_ns, state):
        self.messages += 1
        state = {key: value for key, value in state.items() if value is not None}
        if state != self.latest:
            self.changes += 1
            self.timeline.add(receive_ns, 'flight_state', backend=self.backend, state=state)
            self.latest = state

    def summary(self, topic, msg_type):
        return {'backend': self.backend, 'topic': topic, 'type': msg_type,
                'messages': self.messages, 'state_transitions': self.changes,
                'latest': self.latest}


def _status_payload(status):
    level = status.level
    return {
        'name': status.name,
        'level': int.from_bytes(level, 'little') if isinstance(level, bytes) else int(level),
        'message': status.message,
        'hardware_id': status.hardware_id,
        'values': [{'key': value.key, 'value': value.value} for value in status.values],
    }


def _stamp(message):
    stamp = message.header.stamp
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    valid = sec >= 0 and 0 <= nanosec < 1_000_000_000
    return (sec * 1_000_000_000 + nanosec if valid else None), valid


def _odom_record(message, stamp_ns):
    pose = message.pose.pose
    twist = message.twist.twist
    return {
        'stamp_ns': stamp_ns,
        'position': (pose.position.x, pose.position.y, pose.position.z),
        'orientation': (pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w),
        'velocity': (twist.linear.x, twist.linear.y, twist.linear.z),
        'pose_covariance': message.pose.covariance,
        'twist_covariance': message.twist.covariance,
        'frame_id': message.header.frame_id,
        'child_frame_id': message.child_frame_id,
    }


def _px4_state(message):
    names = ('pre_flight_checks_pass', 'nav_state', 'arming_state', 'failsafe')
    return {name: getattr(message, name, None) for name in names}


def _mavros_state(message):
    names = ('connected', 'armed', 'guided', 'mode', 'system_status')
    return {name: getattr(message, name, None) for name in names}


def _type_leaf(type_name):
    return str(type_name).replace('::', '/').split('/')[-1]


def _expected_type(topic_types, topic, expected, missing):
    actual = topic_types.get(topic)
    if actual is None:
        missing.append({'topic': topic, 'reason': 'topic is absent from bag metadata'})
        return False
    if actual != expected:
        missing.append({'topic': topic, 'reason': 'message type mismatch',
                        'expected_type': expected, 'actual_type': actual})
        return False
    return True


def _px4_status_candidates(topic_types, fmu_prefix):
    base = f'{fmu_prefix}/out/vehicle_status'
    pattern = re.compile(re.escape(base) + r'(?:_v[0-9]+)?$')
    return sorted((topic, type_name) for topic, type_name in topic_types.items()
                  if pattern.fullmatch(topic) and 'px4_msgs' in type_name and
                  _type_leaf(type_name) == 'VehicleStatus')


def load_config(path):
    import yaml

    text = path.read_text(encoding='utf-8')
    backend, fmu_prefix = parse_backend_config(text)
    document = yaml.safe_load(text)
    if not isinstance(document, dict):
        raise ValueError('backend config must contain a YAML mapping')
    bridge = document.get('uav_bridge')
    if not isinstance(bridge, dict):
        bridge = {}
    threshold_keys = (
        'input_timeout_s', 'visual_timeout_s', 'max_position_step_m',
        'max_position_prediction_error_m', 'max_position_variance_m2',
        'max_attitude_variance_rad2', 'max_velocity_variance_m2_s2',
        'max_angular_velocity_variance_rad2_s2', 'max_sample_age_s',
        'max_future_s', 'max_vio_speed_m_s',
    )
    thresholds = {}
    for key in threshold_keys:
        value = _finite(bridge.get(key))
        allow_zero = key in ('max_future_s', 'max_vio_speed_m_s')
        thresholds[key] = value if value is not None and (
            value >= 0.0 and (allow_zero or value > 0.0)
        ) else None
    return {
        'backend': backend,
        'fmu_prefix': fmu_prefix,
        'input_topic': bridge.get('input_topic', '/ov_msckf/odomimu'),
        'visual_health_topic': bridge.get('visual_health_topic', '/ov_msckf/visual_health'),
        'input_frame_id': bridge.get('input_frame_id', 'global'),
        'input_child_frame_id': bridge.get('input_child_frame_id', 'imu'),
        'thresholds': thresholds,
    }


def _installed_config():
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('uav_bringup')) / 'config' / 'uav.yaml'


def _message_class(type_name):
    from rosidl_runtime_py.utilities import get_message

    return get_message(type_name)


def analyze_bag(bag_path, config, stationary=False):
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    topic_types = {}
    with_reader = rosbag2_py.SequentialReader()
    with_reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=''),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'),
    )
    metadata = with_reader.get_all_topics_and_types()
    topic_types = {item.name: item.type for item in metadata}
    missing = []
    odom_topic = config['input_topic']
    health_topic = config['visual_health_topic']
    bridge_topic = BRIDGE_DIAGNOSTICS_TOPIC

    decode_types = {}
    for topic, expected in (
            (odom_topic, 'nav_msgs/msg/Odometry'),
            (health_topic, 'diagnostic_msgs/msg/DiagnosticArray'),
            (bridge_topic, 'diagnostic_msgs/msg/DiagnosticArray')):
        if _expected_type(topic_types, topic, expected, missing):
            decode_types[topic] = expected

    state_topic = None
    state_type = None
    if config['backend'] == 'mavros':
        state_topic, state_type = '/mavros/state', 'mavros_msgs/msg/State'
        if _expected_type(topic_types, state_topic, state_type, missing):
            decode_types[state_topic] = state_type
    else:
        candidates = _px4_status_candidates(topic_types, config['fmu_prefix'])
        if len(candidates) == 1:
            state_topic, state_type = candidates[0]
            decode_types[state_topic] = state_type
        elif not candidates:
            missing.append({
                'topic_pattern': f"{config['fmu_prefix']}/out/vehicle_status[_vN]",
                'reason': 'no PX4 VehicleStatus topic with its actual message type in bag metadata',
                'other_vehicle_status_topics': [
                    {'topic': topic, 'type': type_name}
                    for topic, type_name in topic_types.items()
                    if _type_leaf(type_name) == 'VehicleStatus'
                ],
            })
        else:
            missing.append({'reason': 'ambiguous PX4 VehicleStatus topics for configured prefix',
                            'candidates': [{'topic': topic, 'type': type_name}
                                           for topic, type_name in candidates]})

    classes = {}
    for topic, type_name in decode_types.items():
        try:
            classes[topic] = _message_class(type_name)
        except (ImportError, AttributeError, ModuleNotFoundError, ValueError) as error:
            missing.append({'topic': topic, 'type': type_name,
                            'reason': f'ROS message type is not importable: {error}'})

    timeline = Timeline()
    odom = OdomAnalyzer(config['thresholds'], config['input_frame_id'],
                        config['input_child_frame_id'], timeline)
    visual = VisualAnalyzer(config['thresholds'].get('visual_timeout_s'), timeline,
                            config['input_frame_id'])
    bridge = BridgeAnalyzer(timeline)
    state = StateAnalyzer(config['backend'], timeline)
    topic_stats = {}
    for topic, type_name in topic_types.items():
        topic_stats[topic] = TopicStats(
            topic, type_name,
            collect_intervals=(topic in decode_types or topic in RECORD_TOPICS or
                               type_name in ('sensor_msgs/msg/Imu',
                                             'px4_msgs/msg/SensorCombined')),
        )
    bag_start_ns = None
    bag_end_ns = None
    while with_reader.has_next():
        topic, raw_data, receive_ns = with_reader.read_next()
        receive_ns = int(receive_ns)
        bag_start_ns = receive_ns if bag_start_ns is None else min(bag_start_ns, receive_ns)
        bag_end_ns = receive_ns if bag_end_ns is None else max(bag_end_ns, receive_ns)
        if topic in topic_stats:
            topic_stats[topic].observe(receive_ns)
        message_class = classes.get(topic)
        if message_class is None:
            continue
        try:
            message = deserialize_message(raw_data, message_class)
        except (TypeError, ValueError, RuntimeError) as error:
            missing.append({'topic': topic, 'type': topic_types[topic],
                            'reason': f'message deserialization failed: {error}'})
            classes.pop(topic, None)
            continue

        if topic == odom_topic:
            stamp_ns, _ = _stamp(message)
            odom.observe(_odom_record(message, stamp_ns), receive_ns)
        elif topic == health_topic:
            stamp_ns, stamp_valid = _stamp(message)
            statuses = [_status_payload(status) for status in message.status]
            previous_identity = visual.identity
            visual.observe(receive_ns, message.header.frame_id, stamp_ns or 0,
                           statuses, stamp_valid)
            if previous_identity is not None and visual.identity != previous_identity:
                odom.break_coordinate_continuity('VIO session or initialization changed')
        elif topic == bridge_topic:
            statuses = [_status_payload(status) for status in message.status]
            bridge.observe(receive_ns, statuses)
        elif topic == state_topic:
            if config['backend'] == 'mavros':
                state.observe(receive_ns, _mavros_state(message))
            else:
                state.observe(receive_ns, _px4_state(message))

    if bag_start_ns is None:
        bag_start_ns = bag_end_ns = 0
        missing.append({'reason': 'bag contains no messages'})
    bag_end_ns = bag_end_ns if bag_end_ns is not None else bag_start_ns
    odom_summary = odom.finish(bag_end_ns, stationary)
    visual_summary = visual.finish(bag_end_ns)
    if not odom.count:
        missing.append({'topic': odom_topic, 'reason': 'topic has no odometry messages'})
    if not visual.messages:
        missing.append({'topic': health_topic, 'reason': 'topic has no visual-health messages'})
    if not bridge.messages:
        missing.append({'topic': bridge_topic, 'reason': 'topic has no bridge diagnostics'})
    if state_topic and not state.messages:
        missing.append({'topic': state_topic, 'reason': 'topic has no flight-state messages'})

    unavailable = []
    if not odom.count:
        unavailable.extend((
            'VIO source and receive rates', 'position steps and trajectory displacement',
            'world-velocity integration residuals', 'covariance diagonal diagnostics',
        ))
    if odom.prediction_errors.count == 0:
        unavailable.append('world-velocity integration residual distribution (no valid adjacent pairs)')
    if odom.coordinate_discontinuities:
        unavailable.append('whole-bag displacement/static drift across coordinate resets')
    if not state.messages:
        unavailable.append('flight-state timeline')
    if not visual.messages:
        unavailable.append('visual-health/session timeline')

    visual_summary['configured_input_frame_id'] = config['input_frame_id']
    visual_summary['frame_contract_note'] = (
        'The reused visual-health v1 parser accepts frame_id=global. A custom configured '
        'input_frame_id does not change that parser contract and will be reported invalid.'
    )
    report = {
        'report_version': 1,
        'bag': str(bag_path),
        'backend_from_config': config['backend'],
        'dds_fmu_prefix_from_config': config['fmu_prefix'],
        'stationary_assumption_selected_by_user': bool(stationary),
        'bag_record_time': {
            'start_ns': bag_start_ns, 'end_ns': bag_end_ns,
            'duration_s': _rounded((bag_end_ns - bag_start_ns) / 1e9),
            'meaning': 'rosbag2 record timestamps; used to align the joint timeline',
        },
        'timing_interpretation': (
            'Source header stamp and bag record time are reported separately. Their difference '
            'is meaningful only when their clocks share a domain and is not end-to-end latency.'
        ),
        'configuration_thresholds': config['thresholds'],
        'topics': {
            topic: topic_stats[topic].summary() for topic in sorted(topic_stats)
        },
        'selected_topics': {
            'vio_odometry': {'topic': odom_topic, 'type': topic_types.get(odom_topic)},
            'visual_health': {'topic': health_topic, 'type': topic_types.get(health_topic)},
            'bridge_diagnostics': {'topic': bridge_topic, 'type': topic_types.get(bridge_topic)},
            'flight_state': {'topic': state_topic, 'type': state_type},
            'px4_vehicle_status_candidates': [
                {'topic': topic, 'type': type_name}
                for topic, type_name in _px4_status_candidates(topic_types, config['fmu_prefix'])
            ],
        },
        'vio_odometry': odom_summary,
        'visual_health_v1': visual_summary,
        'bridge_diagnostics_v1': bridge.summary(),
        'flight_state': state.summary(state_topic, state_type),
        'joint_timeline': timeline.finish(bag_start_ns),
        'missing_topics_types_or_data': missing,
        'unavailable_metrics': sorted(set(unavailable)),
        'safety_boundary': (
            'Offline read-only bag analysis. It creates no ROS node or publisher, sends no flight '
            'commands, and cannot establish PX4 EKF2 external-vision fusion.'),
    }
    return report


def write_report(path, payload):
    text = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as output:
        output.write(text)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='rosbag2 directory or URI')
    parser.add_argument('--backend-file', type=Path,
                        help='central uav.yaml (default: installed uav_bringup config)')
    parser.add_argument('--output', type=Path, help='write JSON to a new file; never overwrite')
    parser.add_argument('--stationary', action='store_true',
                        help='explicitly interpret position displacement as static drift')
    args = parser.parse_args(argv)
    if args.output and args.output.exists():
        parser.error(f'refusing to overwrite existing report: {args.output}')
    try:
        config_path = args.backend_file or _installed_config()
        config = load_config(config_path.expanduser())
        report = analyze_bag(args.bag.expanduser(), config, stationary=args.stationary)
        if args.output:
            write_report(args.output, report)
        else:
            print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except FileExistsError as error:
        print(f'analyze_vio_bag.py: refusing to overwrite {error.filename}', file=sys.stderr)
        return 1
    except (ImportError, OSError, ValueError, RuntimeError) as error:
        print(f'analyze_vio_bag.py: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
