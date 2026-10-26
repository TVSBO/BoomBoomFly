"""Small, ROS-independent helpers shared by the VIO check and recorder."""

from math import isfinite


RECORD_TOPICS = (
    '/camera/camera/infra1/image_rect_raw',
    '/camera/camera/infra2/image_rect_raw',
    '/ov_msckf/visual_health',
    '/ov_msckf/odomimu',
    '/uav_bridge/diagnostics',
    '/uav_control/state',
    '/uav_control/target',
    '/uav_mission/diagnostics',
    '/tf',
    '/tf_static',
    '/rosout',
)

MAVROS_CONTROL_TOPICS = (
    '/mavros/state',
    '/mavros/extended_state',
    '/mavros/local_position/pose',
    '/mavros/local_position/velocity_local',
    '/mavros/odometry/out',
    '/mavros/setpoint_position/local',
)

DDS_RECORD_MESSAGES = (
    ('out/sensor_combined', 'SensorCombined'),
    ('out/vehicle_status', 'VehicleStatus'),
    ('out/vehicle_local_position', 'VehicleLocalPosition'),
    ('out/vehicle_attitude', 'VehicleAttitude'),
    ('out/vehicle_land_detected', 'VehicleLandDetected'),
    ('out/vehicle_command_ack', 'VehicleCommandAck'),
    ('in/trajectory_setpoint', 'TrajectorySetpoint'),
    ('in/vehicle_command', 'VehicleCommand'),
    ('in/vehicle_visual_odometry', 'VehicleOdometry'),
)

DDS_UNVERSIONED_TOPICS = ('out/timesync_status', 'in/offboard_control_mode')


def _uint64(value):
    if not value or not value.isascii() or not value.isdigit():
        return None
    result = 0
    for digit in value:
        result = result * 10 + ord(digit) - ord('0')
        if result > 18446744073709551615:
            return None
    return result


def within_stamp_tolerance(stamp_ns, now_ns, max_age_s=0.15, max_future_s=0.01):
    age_s = (now_ns - stamp_ns) / 1e9
    return stamp_ns > 0 and -max_future_s <= age_s <= max_age_s


def new_sample():
    return {
        'count': 0,
        'first_receive': None,
        'last_receive': None,
        'max_receive_gap_s': 0.0,
        'stamp_advances': 0,
        'duplicate_stamps': 0,
        'backwards_stamps': 0,
        'zero_stamps': 0,
        'invalid_stamps': 0,
        'last_stamp_ns': None,
        'stamp_gaps_s': [],
        'stamp_ages_on_receive_s': [],
    }


def observe_sample(sample, receive_steady, receive_ros_ns, stamp_ns=None):
    """Add one received message, keeping wall gaps separate from header stamps."""
    if sample['count'] == 0:
        sample['first_receive'] = receive_steady
    else:
        sample['max_receive_gap_s'] = max(
            sample['max_receive_gap_s'], receive_steady - sample['last_receive'])
    sample['count'] += 1
    sample['last_receive'] = receive_steady

    if stamp_ns is None:
        return
    stamp_ns = int(stamp_ns)
    if stamp_ns < 0:
        sample['invalid_stamps'] += 1
        return
    if stamp_ns == 0:
        sample['zero_stamps'] += 1
        return
    previous = sample['last_stamp_ns']
    if previous is not None:
        if stamp_ns == previous:
            sample['duplicate_stamps'] += 1
        elif stamp_ns < previous:
            sample['backwards_stamps'] += 1
        else:
            sample['stamp_advances'] += 1
            sample['stamp_gaps_s'].append((stamp_ns - previous) / 1e9)
    sample['last_stamp_ns'] = stamp_ns
    sample['stamp_ages_on_receive_s'].append((receive_ros_ns - stamp_ns) / 1e9)


def _percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    return ordered[min(len(ordered) - 1, int(len(ordered) * fraction))]


def summarize_sample(sample, started_steady, ended_steady, now_ros_ns):
    """Return JSON-friendly receive, timestamp, and end-of-window gap metrics."""
    result = {
        key: sample[key] for key in (
            'count', 'stamp_advances', 'duplicate_stamps', 'backwards_stamps',
            'zero_stamps', 'invalid_stamps')
    }
    first = sample['first_receive']
    last = sample['last_receive']
    if first is None:
        result.update({
            'leading_gap_s': round(ended_steady - started_steady, 4),
            'trailing_gap_s': round(ended_steady - started_steady, 4),
            'max_receive_gap_s': round(ended_steady - started_steady, 4),
        })
        return result

    leading = max(0.0, first - started_steady)
    trailing = max(0.0, ended_steady - last)
    result['first_receive_after_start_s'] = round(first - started_steady, 4)
    result['last_receive_age_s'] = round(trailing, 4)
    result['leading_gap_s'] = round(leading, 4)
    result['trailing_gap_s'] = round(trailing, 4)
    result['max_receive_gap_s'] = round(max(sample['max_receive_gap_s'], trailing), 4)
    duration = last - first
    if sample['count'] > 1 and duration > 0.0:
        result['receive_hz'] = round((sample['count'] - 1) / duration, 2)

    stamp_ns = sample['last_stamp_ns']
    if stamp_ns is not None:
        result['stamp_ns'] = stamp_ns
        result['stamp_age_s'] = round((now_ros_ns - stamp_ns) / 1e9, 4)
    stamp_ages = sample['stamp_ages_on_receive_s']
    if stamp_ages:
        result['stamp_age_on_receive_s'] = {
            'median': round(_percentile(stamp_ages, 0.5), 4),
            'p95': round(_percentile(stamp_ages, 0.95), 4),
            'max': round(max(stamp_ages), 4),
        }
    stamp_gaps = sample['stamp_gaps_s']
    if stamp_gaps:
        result['max_advancing_stamp_gap_s'] = round(max(stamp_gaps), 4)
    return result


def parse_visual_health(frame_id, stamp_ns, statuses, stamp_valid=True):
    """Validate the OpenVINS visual-health v1 contract and update eligibility."""
    result = {
        'contract_valid': False,
        'fusion_healthy': False,
        'visual_update': False,
        'reason': '',
        'frame_id': frame_id,
        'stamp_ns': int(stamp_ns),
    }
    if not stamp_valid:
        result['reason'] = 'invalid visual-health timestamp'
        return result
    if frame_id != 'global' or len(statuses) != 1:
        result['reason'] = 'invalid frame or status count'
        return result

    status = statuses[0]
    values = {}
    for entry in status.get('values', []):
        key = entry['key']
        if key in values:
            result['reason'] = f'duplicate visual-health key: {key}'
            return result
        values[key] = entry['value']

    session = status.get('hardware_id', '')
    version = values.get('contract_version', '')
    initialized = values.get('initialized', '')
    counter_text = values.get('reset_counter', '')
    generation = _uint64(counter_text)
    if (status.get('name') != 'openvins/visual_fusion' or not session or
            version != '1' or initialized not in ('true', 'false') or generation is None):
        result['reason'] = 'invalid visual-health v1 contract'
        return result
    level = int(status.get('level', 3))
    result.update({
        'contract_valid': True,
        'status_name': status['name'],
        'level': level,
        'status_message': status.get('message', ''),
        'session_id': session,
        'contract_version': version,
        'initialized': initialized == 'true',
        'reset_counter': generation,
    })
    if initialized != 'true':
        result['reason'] = 'waiting for OpenVINS initialization'
    elif generation == 0:
        result['reason'] = 'OpenVINS reset counter is zero'
    elif level != 0:
        result['reason'] = 'OpenVINS reports no accepted visual fusion'
    elif int(stamp_ns) <= 0:
        result['reason'] = 'visual-fusion timestamp is missing'
    else:
        result['fusion_healthy'] = True
        result['visual_update'] = True
        result['reason'] = 'accepted visual fusion'
    return result


def parse_bridge_diagnostics(statuses):
    """Validate the bridge's v1 odometry diagnostics and freshness fields."""
    result = {'contract_valid': False, 'reason': ''}
    if len(statuses) != 1:
        result['reason'] = 'invalid status count'
        return result
    status = statuses[0]
    values = {}
    for entry in status.get('values', []):
        key = entry['key']
        if key in values:
            result['reason'] = f'duplicate bridge-diagnostic key: {key}'
            return result
        values[key] = entry['value']

    required = (
        'contract_version', 'backend', 'enabled', 'faulted', 'paused', 'pause_reason',
        'visual_fresh', 'visual_age_s', 'visual_stamp_ns', 'input_fresh', 'input_age_s',
        'received', 'adapted', 'published', 'rejected', 'rate_limited',
        'recovery_visual_count', 'recovery_odom_count', 'last_process_result',
        'last_rejection', 'last_process_ms', 'max_process_ms')
    if status.get('name') != 'uav_bridge/odom_bridge' or any(
            key not in values for key in required):
        result['reason'] = 'invalid bridge-diagnostics v1 fields'
        return result
    bool_keys = ('enabled', 'faulted', 'paused', 'visual_fresh', 'input_fresh')
    if (values['contract_version'] != '1' or values['backend'] not in ('mavros', 'dds') or
            any(values[key] not in ('true', 'false') for key in bool_keys)):
        result['reason'] = 'invalid bridge-diagnostics v1 contract'
        return result
    if values['faulted'] != values['paused']:
        result['reason'] = 'bridge paused and faulted fields disagree'
        return result

    counters = (
        'received', 'adapted', 'published', 'rejected', 'rate_limited',
        'recovery_visual_count', 'recovery_odom_count')
    parsed_counters = {key: _uint64(values[key]) for key in counters}
    if any(value is None for value in parsed_counters.values()):
        result['reason'] = 'invalid bridge-diagnostics counter'
        return result
    try:
        numeric = {
            key: float(values[key])
            for key in ('visual_age_s', 'input_age_s', 'last_process_ms', 'max_process_ms')
        }
    except ValueError:
        result['reason'] = 'invalid bridge diagnostic duration'
        return result
    def valid_sample_age(key, fresh_key):
        age = numeric[key]
        return values[fresh_key] == 'false' or age >= -0.01

    if (not all(isfinite(value) for value in numeric.values()) or
            any(numeric[key] < 0.0 for key in ('last_process_ms', 'max_process_ms')) or
            not valid_sample_age('visual_age_s', 'visual_fresh') or
            not valid_sample_age('input_age_s', 'input_fresh')):
        result['reason'] = 'invalid bridge diagnostic duration'
        return result
    visual_stamp_ns = _uint64(values['visual_stamp_ns'])
    if visual_stamp_ns is None or (
            values['visual_fresh'] == 'true' and visual_stamp_ns == 0):
        result['reason'] = 'invalid bridge visual timestamp'
        return result

    result.update({
        'contract_valid': True,
        'name': status['name'],
        'level': int(status.get('level', 3)),
        'status_message': status.get('message', ''),
        **{key: values[key] == 'true' for key in bool_keys},
        'backend': values['backend'],
        'pause_reason': values['pause_reason'],
        'visual_age_s': numeric['visual_age_s'],
        'visual_stamp_ns': visual_stamp_ns,
        'input_age_s': numeric['input_age_s'],
        **parsed_counters,
        'last_process_result': values['last_process_result'],
        'last_rejection': values['last_rejection'],
        'last_process_ms': numeric['last_process_ms'],
        'max_process_ms': numeric['max_process_ms'],
    })
    return result


def parse_backend_yaml(text):
    """Read only the unique ``uav_control.backend`` choice."""
    in_uav_control = False
    control_sections = 0
    backends = []
    for raw_line in text.splitlines():
        line = raw_line.split('#', 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        key, separator, value = line.strip().partition(':')
        if not separator:
            continue
        if indent == 0:
            in_uav_control = key == 'uav_control'
            control_sections += int(in_uav_control)
        elif in_uav_control and indent == 2 and key == 'backend':
            backends.append(value.strip().strip("'\""))
    if control_sections != 1 or len(backends) != 1 or backends[0] not in ('dds', 'mavros'):
        raise ValueError("control configuration must select exactly one of 'dds' or 'mavros'")
    return backends[0]


def parse_backend_config(text):
    """Return the selected backend and its DDS prefix from the central config."""
    backend = parse_backend_yaml(text)
    in_uav_control = False
    in_dds = False
    prefixes = []
    for raw_line in text.splitlines():
        line = raw_line.split('#', 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        key, separator, value = line.strip().partition(':')
        if not separator:
            continue
        if indent == 0:
            in_uav_control = key == 'uav_control'
            in_dds = False
        elif in_uav_control and indent == 2:
            in_dds = key == 'dds'
        elif in_uav_control and in_dds and indent == 4 and key == 'fmu_prefix':
            prefixes.append(value.strip().strip("'\""))
    if backend != 'dds':
        return backend, '/fmu'
    if len(prefixes) > 1:
        raise ValueError('control configuration has multiple DDS fmu_prefix values')
    prefix = prefixes[0] if prefixes else '/fmu'
    if (not prefix.startswith('/') or prefix.endswith('/') or
            any(char.isspace() for char in prefix)):
        raise ValueError('DDS fmu_prefix must be absolute and must not end with /')
    return backend, prefix


def dds_record_topics(fmu_prefix, message_versions):
    """Build PX4 paths from local message versions and explicit unversioned topics."""
    prefix = str(fmu_prefix)
    if (not prefix.startswith('/') or prefix.endswith('/') or
            any(char.isspace() for char in prefix)):
        raise ValueError('fmu_prefix must be absolute and must not end with /')
    topics = [f'{prefix}/{path}' for path in DDS_UNVERSIONED_TOPICS]
    for path, message_name in DDS_RECORD_MESSAGES:
        if message_name not in message_versions:
            raise ValueError(f'missing local px4_msgs version for {message_name}')
        version = int(message_versions[message_name])
        if version < 0:
            raise ValueError(f'invalid local px4_msgs version for {message_name}')
        topics.append(dds_message_topic(prefix, path, version))
    return tuple(topics)


def dds_message_topic(fmu_prefix, topic_path, message_version=0):
    """Build a PX4 ROS topic path using that local message's version constant."""
    prefix = str(fmu_prefix)
    version = int(message_version)
    if (not prefix.startswith('/') or prefix.endswith('/') or
            any(char.isspace() for char in prefix)):
        raise ValueError('fmu_prefix must be absolute and must not end with /')
    if not topic_path or topic_path.startswith('/') or version < 0:
        raise ValueError('PX4 topic path must be relative and message version nonnegative')
    suffix = f'_v{version}' if version else ''
    return f'{prefix}/{topic_path}{suffix}'


def record_topics(backend, fmu_prefix='/fmu', message_versions=None):
    topics = list(RECORD_TOPICS)
    if backend == 'mavros':
        topics.append('/mavros/imu/data_raw')
        topics.extend(MAVROS_CONTROL_TOPICS)
    elif backend == 'dds':
        topics.append('/px4/imu')
        topics.extend(dds_record_topics(fmu_prefix, message_versions or {}))
    else:
        raise ValueError("backend must be 'dds' or 'mavros'")
    return tuple(dict.fromkeys(topics))


def build_record_command(output, backend, fmu_prefix='/fmu', message_versions=None, ros2='ros2'):
    return [ros2, 'bag', 'record', '-o', str(output), *record_topics(
        backend, fmu_prefix, message_versions)]
