from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from vio_tools import dds_message_topic, parse_backend_config, parse_bridge_diagnostics
from vio_tools import record_topics


def _bridge_status(**updates):
    values = {
        'contract_version': '1',
        'backend': 'dds',
        'enabled': 'true',
        'faulted': 'false',
        'paused': 'false',
        'pause_reason': '',
        'visual_fresh': 'true',
        'visual_age_s': '0.02',
        'visual_stamp_ns': '1230000000',
        'input_fresh': 'true',
        'input_age_s': '0.01',
        'received': '10',
        'adapted': '9',
        'published': '8',
        'rejected': '1',
        'rate_limited': '2',
        'recovery_visual_count': '3',
        'recovery_odom_count': '3',
        'last_process_result': 'published',
        'last_rejection': '',
        'last_process_ms': '0.2',
        'max_process_ms': '0.4',
    }
    values.update(updates)
    return {
        'name': 'uav_bridge/odom_bridge',
        'level': 0,
        'message': 'bridge enabled and healthy',
        'values': [{'key': key, 'value': value} for key, value in values.items()],
    }


def test_bridge_diagnostics_require_real_visual_and_input_freshness():
    parsed = parse_bridge_diagnostics([_bridge_status(visual_fresh='false')])

    assert parsed['contract_valid']
    assert parsed['enabled'] and parsed['input_fresh']
    assert not parsed['visual_fresh']
    assert parsed['published'] == 8
    assert parsed['recovery_visual_count'] == 3

    paused = parse_bridge_diagnostics([
        _bridge_status(faulted='true', paused='true', pause_reason='visual timeout')
    ])
    assert paused['contract_valid'] and paused['paused']
    assert paused['pause_reason'] == 'visual timeout'


def test_bridge_diagnostics_reject_old_name_and_inconsistent_pause_state():
    old = _bridge_status()
    old['name'] = 'uav_vio_bridge/stream'
    assert not parse_bridge_diagnostics([old])['contract_valid']

    inconsistent = parse_bridge_diagnostics([
        _bridge_status(faulted='true', paused='false')
    ])
    assert not inconsistent['contract_valid']


def test_bridge_diagnostics_accept_future_tolerance_and_no_sample_sentinel():
    future = parse_bridge_diagnostics([
        _bridge_status(visual_age_s='-0.005', input_age_s='-0.01')
    ])
    assert future['contract_valid'] and future['visual_fresh'] and future['input_fresh']

    waiting = parse_bridge_diagnostics([
        _bridge_status(
            visual_fresh='false', visual_age_s='-1', visual_stamp_ns='0',
            input_fresh='false', input_age_s='-1',
        )
    ])
    assert waiting['contract_valid'] and not waiting['visual_fresh']

    stale_after_clock_rollback = parse_bridge_diagnostics([
        _bridge_status(visual_fresh='false', visual_age_s='-4.0')
    ])
    assert stale_after_clock_rollback['contract_valid']

    too_far_future_while_fresh = parse_bridge_diagnostics([
        _bridge_status(visual_age_s='-0.02')
    ])
    assert not too_far_future_while_fresh['contract_valid']

    negative_duration = parse_bridge_diagnostics([
        _bridge_status(last_process_ms='-0.1')
    ])
    assert not negative_duration['contract_valid']


def test_backend_config_and_recording_topics_follow_selected_backend():
    dds, prefix = parse_backend_config(
        'uav_control:\n  backend: dds\n  dds:\n    fmu_prefix: /px4/fmu\n'
        'uav_bridge:\n  backend: mavros\n')
    assert (dds, prefix) == ('dds', '/px4/fmu')
    common = {
        'SensorCombined': 0,
        'VehicleStatus': 1,
        'VehicleLocalPosition': 1,
        'VehicleAttitude': 0,
        'VehicleLandDetected': 0,
        'VehicleCommandAck': 0,
        'VehicleCommand': 0,
        'VehicleOdometry': 0,
        'TrajectorySetpoint': 1,
    }
    dds_topics = record_topics(dds, prefix, common)
    assert '/px4/fmu/in/vehicle_visual_odometry' in dds_topics
    assert '/mavros/odometry/out' not in dds_topics
    assert '/px4/fmu/out/sensor_combined' in dds_topics
    assert '/px4/fmu/out/vehicle_status_v1' in dds_topics
    assert '/px4/fmu/in/trajectory_setpoint_v1' in dds_topics
    assert '/px4/fmu/in/vehicle_command' in dds_topics
    assert '/px4/fmu/in/offboard_control_mode' in dds_topics
    assert '/px4/imu' in dds_topics
    assert '/uav_bridge/diagnostics' in dds_topics
    assert '/ov_msckf/visual_health' in dds_topics
    assert '/ov_msckf/odomimu' in dds_topics
    assert '/mavros/imu/data_raw' not in dds_topics
    assert '/mavros/setpoint_position/local' in record_topics('mavros')
    assert dds_message_topic('/px4/fmu', 'out/sensor_combined', 0) == \
        '/px4/fmu/out/sensor_combined'
    # A recorder subscribes to command/setpoint topics as evidence; it never publishes them.
    assert len(dds_topics) == len(set(dds_topics))

    mavros, prefix = parse_backend_config('uav_control:\n  backend: mavros\n')
    mavros_topics = record_topics(mavros, prefix)
    assert '/mavros/odometry/out' in mavros_topics
    assert '/mavros/imu/data_raw' in mavros_topics
    assert not any('/fmu/' in topic for topic in mavros_topics)

    try:
        parse_backend_config(
            'uav_control:\n  backend: dds\n  dds:\n    fmu_prefix: /px4/fmu/\n')
    except ValueError:
        pass
    else:
        raise AssertionError('trailing slash must be rejected for fmu_prefix')
