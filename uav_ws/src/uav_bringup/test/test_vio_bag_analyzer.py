from math import isclose, sqrt
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from analyze_vio_bag import OdomAnalyzer, StateAnalyzer, Timeline, TopicStats
from analyze_vio_bag import VisualAnalyzer, _expected_type, _px4_status_candidates
from analyze_vio_bag import rotate_body_velocity_to_world, write_report


NS = 1_000_000_000


def _record(stamp_ns, x=0.0):
    return {
        'stamp_ns': stamp_ns,
        'position': (x, 0.0, 0.0),
        'orientation': (0.0, 0.0, 0.0, 1.0),
        'velocity': (0.0, 0.0, 0.0),
        'pose_covariance': [0.0] * 36,
        'twist_covariance': [0.0] * 36,
        'frame_id': 'global',
        'child_frame_id': 'imu',
    }


def _odom_analyzer(timeout=0.2):
    return OdomAnalyzer(
        {'input_timeout_s': timeout, 'visual_timeout_s': 0.3,
         'max_position_step_m': 1.0,
         'max_position_prediction_error_m': 0.12,
         'max_position_variance_m2': 0.16,
         'max_attitude_variance_rad2': 0.36,
         'max_velocity_variance_m2_s2': 1.44,
         'max_angular_velocity_variance_rad2_s2': 4.0},
        'global', 'imu', Timeline())


def test_body_velocity_is_rotated_from_child_to_world_frame():
    half = sqrt(0.5)
    rotated = rotate_body_velocity_to_world((1, 0, 0), (0, 0, half, half))

    assert isclose(rotated[0], 0.0, abs_tol=1e-9)
    assert isclose(rotated[1], 1.0, abs_tol=1e-9)
    assert isclose(rotated[2], 0.0, abs_tol=1e-9)


def test_source_timestamp_counts_distinguish_duplicate_and_backward_samples():
    analyzer = _odom_analyzer()
    for receive_ns, stamp_ns in enumerate((NS, NS, NS // 2, 2 * NS), start=1):
        analyzer.observe(_record(stamp_ns), receive_ns * NS)

    report = analyzer.finish(5 * NS, stationary=False)
    assert report['duplicate_source_stamps'] == 1
    assert report['backwards_source_stamps'] == 1
    assert report['source_stamp_hz'] is None  # A reset breaks the source clock interval.
    assert report['position'] is None


def test_bag_receive_rate_uses_n_minus_one_over_duration():
    stats = TopicStats('/ov_msckf/odomimu', 'nav_msgs/msg/Odometry', True)
    for stamp_ns in (0, NS, NS, 3 * NS):
        stats.observe(stamp_ns)

    summary = stats.summary()
    assert summary['bag_record_hz'] == 1.0  # (4 records - 1) / 3 seconds
    assert summary['duplicate_bag_record_times'] == 1


def test_static_drift_label_requires_explicit_stationary_selection():
    analyzer = _odom_analyzer()
    analyzer.observe(_record(NS, x=0.0), NS)
    analyzer.observe(_record(2 * NS, x=1.0), 2 * NS)

    moving_report = analyzer.finish(2 * NS, stationary=False)
    assert 'no stationary assumption' in moving_report['position']['label']

    stationary_report = analyzer.finish(2 * NS, stationary=True)
    assert 'static drift' in stationary_report['position']['label']


def test_every_long_odom_gap_and_trailing_gap_gets_a_timeline_event():
    analyzer = _odom_analyzer(timeout=0.2)
    for index in range(4):
        analyzer.observe(_record((index + 1) * NS), index * NS)
    analyzer.finish(4 * NS, stationary=False)

    events = analyzer.timeline.finish(0)['events']
    gaps = [event for event in events if event['event'] == 'vio_odom_supply_gap']
    assert [event['bag_time_s'] for event in gaps] == [0.2, 1.2, 2.2, 3.2]


def test_repeated_visual_health_stamp_does_not_renew_update_timeout():
    timeline = Timeline()
    analyzer = VisualAnalyzer(timeout_s=0.3, timeline=timeline)
    statuses = [{
        'name': 'openvins/visual_fusion', 'level': 0, 'message': 'ok',
        'hardware_id': 'session-a',
        'values': [
            {'key': 'contract_version', 'value': '1'},
            {'key': 'initialized', 'value': 'true'},
            {'key': 'reset_counter', 'value': '1'},
        ],
    }]
    analyzer.observe(NS, 'global', 10 * NS, statuses, True)
    analyzer.observe(2 * NS, 'global', 10 * NS, statuses, True)
    report = analyzer.finish(2 * NS)

    assert report['accepted_advancing_visual_stamps'] == 1
    assert report['duplicate_visual_stamps'] == 1
    assert any(event['event'] == 'visual_advancing_stamp_gap'
               for event in timeline.finish(0)['events'])


def test_flight_state_timeline_records_only_state_changes():
    timeline = Timeline()
    analyzer = StateAnalyzer('dds', timeline)
    analyzer.observe(NS, {'pre_flight_checks_pass': False, 'nav_state': 0,
                          'arming_state': 1, 'failsafe': False})
    analyzer.observe(2 * NS, {'pre_flight_checks_pass': False, 'nav_state': 0,
                             'arming_state': 1, 'failsafe': False})
    analyzer.observe(3 * NS, {'pre_flight_checks_pass': True, 'nav_state': 0,
                             'arming_state': 1, 'failsafe': False})

    events = timeline.finish(NS)['events']
    assert [event['event'] for event in events] == ['flight_state', 'flight_state']
    assert events[-1]['state']['pre_flight_checks_pass'] is True


def test_px4_status_resolution_uses_configured_prefix_and_bag_topic_version():
    topics = {
        '/px4/fmu/out/vehicle_status_v27': 'px4_msgs/msg/VehicleStatus',
        '/fmu/out/vehicle_status_v1': 'px4_msgs/msg/VehicleStatus',
    }

    assert _px4_status_candidates(topics, '/px4/fmu') == [
        ('/px4/fmu/out/vehicle_status_v27', 'px4_msgs/msg/VehicleStatus')]


def test_missing_topic_and_empty_odom_are_reported_without_ros():
    missing = []
    assert not _expected_type({}, '/ov_msckf/odomimu', 'nav_msgs/msg/Odometry', missing)
    assert missing[0]['reason'] == 'topic is absent from bag metadata'

    report = _odom_analyzer().finish(0, stationary=False)
    assert report['messages'] == 0
    assert report['position'] is None
    assert report['world_velocity_integration_residual_m_between_adjacent_raw_samples'][
        'count'] == 0


def test_report_writer_refuses_to_overwrite_existing_file(tmp_path):
    report_path = tmp_path / 'report.json'
    report_path.write_text('preserve me', encoding='utf-8')

    with pytest.raises(FileExistsError):
        write_report(report_path, {'ok': True})

    assert report_path.read_text(encoding='utf-8') == 'preserve me'


def test_world_velocity_predicts_world_displacement_and_rejects_invalid_attitude():
    analyzer = _odom_analyzer()
    half = sqrt(0.5)
    first = _record(NS)
    first.update(orientation=(0, 0, half, half), velocity=(1, 0, 0))
    second = dict(first, stamp_ns=2 * NS, position=(0, 1, 0))
    analyzer.observe(first, NS)
    analyzer.observe(second, 2 * NS)

    residuals = analyzer.finish(2 * NS, stationary=False)[
        'world_velocity_integration_residual_m_between_adjacent_raw_samples']
    assert residuals['count'] == 1 and residuals['max'] == 0.0
    assert rotate_body_velocity_to_world((1, 0, 0), (0, 0, 0, 2)) is None


def test_frame_mismatch_does_not_pollute_position_or_prediction_statistics():
    analyzer = _odom_analyzer()
    analyzer.observe(_record(NS), NS)
    wrong_frame = dict(_record(2 * NS, x=100), frame_id='unrelated_world')
    analyzer.observe(wrong_frame, 2 * NS)
    analyzer.observe(_record(3 * NS), 3 * NS)

    report = analyzer.finish(3 * NS, stationary=True)
    assert report['frame_mismatches'] == {'pose_frame': 1}
    assert report['position']['max_distance_from_first_m'] == 0.0
    assert report['position_step_m_between_adjacent_raw_samples']['count'] == 0
    assert report['world_velocity_integration_residual_m_between_adjacent_raw_samples'][
        'count'] == 0


def test_coordinate_reset_suppresses_cross_session_static_drift():
    analyzer = _odom_analyzer()
    analyzer.observe(_record(NS), NS)
    analyzer.break_coordinate_continuity('VIO session or initialization changed')
    analyzer.observe(_record(2 * NS, x=10), 2 * NS)

    report = analyzer.finish(2 * NS, stationary=True)
    assert report['position'] is None
    assert report['coordinate_discontinuities'] == {'VIO session or initialization changed': 1}
    assert report['world_velocity_integration_residual_m_between_adjacent_raw_samples'][
        'count'] == 0


def test_report_writer_creates_parents_and_rejects_nonfinite_json(tmp_path):
    report_path = tmp_path / 'new-directory' / 'report.json'
    write_report(report_path, {'value': None})
    assert report_path.read_text(encoding='utf-8').strip().startswith('{')
    invalid_path = tmp_path / 'invalid.json'
    with pytest.raises(ValueError):
        write_report(invalid_path, {'value': float('nan')})
    assert not invalid_path.exists()
