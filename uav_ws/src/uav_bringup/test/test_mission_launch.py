"""Check task selection through the unified launch without starting any nodes."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext


@pytest.fixture
def launch_module(monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'launch/uav.launch.py'
    spec = importlib.util.spec_from_file_location('uav_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    return module


def setup_launch(module, tmp_path, configured='hover', override='config', start=True):
    config = {
        'uav_control': {'backend': 'dds', 'dds': {'read_only': True}},
        'uav_mission': {'mission_profile': configured, 'square_side_m': 0.5},
    }
    path = tmp_path / 'uav.yaml'
    path.write_text(yaml.safe_dump(config))
    context = LaunchContext()
    context.launch_configurations.update({
        'config_file': str(path), 'mission_profile': override,
        'start_agent': 'false', 'start_control': 'true',
        'start_mission': 'true' if start else 'false',
        'start_mavros': 'false', 'start_camera': 'false',
        'start_openvins': 'false', 'start_bridge': 'false',
    })
    return module.launch_setup(context)


@pytest.mark.parametrize('profile', ['hover', 'square', 'grid_s'])
def test_configured_mission_and_control_read_only_are_preserved(launch_module, tmp_path, profile):
    control, mission = setup_launch(launch_module, tmp_path, configured=profile)
    assert control['parameters'] == [{'backend': 'dds', 'read_only': True}]
    assert mission['package'] == 'uav_mission'
    assert mission['parameters'] == [{'mission_profile': profile, 'square_side_m': 0.5}]


def test_square_override_only_selects_task(launch_module, tmp_path):
    control, mission = setup_launch(launch_module, tmp_path, override='square')
    assert control['parameters'][0]['read_only'] is True
    assert mission['parameters'][0]['mission_profile'] == 'square'
    assert mission['parameters'][0]['square_side_m'] == 0.5


def test_selecting_square_does_not_start_a_mission(launch_module, tmp_path):
    actions = setup_launch(launch_module, tmp_path, override='square', start=False)
    assert len(actions) == 1
    assert actions[0]['package'] == 'uav_control'


def test_invalid_configured_profile_fails_before_launch(launch_module, tmp_path):
    with pytest.raises(ValueError, match='mission_profile'):
        setup_launch(launch_module, tmp_path, configured='sqaure')


def test_grid_override_uses_existing_mission_node(launch_module, tmp_path):
    control, mission = setup_launch(launch_module, tmp_path, override='grid_s')
    assert control['parameters'][0]['read_only'] is True
    assert mission['executable'] == 'mission_node'
    assert mission['parameters'][0]['mission_profile'] == 'grid_s'
