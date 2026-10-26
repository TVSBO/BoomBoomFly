"""Unified UAV launch entry point backed by config/uav.yaml."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def load_config(path):
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('集中配置必须为 YAML 映射')
    return config


def launch_setup(context):
    config = load_config(LaunchConfiguration('config_file').perform(context))

    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        return value(name) == 'true'

    def section(name):
        parameters = config.get(name)
        if not isinstance(parameters, dict):
            raise ValueError(f'集中配置缺少有效的 {name} 参数分区')
        return parameters

    # Backend is a single shared choice, including when the control node is off.
    control = section('uav_control')
    backend = control.get('backend')
    if backend not in ('mavros', 'dds') or not isinstance(control.get(backend), dict):
        raise ValueError('uav_control 必须选择 mavros 或 dds，并提供对应参数分区')
    fmu_prefix = None
    if backend == 'dds':
        fmu_prefix = control[backend].get('fmu_prefix', '/fmu')
        if not isinstance(fmu_prefix, str) or not fmu_prefix.startswith('/') or fmu_prefix.endswith('/'):
            raise ValueError('uav_control.dds.fmu_prefix 必须为绝对路径且不能以 / 结尾')

    actions = []
    if enabled('start_agent'):
        if backend != 'dds':
            raise ValueError('start_agent 只适用于 dds 后端')
        actions.append(ExecuteProcess(
            cmd=[value('agent_executable'), 'serial', '--dev', value('agent_device'),
                 '-b', value('agent_baud'), '-v', '4'], output='screen',
        ))
    if backend == 'dds' and enabled('start_openvins'):
        actions.append(Node(
            package='uav_bridge', executable='imu_bridge', name='imu_bridge', namespace='/',
            output='screen', parameters=[{'fmu_prefix': fmu_prefix}],
        ))

    if enabled('start_control'):
        actions.append(Node(
            package='uav_control', executable='uav_control_node', name='uav_control',
            parameters=[{**control[backend], 'backend': backend}], output='screen',
        ))

    # Starting a mission never changes the selected backend's read-only setting.
    if enabled('start_mission'):
        mission = dict(section('uav_mission'))
        profile = value('mission_profile')
        if profile != 'config':
            mission['mission_profile'] = profile
        if mission.get('mission_profile', 'hover') not in ('hover', 'square', 'grid_s'):
            raise ValueError('uav_mission.mission_profile 必须为 hover、square 或 grid_s')
        actions.append(Node(
            package='uav_mission', executable='mission_node', name='uav_mission',
            parameters=[mission], output='screen',
        ))

    if enabled('start_mavros'):
        mavros = Path(get_package_share_directory('mavros'))
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(mavros / 'launch/px4.launch')),
            launch_arguments={'fcu_url': value('fcu_url'), 'namespace': ''}.items(),
        )]))

    if enabled('start_camera'):
        actions.append(Node(
            package='realsense2_camera', executable='realsense2_camera_node',
            name='camera', namespace='camera', output='screen',
            parameters=[{
                'enable_infra1': True, 'enable_infra2': True,
                'enable_color': False, 'enable_depth': False,
                'enable_gyro': False, 'enable_accel': False,
                'depth_module.infra_profile': value('infra_profile'),
                **section('camera'),
            }],
        ))

    if enabled('start_openvins'):
        # OpenVINS parses imu0_rostopic from its external calibration YAML.
        # Its ROS2Visualizer declares topic_imu directly, so overriding that
        # parameter at launch would cause a duplicate declaration.
        imu_topic = '/mavros/imu/data_raw' if backend == 'mavros' else '/px4/imu'
        actions.append(Node(
            package='ov_msckf', executable='run_subscribe_msckf', namespace='ov_msckf',
            output='screen', respawn=enabled('vio_respawn'), respawn_delay=2.0,
            parameters=[{
                'config_path': value('openvins_config_file'),
                'verbosity': value('vio_verbosity'),
                'use_stereo': True, 'max_cameras': 2, 'save_total_state': False,
                'imu0_rostopic': imu_topic,
            }],
        ))

    if enabled('start_bridge'):
        bridge = dict(section('uav_bridge'))
        if 'backend' in bridge:
            raise ValueError('uav_control.backend 是唯一后端来源，不要在 uav_bridge 重复配置')
        bridge['backend'] = backend
        if backend == 'dds':
            bridge['fmu_prefix'] = fmu_prefix
        actions.append(Node(
            package='uav_bridge', executable='odom_bridge', name='odom_bridge',
            namespace='/', output='screen', parameters=[bridge],
        ))
    return actions


def generate_launch_description():
    bringup = Path(get_package_share_directory('uav_bringup'))
    arguments = [DeclareLaunchArgument(
        'config_file', default_value=str(bringup / 'config/uav.yaml'),
        description='集中部署配置文件',
    )]
    for name, default, description in (
        ('start_control', 'true', '启动控制节点；后端和只读状态由集中配置决定'),
        ('start_agent', 'false', '同时启动串口 DDS Agent；已有 Agent 时保持关闭'),
        ('start_mission', 'false', '启动自动任务；不修改控制节点的只读状态'),
        ('start_mavros', 'false', '启动 MAVROS 飞控通信'),
        ('start_camera', 'false', '启动 D435i 双目相机'),
        ('start_openvins', 'false', '启动 OpenVINS；DDS 后端会同时启动 imu_bridge'),
        ('start_bridge', 'false', '启动 uav_bridge 的 odom_bridge；回传仍由 uav_bridge.enabled 控制'),
        ('vio_respawn', 'true', 'OpenVINS 退出后等待两秒重启'),
    ):
        arguments.append(DeclareLaunchArgument(
            name, default_value=default, choices=['true', 'false'], description=description,
        ))
    arguments.extend([
        DeclareLaunchArgument('mission_profile', default_value='config',
                              choices=['config', 'hover', 'square', 'grid_s'],
                              description='本次任务类型；config 使用集中配置，square 为方形，grid_s 为九宫格 S 形往返'),
        DeclareLaunchArgument('agent_executable', default_value='MicroXRCEAgent',
                              description='Micro XRCE-DDS Agent 可执行文件或完整路径'),
        DeclareLaunchArgument('agent_device', default_value='/dev/ttyS6',
                              description='DDS Agent 主机串口'),
        DeclareLaunchArgument('agent_baud', default_value='921600',
                              description='DDS Agent 串口波特率'),
        DeclareLaunchArgument('fcu_url', default_value='serial:///dev/ttyS6:921600',
                              description='MAVROS 飞控连接地址'),
        DeclareLaunchArgument('infra_profile', default_value='848x480x30',
                              description='双目分辨率和帧率，须与标定匹配'),
        DeclareLaunchArgument(
            'openvins_config_file',
            default_value=str(bringup / 'config/openvins/d435i_px4/estimator_config.yaml'),
            description='OpenVINS 标定与估计器配置',
        ),
        DeclareLaunchArgument('vio_verbosity', default_value='WARNING',
                              description='OpenVINS 日志级别，诊断时可设为 INFO 或 DEBUG'),
        OpaqueFunction(function=launch_setup),
    ])
    return LaunchDescription(arguments)
