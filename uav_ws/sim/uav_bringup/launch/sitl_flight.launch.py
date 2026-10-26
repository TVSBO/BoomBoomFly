from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = Path(get_package_share_directory('uav_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('config_file',
                              default_value=str(bringup / 'config/uav.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/uav.launch.py')),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'start_mission': 'true',
            }.items(),
        ),
    ])
