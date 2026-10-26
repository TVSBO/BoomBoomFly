#!/usr/bin/env python3
"""Record VIO, sensor, mission, and selected control-backend evidence topics."""

import argparse
from datetime import datetime
from pathlib import Path
import subprocess
import sys

from vio_tools import build_record_command, parse_backend_config


def installed_backend_file():
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('uav_bringup')) / 'config' / 'uav.yaml'


def local_px4_message_versions():
    from px4_msgs.msg import (
        SensorCombined, VehicleAttitude, VehicleCommandAck, VehicleLandDetected,
        VehicleCommand, VehicleLocalPosition, VehicleOdometry, VehicleStatus,
        TrajectorySetpoint,
    )

    messages = {
        'SensorCombined': SensorCombined,
        'VehicleAttitude': VehicleAttitude,
        'VehicleCommandAck': VehicleCommandAck,
        'VehicleCommand': VehicleCommand,
        'VehicleLandDetected': VehicleLandDetected,
        'VehicleLocalPosition': VehicleLocalPosition,
        'VehicleOdometry': VehicleOdometry,
        'VehicleStatus': VehicleStatus,
        'TrajectorySetpoint': TrajectorySetpoint,
    }
    return {
        name: getattr(message, 'MESSAGE_VERSION', 0)
        for name, message in messages.items()
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output',
        help='new bag directory (default: bags/vio_YYYYmmdd_HHMMSS)',
    )
    parser.add_argument(
        '--backend-file', type=Path,
        help='集中配置 uav.yaml（默认使用已安装的 uav_bringup 配置）',
    )
    args = parser.parse_args()
    output = Path(args.output or datetime.now().strftime('bags/vio_%Y%m%d_%H%M%S')).expanduser()
    if output.exists():
        parser.error(f'refusing to record over existing path: {output}')
    try:
        backend_file = args.backend_file or installed_backend_file()
        backend, configured_prefix = parse_backend_config(
            backend_file.read_text(encoding='utf-8'))
        versions = local_px4_message_versions() if backend == 'dds' else None
        output.parent.mkdir(parents=True, exist_ok=True)
        return subprocess.run(
            build_record_command(output, backend, configured_prefix, versions),
            check=False,
        ).returncode
    except KeyboardInterrupt:
        return 130
    except FileNotFoundError:
        print('A required ROS package, config file, or ros2 executable was not found.',
              file=sys.stderr)
        return 127
    except (ImportError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 127


if __name__ == '__main__':
    raise SystemExit(main())
