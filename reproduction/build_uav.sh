#!/usr/bin/env bash
# Build source only. This script does not start ROS nodes or touch flight hardware.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$root/uav_ws"
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --base-paths src/uav_control/uav_interfaces src \
  --packages-up-to uav_interfaces uav_control uav_mission uav_bridge uav_bringup \
    ov_msckf realsense2_camera realsense2_description \
  --executor sequential --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON \
    -DBUILD_MAVROS_INTERFACE=OFF -DBUILD_TESTING=ON "$@"
