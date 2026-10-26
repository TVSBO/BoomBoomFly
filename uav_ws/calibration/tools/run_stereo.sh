#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /kalibr_ws/devel/setup.bash
cd /data
export MPLBACKEND=Agg
xvfb-run -a rosrun kalibr kalibr_calibrate_cameras \
  --models pinhole-radtan pinhole-radtan \
  --target /data/target.yaml \
  --bag /data/continuous_20_144_4hz.bag \
  --topics /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw \
  --approx-sync 0.001 --dont-show-report
