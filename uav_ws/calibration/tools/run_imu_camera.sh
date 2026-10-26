#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /kalibr_ws/devel/setup.bash
cd /data
export MPLBACKEND=Agg
# Use high-rate original images for temporal calibration, not the 4 Hz stereo bag.
xvfb-run -a rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/calibration_full.bag --bag-from-to 20 144 --bag-freq 20 \
  --cams /data/continuous_20_144_4hz-camchain.yaml \
  --imu /data/imu.yaml --imu-models calibrated \
  --target /data/target.yaml --max-iter 30 --dont-show-report --export-poses
