#!/bin/bash
set -e
export DEBIAN_FRONTEND=noninteractive
if ! python3 -c 'import wx, igraph, cairo' >/dev/null 2>&1; then
  apt-get update
  apt-get install -y --no-install-recommends python3-wxgtk4.0 python3-igraph python3-cairo xvfb xauth
fi
source /opt/ros/noetic/setup.bash
source /kalibr_ws/devel/setup.bash
export MPLBACKEND=Agg
# This Kalibr version catches argparse's successful SystemExit and returns 2
# for --help. Check its output rather than treating that as an import failure.
rosrun kalibr kalibr_calibrate_cameras --help > /data/camera_help.txt 2>&1 || test "$?" -eq 2
rosrun kalibr kalibr_calibrate_imu_camera --help > /data/imu_help.txt 2>&1 || test "$?" -eq 2
python3 - <<'PY'
from pathlib import Path
for file,flag in [('camera_help.txt','--models'),('imu_help.txt','--imu')]:
    text=(Path('/data')/file).read_text()
    assert flag in text and 'Traceback' not in text, file
PY
printf 'Kalibr CLI ready\n' | tee /data/environment_ready.txt
exec sleep infinity
