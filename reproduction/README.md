# 复现材料

当前目录用于 TVSBO 单仓库发布版。自研包和 OpenVINS 已随克隆获取，
在主工程根目录执行 python3 Scripts/restore_dependencies.py 恢复其余上游依赖。

## 环境

Ubuntu 22.04、ROS 2 Humble，源机为 aarch64 Orange Pi。安装 ROS 并配置 apt 后：

```bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions \
  python3-rosdep python3-pytest libceres-dev libeigen3-dev libboost-all-dev \
  libopencv-dev libopencv-contrib-dev libspdlog-dev libtinyxml2-dev \
  ros-humble-fastrtps ros-humble-fastcdr ros-humble-mavlink \
  ros-humble-librealsense2 ros-humble-rosbag2-py
# 首次配置且 rosdep 未初始化时才执行 sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths uav_ws/src --ignore-src --rosdistro humble -y
bash reproduction/build_uav.sh
bash reproduction/build_agent.sh
```

RealSense wrapper 4.58.1 要求 SDK >=2.58.0，源机 SDK 为 2.58.4。
本仓库没有安装系统依赖；新机 apt 可用性和板端驱动仍需按硬件核对。

## PX4

restore_px4.sh 恢复官方固定提交 d6f12ad1c4f70ad3230afd7d86e971421e02fef4、
源机已初始化的子模块，并应用 px4-local.patch（V3）。不自动编译、刷写或改参数。
工具链版本见 tools/px4_timesync_fix/build_environment.json，aarch64 工具链 URL 不适用于 x86_64 主机。
需另行准备 ninja-build、gcc/g++、python3-dev、Python venv 与 px4-python-requirements.txt。
固件构建：bash tools/px4_timesync_fix/build_firmware.sh sync_v3。

## 历史与校验

repositories.lock.json、environment.txt、hardware/ 是源设备历史记录。
snapshot_20261025/ 下的 SHA256SUMS、validation.json 和原 README 仅对原导出快照有效，
不校验本次重写的说明和拆分仓库；个人版发布版本以本仓库 Git 提交为准，组织版来源见 organization-imports.lock.json。
原设备完整录包、ULog、编译固件和工具链没有随本次发布上传。
tools 内部分历史实验脚本依赖未导出的日志，不能作为开箱可用入口。
