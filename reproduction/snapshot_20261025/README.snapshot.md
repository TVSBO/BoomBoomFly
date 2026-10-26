# BoomBoomFly 可复现源码快照

提取日期：2026-10-25。来自 Orange Pi 上的实际工作目录，包含尚未提交的本地源码修改。
本目录是一份独立的单仓库源码快照；提取过程只读取原工程，不修改或删除原工程文件。

## 目录

```text
boomboomfly/
├── Scripts/                 原工程辅助脚本（历史多仓库同步/推送用途）
├── docs/                    架构与工程说明
├── manifests/               原多仓库清单
├── uav_ws/
│   ├── src/
│   │   ├── uav_control/     含嵌套 uav_interfaces
│   │   ├── uav_mission/
│   │   ├── uav_bridge/
│   │   ├── uav_bringup/
│   │   └── thirdparty/      OpenVINS、PX4 msgs、MAVROS、RealSense 源码
│   ├── config/              当前 OpenVINS 标定与估计器配置
│   ├── calibration/         标定工具及 YAML 输入/结果配置
│   ├── docs/
│   └── sim/
├── ugv_ws/                  保留本地已有目录与说明
├── swarm_ws/                保留本地已有目录与说明
├── tools/                   DDS Agent 源码、PX4 修复脚本/补丁、参考代码
└── reproduction/            版本、依赖、提取记录及复现入口
```

保留自写 ROS 包与其 package.xml、CMakeLists.txt、测试、launch、配置和许可证。
保留第三方源码的许可证。未复制旧 .git 历史、.repo、build、install、log、缓存、虚拟环境、
录包、飞行日志、编译出的固件/二进制、工具链安装包以及上游 OpenVINS 示例数据集。
PX4 完整源码通过固定提交和本地补丁按需恢复，避免混入其大量工具和构建产物。
详细排除项见 `reproduction/excluded.json`；当前硬件的参数快照与 SD 启动文本见 `reproduction/hardware/`。
UGV 的 car/fpga 四个仓库在源目录中只有 Git 元数据、没有工作树文件；本包保留对应空目录，
不把缺失源码写成已完成模块。Git 本身不记录空目录，ZIP 包会保留目录结构。

## 环境和依赖

目标环境为 Ubuntu 22.04、ROS 2 Humble；当前设备为 aarch64 Orange Pi。
Windows 可用于保存和编辑源码；编译、运行请使用 Linux/相应 ROS 环境。
完整系统包版本清单见 `reproduction/environment.txt`，源码仓库的 URL、提交与本地修改状态见
`reproduction/repositories.lock.json`。这里的本地修改已包含在源码快照中，不需要再拉取远端覆盖。

新机器先安装 ROS 2 Humble 并配置其 apt 源，然后安装构建工具：

```bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions \
  python3-rosdep python3-pytest libceres-dev libeigen3-dev libboost-all-dev \
  libopencv-dev libopencv-contrib-dev libspdlog-dev libtinyxml2-dev \
  ros-humble-fastrtps ros-humble-fastcdr ros-humble-mavlink \
  ros-humble-librealsense2 ros-humble-rosbag2-py
# 仅在首次配置且 /etc/ros/rosdep/sources.list.d/20-default.list 不存在时执行：
# sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths uav_ws/src --ignore-src --rosdistro humble -y
```

RealSense wrapper 为 4.58.1，CMake 要求 SDK >= 2.58.0。本机 apt 清单中 SDK 为 2.58.4；
如需 Orange Pi 特定 USB/内核适配，可参考 `uav_ws/docs/OrangePi5Plus_OpenVINS接入配置.md`
中的手工 SDK 方案。依赖需要从系统包仓库获取，本包不包含 ROS 安装或预编译 SDK。

## 编译与启动检查

在本目录执行，脚本按低内存设备采用单任务构建：

```bash
bash reproduction/build_uav.sh
bash reproduction/build_agent.sh
source /opt/ros/humble/setup.bash
source uav_ws/install/local_setup.bash
ros2 launch uav_bringup uav.launch.py --show-args
```

`build_uav.sh` 构建 DDS 控制、任务、桥接、OpenVINS 与 RealSense，不启动硬件。
包清单仍声明 mavros_msgs，因此也会构建其消息包；MAVROS 控制后端的 C++ 实现默认关闭。
如需编译 MAVROS 后端，可执行 `bash reproduction/build_uav.sh -DBUILD_MAVROS_INTERFACE=ON`；
MAVROS 通信节点及 GeographicLib 数据的配置参考原包文档。
`build_agent.sh` 沿用本机 Agent 的构建选项，链接 ROS Humble 的 FastDDS/FastCDR 和系统 spdlog。

当前实际部署配置以 `uav_ws/src/uav_bringup/config/uav.yaml` 为准：DDS、
`read_only: false`、视觉桥接 `enabled: true`、任务类型 `grid_s`。
旧文档中“只读/MAVROS”的描述属于历史状态。提取保留该实际配置，没有自动改参数。
运行前按目标硬件核对串口、相机标定和配置；单纯检查 launch 参数不会启动飞行链路。
正常启动方法见 `uav_ws/src/uav_bringup/README.md`。源码包制作和完整飞行验收是不同工作。

## 可选：重建 PX4 同步修复固件

```bash
bash reproduction/restore_px4.sh
```

该脚本从官方获取固定提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，恢复源机已初始化的
子模块并应用 `reproduction/px4-local.patch`（当前 V3 改动）。已有 firmware 目录时退出，避免覆盖。
随后依据 `tools/px4_timesync_fix/build_environment.json` 下载 Arm GNU 13.2.Rel1 工具链，
核对其中 SHA256，解压到该文件记录的工具链目录名，并准备 Python 环境：

```bash
python3 -m venv --system-site-packages tools/px4_timesync_fix/venv
tools/px4_timesync_fix/venv/bin/pip install -r reproduction/px4-python-requirements.txt
bash tools/px4_timesync_fix/build_firmware.sh sync_v3
```

还需按 PX4 对 Ubuntu 的要求安装其编译依赖（如 ninja-build、gcc/g++、python3-dev）；
工具链下载 URL 针对 aarch64 主机，x86_64 机器应选择同版本对应架构的主机工具链。
原工程中的 prepare/test 脚本部分依赖已排除的实验日志；不把这些历史实验脚本视为开箱可用入口。
PX4 构建脚本会检查固定 Git 提交，因此请通过上述恢复入口获取 Git 工作树。
本次未下载依赖、重新编译固件或刷写飞控。

## 仓库和完整性

本目录为普通源码副本，不包含 Git 元数据；ZIP 包同样只包含源码和复现材料。
不要在这份单仓库快照直接运行原 `Scripts/sync_ros_packages.sh`、`push_git.sh` 或 repo sync；
这些脚本针对旧的多仓库管理方式。原入口文档保存在 `reproduction/README.original.md`。

`reproduction/source.sha256.json` 记录复制时源文件的哈希；本 README 及根/工作区 .gitignore
是导出副本中的有意调整，原忽略规则保存在 `reproduction/original_gitignore/`。
`reproduction/SHA256SUMS` 校验最终导出文件。版本清单和 SHA256 校验不代表已通过新机器全量构建。
本次检查范围及结果见 `reproduction/validation.json`。
