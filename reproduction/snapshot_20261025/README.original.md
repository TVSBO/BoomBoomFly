# BoomBoomFly

2026-10-20：当前本地UAV工程已切换DDS，默认控制只读；适配情况与验证边界见[DDS链路适配](uav_ws/docs/DDS链路适配_20261020.md)。下文早期MAVROS状态保留为历史记录。

室内无 GPS 无人机科研原型，使用 ROS 2 Humble、PX4 和 MAVROS。
本仓库管理工作区说明与辅助脚本；自写包
[`uav_control`](https://github.com/BoomBoomFly/uav_control)、
[`uav_vio_bridge`](https://github.com/BoomBoomFly/uav_vio_bridge) 和
[`uav_bringup`](https://github.com/BoomBoomFly/uav_bringup)、
[`uav_mission`](https://github.com/BoomBoomFly/uav_mission) 分别独立管理。
共享接口包 `uav_interfaces` 位于 `uav_control/uav_interfaces/`，随控制仓库发布。

## 技术路线与接口隔离

- 定位链路：D435i 左右红外图像 + 飞控 IMU → OpenVINS → uav_vio_bridge → MAVROS / MAVLink
  → PX4 external vision。
- 控制链路：ROS 2 C++ 任务 → MAVROS setpoint → PX4 内部位置/速度控制器。

定位数据接入与任务控制保持独立。当前 `uav_mission` 与 `uav_control` 是独立
ROS 2 节点，通过最小 `uav_interfaces` 包的状态、目标和命令接口通信。
控制节点独占 setpoint 发布；当前只实现 MAVROS，DDS 仍未实现。

当前发送给 MAVROS 的位置使用本地 ENU，由 MAVROS 完成 ENU/NED 转换。
`uav_vio_bridge` 将 OpenVINS 的 `PoseWithCovarianceStamped` 原样转发到
MAVROS `/mavros/vision_pose/pose_cov`，不修改时间戳或坐标。
OpenVINS 接入前仍需检查实际 topic、frame、timestamp 和坐标约定。
EGO-Swarm、DDS 迁移和完整任务框架属于后续工作。

## 当前状态

以下环境与 SITL 状态来自已确认的开发记录，不代表每次文档更新都重新验证：

| 项目 | 状态 |
| --- | --- |
| 开发环境 | Ubuntu 22.04.5、ROS 2 Humble、Gazebo Harmonic（gz sim 8.14.0） |
| MAVROS | ros2 分支源码编译成功，包含 mavlink、mavros_msgs、libmavconn、mavros、mavros_extras；GeographicLib egm96-5 已安装 |
| PX4 | v1.17.0，关键 submodule 已初始化，`make px4_sitl gz_x500` 已实际启动成功 |
| uav_control / uav_mission | MAVROS 控制节点与任务状态机已实现；SITL 编译、接口和飞行结果见本次测试记录 |
| ROS 通信检查 | 隔离 ROS_DOMAIN_ID 下，统一状态、动态目标及 20 Hz MAVROS setpoint 路径通过 smoke 检查 |
| 飞行控制 | 第一版起飞、位置稳定 2 秒、悬停 10 秒、AUTO.LAND 和上锁确认已实现；SITL 结果见 [测试记录](https://github.com/BoomBoomFly/uav_ws/blob/main/sim/PX4_SITL_起飞悬停降落验证.md) |
| 定位 | 最小 `uav_vio_bridge` 已实现，独立编译及 run/launch 启动通过；消息转发测试超时，OpenVINS → PX4 链路尚未验证 |
| 启动编排 | `uav_bringup` 提供 VIO 与 SITL 测试 launch；VIO 消息链路仍未验证，飞行测试证据见下文 |
| 实机 | 飞控尚未到货，H743 串口、固件、MAVLink 参数和 IMU 均未验证 |

## 目录与 Git 归属

```text
BoomBoomFly/
├── Scripts/                     # 同步、推送和环境加载
├── uav_ws/
│   ├── src/
│   │   ├── uav_control/         # 独立 Git 仓库，内含 uav_interfaces ROS 包
│   │   ├── uav_vio_bridge/      # OpenVINS 位姿桥接，独立 Git 仓库
│   │   ├── uav_bringup/         # 启动与 YAML 配置编排，独立 Git 仓库
│   │   ├── uav_mission/         # 起飞、悬停和降落状态机，独立 Git 仓库
│   │   └── thirdparty/          # MAVROS、MAVLink、OpenVINS 等本地源码
│   ├── sim/                    # 自有 SITL 入口和验证记录
│   ├── upstream/PX4-Autopilot/  # PX4 源码
│   ├── build/
│   ├── install/
│   └── log/
├── references/                  # 本地参考仓库
└── docker/kalibr/source/        # 本地第三方源码
```

父仓库忽略三个独立工作区目录，各工作区独立管理说明及仿真文件。
各包源码在各自仓库中提交，不是 Git submodule；`uav_interfaces` 随
`uav_control` 仓库提交。克隆主仓库后可用同步脚本获取三个工作区和四个 UAV 自写包。
UAV 工作区的仓库边界与获取步骤见 [uav_ws README](https://github.com/BoomBoomFly/uav_ws#readme)。

## 同步与编译

```bash
cd /home/aa/BoomBoomFly
./Scripts/sync_ros_packages.sh
source Scripts/setup_ros_environment.bash
cd uav_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS="-j2" colcon build \
  --base-paths src/uav_control/uav_interfaces src \
  --packages-select uav_interfaces uav_control --symlink-install
source install/local_setup.bash
ros2 launch uav_control control.launch.py
```

以上针对当前已有依赖和 install 的开发环境。同步脚本依次克隆或快进更新
清单内的工作区和自写包；有分支时沿其 upstream，detached HEAD 时快进到 origin/main 并保持 detached。
历史分叉或文件冲突时停止。脚本不安装 ROS、不准备第三方、不编译 PX4。
新机器需要先准备上述依赖；环境脚本要求工作区已生成 `install/local_setup.bash`。

获取或更新自写包，然后仅编译 VIO 桥接包：

```bash
cd /home/aa/BoomBoomFly
./Scripts/sync_ros_packages.sh
source /opt/ros/humble/setup.bash
cd uav_ws
colcon build --base-paths src/uav_vio_bridge --packages-select uav_vio_bridge --symlink-install
source install/setup.bash
ros2 launch uav_vio_bridge uav_vio_bridge.launch.py
```

单独更新 Bridge 可执行 `./Scripts/sync_ros_packages.sh uav_vio_bridge`，兼容分支及 detached HEAD。
桥接包参数、验证命令与证据边界见其 [README](https://github.com/BoomBoomFly/uav_vio_bridge#readme)。
推送脚本支持 `uav_vio_bridge` 范围，见下方示例。

## 最小 VIO 启动编排

在 `uav_ws` 下编译并启动：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/uav_control/uav_interfaces src --symlink-install \
  --packages-select uav_interfaces uav_control uav_mission uav_vio_bridge uav_bringup
source install/setup.bash
ros2 launch uav_bringup uav.launch.py
```

也可单独执行 `ros2 launch uav_bringup vio.launch.py`；两个入口分别运行。
当前结构为 `uav.launch.py → vio.launch.py → uav_vio_bridge_node`。
参数集中在 `uav_bringup/config/uav_vio_bridge.yaml`，沿用桥接包现有话题。
尚未编排 OpenVINS、MAVROS、D435i 或 uav_control。
详细配置和验证边界见 [uav_bringup README](https://github.com/BoomBoomFly/uav_bringup#readme)。

## SITL 与 MAVROS

PX4 SITL 的版本、完整命令、参数、GCS 要求、正常测试和任务节点停止测试结果，记录在
[PX4 SITL 起飞—悬停—降落验证文档](https://github.com/BoomBoomFly/uav_ws/blob/main/sim/PX4_SITL_起飞悬停降落验证.md)。
2026-09-19 已使用 PX4 v1.17.0、Gazebo Sim 8.14.0 和 ROS 2 Humble 完成正常 SITL
PASS；没有连接实机。复现时先启动 PX4：

```bash
cd /home/aa/BoomBoomFly/uav_ws/upstream/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500
```

接着打开 QGroundControl，确认 PX4 有有效 GCS 心跳；再启动 MAVROS：

```bash
cd /home/aa/BoomBoomFly
source Scripts/setup_ros_environment.bash
ros2 launch mavros px4.launch \
  fcu_url:=udp://:14540@127.0.0.1:14580
```

该地址对应 PX4 Onboard MAVLink local 14580、remote 14540。之后运行
`ros2 launch uav_bringup sitl_flight.launch.py` 启动独立控制和任务节点；不要同时启动
其他 `/mavros/setpoint_position/local` 固定目标发布器。

## 提交与推送

```bash
# 十二个独立范围，按需执行；会暂存并提交所选范围内全部未忽略改动。
./Scripts/push_git.sh uav_control "更新控制接口"
./Scripts/push_git.sh uav_vio_bridge "更新位姿桥接"
./Scripts/push_git.sh uav_bringup "更新启动编排"
./Scripts/push_git.sh uav_mission "更新飞行任务"
./Scripts/push_git.sh uav_ws "更新 UAV 工作区及仿真"
./Scripts/push_git.sh swarm_ws "更新集群工作区"
./Scripts/push_git.sh ugv_ws "更新地面车工作区"
./Scripts/push_git.sh main "更新工作区脚本与文档"
```

执行前检查对应仓库的 `git status`。`main` 是主仓库范围名，不是固定分支名。
脚本不自动拉取或强制推送；推送失败会保留本地提交。
详细范围、保护规则与离线检查见 [Scripts/README.md](Scripts/README.md)。

三个工作区的仿真说明：[UAV](https://github.com/BoomBoomFly/uav_ws/blob/main/sim/README.md)、[UGV](https://github.com/BoomBoomFly/ugv_ws/blob/main/sim/README.md)、[集群](https://github.com/BoomBoomFly/swarm_ws/blob/main/sim/README.md)。

仓库目录与 repo 获取流程见 [manifest 说明](manifests/README.md)。UGV 四个功能包独立管理，ROS 包名保持不变；集群与足式机器人目录仍是预留说明。

UGV 独立包的拆分与发布记录见 [拆分计划](docs/多仓库拆分计划.md)。
