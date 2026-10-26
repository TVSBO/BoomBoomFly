# BoomBoomFly

室内无 GPS 无人机科研原型与工程验证项目，使用 ROS 2 Humble、PX4 v1.17.0、Micro XRCE-DDS 和 OpenVINS。
当前硬件基线为 Orange Pi（aarch64）、D435i 和 MICOAIR H743 V2。

这是 TVSBO 的单仓库源码版：一次克隆即可获得工作区、自研功能包及修改版 OpenVINS，
无需初始化 submodule、repo 或额外的功能包仓库。
源码来自 2026-10-25 Orange Pi 快照，包含当时未提交的修改；2026-10-26 从
[QU-of-boomboom 分类发布版](https://github.com/QU-of-boomboom/BoomBoomFly) 汇总。
各模块原作者、版权和许可证保持原样，来源见 SOURCE.md 与各模块的 SOURCE.md。

## 技术路线

- 定位：D435i 双目红外图像 + PX4 IMU → OpenVINS → uav_bridge → PX4 外部视觉。
- 控制：uav_mission → uav_interfaces → uav_control → DDS → PX4 内部位置控制器。
- 保留 MAVROS 可选后端；任务与控制独立为 ROS 节点，控制节点独占飞控 setpoint 发布。

## 模块分类

| 目录 | 职责 |
| --- | --- |
| [uav_ws](uav_ws) | 标定、估计器配置、仿真和工程验证文档 |
| [uav_control](uav_ws/src/uav_control) | DDS/MAVROS 后端，内含共享接口 uav_interfaces |
| [uav_bridge](uav_ws/src/uav_bridge) | PX4 IMU 和 OpenVINS Odometry 适配、视觉健康检查 |
| [uav_bringup](uav_ws/src/uav_bringup) | 统一 launch 入口及集中部署配置 |
| [uav_mission](uav_ws/src/uav_mission) | 遥控解锁、起飞、悬停、方形、九宫格 S 形与降落 |
| [open_vins](uav_ws/src/thirdparty/open_vins) | 保留工程修改的 OpenVINS，沿用上游许可证 |
| [tools/px4_timesync_fix](tools/px4_timesync_fix) | PX4 V3 时间同步补丁与历史交接 |
| [reproduction](reproduction) | 构建入口、设备历史记录、PX4 恢复和来源版本 |

以上全部是本仓库的普通目录，不包含嵌套 Git 仓库或 gitlink。
未修改的 px4_msgs、MAVROS、RealSense、DDS Agent 从固定上游提交恢复；
UGV、集群和外部参考工程不在本次 UAV 发布范围。与组织版的来源对应见
[organization-imports.lock.json](reproduction/organization-imports.lock.json)。

## 获取与编译

在 Ubuntu 22.04 / ROS 2 Humble 环境执行；Windows 可保存和编辑源码。

```bash
git clone https://github.com/TVSBO/BoomBoomFly.git
cd BoomBoomFly
python3 Scripts/restore_dependencies.py
```

先按 [环境说明](reproduction/README.md) 安装 ROS、colcon、rosdep 和系统依赖，然后：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths uav_ws/src --ignore-src --rosdistro humble -y
bash reproduction/build_uav.sh
bash reproduction/build_agent.sh
source Scripts/setup_ros_environment.bash
ros2 launch uav_bringup uav.launch.py --show-args
```

构建脚本默认关闭 MAVROS 控制 C++ 后端，可传 -DBUILD_MAVROS_INTERFACE=ON。
uav_interfaces 嵌套于 uav_control，构建脚本已显式包含其发现路径。
恢复依赖不会启动硬件；PX4 可选源码恢复见 bash reproduction/restore_px4.sh，不自动刷写。

## 当前部署配置

[uav.yaml](uav_ws/src/uav_bringup/config/uav.yaml) 当前选择 DDS，read_only=false，
视觉桥接 enabled=true，mission_profile=grid_s，require_vio_health=true；MAVROS 保持只读。
launch 默认只启动控制节点，任务、Agent、相机、OpenVINS 和桥接由各自参数选择。
任务程序等待遥控器解锁后推进 Offboard；实际运行步骤见 [uav_bringup](uav_ws/src/uav_bringup/README.md)。
源设备相机—IMU 标定需按目标硬件和安装位置核对；参考速度与任务容差不代表已达到实飞精度。

## 验证边界

当前包含 DDS 控制、视觉健康检查、局部视觉航向就绪、方形及九宫格任务源码，
以及历史 MAVROS SITL、V3 时间同步和地面视觉融合记录。
历史记录不能替代最新 DDS 修订的完整自主飞行验收。
本次只做单仓库整理、源码一致性和静态检查；未执行 ROS/PX4 全量编译、刷写或飞行测试。
历史文档中的原设备日志路径不是本仓库提供的文件。

## 更新与推送

```bash
git status
git add <修改的文件或目录>
git commit -m "说明本次修改"
git push origin main
```

所有自研模块随本仓库统一提交。恢复出的第三方工作树及 build/install/log、录包和固件二进制均被忽略。
组织多仓库版的脚本与清单只在 docs/multi_repository_reference 下作为历史参考保留，
本版无需运行 restore_release.py、sync_ros_packages.sh 或 repo sync。
