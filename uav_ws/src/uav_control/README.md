# uav_control

> TVSBO 单仓库版：本目录源码已随主仓库克隆获取，无需再克隆独立功能仓库。

DDS/MAVROS 飞控控制与统一状态接口。ROS 2 Humble / PX4 v1.17.0，2026-10-25 源码快照，2026-10-26 分类发布。

## 实际实现

统一可执行程序 uav_control_node，根据集中配置创建 DDS 或 MAVROS 后端。
控制节点独占飞控 setpoint；默认 20 Hz，处理状态新鲜度、Offboard、命令 ACK、人工接管和锁止。
本地视觉航向可通过 allow_local_vision_heading 配置按实际 EKF EV yaw 融合状态判断。

| ROS 接口 | 类型 | 用途 |
| --- | --- | --- |
| /uav_control/target | geometry_msgs/PoseStamped | 持续目标 |
| /uav_control/state | uav_interfaces/VehicleState | 统一状态 |
| /uav_control/command | uav_interfaces/FlightCommand | 飞控命令 |
| /uav_control/reset | std_srvs/Trigger | 地面重新就绪 |

uav_interfaces 随本仓库发布，位于 uav_interfaces/，需要显式构建发现。
当前 CMake 注册 test/test_dds_readiness.py；本次未运行 ROS 测试。
BUILD_MAVROS_INTERFACE 的 CMake 默认值为 ON，主工程构建脚本显式设为 OFF。

## 获取、编译与配置

请从 [主工程](https://github.com/TVSBO/BoomBoomFly) 获取完整工作区，使用其
Scripts/restore_dependencies.py 和 reproduction/build_uav.sh。
常规启动统一使用 ros2 launch uav_bringup uav.launch.py；backend 与参数来自 uav_bringup/config/uav.yaml。
定位标定在 [uav_ws](https://github.com/TVSBO/BoomBoomFly/tree/main/uav_ws)/config/openvins/d435i_px4。

## 来源与验证边界

原作者、基线和许可证说明见 SOURCE.md。本次分类发布不代表重新编译、通信、融合或飞行验收通过。
原包详细说明已保留为 docs/README.snapshot_20261025.md，其中历史默认值与测试记录按记录日期理解。
