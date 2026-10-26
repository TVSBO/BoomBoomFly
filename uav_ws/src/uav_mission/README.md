# uav_mission

> TVSBO 单仓库版：本目录源码已随主仓库克隆获取，无需再克隆独立功能仓库。

起飞、悬停和轨迹任务状态机。ROS 2 Humble / PX4 v1.17.0，2026-10-25 源码快照，2026-10-26 分类发布。

## 实际实现

任务与控制是独立 ROS 节点，通过 uav_interfaces 的状态、目标和命令接口通信。
不直接发布 PX4 setpoint，不处理相机或原始 VIO。

```text
WAIT_READY → WAIT_ARM（遥控器解锁）→ PREPARE_OFFBOARD → REQUEST_OFFBOARD
  → TAKEOFF → STABILIZE → HOVER
  → hover / square / grid_s → REQUEST_LAND → LANDING → FINISHED
```

当前任务不发送 ARM 解锁命令；遥控器解锁后自动推进。
支持悬停、四边形和九宫格 S 形往返；取消/故障走安全降落，人工接管后停止旧任务。
配置：起飞 1 m、稳定 2 s、悬停 10 s；九宫格格距 0.4 m、参考速度 0.15 m/s、往返 1 次。
上述参数约束参考轨迹，不是实机精度保证；当前默认任务类型 grid_s，launch 默认不启动任务。
子任务结果 0=PENDING、1=PASSED、2=FAILED；子任务通过不等于整个飞行验收通过。
测试源码见 test/test_flight_task.cpp，本次未运行 ROS/C++ 测试。

## 获取、编译与配置

请从 [主工程](https://github.com/TVSBO/BoomBoomFly) 获取完整工作区，使用其
Scripts/restore_dependencies.py 和 reproduction/build_uav.sh。
常规启动统一使用 ros2 launch uav_bringup uav.launch.py；backend 与参数来自 uav_bringup/config/uav.yaml。
定位标定在 [uav_ws](https://github.com/QU-of-boomboom/uav_ws)/config/openvins/d435i_px4。

## 来源与验证边界

原作者、基线和许可证说明见 SOURCE.md。本次分类发布不代表重新编译、通信、融合或飞行验收通过。
原包详细说明已保留为 docs/README.snapshot_20261025.md，其中历史默认值与测试记录按记录日期理解。
