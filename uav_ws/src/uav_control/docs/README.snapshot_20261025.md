# uav_control

更新日期：2026-10-15。`main.cpp` 构建唯一的 `uav_control_node`；控制包内部按 `backend`
创建 DDS 或 MAVROS 后端，任务通过 ROS 接口通信。当前修订未重新构建、运行或验收飞控通信。

## 配置与启动

[集中 uav.yaml](https://github.com/QU-of-boomboom/uav_bringup/blob/main/config/uav.yaml) 的 `uav_control.backend` 是唯一选择，取值 dds/mavros。
统一入口提取所选参数分区再传给节点；控制对象不再自行读取拆分 YAML。
完整集中配置不是 ROS 参数文件，不能整份传给节点 `--params-file`。

```bash
ros2 launch uav_bringup uav.launch.py
# 自定义集中配置，启动前核对其中的后端及只读开关
ros2 launch uav_bringup uav.launch.py config_file:=/absolute/path/to/uav.yaml
```

两个后端均默认 `read_only: true`，启动任务不会改变它。只读仍发布统一状态；显式允许控制需修改
所选后端分区并重启。后端同时决定 OpenVINS IMU 输入及桥接输出，见 [bringup](https://github.com/QU-of-boomboom/uav_bringup/blob/main/README.md)。
构建/overlay 命令集中在 [手动构建](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/OpenVINS%E6%89%8B%E5%8A%A8%E7%BC%96%E8%AF%91%E4%B8%8E%E9%AA%8C%E8%AF%81.md)。
`uav_interfaces` 嵌套在本包中，须显式发现并与控制/任务一起重建；MAVROS 后端需 mavros_msgs，
`BUILD_MAVROS_INTERFACE=OFF` 时只包含 DDS，默认 ON。

## ROS 接口

| 接口 | 语义 |
| --- | --- |
| `/uav_control/target`，PoseStamped | 本地 map/ENU 位置和 FLU yaw，时间戳新鲜、递增，目标持续更新 |
| `/uav_control/state`，VehicleState | 飞控/导航/落地有效性、ENU/FLU 位姿与世界速度、命令最终结果和锁止原因 |
| `/uav_control/command`，FlightCommand | SET_OFFBOARD、ARM、LAND、DISARM；dispatched 仅为本地派发 |
| `/uav_control/reset`，Trigger | DDS 地面重新就绪；要求新鲜地面/未解锁、有效定位、非 OFFBOARD、无 pending |

state_valid/connected 需要新鲜状态，DDS 还检查时间同步。position_valid、heading_valid、velocity_valid
分开判断；状态过期时 armed/mode 只是最后观测，不能作为当前状态。
control_inhibited/control_message 与定位有效性独立；恢复数据不会重启旧任务。
MAVROS 未提供的估计健康、预飞和 failsafe 字段保持未知，不伪造检查通过。

## 控制、时间与故障规则

- DDS 目标消息基线为本地 px4_msgs v1.17.0；实际固件须匹配。前缀可配置，版本后缀由 MESSAGE_VERSION 决定。
  要求 UXRCE_DDS_SYNCT=1、Agent/ROS 系统时钟一致；不支持 /clock，不重复加 Timesync offset。
- 统一坐标为 ENU/FLU；DDS 位置转 NED 为 `(y,x,-z)`，yaw 为 `wrap(pi/2-yaw)`，完整姿态同时转换世界/机体轴。
  MAVROS setpoint 由插件完成后续转换，不再手动二次变换。
- 默认 20 Hz，持续预发送新鲜目标后允许 Offboard，确认实际 OFFBOARD 后才允许 ARM。
  只使用位置/yaw控制，未使用的速度、加速度、jerk/yaw-rate 字段不伪造有效测量。
- 一次一个命令，统一 command_id 递增非零；DDS ACK 按 opcode、路由和时间窗关联。
  IN_PROGRESS 不完成或延长命令；最终 ACK 接受仍需确认实际模式和解锁状态。
- DDS ACK 超时后隔离 opcode，reset 不清除。ARM/DISARM 共用 opcode；重新使用需地面更换
  source_component 并重启，不能把未决取消报告为安全完成。
- MAVROS 等待服务与等待应答分别计时，默认各 3 s；服务就绪后重派发会重新计时。
  任务集中配置为 5 s，可能早于后端最长约 6 s 到期，仍需验证并统一等待契约。
  超时清理本地 pending 不撤回飞控命令；排队 ARM 在发送前重新检查目标、状态和模式。
- 目标超时且状态/定位仍有效时尝试交接 LAND；定位无效或断链时停止心跳。
  AUTO.LAND 后停止目标和心跳，地面 DISARM 需要新鲜落地确认。
- 实际 OFFBOARD 后遭人工接管、或本地参考重置时锁止；不抢回旧控制权、不恢复旧目标。
  DDS 地面 reset 后仍需新目标/任务，MAVROS 重新就绪使用重启。

控制停止后的实际动作由现场 PX4 参数决定，测试顺序见 [DDS 流程](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/DDS%E5%BC%80%E5%8F%91%E6%B5%81%E7%A8%8B.md)。

## 验证边界

当前 CMake 未注册包内测试，旧 backend_selection/dds_frames/dds_node/mavros_node 结果只属于历史版本。
[当前源码与证据](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/%E5%BD%93%E5%89%8Dsrc%E9%9D%99%E6%80%81%E5%AE%A1%E6%9F%A5.md)统一维护限制；合成状态、历史构建、ROS 通信、
SITL、PX4 接收和实机飞行分别下结论。
