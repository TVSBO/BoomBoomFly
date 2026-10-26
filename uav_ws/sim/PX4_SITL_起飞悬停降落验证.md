# PX4 SITL 起飞—悬停—降落历史验证

记录日期：2026-09-19，MAVROS 路径、PC PX4/Gazebo SITL，未连接实机、OpenVINS 或相机。
更新说明：2026-10-15 精简重复接口及失效命令；以下结果只属于当时版本。
当前配置、构建和实际文件入口见 [sim README](README.md)，不使用本文历史参数路径启动当前树。

## 当时环境与参数


| 项目 | 本次环境 |
| --- | --- |
| 主机 | Ubuntu 22.04.5 |
| ROS 2 | Humble |
| PX4 | v1.17.0；SITL 模型 `gz_x500` |
| Gazebo Sim | 8.14.0（Harmonic） |
| MAVROS | ROS 2 `ros2` 分支，提交 `e4f39208aa87`，版本 2.15.1 |
| MAVLink SITL 连接 | PX4 Onboard UDP local 14580 / remote 14540；MAVROS `udp://:14540@127.0.0.1:14580` |
| GCS | QGroundControl；PX4 的 `mavlink status` 显示 `GCS heartbeat valid` 后才解锁 |

首次尝试时 PX4 明确拒绝解锁，理由是 `No connection to the GCS`。打开 QGroundControl 后，PX4 确认 GCS 心跳并报告 `Ready for takeoff!`；没有通过关闭健康检查来绕过该阻塞。PX4 SITL 的 Data Link Loss 默认启用，仿真要求连接 GCS、SDK 或其他 MAVLink 应用，见 [PX4 v1.17 SITL failsafe 文档](https://docs.px4.io/v1.17/en/simulation/failsafes)。

PX4 shell 中现场读取的控制通信失效参数如下，测试期间没有修改：

| 参数 | 实测值 | 含义与验证边界 |
| --- | --- | --- |
| `COM_OF_LOSS_T` | `1.0 s` | Offboard 控制流丢失后的等待时间 |
| `COM_OBL_RC_ACT` | `0`（Position） | v1.17 文档定义的 Offboard 丢失动作；实际是否进入该备用动作还与 RC 状态有关 |
| `NAV_DLL_ACT` | `2`（Return） | GCS/Data Link 丢失后的动作 |

PX4 v1.17 要求先连续发送超过 1 秒、频率高于 2 Hz 的 Offboard setpoint 才允许切入 Offboard；低于 2 Hz 后按 `COM_OF_LOSS_T` 和 `COM_OBL_RC_ACT` 执行失效动作。控制节点以 20 Hz 发布，并在准备阶段持续发送当前位置目标。参数解释见 [PX4 v1.17 Offboard 文档](https://docs.px4.io/v1.17/en/flight_modes/offboard)。本次停止的是任务节点，控制节点仍运行并主动请求 Land，因此 PX4 的“Offboard setpoint 完全丢失”备用动作只核对了配置，没有单独触发验证；`COM_OBL_RC_ACT=0` 也不能描述成自动降落。

## 当时任务和软件检查

任务相对起飞位置升高 1.0 m，水平/高度误差阈值为 0.20/0.15 m，连续稳定 2 s 后悬停 10 s，
请求 LAND 并确认 AUTO.LAND、落地和上锁。yaw 误差被记录，不作为悬停通过阈值。
控制发送周期 20 Hz，目标超时 1.0 s，LAND 交接窗口 3.0 s；参数未为获得通过而放宽。

接口、控制、任务和 bringup 曾在 `/tmp/boomboomfly_sitl_20260919` 构建；
隔离 ROS Domain 42 的旧 smoke 检查报告：`PASS: fresh unified state; no default target; dynamic ENU target at 20.27 Hz`。
随后真实 SITL 观察到目标/状态/服务与飞控模式往返，通信证据不只来自合成 smoke。
旧 smoke 文件、拆分参数 YAML 和独立 launch 已不属于当前入口；当前树不包含这些旧临时产物。

## 实际结果与日志


### 正常起飞—悬停—降落：PASS

- PX4 实际进入 `OFFBOARD`，实际解锁；任务从当前位置升高 1.0 m。
- 目标误差连续满足水平 `≤ 0.20 m`、垂直 `≤ 0.15 m` 至少 2 秒后进入悬停；随后悬停完整 10 秒。
- 正常降落阶段观察到 PX4 `AUTO.LAND`；飞控报告地面状态，MAVROS 接受上锁请求，最终状态 `armed=false`。
- 任务节点实际输出：`PASS: Offboard, arm, 1 m takeoff, stable hold, 10 s hover, AUTO.LAND handover, landed and disarmed confirmed`。
- 悬停期间最大水平误差 `0.044356 m`，最大高度误差 `0.042743 m`，最大航向误差 `0.001780 rad`；最终水平位移 `0.037461 m`，最终相对起飞高度 `-0.033986 m`。

### 悬停时停止任务节点：控制层安全降落通过

- 任务日志最后阶段为 `HOVER`，任务进程停止后没有发出正常任务 Land 命令。
- 控制节点在 1 秒目标超时后记录定位仍有效，请求 Land；MAVROS 接受，PX4 切换到 `AUTO.LAND`。
- PX4 控制台记录 `Landing detected` 和 `Disarmed by landing`。最终 ROS 遥测为 `state_valid=true`、`connected=true`、`position_valid=true`、`landed_state=LANDED_ON_GROUND`、`armed=false`；最终绝对 `map` 坐标约为 `(-0.084, -0.094, 0.014) m`。
- 这次子试验验证的是“任务节点停止、控制节点仍工作”的主动降落交接，不是 PX4 Offboard setpoint 全部丢失后的备用动作试验。

### 过程中发现并修正的失败

- 首次联通后本地位置订阅 QoS 不兼容，控制节点收不到 MAVROS best-effort 位置；改为传感器 QoS 后收到新鲜位置。
- PX4 因没有 GCS 心跳拒绝解锁。操作者打开 QGroundControl 后心跳有效，未更改或关闭该健康检查。
- 首次成功解锁时任务仍发布地面高度目标，PX4 未起飞；日志实测起飞目标误差约 1 m 并触发安全降落。修正为确认解锁后更新到起飞高度，并重新运行。
- 首次完整降落时控制节点把 AUTO.LAND 期间停止目标流误判为任务超时，造成最终上锁命令受安全锁拒绝。修正降落交接期间的目标超时判据和安全命令重试后，后续正常流程 PASS。

### 日志路径

- PX4 控制台：`uav_ws/sitl_logs/2026-09-19/px4.log`
- MAVROS 控制台：`uav_ws/sitl_logs/2026-09-19/mavros.log`
- 最终正常流程：`uav_ws/sitl_logs/2026-09-19/mission_normal_pass.log`
- 任务停止试验：`uav_ws/sitl_logs/2026-09-19/mission_stop_task.log`
- 对应控制节点日志：`uav_ws/sitl_logs/2026-09-19/mission_stop_control.log`
- PX4 ULog：`uav_ws/upstream/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-19/12_07_12.ulg`（正常流程）和 `12_08_44.ulg`（任务节点停止试验）。
- 独立临时构建输出：`/tmp/boomboomfly_sitl_20260919/log/`；原工作区的 `uav_ws/build`、`install`、`log` 未清理或覆盖。

## 验证范围

| 层级 | 结果 |
| --- | --- |
| 编译 | 通过：接口、控制、任务、bringup 首轮构建；最终控制和任务修改增量重建通过 |
| ROS 接口与通信 | 通过：隔离 smoke 20.27 Hz；SITL 中状态、目标、服务请求和飞控状态确认均有实际往返 |
| PX4 SITL | 通过：正常飞行 PASS；任务节点停止时控制层成功请求降落并最终着陆上锁 |
| 实机 | 未验证：没有连接真实飞控、飞机、相机或机载定位系统 |
| PX4 Offboard 丢流备用动作 | 参数已现场读取，行为未单独触发；任务停止试验由仍运行的控制节点主动发出 Land |
| VIO / OpenVINS / D435i | 未接入、未验证 |

历史产物路径保留作追溯，不保证当前磁盘上存在；当前版本要重新保存版本、配置、ROS 证据和 ULog。
