# 当前源码与验证边界

更新日期：2026-10-15。本文合并原代码审查和桥接迁移说明，只维护当前契约、限制及证据入口。
本次文档整理未构建、未运行测试、未启动 ROS/SITL、未操作硬件，未进行运行验证。

## 当前结构

| 位置 | 职责 |
| --- | --- |
| `src/uav_control`，含嵌套 `uav_interfaces` | 唯一选中的 DDS/MAVROS 控制后端；目标、状态、命令和地面 reset |
| `src/uav_mission` | 本地起飞、稳定、悬停、降落及取消状态机 |
| `src/uav_bridge` | DDS IMU 转换、OpenVINS 里程计门控与单后端输出 |
| `src/uav_bringup` | 集中 `uav.yaml`、统一 launch、只读检查、录制与离线分析 |
| `src/thirdparty/open_vins`、`px4_msgs` | 独立第三方仓库；本地消息基线为 px4_msgs v1.17.0 |
| `config/openvins/d435i_px4`、`sim`、`docs` | 工作区拥有的标定/估计配置、仿真入口及说明 |

四个自研包与第三方源码分别管理。`git status`、HEAD 和工作树共同确定实际版本，不能只用父工作区 SHA
代表部署内容。远程项目名 `uav_vio_bridge` 与本地包名 `uav_bridge` 的差异只影响仓库同步参数。
当前父仓库 `.gitignore` 的桥接规则仍为 `/src/uav_vio_bridge/`，未覆盖新路径；本次只整理文档，
没有修改忽略规则。未跟踪 `src/` 不应被误认为需要统一导入源码。

## 当前共享契约

- 后端唯一来源是 [uav.yaml](https://github.com/QU-of-boomboom/uav_bringup/blob/main/config/uav.yaml) 的 `uav_control.backend`。
  DDS/MAVROS 选择同时作用于控制、OpenVINS IMU 输入和桥接输出；运行中切换需重启。
- DDS：`${fmu_prefix}/out/sensor_combined` → `imu_bridge` → `/px4/imu`；MAVROS：直接使用
  `/mavros/imu/data_raw`。DDS 输入 FRD→FLU 只转换一次。时间同步要求、有效性和滤波见
  [桥接接口](https://github.com/QU-of-boomboom/uav_vio_bridge/blob/main/README.md)。
- OpenVINS：`/ov_msckf/odomimu` 的位置在 `global` 世界系，twist 在 `imu` 系。
  预测位置增量前需用姿态把机体系速度旋转到世界系；外参、杆臂及协方差须按同一契约处理。
- MAVROS 输出为 `/mavros/odometry/out`；DDS 输出为 `${fmu_prefix}/in/vehicle_visual_odometry`，
  局部任意航向使用 FRD，速度使用 BODY_FRD。完整坐标、时间和方差语义以桥接 README 为准。
- `/ov_msckf/visual_health` 使用 `openvins/visual_fusion` v1：session、初始化代数和真实视觉融合采样时间。
  初始化、纯 IMU/ZUPT、状态发布和重复 stamp 均不能代替新的视觉更新。
- `/uav_bridge/diagnostics` 的唯一状态名是 `uav_bridge/odom_bridge`，v1。
  `published` 只计 publisher 调用；`last_process_ms/max_process_ms` 只覆盖 adapter，均不证明飞控接收或端到端性能。
- 任务仅发送 `/uav_control/target` 和 `/uav_control/command`，消费 `/uav_control/state`。
  派发、最终 ACK 与实际 OFFBOARD/armed/AUTO.LAND/landed 状态分别确认。
- `position_valid`、`heading_valid`、`velocity_valid` 分别表示数据有效性；DDS 的估计健康只是所实现字段的检查，
  不是完整 EKF 摘要。MAVROS 未提供的健康/预飞/failsafe 字段按未知处理。
- `require_vio_health` 默认 false；室内定位任务可在集中配置副本的 `uav_mission` 分区显式设 true。
  它检查桥接诊断、开关及真实供数新鲜度；VIO 转发恢复不会重启旧任务。

## 已写入的改动与保留限制

| 范围 | 当前源码行为 | 尚需验证 |
| --- | --- | --- |
| 预测门控 | 异常样本不推进比较基线；连续残差超限暂停，所有原始输入先检查再限频 | 单帧尖峰、持续偏移、慢速输入与超时恢复 |
| 可选速度门控 | `max_vio_speed_m_s=0.0` 默认禁用；有限非负，启用后超限复用暂停流程 | 用板端原始样本定标，不能照搬参考值 |
| 恢复 | 3 个独立新视觉样本及 3 个有效递增 Odometry；重复样本不续期、不计数 | 遮挡、会话变化、时钟异常、退出和重启 |
| OpenVINS | 默认单线程直接处理队列，显式异步模式保留成员线程与退出 join | 图像处理仍占用 IMU 回调，队列锁范围未缩小；板端延迟待测 |
| 任务/命令 | LAND 接受和实际交接后清理等待；取消保留未决 ARM 关联，地面 DISARM 单独判断 | 迟到命令、超时、接管和降落期路径的真实通信验证 |
| MAVROS 等待 | 服务等待和应答分别计时，默认各 3 s；服务就绪后重新 send_command 会重启计时。集中配置任务窗口为 5 s，可能先于后端最长约 6 s 到期 | 仍需统一等待契约或调整经验证的任务窗口；本次只记录问题，不修改代码。清理本地 pending 不撤回飞控命令 |
| DDS ACK | 超时 opcode 隔离，reset 不清除；ARM/DISARM 共用 opcode | 地面更换 source_component 并重启后的迟到 ACK 隔离 |
| 离线分析 | 有界数值统计、真实话题类型/版本识别、联合时间线；不发布消息 | 新测试与真实 bag 读取均待执行，不能据状态时间线推定 EKF2 融合 |

参考 `dofarifa/Unmanned-Aerial-Vehicle` 的固定快照 `f4ad78e`，仅借鉴输入检查、有效基线与分析思路。
未移植其设备标定、固定门限、健康心跳、启动宽限期或 `poseimu` 后备。无需添加新的线程池或恢复框架。

## 当前检查入口与证据

已完成源码/配置静态检查、Python AST、包 XML、安装/测试路径及差异格式核对；标定与原有安全开关保留。
当前 [bridge CMake](https://github.com/QU-of-boomboom/uav_vio_bridge/blob/main/CMakeLists.txt) 注册 `test_vio_safety_gate`；
[bringup CMake](https://github.com/QU-of-boomboom/uav_bringup/blob/main/CMakeLists.txt) 注册 `vio_tools_test`、`vio_bag_analyzer_test`。
控制与任务当前 CMake 没有注册测试；旧 `dds_node_test`、`flight_task_test` 等记录不代表当前文件仍在或已通过。
构建及可执行检查见 [手动验证](OpenVINS手动编译与验证.md)。

| 历史记录 | 当时证据与边界 |
| --- | --- |
| 2026-09-19 MAVROS SITL | 正常飞行及任务进程停止后的主动 LAND；[原记录](../sim/PX4_SITL_起飞悬停降落验证.md)，未接入 VIO |
| 2026-10-10 DDS 软件 | 旧控制/任务构建、合成状态测试和只读启动；未证明 Agent/PX4 消息交付；输出目录为 `build/install/dds-package-dev`、`backend-selection`、`control-cleanup` |
| 2026-10-12 旧 VIO | 旧 bridge/bringup 的局部构建及只读短台架采样；[保留测量](Position启动与验证.md)，视觉年龄超限、输出为 0 |
| 2026-10-15 当前修订 | 源码、配置和静态检查；新增 C++/Python 回归源码未执行，未重新构建、ROS/SITL、PX4 接收或融合验收 |

旧 DDS 结果位置保留作追溯：`log/dds-package-dev/build_2026-10-10_18-59-52/`；
`log/backend-selection/{build_2026-10-10_19-31-33,test_2026-10-10_19-32-27}/`；
`log/control-cleanup/{build_2026-10-10_19-37-30,build_2026-10-10_19-39-21,test_2026-10-10_19-39-45}/`；
`log/backend-config/{build_2026-10-10_19-48-10,build_2026-10-10_19-52-02,test_2026-10-10_19-52-27}/`。
它们是历史本地产物路径，本轮未重读结果，不作为当前通过证据。

后续顺序：当前构建与必要回归 → PC SITL → Orange Pi/飞控只读台架 → PX4 接收与 EKF2 融合 → 实飞。
