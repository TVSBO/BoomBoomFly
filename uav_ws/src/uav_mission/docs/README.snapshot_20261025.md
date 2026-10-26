# uav_mission

更新日期：2026-10-23。任务与控制是独立 ROS 2 节点，通过 `uav_interfaces`
的目标、状态和命令接口通信。任务可使用 MAVROS 或 DDS 后端，不持有跨包控制对象，
不直接处理 PX4 消息、原始 VIO 测量、相机或飞控内部控制器。可选健康门控与新增状态字段见 [共享契约](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/%E5%BD%93%E5%89%8Dsrc%E9%9D%99%E6%80%81%E5%AE%A1%E6%9F%A5.md)。

## 实际实现

`mission_node.cpp` 接收统一状态，以 20 Hz 驱动 `FlightTask`，为目标附加当前 ROS 时间戳，
调用飞行命令 Service，并提供取消入口。`flight_task.cpp` 负责阶段、超时、误差和结果。
节点启动后自动等待就绪并推进任务；当前没有独立的启动 Service、任务 Action 或集群任务入口。
任务阶段及命令结果通过诊断话题和日志报告，接口见 [共享契约](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/%E5%BD%93%E5%89%8Dsrc%E9%9D%99%E6%80%81%E5%AE%A1%E6%9F%A5.md)，完成后节点退出。

```text
WAIT_READY → WAIT_ARM（遥控器解锁）→ PREPARE_OFFBOARD → REQUEST_OFFBOARD
  → TAKEOFF → STABILIZE → HOVER
  → hover: REQUEST_LAND
    square: SQUARE_LEFT → SQUARE_FORWARD → SQUARE_RIGHT → SQUARE_BACKWARD → REQUEST_LAND
    grid_s: GRID_S_OUT_1..8 → GRID_S_RETURN_1..8 → REQUEST_LAND
  → LANDING
  → 地面仍解锁时 REQUEST_DISARM → FINISHED

故障／空中取消 → SAFETY_LAND → 命令处理及新鲜地面/未解锁确认 → FINISHED
人工接管 → 停止自动目标和命令 → FINISHED（未确认安全降落）
```

子任务统一使用 `FlightTask::TaskResult` 数值接口：`0=PENDING`（未完成，含未开始）、
`1=PASSED`、`2=FAILED`。`Output.hover_result` / `Output.square_result` / `Output.grid_result` 分别返回结果，
同名字段也发布在 `/uav_mission/diagnostics`，结果变化会写入日志。
只有确认在空中、完成稳定判定且完整悬停通过，`hover_result` 才变为 1；此时
`mission_profile=hover` 选择降落分支，`square` 选择方形分支。方形四边完成后
`square_result=1`，再请求降落。子任务成功不等于整个飞行成功：后续失败不抹掉已通过的悬停结果。
以后新增任务沿用该结果类型，在成功分支接续；不是另起控制节点，也不自动恢复失败任务。

悬停和方形任务均要求：先在未解锁状态启动节点，等待日志出现 `WAIT_ARM`，再用遥控器解锁。
等待期间不发送目标、模式切换或解锁命令，等待遥控解锁本身没有超时；初始就绪检查仍使用 `ready_timeout_s`。
只有任务运行期间观察到新鲜的未解锁→已解锁变化，且地面、定位、航向、视觉和预检正常，才开始准备 Offboard。
任务不发送 ARM；统一状态中的 armed 变化是授权依据，不额外区分其他外部解锁来源，运行时应保持遥控器作为唯一解锁来源。
准备阶段连续发布解锁时的地面位置目标，按 `prestream_s` 等待，默认 1.2 s；该值需大于控制后端的预发送要求。
Offboard 命令最终接受与实际模式均确认后，才发布起飞高度目标。
起点及方形坐标轴捕获于遥控解锁时，高度相对该本地位置，不使用绝对 z=1。
已解锁时启动、在未就绪时解锁、或状态中断后只能看到已解锁状态，均退出且不会恢复后自动起飞，需上锁后重新启动任务。
准备期间人工切换模式、人工起飞或重新上锁会终止自动任务。

当前阶段只由 `FlightTask::Stage` 定义；未使用的旧阶段枚举文件已删除。没有后端工厂、插件或 MissionExecutor。
方形任务直接扩展当前阶段，搜索、视觉降落或多任务编排按实际需求扩展。

## 方形航点

航点逻辑来自 [dofarifa/Unmanned-Aerial-Vehicle 的 waypoint_flight_node.cpp](https://github.com/dofarifa/Unmanned-Aerial-Vehicle/blob/f4ad78e83a03627eae62552dcf1d7eddc16fbf2e/src/uav_controller/src/waypoint_flight_node.cpp)，
固定参考提交 `f4ad78e83a03627eae62552dcf1d7eddc16fbf2e`。实际源码顺序为左、前、右、后；
移植的是 `set_square_target`、角点稳定判定、单边超时和飞行范围检查，控制命令通过当前统一后端发送。

任务先完成原有相对起飞、连续稳定和初始悬停，再飞一圈，回起点上方后交接当前 AUTO.LAND 流程。
令边长为 L，以遥控解锁时的起点和航向建立前/左方向，四个角点为：
`左 L → 左 L + 前 L → 前 L → 起点`。每个目标由固定起点计算，避免角点误差逐段累积；
全程保持初始航向和相对起飞高度。

参考源码使用 NED，这里通过起始 map yaw 换算为现有 z-up map 位置；
map 可能是任意局部航向，不把“前、左”解释为地理北、西。
默认边长 0.5 m、三维到点距离不超过 0.08 m、三维速度不超过 0.15 m/s，连续满足 0.5 s 才换边。
`square_max_speed_m_s` 是角点到达速度条件，不限制飞边过程中的最大速度；飞行速度与响应仍由 PX4 位置控制参数决定。
每边 10 s 超时；距起点水平半径超过 `sqrt(2)*L+0.25 m` 或高度误差超过 0.25 m 时进入已有安全降落路径。
范围检查不提供避障功能，起飞点位于方形的一个角，场地需覆盖整条航线及余量。
完成四边、确认 AUTO.LAND、着陆和上锁后才报告 PASS；诊断和结果包含 `mission_profile`、`completed_square_legs`。
`hover_max_*` 仍只统计方形前的初始悬停，不代表整圈航线误差。

## 接口与失败处理

| 接口 | 当前行为 |
| --- | --- |
| `/uav_control/target`，PoseStamped | 发布本地 map／ENU 位置与 yaw，控制节点维护飞控发送循环 |
| `/uav_control/state`，VehicleState | 读取状态／定位／落地有效性、命令结果及控制锁止 |
| `/uav_control/command`，FlightCommand | 递增非零 command_id；仅请求 SET_OFFBOARD、LAND、DISARM，解锁由遥控器完成 |
| `/uav_mission/cancel`，Trigger | 接受取消；若可能已解锁，转入安全降落并等待地面／未解锁确认 |

`dispatched=true` 不代表 PX4 接受，更不代表实际状态已经改变。
`control_inhibited`／`control_message` 使任务识别只读或故障锁止，
防止定位／通信恢复后继续旧任务。降落交接阶段仍能确认 AUTO.LAND、落地和未解锁，
不会仅因停止 Offboard 心跳而报错。MAVROS 后端也发布实际锁止状态及原因。

启用 `require_vio_health` 时，任务订阅 `/uav_bridge/diagnostics` 的
`uav_bridge/odom_bridge` v1 状态；只有桥接已启用、未暂停且真实视觉更新和输入都新鲜时才继续。
桥接的 `published` 计数只表示本地 publisher 调用，不能证明 PX4 收到或 EKF2 融合。
任务运行中 VIO 健康失效会进入既有失败／安全降落路径；桥接恢复供数不会恢复旧任务，
需完成地面复位后显式启动新任务。

空中失败或取消时，仍拥有 OFFBOARD 控制且定位/状态有效才请求 LAND。后端已经锁止时，任务只观察兜底降落，避免重复 LAND。
等待遥控解锁或尚未请求 Offboard 时取消，直接停止任务并将控制保留在遥控器。
已请求 Offboard 后取消会进入安全处理，地面只在新鲜落地状态下请求 DISARM，空中只在仍持有 Offboard 时请求 LAND。
安全 LAND 最终拒绝/超时后停止任务侧 LAND 重试和目标流；地面 DISARM 与此前 LAND 失败分别处理，限制见 [共享审查](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/%E5%BD%93%E5%89%8Dsrc%E9%9D%99%E6%80%81%E5%AE%A1%E6%9F%A5.md)。
已解锁时启动任务直接失败；人工切到非 OFFBOARD/AUTO.LAND 模式后停止自动控制。
链路／定位无效时，不把旧观测当成落地结果，安全阶段可能持续等待。
控制节点停止后的实际 Offboard-loss 行为取决于 PX4 配置，任务超时与心跳丢失需分开验证。
地面重新就绪由 DDS `/uav_control/reset` 完成，之后显式启动新任务。
任务节点不自动调用 reset，也不自动重新开始旧任务。

取消命令需要先加载与任务相同的 overlay：

```bash
ros2 service call /uav_mission/cancel std_srvs/srv/Trigger '{}'
```

取消 Service 成功只证明取消请求被接受；完成需核对最终日志与飞控状态。

## 参数与启动

实际参数位于集中配置 [`uav_bringup/config/uav.yaml`](https://github.com/QU-of-boomboom/uav_bringup/blob/main/config/uav.yaml) 的 `uav_mission` 分区：

| 参数 | 默认值 |
| --- | --- |
| mission_profile | 集中配置 `grid_s`；代码回退 `hover` |
| square_side_m | 0.5 m |
| square_position_tolerance_m / square_max_speed_m_s | 0.08 m（三维距离）/ 0.15 m/s（三维速度） |
| square_settle_duration_s / square_leg_timeout_s | 0.5 / 10.0 s |
| square_boundary_margin_m / square_max_altitude_error_m | 0.25 / 0.25 m |
| takeoff_height_m | 1.0 m |
| horizontal_tolerance_m / vertical_tolerance_m | 0.2 / 0.15 m |
| stable_duration_s / hover_duration_s | 2.0 / 10.0 s |
| prestream_s | 1.2 s；大于控制后端预发送要求 |
| ready_timeout_s / command_timeout_s | 集中配置 30.0 / 5.0 s；command_timeout_s 代码默认 7.0 s |
| takeoff_timeout_s / landing_timeout_s | 30.0 / 45.0 s |
| state_timeout_s | 1.0 s |
| require_vio_health / vio_health_timeout_s | 集中配置 true / 2.0 s；代码默认 false / 2.0 s |

yaw 误差会记录，但当前不是悬停通过阈值。表中区分集中配置与代码回退值，实飞条件另行冻结。
构建与 overlay 命令见 [手动构建](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/OpenVINS%E6%89%8B%E5%8A%A8%E7%BC%96%E8%AF%91%E4%B8%8E%E9%AA%8C%E8%AF%81.md)；
嵌套接口路径为 `src/uav_control/uav_interfaces`，全部消费者需同步重建新增的状态字段。

当前统一入口读取集中配置。DDS/MAVROS 后端和只读状态分别由 `uav_control.backend` 与所选后端分区决定；
`start_mission:=true` 只启动任务节点，不会关闭只读保护。自动任务在配置为允许控制并完成飞控、定位验证后才可启动：

```bash
# Agent、VIO 和控制链路已启动、未解锁且完成地面复位后，单独启动方形任务。
# 等日志显示 WAIT_ARM，再用遥控器解锁，随后自动起飞。
ros2 launch uav_bringup uav.launch.py start_control:=false start_mission:=true mission_profile:=square

# 原悬停任务；不指定 mission_profile 时读取集中配置（本机当前为 grid_s）。
ros2 launch uav_bringup uav.launch.py start_control:=false start_mission:=true mission_profile:=hover
```

`mission_profile` launch 参数默认 `config`，表示读取集中配置；显式指定 `hover`/`square` 仅覆盖本次任务。
边长、高度、悬停时间等统一编辑 `uav_bringup/config/uav.yaml` 后重新构建/加载安装配置。
以上两条命令复用已有链路，不重复启动 Agent、控制节点或相机；同次试验只运行一个任务目标来源。
`uav_mission` 没有独立任务 launch；运行 `mission_node` 会自动推进任务，不能用于只读检查。

## 验证边界

当前 CMake 注册 `test_flight_task`，涵盖两个任务的遥控解锁等待与新鲜变化判定、已解锁启动拒绝、
准备期间取消/上锁/接管、原悬停流程、方形闭合/旋转、连续到点判定、超时、范围及视觉失效处理。
方形功能的离线测试与编译不等于实飞验收；当前方形功能尚未进行实机飞行验证。接口变更需与全部消费者一起重建。
待验收项与历史证据集中见 [当前源码与验证边界](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/%E5%BD%93%E5%89%8Dsrc%E9%9D%99%E6%80%81%E5%AE%A1%E6%9F%A5.md)，
运行与故障顺序见 [DDS 开发流程](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/DDS%E5%BC%80%E5%8F%91%E6%B5%81%E7%A8%8B.md)。

## 九宫格 S 形往返（grid_s）

仍由同一个 mission_node 在悬停成功后执行。九宫格是 3×3 个位置点，相对遥控解锁时的起点和初始航向，
飞机从右下角点 1 出发；左/前是机体初始方向，航向和高度在整个路径中保持不变。

```text
9 ← 8 ← 7
        ↑
4 → 5 → 6
↑
3 ← 2 ← 1（起点）
```

去程 1→2→3→4→5→6→7→8→9，回程严格 9→8→7→6→5→4→3→2→1。
一个完整往返共16段；可通过 grid_round_trips 设置有限重复次数，全部完成后交接原有 AUTO.LAND。
本次部署相邻点0.4米、往返1次，覆盖0.8×0.8米，路径长度6.4米。

| 集中配置参数 | 当前值 | 含义 |
| --- | --- | --- |
| grid_spacing_m / grid_round_trips | 0.4 / 1 | 相邻点间距和完整往返次数 |
| grid_speed_m_s / grid_acceleration_m_s2 | 0.15 / 0.2 | 连续位置参考的速度和加速度上限 |
| grid_position_tolerance_m / grid_arrival_speed_m_s | 0.08 / 0.15 | 到点三维距离和实际估计速度门限 |
| grid_settle_duration_s / grid_leg_timeout_s | 0.5 / 15 | 到点连续稳定时间和单段超时（秒） |
| grid_boundary_margin_m / grid_max_altitude_error_m | 0.25 / 0.25 | 九宫格边界外余量和高度偏差上限（米） |

每段使用按时间推进的五次多项式位置参考，端点速度、加速度为零；阶段切换不累积实测到点误差。
只有参考走完整段、飞机接近终点且速度达标并连续稳定后，才能进入下一段。
速度/加速度限制针对发送的位置参考，不是机体实际速度的硬限幅。当前单段参考约5秒，
16段加停稳、起飞悬停和降落预计约2分钟，实际依到点情况变化；不会以持续时间替代到点判据。

`grid_result=0/1/2` 表示未完成/成功/失败，`completed_grid_legs` 和 `completed_grid_round_trips`
在 `/uav_mission/diagnostics` 报告进度。任务失败不恢复旧路径；人工切模式接管后停止目标和命令。
当前起降偏移和定位精度仍需独立实机验证，软件回归通过不等于飞行验收。

已有链路就绪、地面未解锁且完成复位后，在加载同一 overlay 的另一个终端执行：

```bash
ros2 launch uav_bringup uav.launch.py start_control:=false start_mission:=true mission_profile:=grid_s
```

等待 WAIT_ARM 后遥控解锁。任务本身不发送 ARM，不重复启动 Agent/相机/控制链。
