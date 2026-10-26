# uav_bringup

ROS 2 Humble 启动与配置编排包。统一入口为 `launch/uav.launch.py`，部署参数集中在
`config/uav.yaml`。入口逐个提取节点所属分区，不能把整份集中配置作为 ROS 参数文件传给节点。

## 启动

默认只启动 `uav_control`。本机集中配置选择 DDS，且 `read_only: false`；不会启动自动任务、MAVROS、相机、OpenVINS 或视觉里程计桥接。

```bash
ros2 launch uav_bringup uav.launch.py

# 本机 DDS 一体启动：Agent、飞控 IMU、相机、OpenVINS、桥接和控制。
# 启动前退出已有同类进程；当前集中配置 uav_bridge.enabled=true。
ros2 launch uav_bringup uav.launch.py start_agent:=true \
  agent_executable:=/home/orangepi/BoomBoomFly/tools/micro_xrce_dds_agent/install/bin/MicroXRCEAgent \
  start_camera:=true start_openvins:=true start_bridge:=true

# 启动相机和 OpenVINS；DDS 后端会自动启动 imu_bridge，uav_bridge 输出保持关闭
ros2 launch uav_bringup uav.launch.py start_control:=false \
  start_camera:=true start_openvins:=true

# 启动 odom_bridge 进程；只有配置 uav_bridge.enabled:=true 才会回传视觉里程计
ros2 launch uav_bringup uav.launch.py start_control:=false \
  start_camera:=true start_openvins:=true start_bridge:=true

# 使用已有 MAVROS 时启动相机、OpenVINS 与 odom_bridge
ros2 launch uav_bringup uav.launch.py config_file:=/path/to/uav.yaml \
  start_control:=false start_camera:=true start_openvins:=true start_bridge:=true
```

`start_agent`、`start_control`、`start_mission`、`start_mavros`、`start_camera`、`start_openvins` 和
`start_bridge` 分别控制进程是否启动。控制后端没有独立的 launch 参数，唯一来源是
`uav_control.backend`，可选 `dds` 或 `mavros`。即使 `start_control:=false`，同一选择仍决定
OpenVINS 的 IMU 来源和 `odom_bridge` 的唯一输出目的地。

MAVROS 后端下，OpenVINS 直接订阅标定配置使用的 `/mavros/imu/data_raw`，不启动
`imu_bridge`。DDS 后端下，启动 OpenVINS 会同时启动 `imu_bridge`，按
`uav_control.dds.fmu_prefix` 订阅 `${prefix}/out/sensor_combined`，转换后发布 `/px4/imu`，
并让 OpenVINS 订阅该话题。`odom_bridge` 使用同一前缀，只选择 MAVROS 的
`/mavros/odometry/out` 或 DDS 的 `${prefix}/in/vehicle_visual_odometry` 之一。

`uav_bridge.enabled` 是独立的视觉里程计输出门，本机已配置为 `true`。启动 `odom_bridge`、任务或
控制节点都不会隐式打开该门，也不会改变控制后端的 `read_only` 设置。MAVROS 和 DDS 控制
只读状态分别由集中 YAML 的对应后端分区决定，本机当前 DDS 允许控制、MAVROS 保持只读。

入口不会启动 PX4 或 Gazebo。DDS Agent 可由 `start_agent:=true` 启动，串口和波特率
分别由 `agent_device`（默认 `/dev/ttyS6`）、`agent_baud`（默认 `921600`）指定。
`agent_executable` 默认为 PATH 中的 `MicroXRCEAgent`，也可指定完整路径。
已有 Agent 时保持 `start_agent:=false`，避免重复打开串口。
已有 MAVROS 时可不设置 `start_mavros`；若需由入口
启动 MAVROS，可设置 `start_mavros:=true` 并通过 `fcu_url` 覆盖串口地址。

本机视觉使用任意航向 LOCAL_FRD。`uav_control.dds.allow_local_vision_heading=true`
允许用新鲜的飞控 EV yaw 融合、倾斜对齐且无航向故障/拒绝状态，判定局部航向有效；
它不改变视觉坐标标签，不修改 PX4 的 `yaw_align`，也不绕过飞控预飞检查。
`map` 是 PX4 当前局部坐标经 `(y,x,-z)` 换轴后的 z-up 坐标，不表示地理东、北。
位置、速度、姿态和目标采用成对的同一换轴，任务高度为起点上方的相对高度。
初始化或估计器重置后，需等融合稳定、地面未解锁，再显式执行：

```bash
ros2 service call /uav_control/reset std_srvs/srv/Trigger '{}'
ros2 topic echo /uav_control/state --once
```

只读模式会继续显示 `control_inhibited=true`，不能起飞。实际飞行准备完成后，才将集中
配置的 `uav_control.dds.read_only` 改为 `false` 并重启上述链路，复位并确认状态和飞控
预检通过。飞控预检标志针对当前模式：室内无全局定位时不要停在 Hold/AUTO.LOITER
等待该模式的全局位置要求；先连接遥控器，在未解锁状态下切到定高 ALTCTL 并核对预检。
两个任务均由遥控解锁触发：飞机保持未解锁，链路检查和地面复位完成后另开已加载环境的终端执行：

```bash
ros2 launch uav_bringup uav.launch.py start_control:=false start_mission:=true

# 方形任务：原起飞与悬停后，按初始航向左/前/右/后飞一圈，回起点降落。
# 与上一条二选一；当前边长 0.5 m、相对高度 1.0 m，尚待方形实飞验收。
ros2 launch uav_bringup uav.launch.py start_control:=false start_mission:=true mission_profile:=square
```

第二条任务入口复用已运行的 Agent、VIO 和控制进程，不重复启动它们。
启动任务后等待日志显示 `WAIT_ARM`，再用遥控器解锁；随后自动准备 Offboard、起飞并执行所选任务。
任务程序不发送解锁命令。已解锁时启动任务会退出，需先上锁再重新启动任务。
本机任务已启用 `require_vio_health=true`，视觉诊断过期或异常会中止正常任务。
`mission_profile` 默认为 `config`（使用集中配置），可按本次任务覆盖为 `hover` 或 `square`。
边长、角点容差、连续稳定时间等位于 `config/uav.yaml` 的 `uav_mission` 分区；
`square_max_speed_m_s` 是到点速度判据，不是飞边限速，详见 `uav_mission/README.md`。

## 相机与 OpenVINS

默认 D435i 双目使用 `848x480x30`，与 `config/openvins/d435i_px4/` 中的相机标定一致；安装时
该标定目录会复制到 `uav_bringup` share 下的 `config/openvins/d435i_px4/`。相机驱动保留原始
848×480 图像，OpenVINS 配置负责内部降采样。不要用其他分辨率或相机标定替换此配置。

```bash
ros2 launch uav_bringup uav.launch.py start_control:=false \
  start_camera:=true start_openvins:=true openvins_config_file:=/path/to/estimator_config.yaml
ros2 run uav_bringup check_vio.py --seconds 10
```

OpenVINS 使用当前源码的 `run_subscribe_msckf` 接口：`config_path`、`verbosity`、
`use_stereo`、`max_cameras`、`save_total_state`。IMU 话题通过 OpenVINS external YAML 覆盖项
`imu0_rostopic` 选择，避免与 `ROS2Visualizer` 对 `topic_imu` 的直接参数声明冲突。相机话题和
标定仍由安装的 OpenVINS 配置提供。

诊断录制入口只订阅选定后端、相机、IMU、OpenVINS、桥接、控制和任务话题：

```bash
ros2 run uav_bringup record_vio.py --output bags/vio_bench_01
```

输出目录必须尚不存在；图像录制会较快占用磁盘。OpenVINS 的 INFO 处理耗时写入启动终端，
需要分析时应一并保存 launch 终端日志。录制不发布飞控命令。

录制完成后可离线生成 VIO 与飞控状态联合 JSON 报告：

```bash
ros2 run uav_bringup analyze_vio_bag.py bags/vio_bench_01
ros2 run uav_bringup analyze_vio_bag.py bags/vio_bench_01 \
  --backend-file src/uav_bringup/config/uav.yaml --stationary --output reports/vio_bench_01.json
```

工具按 bag metadata 中的类型和实际话题识别 PX4 `VehicleStatus`，DDS 前缀及桥接门限来自
集中配置。频率使用 `(n-1)/duration`，报告将 bag record 时间和消息源时间分别列出；两种时间的差
只有在时钟域相同时才有意义，也不代表端到端延迟。`--stationary` 只有显式给出时才把相对首样本
位移标注为静止漂移。协方差只报告输入矩阵对角线诊断，不替代桥接器的完整 PSD、特征值、坐标变换
和杆臂门控。JSON 默认写到 stdout；指定的输出文件必须是新路径，已有文件不会覆盖。
输出文件的父目录会自动创建。统计逐条处理消息，不累积图像或 IMU 消息；分位数只保存最近
4096 个有效数值，最小/最大值覆盖全部数值；时间线最多保存 5000 个事件并报告截断数量。
IMU 话题的周期统计使用 bag 记录时间，不表示传感器采样抖动；源时间统计针对 VIO Odometry。
位置预测残差统计比较相邻原始样本，不复现桥接器“最后通过样本”基线及完整恢复状态机。
坐标系不符的位置不参与几何统计；检测到源时间倒退或 VIO 会话/初始化变化时，报告不输出
跨重置的整段位移或静止漂移。联合时间线只说明记录到的事件先后，不证明 EKF2 已融合。

## 集中配置分区

`config/uav.yaml` 包含 `camera`、`uav_control`、`uav_bridge` 和 `uav_mission`。
统一入口只向每个节点传入它自己的分区，并从 `uav_control.backend` 派生桥接路由；
`fmu_prefix` 同时派生 PX4 IMU 输入与 DDS 视觉里程计输出。OpenVINS
相机、IMU 标定和估计器配置单独安装在 `config/openvins/d435i_px4/`。

## 验证边界

本轮只做源码和配置静态检查，新增回归测试源码但未运行；未编译、未运行 ROS 节点或硬件。
静态接口匹配不证明相机、
MAVROS、PX4 DDS 收发、PX4 外部视觉接收或 EKF2 融合；这些需要分别在目标板和飞控侧验证。

### DDS 着陆状态超时（2026-10-23）

`uav_control.dds.landed_state_timeout_s=2.5` 专用于 PX4 约 1 Hz 的着陆状态；
消息时间戳与本机接收时间都必须新鲜。`state_timeout_s=1.0` 继续用于飞控状态，
`position_timeout_s=0.5` 继续用于定位。真实着陆状态丢失仍使任务失败；不会自动恢复旧航线。
任务日志分别报告统一状态过期、飞控状态/同步失效、连接断开和着陆状态过期。
修改配置或重新编译后，运行中的节点需在地面退出并按原入口重新启动才生效。
