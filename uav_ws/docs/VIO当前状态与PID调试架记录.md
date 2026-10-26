# VIO 当前状态与 PID 调试架记录

更新日期：2026-10-17。本文是当前验证进度和下一轮采集约定的主入口。

## 后续约定

- 使用 MAVROS，串口 `serial:///dev/ttyS6:921600`；同一时刻只运行一种飞控通信协议。
- **保持 PX4 当前全部参数不变**，包括 EKF、时间同步、日志和 PID 参数。助手未执行此前建议的 `SDLOG_MODE=2`、`SDLOG_PROFILE=129` 修改，后续不采用；实际值以下次只读快照为准。
- 用户将机体固定在 PID 调试架上，手动解锁、控制电机并上锁。助手负责启动采集、观察、标注事件、保存和分析记录；采集流程不发送解锁、模式切换、油门或自动任务命令。
- 本轮目标是比较电机运行前后供数、视觉更新、同步和 PX4 估计状态。调试架约束运动，结果不等同于自由飞行或 PID 验收。
- 助手本次只读分析用户已采集的 ROS/ULog，未启动新的硬件采集、改动 PX4 参数或运行代码。

## 2026-10-17 18:32 最新三轮自稳采集

已取得 `log/vio_ekf_pidrig_20261017_183229_432838/` 和 log104/105/106，见 [本轮分析](../log/vio_ekf_pidrig_20261017_183229_432838/analysis/report.md)、[联合曲线](../log/vio_ekf_pidrig_20261017_183229_432838/analysis/joint_timeline.png)。

- 三轮实际均为 STABILIZED，解锁约 8.6/10.2/11.1 s。VIO 最大估计位置变化约 2.28/5.47/0.79 cm，桥接没有停发，没有本段新增位置重置，已记录的主实例 EV aid-source 样本无拒绝。
- 本轮峰值推力约 0.135，低于上一轮 POSCTL 的 0.583，时长和操作也不同，不能将改善全部归因于模式切换或外推到高推力。解锁期间气压高度仍有约 3.91/3.09/1.34 m 峰峰变化。
- **18:33:32.841 的一次 IMU 时间戳回退约 1.172 ms 已定位为 MAVROS 首次收敛时切换消息头时基。** 原始 PX4 time_usec 递增；第 501 个合格 TIMESYNC 后开始应用同步 offset，此前消息头为主机 now()。OpenVINS 丢弃一个样本后继续运行，无视觉超时和桥接停发。
- 已更新 PID 架启动器：MAVROS 先按原流程预热 15 s，随后等待 IMU 头时间戳首次回退（本次证据显示这是 500 个合格 TIMESYNC 样本后应用 offset 的时基切换），再要求 IMU 时间戳连续递增、TIMESYNC 状态持续更新 5 s，最多等待 120 s。门控通过后才启动 VIO，VIO 本身仍预热 15 s；超时则不启动 VIO、不提示解锁。每次倒退只作为历史事件记录，输入恢复后不再锁存为持续故障。
- 这项门控以已观察到时基切换作为 MAVROS 已应用 offset 的运行证据，随后检查短时稳定性；它不代表完整时钟精度验收。未更改 PX4 参数，也未增加 MAVROS 参数覆盖。
- 三轮 ULog 与 log103 对比，共同参数仅飞行编号和累计飞行时间计数变化；没有 EKF、日志或 PID 配置差异。

## 2026-10-17 18:12 调试架采集与联合分析

已取得 `log/vio_ekf_pidrig_20261017_181232_929973/` 和 log103 ULog，分析见 [本轮报告](../log/vio_ekf_pidrig_20261017_181232_929973/analysis/report.md)、[联合曲线](../log/vio_ekf_pidrig_20261017_181232_929973/analysis/joint_timeline.png)。以下旧表和原采集计划保留为此前状态，最新结论为：

- 本轮实际全程 POSCTL，不是 Stabilized；下一轮保持 PX4 参数不变，按自稳模式做对照采集。
- 已有主 EKF 实例 1 的内部视觉融合证据；约 2 Hz 的 aid-source 记录中，水平位置 50/52 样本 fused、2 条拒绝，速度/高度/航向各 52/52 样本 fused。不是全部视觉输入的逐条融合率；原始 vehicle_visual_odometry 仍未记录。
- 本轮未复现 visual update expired，解锁至 Kill 前有效视觉源时间最大间隔约 133 ms。VIO 在 Kill 前估计位移变化约 2.09 m，气压高度相对首样本最高变化约 21.29 m，需优先排查电机运行相关的估计质量、机械和气压扰动。
- 18:13:32.550 先 Kill，33.929 桥接因运动预测残差暂停，34.027 恢复，随后出现水平位置拒绝，35.729 水平位置重置约 1.23 m；不能倒置事件顺序，将桥接停发当作本次急停原因。
- Kill 后已记录的四路驱动输出归零；不能把仍在计算的 actuator_motors 控制请求当作实际电机转动。
- 内存稳定；已取得 ULog 的运行中参数更改列表为空。本次离线分析未改动 PX4 参数或运行代码。

## 当前部署与参数快照

| 项目 | 当前已知状态 |
| --- | --- |
| 集中配置 | `src/uav_bringup/config/uav.yaml`，`uav_control.backend=mavros` |
| 控制/任务 | MAVROS 控制 `read_only=true`；记录时不启动控制和任务节点 |
| 基础桥接开关 | `uav_bridge.enabled=false`；回传试验使用单轮配置副本启用，不改默认文件 |
| VIO 输入 | `/mavros/imu/data_raw`；MAVROS 后端不启动 DDS imu_bridge |
| 相机 | D435i，双红外 848×480、30 Hz；OpenVINS 内部降采样 |
| 桥接门限 | 样本年龄 0.15 s、未来容差 0.01 s、视觉超时 0.3 s、里程计超时 0.2 s |
| PX4 外部视觉 | 最近读取 `EKF2_EV_CTRL=15`、`EKF2_HGT_REF=3`、`EKF2_EV_DELAY=0` |
| PX4 外部视觉位置外参 | 最近读取 `EKF2_EV_POS_X/Y/Z=0`；只是现值，尚不能据此确认安装正确 |
| PX4 日志 | 最近读取 `SDLOG_MODE=0`、`SDLOG_PROFILE=1`；保留现值 |

PX4 数值来自 [上轮只读快照](../log/vio_ekf_mavros_20261016_233701_177002/px4_parameters_after.json)，不是本次重新读出的实时值。下一轮只读保存参数和固件信息；遇到差异时记录，不自动写回。

## 已完成的工作与证据

| 阶段 | 已有结论 | 边界 |
| --- | --- | --- |
| 早期 C 阶段内存增长 | 当时后端与实际通信不一致，OpenVINS 等待 `/px4/imu`，相机队列积压 | 当前 MAVROS 复测未再复现，不能推断所有负载下均稳定 |
| 2026-10-16 22:49 MAVROS A/B/C | IMU、同步和 VIO 已有消息，内存稳定 | 缺少 mavros_extras/odometry，ROS 有输出不等于已转换成 MAVLink |
| 插件补齐 | 同工作区 MAVROS 2.16.0 的 mavros_extras 已构建安装，odometry 插件加载并订阅 | 第三方 MAVROS 源码未修改 |
| 2026-10-16 23:37 C 复测 | 完成 180 s，主机输出 MAVLink ODOMETRY 6217 条，约 35.11 Hz | 是 UAS→Router 证据，不是逐条飞控接收/融合证明 |
| 飞控返回估计状态 | 活跃输出期间 80/80 样本的位置、高度和水平/垂直速度有效 | 没有配套 ULog，尚未逐样本确认视觉融合/拒绝 |
| 内存与同步 | OpenVINS RSS 最大约 82 MiB；RTT 中位数 3.53 ms、P95 12.06 ms，未见持续增长趋势 | 启动阶段曾有高 RTT，固定预热时间不能代替新鲜性和收敛检查 |
| 2026-10-17 断流分析 | 有效视觉源时间停滞约 1.000/0.969 s，桥接停发约 0.806/0.767 s | 尚不能唯一判定跟踪不足、三角化失败还是残差拒绝 |

主日志：[23:37 C 复测目录](../log/vio_ekf_mavros_20261016_233701_177002/)。

- [复测报告](../log/vio_ekf_mavros_20261016_233701_177002/retest_summary.md)
- [断流分析报告](../log/vio_ekf_mavros_20261016_233701_177002/outage_diagnosis.md)
- [断流逐条证据](../log/vio_ekf_mavros_20261016_233701_177002/C/outage_analysis.json)

两次停滞期间仍收到 17/19 条重复旧视觉时间的健康消息，源码中这些消息在相机处理后发布；IMU 源时间最大间隔约 8 ms，VIO 预测输出持续。问题已缩小到新视觉观测未持续进入有效更新这一环。第二次附近有 USB 警告，第一次没有对应警告，不能认定为共同根因。

## 下一轮调试架采集顺序

1. 用户完成调试架固定，保持未解锁。助手检查重复节点，创建独立 `log/vio_ekf_pidrig_TIMESTAMP/`，保存只读参数快照、软件配置、固件和实际启动命令。
2. 先启动记录和资源监测，再启动需要采集的运行链路，保留初始化日志。MAVROS 预热 15 s 后，等待 IMU 时间戳时基切换并稳定 5 s（最长 120 s）；通过后才启动相机/VIO。相机/VIO 在禁用视觉回传时继续预热至少 15 s。检查 IMU、同步、视觉健康、odometry 插件和 TF 后，使用本轮配置副本启用回传；此时仍未解锁。
3. 采集电机未运行的基线。固定架上不要求旧流程中的“移动一米、返回起点”等手持动作。
4. 助手确认已开始录制后，用户手动解锁。按 `/mavros/state.armed` 实际变化标注解锁时间，持续记录电机操作、视觉停更、桥接停发/恢复、同步异常和模式变化。
5. 解锁期间持续采集，不因 armed=true 终止，不重启或停止运行链路，不主动切换回传开关。出现异常向用户报告，用户决定电机操作及上锁。
6. 用户手动上锁，确认 armed=false 后继续记录约 10 s，再结束 rosbag 和采集进程。若提前结束录制，只停止记录器；停止录包不等于停止电机或上锁。
7. 下载对应 `.ulg`，关联到本轮目录，通过同步字段生成联合时间线。优先使用现有 GCS 链路或之后 SD 卡导出，避免另一个程序同时打开 `/dev/ttyS6`。

**不要用 `run_mavros_vio_validation.py` 采集解锁阶段。** 它会在检测到解锁后退出并清理本次启动的桥接、VIO 和 MAVROS。下一轮使用独立记录器，运行链路与记录器分别管理，保留原脚本的未解锁限制。

### 一键启动完整采集链路

2026-10-17 新增源码入口 `src/uav_bringup/scripts/run_mavros_pidrig_capture.py`。先退出已有 MAVROS、相机、OpenVINS 和桥接，保持未解锁，再执行：

```bash
cd /home/orangepi/BoomBoomFly/uav_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
python3 src/uav_bringup/scripts/run_mavros_pidrig_capture.py
```

入口使用已有地面验证模块的进程/参数只读工具，独立管理本轮采集：先启动 rosbag，MAVROS 预热 15 s，再等待 IMU 时基切换和连续稳定 5 s（最多 120 s），通过后才启动相机/VIO 与禁用输出的桥接并再预热 15 s。通过供数、插件和 TF 检查后重启为启用输出的桥接。出现“可以手动解锁”后由用户操作；检测到上锁后继续记录 10 s，再清理本轮进程。输出到唯一 `log/vio_ekf_pidrig_TIMESTAMP/`，包含 bag、节点日志、只读参数快照、资源与事件记录。不需要键盘确认，不录制原始图像。该入口已在 2026-10-17 两轮 PID 架采集中运行；本次门控代码更新尚未实机复验，不把静态检查记为硬件验收通过。

解锁后不会因视觉/IMU 告警自动清理链路，也不会自动上锁；Ctrl-C 时如仍解锁或无法确认上锁，保持链路并等待用户上锁/恢复状态连接。强制关闭终端或进程无法提供这一等待行为，且不会停止电机。若录包进程异常退出，终端会报告，不能将仍运行的通信节点当作录制正常。

本轮固定架供数/振动日志采集建议使用 Stabilized（自稳）模式，由用户控制油门。Position 不是启动 ULog 或融合视觉的必要条件；其位置/高度保持控制在受限架上可能因估计误差持续纠偏。此项是针对固定架约束的工程判断，不是自由飞行模式建议。不要为采集临时修改模式映射、解除预飞检查或强制解锁。

最近读取的 SDLOG_MODE=0 会按解锁/上锁启停飞控日志，但脚本不通过 ROS 判断 SD 卡是否正在写入，不保证 PROFILE=1 包含所有视觉话题。用户可通过已有 QGC 连接只读执行 `logger status`；采集结束后下载对应 `.ulg`，ROS bag 不包含该文件。

### 独立记录器入口

以下命令只录制已运行链路，允许状态转为解锁；不启动相机/VIO/MAVROS，不自动启用桥接，不修改 PX4 参数。下一轮由助手执行，当前不运行。每轮使用新目录：

```bash
cd /home/orangepi/BoomBoomFly/uav_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
VIO_RIG_LOG_DIR="$PWD/log/vio_ekf_pidrig_20261017_01"
mkdir -p "$VIO_RIG_LOG_DIR"
cp src/uav_bringup/config/uav.yaml "$VIO_RIG_LOG_DIR/uav_source.yaml"
ros2 bag record --storage sqlite3 --max-cache-size 33554432 \
  -o "$VIO_RIG_LOG_DIR/bag" \
  /mavros/state /mavros/extended_state /mavros/imu/data_raw \
  /mavros/timesync_status /mavros/estimator_status \
  /mavros/local_position/pose /mavros/local_position/velocity_local \
  /mavros/odometry/out /mavros/odometry/in /mavros/statustext/recv \
  /mavros/rc/in /mavros/rc/out /mavros/imu/data /mavros/imu/static_pressure \
  /ov_msckf/odomimu /ov_msckf/visual_health \
  /uav_bridge/diagnostics /diagnostics \
  /uas1/mavlink_sink /uas1/mavlink_source /tf /tf_static /rosout \
  > "$VIO_RIG_LOG_DIR/recorder.log" 2>&1
```

本命令不录制原始图像，32 MiB 是 bag 缓存限制，不是全系统内存上限；不存在的话题不会自动产生消息。助手还需保存 MAVROS/VIO/桥接 stdout、每约 5 s 的内存/CPU/温度、本轮实际回传配置及 `events.jsonl`。事件含主机本地时间和单调时间、解锁状态、模式及来源；用户描述的操作与状态观测分别标注。

## 参数不变时 ULog 的边界

最近的 `SDLOG_MODE=0` 表示解锁时启动记录、上锁时停止。下一轮通过用户实际解锁获得这段 ULog，不设置 SDLOG_MODE=2、不执行 logger on 或其他日志配置更改。解锁前和上锁后基线由 ROS bag 和主机日志保存。采集前只读检查 logger/SD 卡，不能仅凭参数值保证文件存在。

`SDLOG_PROFILE=1` 不保证有 vehicle_visual_odometry 或全部 EV aid-source；不修改该值。下载后先列出实际话题、实例、频率和时间范围，再确定结论：

- 有 vehicle_visual_odometry 时，检查内容、源时间和断流。
- 有 estimator_status_flags、EV aid-source 时，检查主 EKF 融合/拒绝、创新及最近融合时间；低频数据不能当作逐样本证据。
- 缺视觉原始话题或 aid-source 时，记录缺项，只报告已有估计状态和主机传输证据，不标为完整融合验证通过。
- 保存 ULog 中的参数复核参数不变。PX4 和 ROS 原始时钟数值不能直接相减，先做同步换算。

## 待完成

1. 已取得首轮调试架 ROS/ULog 并完成联合分析；下一轮在 Stabilized 下做相同条件的对照记录。
2. 对比电机运行前后视觉停更、输入时序、同步、USB/供电现象、资源和估计状态；相关性与因果分开报告。
3. 补充 OpenVINS 逐帧跟踪数量、MSCKF/SLAM 候选与接受数量、三角化及残差拒绝统计，必要时采集异常前后少量图像。现有健康消息没有这些统计，增加 rosbag 话题不能补出未发布的数据。
4. 按现有话题核实 PX4 接收和融合；运动方向、尺度、安装外参、自由飞行及 PID 性能仍需独立验证。
