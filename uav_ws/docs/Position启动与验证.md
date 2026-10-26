# Position 定位验证与历史台架记录

更新日期：2026-10-15。当前启动、录制和离线分析命令统一见 [手动验证](OpenVINS手动编译与验证.md)。
本文只维护定位验收判据和历史证据；以下旧数据不覆盖当前 uav_bridge 迁移版，本次未操作硬件。

## 当前定位验收

1. 先检查安装 overlay、后端、相机 profile、标定及 IMU 轴向/测量点，保持控制只读、VIO 输出关闭。
   确认没有重复相机或估计器；按 [板端配置](OrangePi5Plus_OpenVINS接入配置.md) 核对供电、USB 和时钟。
2. 静止初始化后手持平移/转动，检查方向、尺度、速度和回到起点误差；静止健康不能代替运动验证。
   同时记录双图、原始/转换后 IMU、OpenVINS health/odom、桥接诊断、飞控状态及 launch stdout。
3. 观察真实视觉 stamp 推进、输入/视觉年龄、重复/倒退与断流。当前默认采样年龄 0.15 s、未来容差 0.01 s、
   输入断流 0.2 s、视觉停更 0.3 s；这些是工程初始值，需原始样本调定。可选速度上限默认禁用。
4. 检查单帧尖峰返回、持续偏移、协方差异常、遮挡及会话变化；异常不应被限频隐藏。
   停发后恢复需 3 个新视觉与 3 个有效递增 Odometry，不回放旧样本，不恢复旧任务。
5. 物理安装、坐标、时间和定位质量核对后，再单独评估开启 `uav_bridge.enabled`。
   `check_vio.py --require-output` 只能检查 ROS 输出；PX4 接收和 EKF2 融合须另取飞控证据。

| 层次 | 需要的独立证据 |
| --- | --- |
| ROS 供数 | 实际输入/输出、采样与接收时间、frame、协方差、拒绝原因；published 计数只是本地调用 |
| PX4 接收 | vehicle_visual_odometry 的内容、时刻、坐标及重置；缺该数据不能精确归因上游中断 |
| EKF2 融合 | 当前主 EKF 的视觉位置/速度/高度/航向融合标志、创新与拒绝/超时；不要只看参数开关 |
| 实际运动 | 已知距离、方向、回环误差及约束条件；调试架上的估计漂移不等于飞机运动 |

离线报告区分 bag 时间与源时间，只在物理静止且无坐标重置时解释静止漂移；联合时间线不能证明融合因果。
发布型回放须同时隔离 ROS Domain 与全部飞控写入/命令话题，当前 analyze 工具只读取 bag。

## 2026-10-12 旧版 D435i/PX4 只读台架

当时 Orange Pi 5 Plus/Humble 使用旧 OpenVINS v2.7 补丁和 uav_vio_bridge/MAVROS。
原配置与台架 ZUPT 配置各采样约 20 s，均显式关闭桥接回传，PX4 全程观测为 connected=true、armed=false、AUTO.LOITER。
旧三包曾增量构建，预测/视觉 2 个及 Bridge 17 个 GTest、launch 参数和配置安装检查有历史通过记录，
仅属于该旧树。迁移后的当前测试名称和构建范围以 [手动验证](OpenVINS手动编译与验证.md) 为准。

| 观测项 | 原配置 | 当时台架配置 |
| --- | --- | --- |
| FCU IMU 接收频率 | 242.0 Hz | 241.5 Hz |
| 双目图像尺寸 | 848×480 mono8 | 848×480 mono8 |
| 双目接收频率 | 22.4 / 22.5 Hz | 13.1 / 13.2 Hz |
| 初始化 | 静止时等待运动 | 静止初始化成功，generation=1 |
| VIO Odometry | 无输出 | 232.6 Hz，global/imu，采样年龄 0.0531 s |
| 视觉健康 | WARN，未初始化 | 有真实融合且 stamp 推进，末次观测年龄 1.1922 s |
| 图像末次年龄 | 0.4189 s | 0.8869 s |
| Bridge 发布数 / 检查退出码 | 0 / 1 | 0 / 1 |

台架末次位置约 `[-0.0012,0.0024,0.0032] m`；Bridge `adapted=0/published=0`，因视觉和部分位姿过期而拒绝。
未放宽视觉超时或用发布时刻刷新健康。旧版四次启动/停止未再复现 OpenVINS 退出段错误；
MAVROS 仍有 SIGINT 后需要 SIGTERM、部分试运行需要 SIGKILL 的历史问题。
首轮还观察到 USB endpoint 131 watchdog 和 PX4 心跳超时，后两次短采样未复现，不代表长时稳定。

历史本地输出：`log/vio-hover-dev/hardware/{flight_topics,bench_topics}.json`、
`log/vio-hover-dev/hardware/first_attempt/bench.log`；早期退出问题见
`log/hover_bringup_20261012_1625/`。它们不是当前部署产物，本轮未重新读取。
后续 OpenVINS 切换到 BoomBoomFly fork、合并 estimator 配置和关闭 ZUPT，没有新的板端测量覆盖上表。

## 受限调试架 ULog

### 2026-10-12 log101

原始 ULog（本地 `ulg/2026_10_12/log101.ulg`）、分析报告（本地 `ulg/2026_10_12/log101_analysis/report.md`）、
曲线图（本地 `ulg/2026_10_12/log101_analysis/overview.png`）。
当时 `EKF2_EV_CTRL=15`、`EKF2_HGT_REF=3`、`EKF2_GPS_CTRL=0`，仅为历史配置，不作为待执行推荐。

- 约 23:00:12 解锁，模式为 Position。
- 解锁后约 10.5 s，估计向下位置约 1.6 m，推力指令约 0.71。
- 约 11 s 视觉停融、13.8 s 气压高度停融，15.1–15.7 s 依次退到 Altitude、Stabilized。
- 约 25.3 s 急停后执行器输出归零。气压高度随推力变化约 30 m，需结合气流和安装环境排查。

用户确认调试架限制运动、观察到持续加大油门；现有数据不足以判断 PID 过大或自由飞行性能。
ROS 文本未完整覆盖这次 EKF 停融，后续需录制异常前后的完整原始 VIO 与飞控证据。

### 2026-10-13 其他记录

log100 分析（本地 `ulg/2026_10_13/log100_analysis/report.md`）；
19:00 联合分析（本地 `ulg/2026_10_13_1900/analysis/joint_report.md`）及
时间同步复核（本地 `ulg/2026_10_13_1900/analysis/timesync_review.md`）。
这些是各次受限试验的记录，不代替当前源码、板端时延、PX4 接收或飞行验收。
