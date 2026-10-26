# DDS 同步修复候选（未刷写）

2026-10-22 最终状态更新：V3 已刷入并完成 11 分钟同步复测，测试进程已停止。连接无重建、滤波无清零，但仍有最大约 0.53 ms 的微小提前时间戳；未进行本轮视觉融合或自主飞行验收。当前状态以 [换账号交接](交接_20261022_DDS同步.md) 和 release_20261022_v3/deployment.json 为准。下文未刷写与 V1 包说明为历史记录。

最新准备包：[release_20261022](release_20261022/README.md)，含明确区分文件名的修复固件、从 PX4 官方 Release 下载的原版回退固件，以及参数、SD 启动配置和任务存储备份。2026-10-22 本次重编译的包哈希以该目录 manifest.json/SHA256SUMS 为准；此前构建记录的包哈希不用于本次交付。

2026-10-22 更新：已通过 USB 实时核实当前飞控为 MICOAIR_H743_V2，固件 SHA 与本目录基线 d6f12ad1c4f70ad3230afd7d86e971421e02fef4 一致，UXRCE_DDS_SYNCT=1。当前采用只改同步调度的版本 `artifacts/sync_only/micoair_h743-v2_default.px4`；只改一个 DDS client 源文件，保持原版话题频率和参数定义。说明见 [同步修复交接](同步修复交接_20261022.md)。未刷写，尚不能宣称实机故障已修复。

以下为此前同时包含 IMU 200 Hz 改动的候选记录。旧 `artifacts/patched` 不作为本次同步修复候选；下文关于“尚无 USB、当前 SHA 未核实”的描述是历史状态。

## 此前 IMU 200 Hz 与同步采样周期候选记录

最新进展：原版及修复版完整固件均已编译并通过板号、大小、解压镜像与生成 DDS 配置检查。修复版位于 `artifacts/patched/micoair_h743-v2_default.px4`。详见 [固件构建与刷写交接](固件构建与刷写交接.md)。尚未刷写，实机时间对齐尚未通过。

本目录针对官方提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 准备补丁。该 SHA 来自历史飞行日志，目前尚未通过独立 USB 通道重新确认正在运行的固件 SHA。

## IMU 核对

- 本次切换 DDS 前实机参数备份：`IMU_INTEG_RATE=200`、`IMU_GYRO_RATEMAX=400`。本次未改动两项参数。
- 重新启动 Agent 后，10 秒只读实测 `/fmu/out/sensor_combined` 约 100.57 Hz，尚未达到 200 Hz。
- 同次消息 `gyro_integral_dt=4943 µs`、`accelerometer_integral_dt=4943 µs`，支持内部约 200 Hz 积分，但 DDS 抽样为约 100 Hz。单条积分时长不是整段频率统计。
- 官方 `dds_topics.yaml` 未对 SensorCombined 指定 rate_limit；`dds_topics.h.em` 默认 10 ms interval，对应 100 Hz。修改内部积分率不等于修改 DDS 输出率。
- 补丁只为 SensorCombined 添加 `rate_limit: 200.`，不全局提高其他话题频率。刷写后必须实测 `/fmu/out/sensor_combined` 和 `/px4/imu`，包括丢包/间隔；源码设置不能代替实测。

## 同步修复

保留原版 Timesync 滤波器和容差，只修正 DDS client 的同步调度：

1. 初始化及运行期都采用 100 ms 请求间隔，以请求开始时间计时，避免 RTT 被计入周期。
2. 收到同步回复后更新调度时间，即使滤波正在重新收敛，也保持相同周期。
3. 只有收敛后才执行原有可选系统时钟设置。

原因：原版初始化循环每次约 10 ms sleep 加 RTT，运行期却变为约 1 s；Timesync 的 skew 使用每次采样单位。周期切换会改变其含义，在本机实测约 1330 ppm 的相对速率差下产生长时间残差积累。

代价：500 次收敛样本约需 50 秒启动时间；稳态同步约 10 Hz，增加少量串口流量。还需验证运行调度、串口余量和异常 RTT 条件。没有改公共 Timesync 库，不影响原 MAVLink 同步代码。

## 已完成验证及限制

`prepare_and_test.py` 编译执行 **未经修改的官方 Timesync.cpp**，uORB/hrt 使用最小桩，以不同同步调度对比算法表现。覆盖零、正负 1330 ppm、合成/实测去趋势噪声，以及调度抖动和间歇漏采样，共 14 个场景，每场景模拟约 650 秒。

- 原调度正负 1330 ppm 场景：约 115 ms 最大保持误差、6 次重置，复现实机现象。
- 固定周期场景：零重置；包含调度抖动和漏采样的候选场景最大保持误差约 1.69 ms。
- 实测噪声来自 120 秒 DDS observed_offset，去掉线性趋势后循环使用；不是对完整板端运行的精确重放。阈值为工程验证用，不能保证所有场景。
- 补丁已通过 `git apply --check`，应用后与 candidate 文件逐字节一致。
- 已使用项目内独立安装的 Arm GNU 13.2.rel1 编译两版完整固件，.px4 板号均为 1179、镜像未超限，解压后与 .bin 一致。修复版镜像增加 80 字节，实际生成的 IMU 发送间隔为 5 ms。尚未刷写，实机 IMU 输出仍为约 100 Hz，同步问题仍存在；主机仍未检测到飞控 USB 维护通道。

复现命令：

```bash
python3 tools/px4_timesync_fix/prepare_and_test.py
```

补丁：`dds_200hz_and_sync_cadence.patch`；完整模拟结果：`results.json`。

## 下一步

现有 GPS1/UART3 串口可继续用于 DDS 遥测和验证，USB 并非排查或离线构建的前提。源码、工具链和 `micoair_h743-v2_default` 两版构建已就绪；下一步需要通过合适维护通道实时确认当前固件版本与参数，再安排刷写和地面复测。刷写属于替换飞控固件，具体固件和恢复说明见交接文档。

已核对同一提交的 `boards/micoair/h743-v2/src/hw_config.h`：bootloader 头文件定义 USB 及 USART `/dev/ttyS0,115200`，但 bootloader defconfig 未看到 USART 启用项。bootloader 与应用固件的设备枚举可能不同，不能仅凭 ttyS 编号判断其物理 UART，也不能仅凭头文件认定 UART3 刷写可用或认定实机 bootloader 与源码一致。当前这条 DDS 链路不能直接当作已验证的固件上传通道。参考：[bootloader 硬件定义](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/micoair/h743-v2/src/hw_config.h)、[bootloader 配置](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/micoair/h743-v2/nuttx-config/bootloader/defconfig)。

复测应覆盖 IMU 实际 200 Hz、至少 10 分钟时间同步、无滤波重置、无未来时间戳、持续视觉融合与航向有效性；这些通过前不进行 Offboard/悬停。

源码：[DDS client](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/uxrce_dds_client.cpp)、[DDS 话题表](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.yaml)、[生成模板](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em)。
