# VIO 地面延迟与负载测试

> 历史方案说明：本文由远端原有提交保留，使用旧 launch 和本地 `ulg/` 自动测试脚本；这些脚本未纳入本次发布。当前统一入口、输出开关和人工验证步骤以 [手动构建与验证](OpenVINS手动编译与验证.md) 为准，不直接套用下列旧命令。

目的：复现 2026-10-13 第三次的 TIMESYNC 延迟增长，定位延迟最先出现在 MAVROS/IMU、相机、视觉更新还是 bridge。三轮对比，不改参数或时间门限，不需要重新编译。

**全程拆下螺旋桨，飞控保持未解锁。** 碰撞后先检查相机支架、连接线和机体固定情况。记录过程不需要电机转动；地面未复现不代表已排除电机振动、供电变化或飞行时视觉场景因素。每轮结束后 Ctrl-C 退出该轮全部节点，再启动下一轮。

监测脚本只订阅 ROS、读取 `/proc`/温度/频率并写本地文件，没有 publisher、解锁、参数设置或控制服务。默认只订阅相机元数据，不订阅完整图像；避免一开始就用完整图像 rosbag 增加磁盘与 CPU 负载。

## 一个终端自动完成三轮（推荐）

先退出已有硬件 launch。已拆桨、飞控保持未解锁后，只需在一个终端运行：

```bash
cd /home/orangepi/BoomBoomFly/uav_ws
bash ulg/2026_10_13_1900/analysis/run_latency_test.sh
```

脚本自动加载当前工作区 ROS 环境，依次执行 A/B/C，默认每轮先静置预热 60 秒，再采样测试 180 秒，总计约 12 分钟加节点退出时间。预热阶段也保存采样，但每轮 `result.json` 的统计排除预热。终端会实时提示：预热、继续静置、缓慢移动、放回静置。

A 全程静置；B/C 的测试阶段按三个等长区间进行：静置、缓慢前后/左右移动并转向约 90°、放回静置。脚本会打印动作提示。默认每轮结束自动停止该轮节点并开始下一轮；需要自行准备间隔时增加 `--pause-between`，在同一个终端按回车开始下一轮。

Ctrl-C 会中止整套测试并停止本次启动的进程；已经写入的日志保留。脚本不解锁、不发送电机控制指令、不改变飞控参数；C 会向 PX4 发送健康视觉 odometry。输出自动创建在 `log/vio_latency_test_日期_时间/`，终端显示具体路径。各轮包含 `console.log`、`monitor.log`、`ros/`、`metrics/`、`commands.json`、`result.json`，总目录包含 `results.json`。这里“完成”只表示采样完成，不代表达到飞行条件。

只测其中一轮也可以：

```bash
bash ulg/2026_10_13_1900/analysis/run_latency_test.sh --phases B
```

默认加载 `install/local_setup.bash`。如果实际试飞使用另一份 overlay，先手动加载相同环境，直接执行 Python 入口，避免 shell 包装器重新加载默认安装：

```bash
python3 ulg/2026_10_13_1900/analysis/run_latency_test.py
```

只查看启动命令，不启动硬件：

```bash
bash ulg/2026_10_13_1900/analysis/run_latency_test.sh --dry-run
```

以下双终端命令保留为手动分步方式。

## 终端环境

所有终端先执行：

```bash
cd /home/orangepi/BoomBoomFly/uav_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

如果实际飞行使用额外 overlay，在每个终端继续加载同一份 overlay；保持与试飞一致。当前工作区检查没有 `install/openvins-fork/`，不要直接照搬不存在的安装路径。

下列使用 `log/vio_latency_test_01/`；重复试验时改成 `_02` 等新目录。监测输出目录必须是新目录，避免覆盖证据。

## A：只运行 MAVROS，180 秒

终端 1：

```bash
mkdir -p log/vio_latency_test_01/A/ros
ROS_LOG_DIR="$PWD/log/vio_latency_test_01/A/ros" \
  ros2 launch mavros px4.launch namespace:=/ \
  fcu_url:=serial:///dev/ttyS6:921600 \
  2>&1 | tee log/vio_latency_test_01/A/console.log
```

终端 2，开始监测：

```bash
python3 ulg/2026_10_13_1900/analysis/monitor_latency.py \
  --seconds 180 --phase A_mavros \
  --out log/vio_latency_test_01/A/metrics
```

飞机静置。第一轮没有相机/VIO/bridge 消息是正常的。等待 MAVROS 连接及同步收敛；保留启动阶段记录，不把发现节点或初始化阶段的缺包直接当故障。实际有效观测需有 `/mavros/imu/data_raw` 和 `/mavros/timesync_status` 数据；缺失则先看 console，不要继续用空数据比较。

命令行使用 `namespace:=/` 指定根命名空间。不要写 `namespace:=''`：shell 会去掉空引号，ROS 2 实际收到 `namespace:=`，报 `malformed launch argument` 并立即退出。自动脚本此时会停止监测进程，监测中的中断或 context 无效报错属于退出时的后续现象。

## B：完整视觉链路，禁止向 PX4 回传，180 秒

退出 A 的 MAVROS 后，终端 1：

```bash
mkdir -p log/vio_latency_test_01/B/ros
ROS_LOG_DIR="$PWD/log/vio_latency_test_01/B/ros" \
  ros2 launch uav_bringup d435i_px4_vio.launch.py \
  bridge_enabled:=false vio_verbosity:=INFO \
  2>&1 | tee log/vio_latency_test_01/B/console.log
```

终端 2：

```bash
python3 ulg/2026_10_13_1900/analysis/monitor_latency.py \
  --seconds 180 --phase B_vio_no_output \
  --out log/vio_latency_test_01/B/metrics
```

动作按监测 elapsed 秒计：0～60 秒静置；60～120 秒缓慢手持前后、左右移动并转向约 90°；120～180 秒放回静置。启动前 5 秒保持静止供初始化。面对有自然纹理、距离合适的场景，不遮相机、不快速晃动。不要在这轮中重启节点，否则断流属于人为停止。

`bridge_enabled=false` 只禁止发布到 PX4；桥接适配与健康检查仍运行，因此仍可出现 `faulted=1`。该轮 `/mavros/odometry/out` 无消息是预期行为，不能当作故障。

## C：完整链路并回传，180 秒

确认已拆桨、未解锁；退出 B 的所有节点后，终端 1：

```bash
mkdir -p log/vio_latency_test_01/C/ros
ROS_LOG_DIR="$PWD/log/vio_latency_test_01/C/ros" \
  ros2 launch uav_bringup d435i_px4_vio.launch.py \
  bridge_enabled:=true vio_verbosity:=INFO \
  2>&1 | tee log/vio_latency_test_01/C/console.log
```

终端 2：

```bash
python3 ulg/2026_10_13_1900/analysis/monitor_latency.py \
  --seconds 180 --phase C_vio_output \
  --out log/vio_latency_test_01/C/metrics
```

重复 B 的动作顺序。C 增加实际 odometry 回传和飞控融合负载，最接近此次通信链路。无需解锁。出现提示后继续保留约 15～30 秒记录，观察 RTT 是否恢复而 bridge 仍锁定，再退出。需要另一次复现时新建目录并重新启动，不在同一轮中清除故障。

## 记录内容和判断

每个 `metrics/` 中：

- `samples.jsonl`：约每秒一次，包含各话题该秒频率、接收时 age 的 min/p50/p95/max、接收间隔、时间戳推进间隔、静默时长、重复/倒退时间戳，以及最新健康状态/定位/遥控状态。
- `summary.json`：计数、最大 RTT、最大接收年龄、结束时发布者数量。它不判断是否达到飞行条件。
- 主机信息：每核及总 CPU、各相关进程 CPU（100% 为一个核，可超过 100%）、RSS、可用内存、swap 页计数、温度及 CPU 频率。
- `ros_minus_steady_delta_s`：ROS 时钟增量与单调时钟增量之差，用来提示主机/ROS 时间跳变。

CPU 和进程 CPU 首次采样没有差分值，属于正常情况。CPU load 不是单独判据：一个处理线程或回调堵塞时，总 CPU 不一定到 100%。温度或频率变化需与延迟一起判断，不能单凭降频就认定过热；swap 页计数需看增量。

| 首先出现的现象 | 下一步排查 |
| --- | --- |
| A 静置时 RTT 已持续增长，IMU age 同步增加 | MAVROS 接收/回调、串口队列、主机调度；不能先归因于视觉算法 |
| A 正常，B 的 RTT/IMU age 开始增长，伴随某核高负载、可用内存下降或 swap 增加 | 加入相机和估计器后产生的资源竞争/处理积压 |
| RTT、IMU、相机时间均及时，视觉健康 stamp 不再推进 | 特征跟踪或可接受视觉更新中断；查看 OpenVINS INFO/DEBUG |
| 相机元数据 stamp/接收先停，IMU 继续 | 相机/USB/图像链路；元数据缺失本身不能等同图像断流 |
| B 正常，C 开始增长 | 额外 odometry 回传、串口发送/回调、飞控融合负载 |
| ROS 与单调时钟增量突然分离，或时间戳倒退 | 主机校时/时间基准改变；结合同步 offset 和驱动日志 |
| bridge 故障时 received 继续增加但 published 固定 | 接收流仍在，健康门控锁定转发；具体原因看完整警告前半句 |

参考门限来自当前配置：输入 odometry 年龄 `max_sample_age_s=0.15`；视觉健康观测年龄或有效更新接收间隔 `visual_timeout_s=0.3`。持续接近门限就值得排查；超过门限时核对桥接日志。单纯重复的 health stamp 不会续期。

RTT 通常应保持在基线的几毫秒量级；连续超过 10 ms 或不断增长要检查。RTT 是同步交换的往返时间，不能直接当作 IMU、图像或视觉 age。主机 MAVROS 侧 RTT 和此前 ULog 中的 PX4 侧 RTT 也是两个方向的测量，不能逐样本等同。

监测回调测到的 age 含本监测进程的调度和 DDS 延迟；若该进程自身 report interval 拉长、CPU 占用明显，先控制测量开销再推断原链路。日志中的 age 比正常大，并不自动证明它全部来自串口。

## 没有相机元数据时

检查 `summary.json` 的 `publishers_at_end`。元数据 topic 没有发布者，不能判断相机停帧；仅此时再开新一轮监测，在原命令增加 `--images`，直接记录图像 header 时间及接收间隔，不保存图像像素。额外图像订阅仍会增加 DDS/反序列化负载，需与不订阅图像的结果对比。

若只有 INFO 日志的运行出现异常，用新目录、相同动作复测 `vio_verbosity:=WARNING`，判断日志输出是否影响负载。只有健康更新中断而其他时间正常时，再开 DEBUG 定位特征问题。

## 本次已验证的范围

监测脚本完成无 ROS 的主机采样冒烟检查、重复/停滞时间戳与间隔计算检查，以及本地 ROS 消息导入检查。当前执行环境没有映射 `/dev/ttyS6`，没有启动实际硬件链路。运行 A/B/C 的结果才是硬件验证证据。

单终端入口完成环境加载与 dry-run 检查，并用模拟子进程验证顺序启动、日志保存、正常结束、监测异常退出及 Ctrl-C 后的进程清理。这些检查没有连接飞控或相机。

收集 `log/vio_latency_test_01/` 整个目录即可，优先检查故障前后约 10 秒，按最早变坏的指标定位。地面全部正常后，可做更长时间、重复重启/搬动测试；拆桨测试仍不覆盖带桨振动与飞行闭环。
