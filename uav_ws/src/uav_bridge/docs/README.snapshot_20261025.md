# uav_bridge

ROS 2 节点负责 PX4 IMU 到 OpenVINS 的输入适配，以及 OpenVINS 里程计到单一飞控接口的适配。
本包不实现控制、解锁、飞行模式、任务、PX4 参数配置或相机标定。

```text
PX4 ${fmu_prefix}/out/sensor_combined
  → imu_bridge → /px4/imu → OpenVINS

/ov_msckf/odomimu + /ov_msckf/visual_health
  → odom_bridge → /mavros/odometry/out
                或 ${fmu_prefix}/in/vehicle_visual_odometry
```

## 统一启动与配置

通过 `uav_bringup` 的统一入口启动。飞控后端只从 `uav_control.backend` 读取，并由 launch
传给 `odom_bridge`；桥接节点的 `backend` 默认空值，未注入 `mavros` 或 `dds` 时会拒绝启动。
不要在 `uav_bridge` 配置分区重复选择后端。DDS 前缀也从同一个后端配置分区传入。

```bash
ros2 launch uav_bringup uav.launch.py start_control:=false \
  start_camera:=true start_openvins:=true start_bridge:=true
```

`uav_bridge.enabled` 默认 `false`。启动节点会执行检查、适配并发布诊断，但不会向选定的飞控
接口发布 Odometry。完成机载时间、坐标轴和参考点核对后，才在集中配置中显式启用。
DDS 后端启动 OpenVINS 时会同时启动 `imu_bridge`；MAVROS 后端下 OpenVINS 直接使用
`/mavros/imu/data_raw`，不启动该节点。

`uav_control.backend` 仅可为 `dds` 或 `mavros`。DDS 使用 `${uav_control.dds.fmu_prefix}`，
默认 `/fmu`，同时决定 PX4 IMU 输入和视觉里程计输出话题。MAVROS 输出固定为
`/mavros/odometry/out`。运行期间参数只读，切换后端需重启。

## IMU 输入

`imu_bridge` 以 sensor-data QoS 订阅 `${fmu_prefix}/out/sensor_combined`，并发布
`sensor_msgs/Imu` 到 `/px4/imu`。它将 PX4 FRD 数据按 `(x, -y, -z)` 转为 ROS FLU；拒绝无效、
重复或回退时间戳、无效加速度计相对时间、传感器 clipping 和非有限数值。
`moving_average_samples` 为只读奇数窗口，范围 1 到 9，默认 1；窗口大于 1 时，输出 stamp
取窗口中间样本的时间。

IMU 消息仅将 `orientation_covariance[0]` 设为 `-1`，表示未提供姿态。角速度和线加速度协方差
保持全零，表示未知；本地 OpenVINS 仍从 Kalibr 配置读取 IMU 噪声。桥接不伪造测量方差。

IMU stamp 保留 PX4 样本时刻；工程要求 PX4 配置 `UXRCE_DDS_SYNCT=1`，并让 DDS Agent 与
ROS 主机使用一致系统时钟。节点不再叠加固定时移。这是源码和配置契约，不能替代机载时钟核对。

## Odometry 输入和安全门控

输入契约为 `global` 世界位置和 `imu` 坐标系线速度、角速度。OpenVINS 状态传播中将世界系速度
转换到 IMU 系后输出，因此预测位置增量时必须用姿态将 IMU 系速度转回世界系。源码在
[Propagator.cpp](https://github.com/QU-of-boomboom/open_vins/blob/master/ov_msckf/src/state/Propagator.cpp) 中执行该转换；
不要把 Visualizer 的局部坐标注释当作 Odometry twist 的坐标契约。

每个唯一的新输入在输出限频前都检查时间戳、frame、有限值、四元数范数、完整协方差的对称性和
半正定性、坐标变换后的协方差限制，以及位置/姿态硬 step。位置预测使用最后一个通过预测检查的
原始位置和世界系速度，对当前位置增量与梯形积分比较；被拒绝的样本不会推进比较基线，残差检查
不会因样本间隔变长而跳过。单次预测残差超限会拒绝该样本；三个连续残差超限会暂停已开始的转发。
可选速度限制使用输入 IMU 坐标系线速度模长（旋转到世界系后模长相同），对每个原始样本检查；0 表示禁用。
超限样本会被拒绝，若转发已开始则进入相同的暂停和自动恢复流程。该限制需要用板端原始 VIO 样本
调定，集中配置默认禁用。位置和姿态硬 step 独立检查。

发生视觉或有效 Odometry 超时、时间回退、VIO 会话/初始化代数变化或明显位姿跳变后，节点暂停
输出并继续检查后续 Odometry。恢复必须同时满足：

- 3 个时间戳各自更新、状态为 OK 的视觉健康样本；重复健康时间戳不计数，也不续期。
- 3 个连续有效、时间递增且位姿连续的 Odometry 样本；每个计数样本的时间戳不早于本轮首个
  有效视觉样本。重复 Odometry 时间戳不重新累计预测异常，也不打断恢复计数。

恢复期间收到无效的新健康或 Odometry 样本，或健康/输入看门狗再次超时，会清除本轮计数。
暂停时仍运行样本检查，不会因视觉样本数尚未达到 3 而跳过 Odometry 校验。恢复后发布当前样本并
重新建立原始输入基线，不回放停发期间的数据、不更改坐标参数，也不需要重启节点。

本包没有 startup grace，也没有 `poseimu` 后备。视觉健康必须来自修复版 OpenVINS 的实际接受
视觉观测时间；纯 IMU 传播、状态发布或重复心跳不能续期。它只能说明近期接受过视觉更新，不能
证明定位无漂移或 PX4 已融合。自动恢复 VIO 转发不会恢复任务、解锁或切换飞行模式。

当前工程标定见 [相机—IMU 标定](https://github.com/QU-of-boomboom/uav_ws/blob/main/config/openvins/d435i_px4/kalibr_imucam_chain.yaml)、
[IMU 标定](https://github.com/QU-of-boomboom/uav_ws/blob/main/config/openvins/d435i_px4/kalibr_imu_chain.yaml) 和
[OpenVINS 配置](https://github.com/QU-of-boomboom/uav_ws/blob/main/config/openvins/d435i_px4/estimator_config.yaml)。`T_cam_imu` 满足
`p_C = R_CI p_I + t_CI`；本地 OpenVINS 在需要 `T_imu_cam` 时自动求逆，Bridge 不重复乘相机
外参。`calib_cam_extrinsics: false` 表示固定使用所给外参，不表示没有外参。IMU 标定文件中的
`T_i_b=I` 是 Kalibr 的参考 IMU0 到 IMU0 关系，既不是 OpenVINS 使用的机体质心测量，也不是
IMU 到飞行器重心的安装标定；当前 OpenVINS 没有读取这个字段。

Bridge 默认保留 IMU 原点和坐标轴：`rotation_body_imu_xyzw` 为 `R_BI`，默认单位旋转；
`position_body_in_imu_m` 表示从 IMU 原点到输出测量点的向量，默认零向量。
这不是飞机质心的安装测量。需要改输出参考点时，必须使用独立测得的机体外参，不能从相机平移
推导。`world_yaw_rad` 只旋转局部水平轴，不建立真北观测；`vio_odom` 不声明北向对齐。

输入 pose 协方差顺序为 `[p_G, theta_I]`，twist 为 `[v_I, omega_I]`，均按行优先 6×6 处理。
转换包含杆臂 Jacobian。对 pose 的位置—姿态交叉块范数 `s`，实现将位置协方差加上 `sI`，
并以 `(最大姿态特征值+s)I` 替代姿态块后清除交叉项。这是保守的半正定上界近似，不是完整
相关性保留或统计最优转换。MAVROS 输出还发布两条零平移的标准轴 TF；它们不是安装外参或动态位姿。

DDS `VehicleOdometry` 每组只携带 3 个方差，不传完整交叉相关项。完整矩阵用于公共输入有效性、
特征值门控和几何协方差传播；DDS 线上数据不代表完整的统计最优协方差变换。

## 后端消息

| 后端 | 输出 | 坐标与时间语义 |
| --- | --- | --- |
| MAVROS | `/mavros/odometry/out` (`nav_msgs/Odometry`) | 发布适配后的世界/机体 frame；MAVROS 负责后续 MAVLink 坐标转换。 |
| DDS | `${fmu_prefix}/in/vehicle_visual_odometry` (`px4_msgs/VehicleOdometry`) | `pose_frame=FRD` 支持局部任意航向；速度使用合法的 `VELOCITY_FRAME_BODY_FRD`。在 `UXRCE_DDS_SYNCT=1` 的系统时钟契约下，`timestamp_sample` 原样使用输入样本时刻换算成微秒，不重复加偏移；`timestamp=0` 由 PX4 处理接收时刻。 |

DDS 写入前先检查 float32 范围；位置、姿态和速度协方差的 9 个方差缩窄后必须有限且严格大于
零，否则拒绝，不填充人工 epsilon。`quality` 保持为 0：视觉健康契约不提供数值质量分，且本地
[`VehicleOdometry.msg`](https://github.com/PX4/px4_msgs/blob/86d8239e962f6939e05c3737784f60c02fa884db/msg/VehicleOdometry.msg) 将该字段标为 unused。部署时仍需核对 PX4 的 `EKF2_EV_QMIN` 配置及实际
接收行为；本包不改飞控参数。

## 诊断与参数

诊断发布到 `/uav_bridge/diagnostics`，约每秒一个 `DiagnosticArray`，唯一状态名为
`uav_bridge/odom_bridge`。`faulted=true` 表示正在暂停并等待恢复。`received`、`adapted`、
`published`、`rejected` 和 `rate_limited` 用于区分输入、适配、调用 publisher、拒绝和限频；
`published` 不证明 MAVROS/PX4 收到或 EKF2 融合。
`last_process_ms` 只计 `adapt_odometry` 内的检查、门控和几何转换，不包括限频、后端坐标转换、
序列化、传输或 OpenVINS。

参数在运行期只读。默认值保持当前源码与集中配置中的原值：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `enabled` | `false` | 是否向选定后端发送适配后 Odometry |
| `input_topic` / `visual_health_topic` | `/ov_msckf/odomimu` / `/ov_msckf/visual_health` | OpenVINS 里程计和视觉健康 |
| `input_frame_id` / `input_child_frame_id` | `global` / `imu` | 输入 frame 约束 |
| `output_frame_id` / `output_child_frame_id` | `vio_odom` / `vio_body` | MAVROS 输出 frame |
| `rotation_body_imu_xyzw` / `position_body_in_imu_m` | 单位旋转 / 零向量 | IMU 到输出测量点的配置变换 |
| `world_yaw_rad` | `0.0` | 局部水平轴旋转 |
| `publish_rate_hz` | `40.0` | 输出限频；不限制原始样本检查 |
| `max_sample_age_s` / `max_future_s` | `0.15` / `0.01` | 输入时间界限 |
| `visual_timeout_s` / `input_timeout_s` | `0.3` / `0.2` | 健康和输入看门狗 |
| `max_position_step_m` / `max_attitude_step_rad` | `1.0` / `0.75` | 相邻位置/姿态硬 step 限制 |
| `max_position_variance_m2` / `max_attitude_variance_rad2` | `0.16` / `0.36` | 转换后协方差块最大特征值 |
| `max_velocity_variance_m2_s2` / `max_angular_velocity_variance_rad2_s2` | `1.44` / `4.0` | 转换后协方差块最大特征值 |
| `max_position_prediction_error_m` | `0.12` | 通过门控样本的位置增量与世界速度积分的最大残差 |
| `max_vio_speed_m_s` | `0.0` | 可选线速度模长上限，单位 m/s；0 禁用，启用前需板端定标 |
| `moving_average_samples` | `1` | IMU 奇数样本平均窗口，范围 1 到 9 |

## 源码参考与验证边界

桥接直接发布 `px4_msgs::msg::VehicleOdometry`，不链接 px4-ros2-interface-lib。参考源码快照为
[`Unmanned-Aerial-Vehicle` `f4ad78e`](https://github.com/dofarifa/Unmanned-Aerial-Vehicle/tree/f4ad78e83a03627eae62552dcf1d7eddc16fbf2e)。
该参考项目固定的接口库 gitlink 为 `4a3370f084ac6f1ef001a4afa2b007845ffd0837`；此处只用于
核对接口契约，不作为本包运行依赖。本工作区 [`px4_msgs` `86d8239`](https://github.com/PX4/px4_msgs/tree/86d8239e962f6939e05c3737784f60c02fa884db/.)
对应完整提交 `86d8239e962f6939e05c3737784f60c02fa884db`。

本轮只做源码、配置和文档静态核对；未编译、未运行测试、未启动 ROS、未操作硬件。当前结果不
证明 DDS/MAVROS 传输、部署时钟对齐、PX4 接收、EKF2 融合或飞行效果。
新增 `test/test_vio_safety_gate.cpp` 回归源码，覆盖异常基线、持续偏移、速度限制和恢复计数；
通过 `BUILD_TESTING` 注册为 CTest，本轮未执行。

## 历史测量

旧包的构建和 D435i 短台架数据已合并到 [Position 历史记录](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/Position%E5%90%AF%E5%8A%A8%E4%B8%8E%E9%AA%8C%E8%AF%81.md)。
历史包名、launch、测试数和采样结果不作为当前迁移版的验收证据；当前构建与回归入口见
[手动验证](https://github.com/QU-of-boomboom/uav_ws/blob/main/docs/OpenVINS%E6%89%8B%E5%8A%A8%E7%BC%96%E8%AF%91%E4%B8%8E%E9%AA%8C%E8%AF%81.md)。
