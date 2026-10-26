# Orange Pi 5 Plus 4GB：D435i / PX4 IMU 接入配置

更新日期：2026-10-15。本文维护板端驱动准备、标定和资源约束；运行步骤见
[手动构建与验证](OpenVINS手动编译与验证.md)，测量及接收/融合验收见 [Position 验证](Position启动与验证.md)。
本次未连接目标板，下面的历史快照和候选驱动组合不表示当前安装或流稳定性已验收。

## 板端与链路

目标为 Orange Pi 5 Plus 4GB、Ubuntu 22.04、ROS 2 Humble、D435i 双目红外与外部 PX4 IMU。
后端由 [集中配置](https://github.com/QU-of-boomboom/uav_bringup/blob/main/config/uav.yaml) 统一选择；当前默认 DDS，MAVROS 仍受支持：

| 后端 | IMU 路径 | VIO 回传 |
| --- | --- | --- |
| DDS | SensorCombined → imu_bridge，FRD→FLU → `/px4/imu` | `${fmu_prefix}/in/vehicle_visual_odometry` |
| MAVROS | `/mavros/imu/data_raw`，沿用 MAVROS 的 FLU 数值 | `/mavros/odometry/out` |

两个后端控制均默认只读，桥接默认关闭输出。相机内置 IMU 未用于本配置；如需改用它，须重新核对时间、
消息组织及外参，不能沿用飞控 IMU 标定。通信与供数方式以当前源码为准，不再使用旧独立后端 YAML。

用户 2026-10-07 系统快照：aarch64、Orange Pi 1.2.2 Jammy/Ubuntu 22.04.5、
内核 `6.1.99-rockchip-rk3588`；内存 3.8 GiB、available 约 3.0 GiB、swap 1.9 GiB。
D435i 枚举为 `8086:0b3a`；相机接口速率仍需 `lsusb -t` 复核，不能用根 hub 的 5000M 代替相机数据。
低 free 但高 cache 不表示内存耗尽；工具不在 PATH 也不能单凭此认定 SDK 未安装。

板端核对：

```bash
uname -m
uname -r
cat /etc/os-release
free -h
lsusb -t
source /opt/ros/humble/setup.bash
ros2 pkg prefix realsense2_camera
ros2 pkg prefix cv_bridge
dpkg-query -W 'librealsense*' 'ros-humble-realsense2*' 2>/dev/null
```

稳定供电需覆盖板与外设负载，检查散热、降频、USB 重连及持续换页；RViz 放 PC，日志优先写 eMMC/NVMe。
厂商内核不等于通用 Ubuntu 内核，不直接套用 x86 内核补丁。OpenVINS 当前没有可直接启用的 RK3588 NPU 开关，
不能由 TOPS 推定 VIO 性能。USB 枚举、SDK 识别、ROS 出图与 VIO 稳定性分别验收。

## RealSense 驱动准备

沿用之前选定的 SDK `v2.58.1` 与 wrapper `4.58.1` 候选，不表示已验证兼容或是最新推荐版本。
先核对已有安装，避免源码 SDK 与系统包混用。第三方目录需自行准备，本工作树不包含驱动源码。
以下保留板端 RSUSB 构建方案；wrapper 和相机支持的 profile、固件及时间戳仍需实际验证。


以下命令供 Orange Pi 上执行，前提是已安装 ROS 2 Humble，且两个源码目录已下载。按顺序执行，每一步成功后继续。助手只核对源码和文档，没有在板端执行构建或采流。

1. 设定实际工作区路径，核对版本，安装 SDK 编译依赖：

   ```bash
   uav_ws_dir="$HOME/uav_ws"
   rs_sdk_src="$uav_ws_dir/upstream/librealsense"
   rs_ros_src="$uav_ws_dir/src/thirdparty/realsense-ros"

   git -C "$rs_sdk_src" describe --tags --exact-match
   git -C "$rs_ros_src" describe --tags --exact-match

   sudo apt update
   sudo apt install -y build-essential cmake pkg-config \
     libusb-1.0-0-dev libudev-dev libssl-dev v4l-utils \
     python3-colcon-common-extensions python3-rosdep
   ```

   两个标签应分别为 `v2.58.1` 与 `4.58.1`。关闭占用相机的程序，拔下 D435i，再安装设备权限规则：

   ```bash
   cd "$rs_sdk_src"
   ./scripts/setup_udev_rules.sh
   ```

2. 采用 RSUSB 后端，编译并安装 SDK：

   ```bash
   cmake -S "$rs_sdk_src" -B "$rs_sdk_src/build-rsusb" \
     -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_INSTALL_PREFIX=/usr/local \
     -DCMAKE_INSTALL_LIBDIR=lib \
     -DFORCE_RSUSB_BACKEND=ON \
     -DBUILD_EXAMPLES=OFF \
     -DBUILD_GRAPHICAL_EXAMPLES=OFF \
     -DBUILD_GLSL_EXTENSIONS=OFF \
     -DBUILD_TOOLS=ON \
     -DBUILD_WITH_DDS=OFF \
     -DBUILD_ROSBAG2=OFF \
     -DCHECK_FOR_UPDATES=OFF

   cmake --build "$rs_sdk_src/build-rsusb" --parallel 1
   sudo cmake --install "$rs_sdk_src/build-rsusb"
   sudo ldconfig

   PKG_CONFIG_PATH=/usr/local/lib/pkgconfig pkg-config --modversion realsense2
   ```

   版本应输出 `2.58.1`。这是针对当前厂商内核的板端验证方案，无需运行 Ubuntu 通用内核补丁脚本。关闭的是 SDK 的相机 DDS 与内部 ROS bag2 支持，不影响 ROS wrapper 发布话题、PX4 DDS 或 `ros2 bag record`。保留 `BUILD_TOOLS` 可提供 `rs-enumerate-devices`。[SDK 构建选项](https://github.com/realsenseai/librealsense/blob/v2.58.1/CMake/lrs_options.cmake)、[工具构建入口](https://github.com/realsenseai/librealsense/blob/v2.58.1/tools/CMakeLists.txt)

   重新插入 D435i，执行 `rs-enumerate-devices` 和 `lsusb -t`，核对设备、固件、支持的 profile 及实际 USB 链路。

3. 安装 ROS 依赖，仅构建相机相关三个包：

   ```bash
   source /opt/ros/humble/setup.bash

   if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
     sudo rosdep init
   fi
   rosdep update
   rosdep install --from-paths \
     "$rs_ros_src/realsense2_camera" \
     "$rs_ros_src/realsense2_camera_msgs" \
     "$rs_ros_src/realsense2_description" \
     --ignore-src --rosdistro humble --skip-keys librealsense2 -y

   cd "$uav_ws_dir"
   export CMAKE_BUILD_PARALLEL_LEVEL=1
   export MAKEFLAGS="-j1"

   colcon --log-base log/realsense build \
     --base-paths "$rs_ros_src" \
     --build-base build/realsense \
     --install-base install/realsense \
     --packages-select realsense2_camera_msgs realsense2_camera realsense2_description \
     --executor sequential \
     --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
       -Drealsense2_DIR=/usr/local/lib/cmake/realsense2

   source "$uav_ws_dir/install/realsense/local_setup.bash"
   ros2 pkg prefix realsense2_camera
   ```

   跳过 `librealsense2` 的 rosdep 安装，使用刚构建的 SDK；显式指定其 CMake 路径，并用独立构建输出避免与已有 UAV 构建混用。[ROS wrapper 源码安装说明](https://github.com/realsenseai/realsense-ros/blob/4.58.1/README.md#installation-on-ubuntu)

4. 核对设备支持标定所用 `848x480x30` 后，单独验证左右红外采流：

   ```bash
   ros2 launch realsense2_camera rs_launch.py \
     enable_infra1:=true enable_infra2:=true \
     enable_color:=false enable_depth:=false \
     enable_gyro:=false enable_accel:=false \
     depth_module.infra_profile:=848x480x30 \
     depth_module.emitter_enabled:=0 \
     pointcloud.enable:=false align_depth.enable:=false
   ```

   保留当前飞控 IMU 方案。另开终端，source Humble 和上述安装目录后，通过 `ros2 topic list` 确认实际话题，再测量两路图像频率；构建成功、设备枚举和采流结果分别记录，不能代替 OpenVINS 与飞行验证。

## 相机输入与标定

当前统一入口使用左右红外 `848x480x30`，关闭 RGB、深度和相机 IMU，`depth_module.emitter_enabled=0`。
单独测试相机后先停止该进程，再运行统一入口；不能让两套驱动同时占用设备。
核对左右曝光、运动模糊、stamp 配对和 QoS。驱动内 `enable_sync` 不能同步外部飞控 IMU。

| 文件 | 职责 |
| --- | --- |
| [estimator_config.yaml](../config/openvins/d435i_px4/estimator_config.yaml) | 估计器、跟踪负载、初始化及其他标定文件的相对路径 |
| [kalibr_imucam_chain.yaml](../config/openvins/d435i_px4/kalibr_imucam_chain.yaml) | 双目内参/畸变/分辨率、T_cam_imu、timeshift_cam_imu |
| [kalibr_imu_chain.yaml](../config/openvins/d435i_px4/kalibr_imu_chain.yaml) | IMU 噪声、采样率及基准话题；launch 按后端覆盖 imu0_rostopic |

这些是带 `%YAML:1.0` 的 OpenCV YAML，由 OpenVINS `config_path` 读取，不是 ROS 参数文件。
输入图像的校正状态须与内参/畸变模型一致，避免对已校正图像重复应用原始畸变。
CameraInfo 不能提供相机到外部飞控 IMU 的安装外参；外参应在刚性固定的实际装配上核对。

`T_cam_imu` 满足 `p_C=R_CI p_I+t_CI`；OpenVINS 请求反向变换时自动取逆，Bridge 不重复施加相机外参。
`calib_cam_extrinsics=false` 表示固定使用给定外参。`T_i_b=I` 是 Kalibr 参考 IMU 的关系，
不证明 IMU 位于飞机重心。输出参考点和机体轴向须独立测量，不能从相机平移推导。

## IMU 与时间

DDS imu_bridge 使用 SensorCombined 的 gyro timestamp，检查有效相对时间、clipping、重复/回退和非有限值。
加速度真实时刻为 timestamp 加 accelerometer_timestamp_relative；当前适配没有独立重采样，不能声称两传感器时刻已严格对齐。
默认平均窗口 1，可选奇数 1–9，窗口大于 1 时 stamp 取中心样本。噪声来自 Kalibr 配置，不能照搬参考设备。

角速度单位 rad/s，加速度 m/s²；保留重力相关测量项，不使用飞控融合姿态替代原始惯性数据。
YAML 的 update_rate 不能提高真实发布频率；核对实际频率、最大间隔、数据年龄和传输带宽。
DDS 要求 `UXRCE_DDS_SYNCT=1` 与系统时钟，不重复叠加 offset；MAVROS 路径也须核对相机/IMU 同一时基。
不用 `now()` 改写源采样时间。固定时间偏移、随机通信/调度抖动和 VIO 计算延迟是不同问题，不能靠单个 delay 参数互相掩盖。

## 当前 4GB 估计配置

| 配置 | 当前值 |
| --- | --- |
| 双目 KLT、总特征/相机数 | 120 / 2，即每相机目标 60 |
| max_clones / max_slam / max_slam_in_update | 8 / 20 / 20 |
| track_frequency / downsample_cameras | 31.0 Hz / true |
| num_opencv_threads / multi_threading_subs | 2 / false |
| init_window_time / init_wait_for_jerk / init_dyn_use | 5.0 s / false / false |
| try_zupt | false，静止初始化后继续视觉处理 |
| 相机内参/外参/时间偏移在线标定 | 关闭，使用所给配置 |

驱动及标定保持 848×480，估计器内部降至 424×240 并同步缩放内参；不在驱动再降分辨率。
减少像素/特征和窗口只说明负载配置变化，不是板端 CPU、时延或稳健性改善的测量结果。
31 Hz 是跟踪上限，不能增加相机帧率；单线程直接处理队列仍会占用 IMU 回调。
启动时先覆盖静止初始化窗口，随后手持检查运动方向、尺度、速度和实际视觉更新时间。
如自定义配置启用 ZUPT，纯零速更新仍不续期 Bridge 的真实视觉健康。

2026-10-22 针对 D435 约 0.05004 m 基线补充特征三角化配置：
`fi_max_dist=5.0`、`fi_max_baseline=100.0`、`fi_max_cond_number=40000.0`。
默认 baseline ratio 为 40，静止时仅靠双目基线会限制约 2 m 以外的特征；本机场景实测
121 个有效立体匹配中，73 个位于 2～4 m，原配置仅建立 2 个稳定 SLAM 地标。
这些参数控制特征三角化，不是飞行范围，也不改变 Bridge 的 0.16 m² 位置方差门限、
0.36 rad² 姿态方差门限或像素创新检查。当前只验证未解锁静止条件，不能代替运动或自由飞行验证。
配置修改前备份和地面对比数据位于工作区外层
`log/dds_local_heading_20261022/covariance_audit/`。

记录 CPU、RSS、温度、频率、swap、相机/IMU 间隔、视觉年龄及 OpenVINS INFO 的耗时与 behind 指标。
使用同一数据比较配置，避免通过放宽超时掩盖积压。当前门控边界及杆臂/协方差语义见
[uav_bridge](https://github.com/QU-of-boomboom/uav_vio_bridge/blob/main/README.md)；真实 PX4 接收和融合另行验收。
