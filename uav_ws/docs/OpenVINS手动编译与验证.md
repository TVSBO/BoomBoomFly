# 手动构建与 VIO 验证

更新日期：2026-10-15。命令从实际 `uav_ws` 根目录执行，适用于 ROS 2 Humble。
以下是后续人工步骤，本次未执行构建、测试、ROS 或硬件操作。板端驱动准备见
[Orange Pi 接入配置](OrangePi5Plus_OpenVINS接入配置.md)。

## 源码与构建

OpenVINS 位于 `src/thirdparty/open_vins`，由 BoomBoomFly fork 的 `master` 维护，不能给当前树重复应用旧补丁。
先检查所有参与仓库的 `git status` 与 HEAD；工作区提交不包含独立源码仓库的未提交内容。
板端 4GB 使用串行构建，独立输出避免旧 v2.7 缓存。新终端先只加载 Humble：

```bash
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --log-base log/vio-current build \
  --base-paths src/thirdparty/px4_msgs src/thirdparty/open_vins src/uav_bridge src/uav_bringup \
  --packages-select px4_msgs ov_core ov_init ov_msckf uav_bridge uav_bringup \
  --executor sequential --build-base build/vio-current --install-base install/vio-current \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON -DBUILD_TESTING=ON
```

同时重建 ov_core、ov_init、ov_msckf；bridge 依赖的 px4_msgs 已显式包含。
主要依赖为 Eigen、Boost、Ceres、OpenCV/cv_bridge、image_transport、message_filters、tf2、diagnostic_msgs；
测试还需 ament_cmake_gtest/ament_cmake_pytest，离线分析需 rosbag2_py、rosidl_runtime_py 和对应消息包。
OpenCV 与 cv_bridge 必须兼容，不混入 ROS 1 环境。
本命令不构建控制/任务、MAVROS 或 RealSense；运行相应功能前须有匹配安装。

如需单独重建控制/任务，显式发现嵌套接口；DDS-only 示例：

```bash
MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --log-base log/control-current build \
  --base-paths src/uav_control/uav_interfaces src/uav_control src/uav_mission \
    src/uav_bringup src/thirdparty/px4_msgs \
  --packages-select px4_msgs uav_interfaces uav_control uav_mission uav_bringup \
  --executor sequential --build-base build/control-current --install-base install/control-current \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_MAVROS_INTERFACE=OFF
```

MAVROS 后端需可用的 mavros_msgs，并以 `BUILD_MAVROS_INTERFACE=ON` 构建。
切换模式使用独立输出目录。全源发现使用 `--base-paths src/uav_control/uav_interfaces src`。

## 当前离线回归入口

在 VIO 构建成功后执行；只列当前 CMake 确实注册的测试，不沿用已删除的旧测试名：

```bash
source install/vio-current/local_setup.bash
ctest --test-dir build/vio-current/ov_msckf -R '^fast_propagation_test$' --output-on-failure
ctest --test-dir build/vio-current/uav_bridge -R '^test_vio_safety_gate$' --output-on-failure
ctest --test-dir build/vio-current/uav_bringup \
  -R '^(vio_tools_test|vio_bag_analyzer_test)$' --output-on-failure
```

分别检查预测时基、有效基线/速度限制/恢复计数、诊断配置契约及离线分析。
控制/任务当前 CMake 未注册测试；历史通过结果不能视为当前覆盖。无匹配测试也不算通过。
C++ 编译、测试、真实 bag 解析、ROS 交付及飞控融合分别记录。

## 核对安装与 overlay

新终端先 Humble，再加载确实需要的驱动/控制 overlay，最后加载刚构建的 VIO overlay：

```bash
source /opt/ros/humble/setup.bash
# 如驱动或控制位于其他安装，先 source 该安装的 local_setup.bash。
source install/vio-current/local_setup.bash
ros2 pkg prefix ov_core
ros2 pkg prefix ov_init
ros2 pkg prefix ov_msckf
ros2 pkg prefix uav_bridge
ros2 pkg prefix uav_bringup
ros2 launch uav_bringup uav.launch.py --show-args
```

核对五个包来自预期目录，避免旧安装残留。`--show-args` 只验证入口解析。
launch 默认读取安装目录的 uav.yaml；编辑源码 YAML 不会自动改变已安装的配置。
以下使用源码集中配置的绝对路径，实际运行仍应记录该文件内容。

## 只读台架、录制和离线报告

先准备所选后端的 PX4/Agent 或 MAVROS 通信；统一入口不自动启动 PX4、Agent 或 Gazebo。
确认配置保持控制只读、桥接关闭输出，并避免重复相机/估计器/桥接进程：

```bash
mkdir -p log/vio-bench
ros2 launch uav_bringup uav.launch.py config_file:="$PWD/src/uav_bringup/config/uav.yaml" \
  start_control:=false start_camera:=true start_openvins:=true start_bridge:=true \
  vio_respawn:=false vio_verbosity:=INFO 2>&1 | tee log/vio-bench/launch.log
```

其他终端加载相同环境，在同一工作区执行：

```bash
ros2 run uav_bringup check_vio.py --seconds 20 --backend-file src/uav_bringup/config/uav.yaml
ros2 param get /odom_bridge enabled
ros2 run uav_bringup record_vio.py --backend-file src/uav_bringup/config/uav.yaml \
  --output bags/vio_bench_01
```

录制覆盖静止、已知距离手持平移/转动、遮挡、恢复、估计器停止/重启；bag 目录须不存在。
OpenVINS INFO 耗时在终端 stdout，不能仅靠 `/rosout`，因此同时保留 launch.log。
录制本身也会增加存储负载，比较录制前后的间隔与消息年龄。
停止录制后只读分析：

```bash
ros2 run uav_bringup analyze_vio_bag.py bags/vio_bench_01 \
  --backend-file src/uav_bringup/config/uav.yaml --output reports/vio_bench_01.json
```

只有确认整段物理静止时才加 `--stationary`。报告拒绝覆盖已有文件；分位数使用最近 4096 个数值，
时间线保留最多 5000 个事件并报告截断。源时间与 bag 时间分开；二者的差不是端到端延迟。
相邻原始样本残差及输入协方差对角线分析不复现 C++ 完整门控，也不能证明 EKF2 融合。

## 后续验收

方向、安装、时间与视觉质量核对后，再独立评估启用回传，并用 `check_vio.py --require-output` 检查实际 ROS 输出。
新配置应同时传给 launch/check/record/analyze，不能混用源码配置和旧安装配置。
板端完整判据和历史测量见 [Position 验证](Position启动与验证.md)，参数含义见
[uav_bridge](https://github.com/QU-of-boomboom/uav_vio_bridge/blob/main/README.md)。新速度门限默认禁用，其他门限也仍待板端数据调定。

本轮不增加发布型回放。后续若用 `ros2 bag play`，须同时使用独立 ROS_DOMAIN_ID，并把所有 MAVROS/DDS
飞控写入和控制话题重映射到隔离命名空间；仅改域或仅映射部分命令不构成隔离。
