# OpenVINS — BoomBoomFly 工程版本

> TVSBO 单仓库版：本目录源码已随主仓库克隆获取，无需再克隆独立功能仓库。

本仓库来自 BoomBoomFly/open_vins 的 2026-10-25 实际工作目录，含本地未提交修改。
用于 ROS 2 Humble、D435i 双目红外图像与 PX4 IMU 的室内视觉惯性定位。
上游项目为 https://github.com/rpng/open_vins ，原有许可证、版权和文件头保持原样。
本仓库是无旧 Git 历史的源码快照导入，不是声称保留完整历史的 fork。

## 工程接口

- /ov_msckf/odomimu：里程计，供 uav_bridge 适配。
- /ov_msckf/visual_health：实际接受视觉观测的健康状态；纯 IMU 传播不表示视觉恢复。
- 工程输入、标定及估计器配置由 uav_ws/config/openvins/d435i_px4 管理。
- DDS 后端通过 imu_bridge 提供 /px4/imu；MAVROS 后端使用 /mavros/imu/data_raw。

导出时未提交修改涉及 ov_msckf/launch/subscribe.launch.py、ROS2Visualizer.cpp/.h 和
run_subscribe_msckf.cpp。完整基线及状态见 SOURCE.md 和主工程 reproduction/repositories.lock.json。

## 编译与资料

从 [主工程](https://github.com/TVSBO/BoomBoomFly) 恢复完整依赖后，运行
bash reproduction/build_uav.sh。该构建使用 ENABLE_ROS=ON 和 ROS 2 Humble。
原始上游说明保留在 docs/ReadMe.upstream.md；本次未执行完整构建或定位精度验收。
