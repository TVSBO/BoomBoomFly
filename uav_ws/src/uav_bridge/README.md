# uav_bridge

> TVSBO 单仓库版：本目录源码已随主仓库克隆获取，无需再克隆独立功能仓库。

PX4 IMU 与 OpenVINS 里程计桥接。ROS 2 Humble / PX4 v1.17.0，2026-10-25 源码快照，2026-10-26 分类发布。

## 实际实现

远程仓库名为 uav_vio_bridge，本地目录及 ROS 包名为 uav_bridge。

```text
/fmu/out/sensor_combined → imu_bridge → /px4/imu → OpenVINS
/ov_msckf/odomimu + /ov_msckf/visual_health → odom_bridge
  → DDS /fmu/in/vehicle_visual_odometry 或 MAVROS /mavros/odometry/out
```

包含时间戳、完整协方差、坐标转换、视觉健康、位姿跳变和预测残差检查。
异常时暂停转发，满足健康与连续有效里程计条件后恢复；恢复转发不恢复旧任务。
当前集中配置 enabled=true；节点代码的默认值与部署配置可能不同。
test/test_vio_safety_gate.cpp 经 BUILD_TESTING 注册，本次未运行。

## 获取、编译与配置

请从 [主工程](https://github.com/TVSBO/BoomBoomFly) 获取完整工作区，使用其
Scripts/restore_dependencies.py 和 reproduction/build_uav.sh。
常规启动统一使用 ros2 launch uav_bringup uav.launch.py；backend 与参数来自 uav_bringup/config/uav.yaml。
定位标定在 [uav_ws](https://github.com/TVSBO/BoomBoomFly/tree/main/uav_ws)/config/openvins/d435i_px4。

## 来源与验证边界

原作者、基线和许可证说明见 SOURCE.md。本次分类发布不代表重新编译、通信、融合或飞行验收通过。
原包详细说明已保留为 docs/README.snapshot_20261025.md，其中历史默认值与测试记录按记录日期理解。
