# uav_bringup

> TVSBO 单仓库版：本目录源码已随主仓库克隆获取，无需再克隆独立功能仓库。

启动编排与集中部署配置。ROS 2 Humble / PX4 v1.17.0，2026-10-25 源码快照，2026-10-26 分类发布。

## 实际实现

唯一常规入口为 launch/uav.launch.py，集中配置为 config/uav.yaml。
配置由 launch 按节点分区提取，不能整份作为 --params-file 传给节点。
当前 backend=dds、DDS read_only=false、uav_bridge.enabled=true、mission_profile=grid_s。

默认只启动控制节点。Agent、相机、OpenVINS、桥接和任务均由独立参数选择。

```bash
ros2 launch uav_bringup uav.launch.py --show-args
# 已完成部署核对后启动定位与控制链路，不启动自动任务
ros2 launch uav_bringup uav.launch.py start_agent:=true \
  agent_executable:="$PWD/tools/micro_xrce_dds_agent/install/bin/MicroXRCEAgent" \
  start_camera:=true start_openvins:=true start_bridge:=true
# 另一个终端复用已启动的链路，只启动任务，等待遥控器解锁
ros2 launch uav_bringup uav.launch.py start_control:=false start_mission:=true mission_profile:=grid_s
```

以上 Agent 路径按主工程根目录启动时填写。默认串口 /dev/ttyS6，波特率 921600；需与目标设备一致。
mission_profile 可选 config、hover、square、grid_s；任务已解锁时启动会退出。
配置仍保留源设备标定与参数，本次没有改变飞行算法、串口或任务参数。

## 获取、编译与配置

请从 [主工程](https://github.com/TVSBO/BoomBoomFly) 获取完整工作区，使用其
Scripts/restore_dependencies.py 和 reproduction/build_uav.sh。
常规启动统一使用 ros2 launch uav_bringup uav.launch.py；backend 与参数来自 uav_bringup/config/uav.yaml。
定位标定在 [uav_ws](https://github.com/TVSBO/BoomBoomFly/tree/main/uav_ws)/config/openvins/d435i_px4。

## 来源与验证边界

原作者、基线和许可证说明见 SOURCE.md。本次分类发布不代表重新编译、通信、融合或飞行验收通过。
原包详细说明已保留为 docs/README.snapshot_20261025.md，其中历史默认值与测试记录按记录日期理解。
