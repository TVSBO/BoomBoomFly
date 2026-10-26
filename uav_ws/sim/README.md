# UAV 仿真

更新日期：2026-10-15。这里只保存自有入口和 [2026-09-19 历史验证](PX4_SITL_起飞悬停降落验证.md)。
PX4/Gazebo、Agent 或 MAVROS 需单独准备，本地当前树不包含 PX4 源码和当时的 SITL 日志。
当前入口只做源码核对，未重新构建或运行。

## 当前入口

[sitl_flight.launch.py](uav_bringup/launch/sitl_flight.launch.py) 复用统一 uav.launch.py，
只传入 config_file 并设置 start_mission=true。它不启动 PX4、Gazebo、Agent 或 MAVROS。
当前 bringup CMake 只安装包内 launch，**没有安装该 sim 文件**；在完整工作区根目录按文件路径启动：

```bash
# 先准备仿真与通信，并加载匹配的控制/任务/bringup overlay
ros2 launch "$PWD/sim/uav_bringup/launch/sitl_flight.launch.py" \
  config_file:=/absolute/path/to/uav_sitl.yaml
```

配置副本来自 [uav.yaml](https://github.com/QU-of-boomboom/uav_bringup/blob/main/config/uav.yaml)：
MAVROS 仿真显式设 uav_control.backend=mavros，DDS 仿真显式设 dds；
仿真与控制核对后再将选中后端的 read_only 设为 false。该入口会启动自动任务，不用于只读检查。
旧 `ros2 launch uav_bringup sitl_flight.launch.py` 包内命令不适用于当前安装规则。

构建与嵌套接口发现见 [手动构建](../docs/OpenVINS手动编译与验证.md)；故障用例见
[DDS 流程](../docs/DDS开发流程.md)。当前控制/任务没有注册包内测试，不能沿用旧测试名声称已覆盖。
同次试验只运行一个控制后端和一个任务目标来源；各节点需要同一版本的 uav_interfaces。

历史 MAVROS 连接为 `udp://:14540@127.0.0.1:14580`、PX4 gz_x500，
不代表当前端口、固件和参数已确认。正常任务、任务停止和控制停止分别验证；后两者分别对应主动 LAND
与 PX4 Offboard-loss，不能互相替代。SITL 通过不证明 VIO、真实飞控或自由飞行通过。
