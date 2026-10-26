# UAV 工作区

> TVSBO 单仓库版：本目录源码已随主仓库克隆获取，无需再克隆独立功能仓库。

ROS 2 Humble 室内无人机工作区，负责标定、配置、仿真及工程验证文档。
本仓库来自 2026-10-25 Orange Pi 源码快照，2026-10-26 分类发布。

## 技术路线与当前配置

D435i 双目红外图像与 PX4 IMU 输入 OpenVINS，经 uav_bridge 回传外部视觉；
uav_mission 通过共享接口驱动 uav_control，当前使用 DDS 与 PX4 通信，保留 MAVROS 后端。
集中配置在独立 uav_bringup 仓库的 config/uav.yaml：DDS read_only=false、桥接 enabled=true、
任务 profile=grid_s、require_vio_health=true。launch 默认不启动任务。

## 获取与编译

从 [主工程](https://github.com/TVSBO/BoomBoomFly) 克隆并执行
python3 Scripts/restore_dependencies.py，然后
bash reproduction/build_uav.sh。完整系统依赖、Agent 构建和版本清单均由主工程管理。
src 下自研功能包和修改版 OpenVINS 已作为本仓库普通目录发布；其余上游依赖按锁定版本恢复。
uav_interfaces 在 src/uav_control/uav_interfaces，构建必须显式包含该发现路径。

## 目录

- config/openvins/d435i_px4：源设备相机—IMU 外参、噪声及估计器配置。
- calibration：标定工具及配置；不包含原始录包。
- docs：硬件接入、VIO、DDS、控制与验证记录。
- sim：历史 SITL 说明与验证材料。

## 文档与验证边界

历史 docs 中的 MAVROS/只读状态和原设备日志路径按记录日期理解；当前参数以 uav_bringup/config/uav.yaml 为准。
历史台架、视觉融合或 MAVROS SITL 通过，不代表最新 DDS 与九宫格任务已完成自主飞行验收。
本次仅整理源码并做静态检查，没有启动 ROS、连接硬件或重新标定。
来源、版权与基线见 SOURCE.md，当前发布说明见主工程 docs/发布说明_20261026.md。
