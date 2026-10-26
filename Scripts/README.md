# 单仓库版脚本

- restore_dependencies.py：按 manifests/dependencies.lock.json 恢复未修改的上游依赖。
- setup_ros_environment.bash：通过 source 加载 ROS Humble 与 uav_ws 已构建 overlay。
- reproduction/build_uav.sh：构建自研模块、OpenVINS 和相机驱动。
- reproduction/build_agent.sh：构建 DDS Agent。
- reproduction/restore_px4.sh：固定官方版本加 V3 补丁恢复 PX4 源码，不刷写。

自研模块已包含在本仓库，不需要多仓库同步。修改后使用常规 git add/commit/push。
原组织版多仓库管理脚本在 docs/multi_repository_reference 中仅作为历史参考。
