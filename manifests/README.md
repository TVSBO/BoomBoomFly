# 单仓库依赖清单

dependencies.lock.json 固定未修改的 px4_msgs、MAVROS、RealSense、Micro XRCE-DDS Agent 上游提交。
恢复方式：在工程根目录执行 python3 Scripts/restore_dependencies.py。
自研模块和修改版 OpenVINS 已直接包含在本仓库，不需要额外同步。
来源仓库版本见 reproduction/organization-imports.lock.json；旧多仓库清单归档在 docs/multi_repository_reference。
