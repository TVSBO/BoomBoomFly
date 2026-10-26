# 版本与仓库清单

default.xml 用 repo 描述 6 个子仓库，主工程由普通 Git 管理；均位于 QU-of-boomboom。
uav_vio_bridge 远程名称保留，本地路径为 uav_ws/src/uav_bridge。
open_vins 作为独立工程修改版仓库，其余第三方从上游固定提交恢复。

- 开发获取：bash Scripts/sync_ros_packages.sh，沿 main/当前 upstream 快进。
- 精确获取：python3 Scripts/restore_release.py，按 release.lock.json 固定提交检出。
- 上游依赖：python3 Scripts/restore_dependencies.py，按 dependencies.lock.json 恢复。
- repo 工具：repo init -u https://github.com/QU-of-boomboom/BoomBoomFly.git -b main -m manifests/default.xml。

脚本不覆盖已有非 Git 目录；精确恢复发现已有目录与目标不一致时停止。
工作区与功能包源码独立提交，不把 src 或 uav_ws 当作主工程统一暂存。
主工程提交由 git rev-parse HEAD 单独确定，不把自身提交写入自身文件。
