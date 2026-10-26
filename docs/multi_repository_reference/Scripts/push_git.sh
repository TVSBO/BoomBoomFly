#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# != 2 || -z "${2//[[:space:]]/}" ]]; then
  echo "用法：$0 仓库名 \"提交说明\"" >&2
  exit 1
fi

relative="$(python3 "${project_root}/Scripts/repository.py" "$1")"
target="${project_root}/${relative}"

# 显式检查独立仓库，防止 Git 向上查找误用父仓库。
[[ -e "${target}/.git" ]] || { echo "错误：${target} 不是独立 Git 仓库。" >&2; exit 1; }
branch="$(git -C "${target}" symbolic-ref --quiet --short HEAD)" || {
  echo "错误：当前处于 detached HEAD。" >&2; exit 1;
}
git -C "${target}" remote get-url origin >/dev/null

case "$1" in
  main) excluded=(uav_ws swarm_ws ugv_ws log references reference docker/kalibr/source) ;;
  uav_ws) excluded=(src/uav_control src/uav_bridge src/uav_bringup src/uav_mission src/thirdparty upstream build install log) ;;
  ugv_ws) excluded=(src/fpga_gateway src/car/car_bringup src/car/car_navigation src/car/car_mission src/external build install log) ;;
  *) excluded=() ;;
esac
if [[ ${#excluded[@]} -gt 0 ]]; then
  paths=(.)
  for path in "${excluded[@]}"; do
    # 不自动取消用户的暂存；有越界暂存时在任何修改前停止。
    if ! git -C "${target}" diff --cached --quiet -- "${path}"; then
      echo "错误：暂存区包含独立仓库或第三方路径 ${path}，请先自行处理。" >&2
      exit 1
    fi
    # 已由 ignore 规则排除的路径无需再传给 git add；显式传入会令 Git 报错。
    if ! git -C "${target}" check-ignore -q -- "${path}"; then
      paths+=(":(exclude)${path}")
    fi
  done
  git -C "${target}" add -A -- "${paths[@]}"
else
  git -C "${target}" add -A
fi

git -C "${target}" diff --cached --check
if git -C "${target}" diff --cached --quiet; then
  echo "没有新改动，直接推送已有提交"
else
  git -C "${target}" commit -m "$2"
fi
# 不自动 pull 或 force；推送失败时保留本地提交，供处理后重试。
git -C "${target}" -c push.followTags=false push --set-upstream origin "HEAD:refs/heads/${branch}"
