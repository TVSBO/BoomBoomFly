#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
agent="$root/tools/micro_xrce_dds_agent"
cmake -S "$agent" -B "$agent/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$agent/install" \
  -DUAGENT_SUPERBUILD=OFF -DUAGENT_USE_SYSTEM_FASTDDS=ON \
  -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_LOGGER=ON \
  -DUAGENT_CED_PROFILE=OFF -DUAGENT_P2P_PROFILE=OFF
cmake --build "$agent/build" --parallel 1
cmake --install "$agent/build"
