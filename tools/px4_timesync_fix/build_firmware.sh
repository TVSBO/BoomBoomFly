#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
variant="${1:?Specify baseline, patched, sync_only, sync_v2 or sync_v3}"
case "$variant" in
  baseline|patched|sync_only|sync_v2|sync_v3) ;;
  *) echo 'Expected baseline, patched, sync_only, sync_v2 or sync_v3' >&2; exit 2 ;;
esac
expected_sha=d6f12ad1c4f70ad3230afd7d86e971421e02fef4
test "$(git -C "$task_dir/firmware" rev-parse HEAD)" = "$expected_sha"
export PATH="$task_dir/venv/bin:$task_dir/arm-gnu-toolchain-13.2.Rel1-aarch64-arm-none-eabi/bin:$PATH"
# Per-process proxy overrides the stale global Git setting, including submodules.
export GIT_CONFIG_COUNT=2
export GIT_CONFIG_KEY_0=http.proxy
export GIT_CONFIG_VALUE_0="${https_proxy:-${HTTPS_PROXY:-}}"
export GIT_CONFIG_KEY_1=https.proxy
export GIT_CONFIG_VALUE_1="$GIT_CONFIG_VALUE_0"
mkdir -p "$task_dir/artifacts/$variant"
git -C "$task_dir/firmware" diff > "$task_dir/artifacts/$variant/source.patch"
if [ "$variant" = baseline ]; then
  test ! -s "$task_dir/artifacts/$variant/source.patch"
elif [ "$variant" = patched ]; then
  git -C "$task_dir/firmware" apply --reverse --check "$task_dir/dds_200hz_and_sync_cadence.patch"
elif [ "$variant" = sync_only ]; then
  git -C "$task_dir/firmware" apply --reverse --check "$task_dir/dds_sync_only.patch"
  test "$(git -C "$task_dir/firmware" diff --name-only)" = src/modules/uxrce_dds_client/uxrce_dds_client.cpp
elif [ "$variant" = sync_v2 ]; then
  git -C "$task_dir/firmware" apply --reverse --check "$task_dir/dds_sync_v2.patch"
else
  git -C "$task_dir/firmware" apply --reverse --check "$task_dir/dds_sync_v3.patch"
fi
echo "Building micoair_h743-v2_default ($variant); log: $task_dir/${variant}_build.log"
make -C "$task_dir/firmware" -j2 micoair_h743-v2_default > "$task_dir/${variant}_build.log" 2>&1
output_dir="$task_dir/firmware/build/micoair_h743-v2_default"
for extension in px4 bin elf; do
  cp "$output_dir/micoair_h743-v2_default.$extension" "$task_dir/artifacts/$variant/"
done
cp "$output_dir/src/modules/uxrce_dds_client/dds_topics.h" "$task_dir/artifacts/$variant/dds_topics.h"
git -C "$task_dir/firmware" submodule status --recursive > "$task_dir/artifacts/$variant/submodules.txt"
arm-none-eabi-gcc --version > "$task_dir/artifacts/$variant/compiler.txt"
"$task_dir/venv/bin/python" - <<'PY' > "$task_dir/artifacts/$variant/python_packages.txt"
import importlib.metadata
import sysconfig
packages = importlib.metadata.distributions(path=[sysconfig.get_path('purelib')])
print('\n'.join(sorted(f"{p.metadata['Name']}=={p.version}" for p in packages)))
PY
sha256sum "$task_dir/artifacts/$variant/"*.px4 "$task_dir/artifacts/$variant/"*.bin "$task_dir/artifacts/$variant/"*.elf > "$task_dir/artifacts/$variant/SHA256SUMS"
echo "Build complete: $task_dir/artifacts/$variant"
