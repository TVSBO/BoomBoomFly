#!/usr/bin/env bash
# Optional network step. Only create a new firmware tree; never overwrite one.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
firmware="$root/tools/px4_timesync_fix/firmware"
if [ -e "$firmware" ]; then
  echo "Firmware path already exists; inspect it manually: $firmware" >&2
  exit 1
fi
git clone --no-checkout https://github.com/PX4/PX4-Autopilot.git "$firmware"
git -C "$firmware" checkout --detach d6f12ad1c4f70ad3230afd7d86e971421e02fef4
# Restore only submodules that were populated on the source device.
python3 - "$firmware" "$root/reproduction/px4-submodules.txt" <<'PY'
import subprocess, sys
from pathlib import Path
for line in Path(sys.argv[2]).read_text().splitlines():
    if line.startswith((' ', '+')):
        path = line[1:].split()[1]
        subprocess.run(['git', '-C', sys.argv[1], 'submodule', 'update', '--init', '--recursive', '--', path], check=True)
PY
git -C "$firmware" apply --check "$root/reproduction/px4-local.patch"
git -C "$firmware" apply "$root/reproduction/px4-local.patch"
echo "Restored pinned PX4 with the exported local patch. No firmware was built or flashed."
