#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
specs_text="$(python3 - "$root/manifests/default.xml" "$@" <<'PY'
import sys, xml.etree.ElementTree as ET
projects = {p.attrib['name']:p.attrib['path'] for p in ET.parse(sys.argv[1]).getroot().findall('project')}
names = sys.argv[2:] or list(projects)
if set(names)-projects.keys(): raise SystemExit('Unknown repositories: '+str(set(names)-projects.keys()))
for name in names: print(name+' '+projects[name])
PY
)"
mapfile -t specs <<< "$specs_text"
for spec in "${specs[@]}"; do
  read -r name relative <<< "$spec"
  target="$root/$relative"
  if [[ -e "$target/.git" ]]; then
    if git -C "$target" symbolic-ref --quiet HEAD >/dev/null; then
      git -C "$target" pull --ff-only
    else
      git -C "$target" fetch origin refs/heads/main
      git -C "$target" merge --ff-only --no-autostash FETCH_HEAD
    fi
  elif [[ ! -e "$target" ]]; then
    mkdir -p "$(dirname -- "$target")"
    git clone "https://github.com/QU-of-boomboom/$name.git" "$target"
  elif [[ "$name" == uav_ws && -d "$target" && -z "$(ls -A "$target")" ]]; then
    git clone "https://github.com/QU-of-boomboom/$name.git" "$target"
  else
    echo "Existing non-Git directory left untouched: $target" >&2
    exit 1
  fi
done
