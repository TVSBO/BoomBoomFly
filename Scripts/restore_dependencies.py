#!/usr/bin/env python3
"""Restore pinned upstream dependencies without overwriting existing directories."""
import json, subprocess
from pathlib import Path
root = Path(__file__).resolve().parents[1]
for spec in json.loads((root/'manifests/dependencies.lock.json').read_text()):
    path = root/spec['path']
    if path.exists():
        if not (path/'.git').exists():
            raise SystemExit(f'Existing non-Git directory, left untouched: {path}')
        head = subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'], text=True).strip()
        dirty = subprocess.check_output(['git','-C',str(path),'status','--porcelain','--untracked-files=no'], text=True).strip()
        if head != spec['commit'] or dirty:
            raise SystemExit(f'Existing checkout differs, left untouched: {path}')
        print(f'Already restored: {path}')
        continue
    path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['git','clone','--no-checkout',spec['origin'],str(path)], check=True)
    subprocess.run(['git','-C',str(path),'checkout','--detach',spec['commit']], check=True)
    subprocess.run(['git','-C',str(path),'submodule','update','--init','--recursive'], check=True)
