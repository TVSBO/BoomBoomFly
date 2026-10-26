#!/usr/bin/env python3
"""Restore the published exact commits; never overwrite a different checkout."""
import json, subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[1]
for spec in json.loads((root/'manifests/release.lock.json').read_text()):
    path=root/spec['path']
    if path.exists():
        if not (path/'.git').exists(): raise SystemExit(f'Existing non-Git directory: {path}')
        head=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
        dirty=subprocess.check_output(['git','-C',str(path),'status','--porcelain','--untracked-files=no'],text=True).strip()
        if head != spec['commit'] or dirty: raise SystemExit(f'Existing checkout differs: {path}')
        continue
    path.parent.mkdir(parents=True,exist_ok=True)
    subprocess.run(['git','clone','--no-checkout',spec['url'],str(path)],check=True)
    subprocess.run(['git','-C',str(path),'checkout','--detach',spec['commit']],check=True)
