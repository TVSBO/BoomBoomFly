#!/usr/bin/env python3
"""从 repo manifest 读取自写仓库范围，供 shell 入口使用。"""
from pathlib import Path, PurePosixPath
import sys
import xml.etree.ElementTree as ET


def repositories():
    manifest = Path(__file__).resolve().parent.parent / 'manifests/default.xml'
    result = {'BoomBoomFly': '.'}
    for project in ET.parse(manifest).getroot().findall('project'):
        name = project.attrib['name']
        path = PurePosixPath(project.attrib['path'])
        if path.is_absolute() or '..' in path.parts or str(path) == '.' or name in result:
            raise ValueError(f'invalid repository: {name}')
        result[name] = str(path)
    return result


if __name__ == '__main__':
    repos = repositories()
    if sys.argv[1:] == ['--list']:
        print('\n'.join(name for name in repos if name != 'BoomBoomFly'))
    elif len(sys.argv) == 2:
        name = 'BoomBoomFly' if sys.argv[1] == 'main' else sys.argv[1]
        if name not in repos:
            sys.exit(f'错误：未知仓库 {sys.argv[1]}')
        print(repos[name])
    else:
        sys.exit('用法：repository.py {--list|仓库名}')
