"""Offline drift regressions and package verification; never uploads firmware."""
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import zlib

root = Path(__file__).resolve().parent
folder = root / 'artifacts/sync_only'
subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(root/'stubs'),
                '-I', str(root/'upstream'), str(root/'replay.cpp'),
                str(root/'upstream/Timesync.cpp'), '-o', str(root/'replay')], check=True)
results = []
for drift in [-1330, -22.45, 0, 1313, 1330]:
    for mode in ['original', 'fixed']:
        for jitter in [False, True] if mode == 'fixed' else [False]:
            command = [str(root/'replay'), mode, str(drift), '1000', str(root/'measured_noise_us.txt')]
            if jitter:
                command.append('schedule_jitter')
            result = json.loads(subprocess.check_output(command, text=True))
            results.append(result)
            if mode == 'fixed':
                assert result['resets'] == 0 and result['max_hold_error_us'] < 10000
package_path = folder/'micoair_h743-v2_default.px4'
package = json.loads(package_path.read_text())
image = zlib.decompress(base64.b64decode(package['image']))
assert package['magic'] == 'PX4FWv1' and package['board_id'] == 1179
assert len(image) == package['image_size'] <= package['image_maxsize']
assert image == (folder/'micoair_h743-v2_default.bin').read_bytes()
assert (folder/'dds_topics.h').read_bytes() == (root/'artifacts/baseline/dds_topics.h').read_bytes()
source_patch = subprocess.check_output(['git', '-C', str(root/'firmware'), 'diff'], text=True)
assert source_patch == (folder/'source.patch').read_text()
report = dict(flashed=False, hardware_verified=False, scenarios=results,
              board_id=package['board_id'], image_size=len(image),
              image_maxsize=package['image_maxsize'], topic_configuration_identical_to_baseline=True,
              px4_sha256=hashlib.sha256(package_path.read_bytes()).hexdigest())
(folder/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(dict(scenarios=len(results), fixed_resets=sum(r['resets'] for r in results if r['fixed']),
    fixed_max_hold_error_us=max(r['max_hold_error_us'] for r in results if r['fixed']),
    board_id=report['board_id'], image_size=len(image), px4_sha256=report['px4_sha256']), indent=2))
