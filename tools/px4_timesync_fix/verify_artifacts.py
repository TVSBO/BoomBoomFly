"""Check firmware package identity, size, payload and generated DDS rate."""
import base64
import hashlib
import json
from pathlib import Path
import re
import zlib

root = Path(__file__).resolve().parent
report = {'flashed': False, 'hardware_time_alignment_verified': False, 'variants': {}}
for variant in ['baseline', 'patched']:
    folder = root / 'artifacts' / variant
    path = folder / 'micoair_h743-v2_default.px4'
    package = json.loads(path.read_text())
    image = zlib.decompress(base64.b64decode(package['image']))
    raw = (folder / 'micoair_h743-v2_default.bin').read_bytes()
    assert package['magic'] == 'PX4FWv1'
    assert package['board_id'] == 1179
    assert package['image_size'] == len(image) == len(raw)
    assert image == raw
    assert len(image) <= package['image_maxsize'] == 1966080
    generated = (folder / 'dds_topics.h').read_text()
    section = generated.split('{ ORB_ID(sensor_combined),', 1)[1].split('},', 1)[0]
    rate = re.search(r'\(\(([0-9.]+) > 0\)', section)
    assert rate
    explicit_rate = float(rate[1])
    expected_rate = 0 if variant == 'baseline' else 200
    assert explicit_rate == expected_rate
    report['variants'][variant] = {
        'board_id': package['board_id'], 'git_identity': package['git_identity'],
        'image_size': len(image), 'image_maxsize': package['image_maxsize'],
        'remaining_bytes': package['image_maxsize'] - len(image),
        'px4_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'bin_sha256': hashlib.sha256(raw).hexdigest(),
        'explicit_sensor_rate_hz': explicit_rate,
        'effective_sensor_interval_ms': 10 if variant == 'baseline' else 5,
    }
report['image_size_delta_bytes'] = (
    report['variants']['patched']['image_size'] - report['variants']['baseline']['image_size'])
(root / 'artifacts/validation.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
