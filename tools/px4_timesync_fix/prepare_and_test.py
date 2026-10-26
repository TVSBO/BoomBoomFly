"""Prepare a reviewable firmware patch and evaluate the unchanged PX4 filter."""
import difflib
import json
from pathlib import Path
import statistics
import subprocess

root = Path(__file__).resolve().parent
upstream = root / 'upstream'
candidate = root / 'candidate'
candidate.mkdir(exist_ok=True)
edits = {}
source = (upstream / 'uxrce_dds_client.cpp').read_text()
edits['uxrce_dds_client.cpp'] = source.replace(
    'static constexpr uint8_t TIMESYNC_MAX_TIMEOUTS = 10;',
    "static constexpr uint8_t TIMESYNC_MAX_TIMEOUTS = 10;\n"
    "// Keep the sample interval consistent: Timesync estimates skew per sample.\n"
    "static constexpr hrt_abstime TIMESYNC_INTERVAL_US = 100'000;")
edits['uxrce_dds_client.cpp'] = edits['uxrce_dds_client.cpp'].replace(
    'while (_synchronize_timestamps) {\n\t\tif (uxr_sync_session(session, 1000))',
    'while (_synchronize_timestamps) {\n'
    '\t\tconst hrt_abstime sync_started = hrt_absolute_time();\n'
    '\t\tif (uxr_sync_session(session, 1000))')
edits['uxrce_dds_client.cpp'] = edits['uxrce_dds_client.cpp'].replace(
    "\t\tpx4_usleep(10'000);",
    '\t\tconst hrt_abstime sync_elapsed = hrt_elapsed_time(&sync_started);\n'
    '\n\t\tif (sync_elapsed < TIMESYNC_INTERVAL_US) {\n'
    '\t\t\tpx4_usleep(TIMESYNC_INTERVAL_US - sync_elapsed);\n\t\t}')
edits['uxrce_dds_client.cpp'] = edits['uxrce_dds_client.cpp'].replace(
    'hrt_elapsed_time(&last_sync_session) > 1_s',
    'hrt_elapsed_time(&last_sync_session) >= TIMESYNC_INTERVAL_US').replace(
    'uxr_sync_session(&session, 10) && _timesync.sync_converged()',
    'uxr_sync_session(&session, 10)').replace(
    'if (_param_uxrce_dds_syncc.get() > 0) {\n\t\t\t\t\t\tsyncSystemClock(&session);',
    'if (_timesync.sync_converged() && _param_uxrce_dds_syncc.get() > 0) {\n'
    '\t\t\t\t\t\tsyncSystemClock(&session);')
# Measure the interval from request start in both stages, excluding RTT from cadence.
edits['uxrce_dds_client.cpp'] = edits['uxrce_dds_client.cpp'].replace(
    '\t\t\t\tif (uxr_sync_session(&session, 10)) {\n'
    '\t\t\t\t\tlast_sync_session = hrt_absolute_time();',
    '\t\t\t\tconst hrt_abstime sync_started = hrt_absolute_time();\n\n'
    '\t\t\t\tif (uxr_sync_session(&session, 10)) {\n'
    '\t\t\t\t\tlast_sync_session = sync_started;')
edits['dds_topics.yaml'] = (upstream / 'dds_topics.yaml').read_text().replace(
    'type: px4_msgs::msg::SensorCombined\n',
    'type: px4_msgs::msg::SensorCombined\n    rate_limit: 200.\n')
patch = ''
for name, text in edits.items():
    before = (upstream / name).read_text()
    assert text != before
    (candidate / name).write_text(text)
    path = 'src/modules/uxrce_dds_client/' + name
    patch += ''.join(difflib.unified_diff(before.splitlines(True), text.splitlines(True),
                                          fromfile='a/' + path, tofile='b/' + path))
(root / 'dds_200hz_and_sync_cadence.patch').write_text(patch)

# Detrend observed offsets only; do not use the faulty estimated offset as truth.
evidence = root.parents[1] / 'uav_ws/log/dds_validation_20261021_180541'
rows = json.loads((evidence / 'input_clock_long.json').read_text())
sync = [r for r in rows if r['kind'] == 'sync']
xs = [r['mono'] - sync[0]['mono'] for r in sync]
ys = [r['observed_offset_us'] - sync[0]['observed_offset_us'] for r in sync]
mx, my = statistics.mean(xs), statistics.mean(ys)
slope = sum((x-mx)*(y-my) for x, y in zip(xs, ys)) / sum((x-mx)**2 for x in xs)
noise = [y-my-slope*(x-mx) for x,y in zip(xs,ys)]
noise_path = root / 'measured_noise_us.txt'
noise_path.write_text('\n'.join(str(v) for v in noise)+'\n')

subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(root/'stubs'),
                '-I', str(upstream), str(root/'replay.cpp'), str(upstream/'Timesync.cpp'),
                '-o', str(root/'replay')], check=True)
results = []
for mode in ['original', 'fixed']:
    for drift in [-1330, 0, 1330]:
        for empirical in [False, True]:
            command = [str(root/'replay'), mode, str(drift), '1000']
            if empirical:
                command.append(str(noise_path))
            run = subprocess.run(command, text=True, capture_output=True, check=True)
            result = json.loads(run.stdout)
            result['noise_source'] = 'detrended_measured_offsets' if empirical else 'synthetic_sine'
            results.append(result)
for drift in [-1330, 1330]:
    run = subprocess.run([str(root/'replay'), 'fixed', str(drift), '1000',
                          str(noise_path), 'schedule_jitter'],
                         text=True, capture_output=True, check=True)
    result = json.loads(run.stdout)
    result['noise_source'] = 'detrended_measured_offsets'
    results.append(result)
(root/'results.json').write_text(json.dumps({'measured_drift_us_per_s': slope,
    'noise_min_us': min(noise), 'noise_max_us': max(noise), 'scenarios': results}, indent=2)+'\n')
print(json.dumps(results, indent=2))
