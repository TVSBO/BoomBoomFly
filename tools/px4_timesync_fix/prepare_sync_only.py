"""Prepare the DDS timing-only candidate; does not access flight hardware."""
import difflib
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parent
upstream = root / 'upstream'
firmware = root / 'firmware'
candidate = root / 'candidate_sync_only'
candidate.mkdir(exist_ok=True)
source = (root / 'candidate/uxrce_dds_client.cpp').read_text()
before = '\t\t\t\tconst hrt_abstime sync_started = hrt_absolute_time();\n\n\t\t\t\tif (uxr_sync_session(&session, 10)) {\n\t\t\t\t\tlast_sync_session = sync_started;'
after = '\t\t\t\t// Rate-limit attempts even when replies time out.\n\t\t\t\tlast_sync_session = hrt_absolute_time();\n\n\t\t\t\tif (uxr_sync_session(&session, 10)) {'
assert source.count(before) == 1
source = source.replace(before, after).rstrip() + '\n'
(candidate / 'uxrce_dds_client.cpp').write_text(source)
original = (upstream / 'uxrce_dds_client.cpp').read_text()
patch = ''.join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
    fromfile='a/src/modules/uxrce_dds_client/uxrce_dds_client.cpp',
    tofile='b/src/modules/uxrce_dds_client/uxrce_dds_client.cpp'))
(root / 'dds_sync_only.patch').write_text(patch)
(firmware / 'src/modules/uxrce_dds_client/uxrce_dds_client.cpp').write_text(source)
topics = subprocess.check_output(['git', '-C', str(firmware), 'show', 'HEAD:src/modules/uxrce_dds_client/dds_topics.yaml'])
(firmware / 'src/modules/uxrce_dds_client/dds_topics.yaml').write_bytes(topics)
changed = subprocess.check_output(['git', '-C', str(firmware), 'diff', '--name-only'], text=True).splitlines()
assert changed == ['src/modules/uxrce_dds_client/uxrce_dds_client.cpp'], changed
subprocess.run(['git', '-C', str(firmware), 'apply', '--reverse', '--check', str(root / 'dds_sync_only.patch')], check=True)
subprocess.run(['git', '-C', str(firmware), 'diff', '--check'], check=True)
print('Prepared one-file DDS timing-only patch; topic rates and parameter definitions unchanged.')
