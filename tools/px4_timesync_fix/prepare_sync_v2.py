"""Prepare elapsed-time DDS skew tracking; other Timesync users keep legacy behavior."""
import difflib
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parent
fw=root/'firmware'
paths=['src/lib/timesync/Timesync.cpp','src/lib/timesync/Timesync.hpp',
       'src/modules/uxrce_dds_client/uxrce_dds_client.cpp','src/modules/uxrce_dds_client/uxrce_dds_client.h']
original={p:subprocess.check_output(['git','-C',str(fw),'show','HEAD:'+p]).decode() for p in paths}
edits=dict(original)
def change(path,before,after):
    assert edits[path].count(before)==1,(path,before)
    edits[path]=edits[path].replace(before,after)

h=paths[1];c=paths[0];client=paths[2];header=paths[3]
change(h,'Timesync(uint8_t source = timesync_status_s::SOURCE_PROTOCOL_UNKNOWN) : _source(source) {};',
       'Timesync(uint8_t source = timesync_status_s::SOURCE_PROTOCOL_UNKNOWN, bool time_based = false) :\n\t\t_source(source), _time_based(time_based) {};')
change(h,'int64_t offset() const { return (int64_t)_time_offset; }',
       'int64_t offset() const { return (int64_t)_time_offset; }\n\n'
       '\t/** Extrapolate a DDS clock estimate between valid measurements. */\n'
       '\tint64_t offset_at(uint64_t now_us) const\n\t{\n'
       '\t\tif (!_time_based || _last_sample_us == 0 || now_us <= _last_sample_us) { return offset(); }\n'
       '\t\treturn (int64_t)(_time_offset + _time_skew * ((now_us - _last_sample_us) / 1e6));\n\t}')
change(h,'void add_sample(int64_t offset_us);','void add_sample(int64_t offset_us, uint64_t now_us);')
change(h,'uint8_t _source{};','uint8_t _source{};\n\tbool _time_based{false};\n\tuint64_t _last_sample_us{0};')
change(c,'\t// Message originating from this system, compute time offset from it',
       '\tif (_time_based && _last_sample_us != 0 && now_us <= _last_sample_us) { return; }\n\n'
       '\t// Message originating from this system, compute time offset from it')
change(c,'add_sample(offset_us);','add_sample(offset_us, now_us);')
change(c,'void Timesync::add_sample(int64_t offset_us)','void Timesync::add_sample(int64_t offset_us, uint64_t now_us)')
change(c,'double time_offset_prev = _time_offset;',
       'double time_offset_prev = _time_offset;\n'
       '\tconst double dt = (_time_based && _last_sample_us != 0) ? (now_us - _last_sample_us) / 1e6 : 1.0;\n'
       '\t_last_sample_us = now_us;')
change(c,'(_time_offset + _time_skew);','(_time_offset + _time_skew * dt);')
change(c,'_filter_beta * (_time_offset - time_offset_prev)', '_filter_beta * ((_time_offset - time_offset_prev) / dt)')
change(c,'\t_sequence = 0;','\t_sequence = 0;\n\t_last_sample_us = 0;')
change(header,'Timesync _timesync{timesync_status_s::SOURCE_PROTOCOL_DDS};',
       'Timesync _timesync{timesync_status_s::SOURCE_PROTOCOL_DDS, true};')
change(client,'static constexpr uint8_t TIMESYNC_MAX_TIMEOUTS = 10;',
       "static constexpr uint8_t TIMESYNC_MAX_TIMEOUTS = 10;\nstatic constexpr hrt_abstime TIMESYNC_INTERVAL_US = 100'000;")
change(client,'session->time_offset = -timesync->offset() * 1000;',
       'session->time_offset = -timesync->offset_at(current_time / 1000) * 1000;')
change(client,'int timeout_ms = 1\'000; // 1 second',
       'int timeout_ms = 0; // Receive PONG asynchronously in the main loop; do not stall timestamps.')
change(client,'uxr_ping_agent_session(session, timeout_ms, attempts);',
       '_had_ping_reply = uxr_ping_agent_session(session, timeout_ms, attempts);')
change(client,'\n\t\t\t_had_ping_reply = false;\n', '\n')
change(client,'hrt_elapsed_time(&last_sync_session) > 1_s',
       'hrt_elapsed_time(&last_sync_session) >= TIMESYNC_INTERVAL_US')
change(client,'\t\t\t\tif (uxr_sync_session(&session, 10) && _timesync.sync_converged()) {\n\t\t\t\t\tlast_sync_session = hrt_absolute_time();',
       '\t\t\t\tlast_sync_session = hrt_absolute_time();\n\n'
       '\t\t\t\tif (uxr_sync_session(&session, 10) && _timesync.sync_converged()) {')
change(client,'\t\t\t/* Wait for topic updates for max 10 ms */',
       '\t\t\tif (_synchronize_timestamps && _timesync.sync_converged()) {\n'
       '\t\t\t\tsession.time_offset = -_timesync.offset_at(hrt_absolute_time()) * 1000;\n\t\t\t}\n\n'
       '\t\t\t/* Wait for topic updates for max 10 ms */')
patch=''
for p in paths:
    (fw/p).write_text(edits[p])
    patch+=''.join(difflib.unified_diff(original[p].splitlines(True),edits[p].splitlines(True),fromfile='a/'+p,tofile='b/'+p))
(root/'dds_sync_v2.patch').write_text(patch)
subprocess.run(['git','-C',str(fw),'diff','--check'],check=True)
assert set(subprocess.check_output(['git','-C',str(fw),'diff','--name-only'],text=True).splitlines())==set(paths)
print('Prepared elapsed-time skew tracking, fast original startup, predictive offset and nonblocking ping.')
