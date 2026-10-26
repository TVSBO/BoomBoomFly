"""Add valid-session liveness and four-timestamp offset to the elapsed-time candidate."""
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parent
subprocess.run(['python3',str(root/'prepare_sync_v2.py')],check=True)
fw=root/'firmware'
p=fw/'src/modules/uxrce_dds_client/uxrce_dds_client.cpp'
source=p.read_text()
before='static void on_time(uxrSession *session, int64_t current_time, int64_t client_transmit_timestamp,\n\t\t    int64_t agent_receive_timestamp, int64_t originate_timestamp, void *args)'
after='static void on_time(uxrSession *session, int64_t current_time, int64_t agent_receive_timestamp,\n\t\t    int64_t agent_transmit_timestamp, int64_t originate_timestamp, void *args)'
assert source.count(before)==1
source=source.replace(before,after)
before='\t\ttimesync->update(current_time / 1000, agent_receive_timestamp, originate_timestamp);'
after=('\t\t// NTP midpoint excludes Agent processing time from the clock offset.\n'
       '\t\tconst int64_t agent_midpoint = agent_receive_timestamp + (agent_transmit_timestamp - agent_receive_timestamp) / 2;\n'
       '\t\ttimesync->update(current_time / 1000, agent_midpoint, originate_timestamp);')
assert source.count(before)==1
source=source.replace(before,after)
before='\t\t\t\tif (uxr_sync_session(&session, 10) && _timesync.sync_converged()) {\n\n\t\t\t\t\tif (_param_uxrce_dds_syncc.get() > 0) {'
after=('\t\t\t\tif (uxr_sync_session(&session, 10)) {\n'
       '\t\t\t\t\t// A TIMESTAMP reply proves session liveness, even if PONG is delayed.\n'
       '\t\t\t\t\t_had_agent_reply = true;\n\n'
       '\t\t\t\t\tif (_timesync.sync_converged() && _param_uxrce_dds_syncc.get() > 0) {')
assert source.count(before)==1
source=source.replace(before,after).replace('_had_ping_reply','_had_agent_reply')
p.write_text(source)
p=fw/'src/modules/uxrce_dds_client/uxrce_dds_client.h'
p.write_text(p.read_text().replace('_had_ping_reply','_had_agent_reply'))
subprocess.run(['git','-C',str(fw),'diff','--check'],check=True)
(root/'dds_sync_v3.patch').write_bytes(subprocess.check_output(['git','-C',str(fw),'diff']))
print('Prepared elapsed-time tracking, four-timestamp offset and session-reply liveness.')
