"""Exercise production timestamp callback and connectivity method with protocol faults."""
from pathlib import Path
import json
import subprocess

root=Path(__file__).resolve().parent
out=root/'artifacts/sync_v3';out.mkdir(exist_ok=True)
client=(root/'firmware/src/modules/uxrce_dds_client/uxrce_dds_client.cpp').read_text()
callback=client.split('static void on_time(',1)[1].split('static void on_time_no_sync',1)[0]
callback='static void on_time('+callback
main=r'''
int main() {
    Timesync filter(2, true); uxrSession session{};
    const int64_t agent_offset = 1792660000000000000LL;
    int64_t origin = 1000000000;
    for (int i=0; i<1000; ++i) {
        const int64_t network = 500000;
        const int64_t processing = (i%2) ? 3000000 : 200000;
        const int64_t received = origin + network + agent_offset;
        const int64_t transmitted = received + processing;
        const int64_t now = origin + 2*network + processing;
        on_time(&session, now, received, transmitted, origin, &filter);
        origin += 13000000;
    }
    std::cout << std::llabs(session.time_offset-agent_offset) << '\n';
    return !filter.sync_converged() || std::llabs(session.time_offset-agent_offset)>1000;
}
'''
prefix='#include "Timesync.hpp"\n#include <iostream>\n#include <cstdlib>\nstruct uxrSession { int64_t time_offset{}; };\n'
changed=root/'firmware/src/lib/timesync'
def build_callback(name,text):
    path=out/(name+'.cpp');path.write_text(prefix+text+main)
    target=out/name
    subprocess.run(['g++','-std=c++17','-O2','-I',str(root/'stubs'),'-I',str(changed),str(path),str(changed/'Timesync.cpp'),'-o',str(target)],check=True)
    return subprocess.run([str(target)],capture_output=True,text=True)
good=build_callback('callback_production',callback)
assert good.returncode==0,good.stdout+good.stderr
bad=build_callback('callback_three_timestamp_control',callback.replace('timesync->update(current_time / 1000, agent_midpoint,','timesync->update(current_time / 1000, agent_transmit_timestamp,'))
assert bad.returncode!=0,'Regression test failed to detect known processing-time bias'

connectivity=client.split('void UxrceddsClient::checkConnectivity(',1)[1].split('void UxrceddsClient::resetConnectivityCounters',1)[0]
connectivity='void UxrceddsClient::checkConnectivity('+connectivity
prefix=r'''
#include <cstdint>
#include <cassert>
using hrt_abstime = uint64_t;
uint64_t clock_us=0;
uint64_t hrt_absolute_time() { return clock_us; }
uint64_t hrt_elapsed_time(const uint64_t *then) { return clock_us-*then; }
constexpr uint64_t operator""_s(unsigned long long x) { return x*1000000; }
#define PX4_ERR(...) ((void)0)
struct uxrSession {};
bool uxr_ping_agent_session(uxrSession*,int timeout_ms,uint8_t) { assert(timeout_ms==0); return false; }
struct Param { int32_t value; int32_t get() const { return value; } };
class UxrceddsClient {
public:
    int _last_payload_tx_rate=100,_last_payload_rx_rate=0;
    int _num_tx_rate_zero=0,_num_rx_rate_zero=0,_num_pings_missed=0;
    bool _connected=true,_had_agent_reply=false;
    uint64_t _last_ping=0;
    Param _param_uxrce_dds_tx_to{3},_param_uxrce_dds_rx_to{-1};
    void checkConnectivity(uxrSession*);
};
'''
main=r'''
int main() {
    uxrSession session; UxrceddsClient client;
    // A responding TIMESTAMP peer must stay connected despite delayed PONG.
    for (int i=0;i<100;++i) {
        clock_us+=100000;client._had_agent_reply=true;
        client.checkConnectivity(&session);assert(client._connected);
    }
    // When every response stops, the original 3-missed-probe policy still works.
    client._had_agent_reply=false;
    for (int i=0;i<50;++i) {clock_us+=100000;client.checkConnectivity(&session);}
    assert(!client._connected);
    // Genuine payload TX failure must still trip its configured timeout.
    UxrceddsClient silent; silent._last_payload_tx_rate=0;silent._last_ping=clock_us;
    for(int i=0;i<50;++i) {clock_us+=100000;silent._had_agent_reply=true;silent.checkConnectivity(&session);}
    assert(!silent._connected);
}
'''
path=out/'connectivity_production.cpp';path.write_text(prefix+connectivity+main)
subprocess.run(['g++','-std=c++17','-O2',str(path),'-o',str(out/'connectivity_production')],check=True)
subprocess.run([str(out/'connectivity_production')],check=True)
report=dict(four_timestamp_callback_error_ns=int(good.stdout),three_timestamp_control_error_ns=int(bad.stdout),
    active_timestamp_with_missing_pong_passed=True,complete_link_loss_disconnect_passed=True,payload_tx_timeout_passed=True)
(out/'protocol_validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
