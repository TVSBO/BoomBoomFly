"""Test real filter code for legacy equivalence and variable-interval DDS tracking."""
import json
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parent
out=root/'artifacts/sync_v2'
out.mkdir(exist_ok=True)
changed=root/'firmware/src/lib/timesync'
def compile_filter(source,library,include,output):
    subprocess.run(['g++','-std=c++17','-O2','-I',str(root/'stubs'),'-I',str(include),str(source),str(library),'-o',str(output)],check=True)
compile_filter(root/'replay.cpp',root/'upstream/Timesync.cpp',root/'upstream',out/'replay_original')
compile_filter(root/'replay.cpp',changed/'Timesync.cpp',changed,out/'replay_legacy')
source=(root/'replay.cpp').read_text().replace('Timesync filter;','Timesync filter(0, true);')
source=source.replace('const bool fixed = std::string(argv[1]) == "fixed";','const bool fixed = true;')
source=source.replace('fixed ? 100000 : (filter.sync_converged() ? 1000000 : 13000)', '(filter.sync_converged() ? 100000 : 13000)')
source=source.replace('if (samples % 500 == 0) step += 100000;', 'if (samples % 500 == 0) step += 1000000;')
source=source.replace('std::abs(residual + drift * step / 1e6)', 'std::abs(truth + drift * step / 1e6 - filter.offset_at(now + step))')
(out/'replay_time_based.cpp').write_text(source)
compile_filter(out/'replay_time_based.cpp',changed/'Timesync.cpp',changed,out/'replay_time_based')
legacy=[];dds=[]
for drift in [-1330,-22.45,0,1313,1330]:
    for mode in ['original','fixed']:
        args=[mode,str(drift),'1000',str(root/'measured_noise_us.txt')]
        before=json.loads(subprocess.check_output([str(out/'replay_original'),*args],text=True))
        after=json.loads(subprocess.check_output([str(out/'replay_legacy'),*args],text=True))
        assert before==after,(before,after)
        legacy.append(dict(mode=mode,drift=drift,identical=True))
    for jitter in [False,True]:
        args=['time_based',str(drift),'1000',str(root/'measured_noise_us.txt')]
        if jitter:args.append('schedule_jitter')
        result=json.loads(subprocess.check_output([str(out/'replay_time_based'),*args],text=True))
        assert result['resets']==0 and result['max_hold_error_us']<10000,result
        dds.append(result)
report=dict(legacy_cases=legacy,dds_cases=dds,flashed=False,hardware_verified=False)
(out/'filter_validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(dict(legacy_cases_identical=len(legacy),dds_cases=len(dds),dds_resets=sum(r['resets'] for r in dds),
    max_predicted_error_us=max(r['max_hold_error_us'] for r in dds)),indent=2))
