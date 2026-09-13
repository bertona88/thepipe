"""Record a compact reproducible nominal/fault matrix; full replays go in out/."""
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pipe_micro.contracts import ROOT,load,FAULTS
from pipe_micro.runtime import run,source_identity

m,o=load();source=source_identity()
if source['dirty']:raise SystemExit('Commit the source before recording evidence')
cases=[]
(ROOT/'out').mkdir(exist_ok=True)
for fault in FAULTS:
    result=run(m,{**o,'fault':fault},source)
    (ROOT/'out'/f'pipe_micro_{fault}.json').write_text(json.dumps(result,sort_keys=True,separators=(',',':'))+'\n')
    cases.append({'fault':fault,'completed':result['completed'],'refusal':result['refusal'],
                  'ticks':len(result['frames']),'replay_sha256':result['record_sha256'],
                  'events':result['events'],'operation_hash':result['operation_hash']})
output=ROOT/'docs/evidence/pipe_micro_v1.json';output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps({'schema':'pipe-micro-evidence-matrix/v1','source':source,'machine_hash':result['machine_hash'],
   'command':'python3 scripts/record_micro_evidence.py','evidence_class':'synthetic_contact',
   'hardware_qualified':False,'cases':cases},indent=2)+'\n')
print(output)
