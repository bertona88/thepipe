import argparse
import json
from pathlib import Path
from .contracts import load, MODEL, OPERATION, FAULTS
from .runtime import run, verify_replay


def main():
    p=argparse.ArgumentParser(description='Pipe µ synthetic operation; no hardware qualification')
    p.add_argument('--machine',type=Path,default=MODEL)
    p.add_argument('--operation',type=Path,default=OPERATION)
    p.add_argument('--fault',choices=FAULTS)
    p.add_argument('--report',type=Path)
    p.add_argument('--verify-replay',type=Path)
    a=p.parse_args()
    try:
        if a.verify_replay:
            verify_replay(json.loads(a.verify_replay.read_text()));print('Replay verified');return 0
        m,o=load(a.machine,a.operation)
        if a.fault:o['fault']=a.fault
        report=run(m,o)
        if a.report:
            a.report.parent.mkdir(parents=True,exist_ok=True)
            a.report.write_text(json.dumps(report,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n')
        print(json.dumps({k:report[k] for k in ['completed','refusal','evidence_class','hardware_qualified','machine_hash']}))
        return 0 if report['completed'] else 2
    except (ValueError,KeyError,TypeError,OSError) as e:
        p.exit(1,f'Invalid/unavailable: {e}\n')

if __name__=='__main__':raise SystemExit(main())
