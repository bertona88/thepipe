"""Export authoritative reduced CAD boxes from a verified replay frame.
OpenSCAD output is an inspection envelope, explicitly not fabrication-ready CAD.
"""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pipe_micro.runtime import verify_replay
from pipe_micro.contracts import digest


def export(report,tick):
    verify_replay(report)
    frame=report['frames'][tick]
    parts=frame['geometry']
    lines=['// Pipe µ reduced collision envelopes; dimensions in mm.',
           '// Not fabrication-ready; source replay '+report['record_sha256']]
    for b in parts:
        c=[x*1000 for x in b['center_m']];size=[x*2000 for x in b['half_extents_m']]
        lines += ['// '+b['id'],f'translate({json.dumps(c)}) cube({json.dumps(size)}, center=true);']
    metadata={'schema':'pipe-micro-cad-envelopes/v1','source':report['source'],
      'machine_hash':report['machine_hash'],'replay_hash':report['record_sha256'],
      'tick':frame['tick'],'units':'m','geometry':parts,'geometry_hash':digest(parts),
      'fabrication_ready':False,'representation':'axis-aligned conservative envelopes'}
    return '\n'.join(lines)+'\n',metadata


def main():
    p=argparse.ArgumentParser();p.add_argument('replay',type=Path);p.add_argument('--tick',type=int,default=-1);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    try:
        scad,meta=export(json.loads(a.replay.read_text()),a.tick)
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.with_suffix('.scad').write_text(scad)
        a.output.with_suffix('.json').write_text(json.dumps(meta,indent=2)+'\n')
    except (ValueError,KeyError,IndexError,OSError) as e:p.exit(1,f'CAD unavailable: {e}\n')

if __name__=='__main__':main()
