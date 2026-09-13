"""Fixed-step orchestration and replay. Records are the only visualization input."""
import copy
import subprocess
from .contracts import validate, digest, ROOT, require
from .plant import Plant
from .sensor import observe
from .controller import Controller
from .geometry import envelopes


def source_identity():
    revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    paths=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard'],cwd=ROOT,text=True).splitlines()
    files={p:(ROOT/p).read_bytes().hex() for p in sorted(set(paths)) if (ROOT/p).is_file() and not p.startswith('docs/evidence/')}
    dirty=bool(subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip())
    return {'revision':revision,'dirty':dirty,'working_tree_sha256':digest(files)}


def run(m,o,source=None):
    validate(m,o);plant=Plant(m,o);control=Controller(m,o);frames=[]
    for tick in range(o['max_ticks']):
        obs=observe(m,o,plant.state,tick)
        before=copy.deepcopy(plant.state)
        command=control.decide(obs,tick)
        plant.step(command)
        if plant.failure:
            command=control.hold(plant.failure);plant.step(command)
        frame={'tick':tick,'time_s':tick*o['dt_s'],'observation':obs,'command':command,
          'physical_before':before,'physical_after':copy.deepcopy(plant.state),
          'control':{'phase':control.phase,'responsibility':control.responsibility},
          'geometry':envelopes(m,plant.state)}
        frames.append(frame)
        if control.reason or control.completed:break
    if not control.completed and not control.reason:
        control.hold('operation_timeout');plant.step({'kind':'hold'})
        # Last sample must record the actually executed stop.
        frames[-1]['physical_after']=copy.deepcopy(plant.state)
        frames[-1]['command']=control.hold('operation_timeout')
    report={'schema':'pipe-micro-replay/v1','source':source or source_identity(),
      'machine':m,'operation':o,'machine_hash':digest(m),'operation_hash':digest(o),
      'evidence_class':'synthetic_contact','hardware_qualified':False,
      'completed':control.completed,'refusal':control.reason,'events':control.events,'frames':frames,
      'coverage':{'orientation':'constrained, not measured','contact':'unmeasured seeded capacity and pull-off bounds',
        'mechanics':'bounded linear translational flexure, quasistatic loading',
        'optics':'synthetic independent centroid noise and box occlusion; not camera reconstruction',
        'collision':'discrete conservative boxes; no continuous deforming-solid proof',
        'safe_response':'executed freeze with retained load; no hardware safety qualification'}}
    report['record_sha256']=digest(report)
    return report


def verify_replay(report):
    require(report.get('schema')=='pipe-micro-replay/v1','unsupported replay')
    copy_report=copy.deepcopy(report);claimed=copy_report.pop('record_sha256',None)
    require(claimed==digest(copy_report),'replay integrity mismatch')
    require(report['hardware_qualified'] is False and report['evidence_class']=='synthetic_contact','invalid qualification')
    source=report['source'];require(set(source)=={'revision','dirty','working_tree_sha256'},'source identity missing')
    require(len(source['revision'])==40 and len(source['working_tree_sha256'])==64,'invalid source identity')
    current=source_identity()
    require(source['revision']==current['revision'] and source['working_tree_sha256']==current['working_tree_sha256'],'source revision/tree mismatch; reproduce at recorded source')
    require(report['machine_hash']==digest(report['machine']) and report['operation_hash']==digest(report['operation']),'configuration mismatch')
    # Replay verifies against current implementation; no animation or estimate fallback.
    expected=run(report['machine'],report['operation'],source)
    require(expected==report,'recorded execution does not reproduce with this implementation')
    return True
