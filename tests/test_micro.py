import ast
import copy
import json
import unittest
from pathlib import Path
from pipe_micro.contracts import load,validate,FAULTS,digest,ROOT
from pipe_micro.runtime import run,verify_replay,source_identity
from pipe_micro.controller import Controller
from pipe_micro.plant import Plant
from pipe_micro.sensor import observe
from pipe_micro.geometry import envelopes,overlap,frame_point

SOURCE=source_identity()

class MicroTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m,cls.o=load();cls.report=run(cls.m,cls.o,SOURCE)

    def test_executed_support_transition(self):
        r=self.report;self.assertTrue(r['completed'],r['refusal']);self.assertFalse(r['hardware_qualified'])
        events=[e['event'] for e in r['events']]
        ordered=['base_positioned','donor_retained','dual_supported','release_commanded','separation_observed','retention_verified','donor_withdrawn','final_pose_verified']
        self.assertEqual(sorted(ordered,key=events.index),ordered)
        self.assertEqual(r['frames'][-1]['physical_after']['supports'],['destination'])
        self.assertTrue(any(set(f['physical_after']['supports'])=={'donor','receiver'} for f in r['frames']))
        self.assertTrue(any(f['command']['kind']=='move' for f in r['frames']))
        self.assertTrue(all(e['evidence_ids'] for e in r['events']))

    def test_all_faults_refuse_and_execute_stop(self):
        expected=dict(stale='stale_observation',wrong_calibration='observation_identity',wrong_identity='missing_or_invalid_part',occlusion='missing_or_invalid_part',unobservable='unobservable_required_translation',donor_adhesion='separation_unverified',receiver_slip='phase_timeout_ReceiverContact',residual_bridge='separation_unverified',ambiguous_gap='separation_unverified',missing_destination='missing_or_invalid_part',blocked_jaw='blocked_jaw',blocked_withdrawal='blocked_withdrawal',service_wrap='service_wrap',part_rotation='orientation_departure')
        for fault in FAULTS[1:]:
            with self.subTest(fault=fault):
                o={**self.o,'fault':fault};r=run(self.m,o,SOURCE)
                self.assertFalse(r['completed']);self.assertEqual(r['refusal'],expected[fault])
                self.assertTrue(r['frames'][-1]['physical_after']['stopped'])
                self.assertEqual(r['frames'][-1]['command']['kind'],'hold')
                self.assertNotIn('final_pose_verified',[e['event'] for e in r['events']])

    def test_determinism_and_replay_tampering(self):
        self.assertEqual(self.report,run(self.m,self.o,SOURCE));self.assertTrue(verify_replay(self.report))
        for mutation in ('geometry','observation','source'):
            r=copy.deepcopy(self.report)
            if mutation=='geometry':r['frames'][4]['geometry']=[]
            elif mutation=='observation':r['frames'][4]['observation']['objects']['part']['xyz_m']=[0,0,0]
            else:r['source']={}
            r.pop('record_sha256');r['record_sha256']=digest(r)
            with self.assertRaises(ValueError):verify_replay(r)

    def test_strict_legacy_and_missing_fields(self):
        for change in ({'schema':'machine_baseline_v1'}, {'model_id':'nominal'}, {'evidence_class':'hardware_observed'}):
            with self.assertRaises(ValueError):validate({**self.m,**change},self.o)
        for field in self.m:
            m=copy.deepcopy(self.m);del m[field]
            with self.assertRaises(ValueError):validate(m,self.o)
        m=copy.deepcopy(self.m);m['flexure']['stiffness_n_m']=float('nan')
        with self.assertRaises(ValueError):validate(m,self.o)

    def test_reach_coupling_and_uncertainty(self):
        for key,value in [('coupling','independent'),('continuous_rotation',True),('service_theta_limit_rad',.00001)]:
            m=copy.deepcopy(self.m);m['transport'][key]=value
            with self.assertRaises(ValueError):validate(m,self.o)
        m=copy.deepcopy(self.m);m['shoulders']['donor']=[-.04,0,0]
        with self.assertRaises(ValueError):validate(m,self.o)
        m=copy.deepcopy(self.m);m['optics']['sigma_m']=.001
        with self.assertRaises(ValueError):validate(m,self.o)

    def test_truth_firewall_and_no_owner_support(self):
        source=(ROOT/'pipe_micro/controller.py').read_text();tree=ast.parse(source)
        self.assertFalse(any(isinstance(n,ast.ImportFrom) and n.module in ('plant','sensor','runtime') for n in ast.walk(tree)))
        plant=Plant(self.m,self.o);obs=observe(self.m,self.o,plant.state,0)
        c=Controller(self.m,self.o);c.phase='Pickup';c.responsibility='donor'
        cmd=c.decide(obs,0);self.assertEqual(c.phase,'Pickup');self.assertEqual(cmd['kind'],'jaw')
        c.phase='DonorUnloading';obs['contacts']=['receiver'];obs['donor_gap_m']=None
        obs['objects']['donor']['jaw_m']=self.m['tool']['jaw_open_m']
        c.decide(obs,0);self.assertEqual(c.reason,'separation_unverified')

    def test_independent_part_observation_and_invalid_covariance(self):
        p=Plant(self.m,self.o);obs=observe(self.m,self.o,p.state,0)
        self.assertNotEqual(obs['objects']['part']['sample_id'],obs['objects']['donor']['sample_id'])
        obs['objects']['part']['covariance_m2']=[float('nan')]*3
        self.assertEqual(Controller(self.m,self.o).decide(obs,0)['kind'],'hold')

    def test_geometry_and_loads_over_executed_states(self):
        for f in self.report['frames'][::37]:
            s=f['physical_after'];self.assertEqual(envelopes(self.m,s),f['geometry'])
            ids={b['id'] for b in f['geometry']}
            self.assertTrue({'donor/palm','receiver/pad/0','donor/service','receiver/flexure','destination/fixture','part'}<=ids)
            for tool in s['tools'].values():
                self.assertLessEqual(tool['tension_n'],self.m['flexure']['max_tension_n'])
                self.assertGreaterEqual(tool['energy_j'],0)

    def test_cad_matches_multiple_executed_states(self):
        from cad.export_micro import export
        for tick in [0,200,-1]:
            scad,meta=export(self.report,tick)
            self.assertEqual(meta['geometry'],self.report['frames'][tick]['geometry'])
            self.assertEqual(meta['geometry_hash'],digest(meta['geometry']))
            self.assertFalse(meta['fabrication_ready'])
            self.assertIn('cube(',scad)

    def test_cli_rejects_legacy_alias_and_invalid_replay(self):
        import subprocess,sys,tempfile
        result=subprocess.run([sys.executable,'-m','pipe_micro','--operation','nominal'],cwd=ROOT,capture_output=True)
        self.assertEqual(result.returncode,1)
        (ROOT/'out').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT/'out') as temp:
            path=Path(temp)/'bad.json';path.write_text('{}')
            result=subprocess.run([sys.executable,'-m','pipe_micro','--verify-replay',str(path)],cwd=ROOT,capture_output=True)
            self.assertEqual(result.returncode,1)

    def test_historical_evidence_unchanged_and_inventory_complete(self):
        import subprocess,hashlib
        manifest=json.loads((ROOT/'docs/migration/pipe_micro_disposition.yaml').read_text())
        original=set(subprocess.check_output(['git','ls-tree','-r','--name-only',manifest['baseline_revision']],cwd=ROOT,text=True).splitlines())
        self.assertEqual(original,{x['path'] for x in manifest['paths']})
        for x in manifest['paths']:
            if x['disposition']=='archive':self.assertEqual(hashlib.sha256((ROOT/x['destination']).read_bytes()).hexdigest(),x['baseline_sha256'])
            if x['disposition']=='retire':self.assertFalse((ROOT/x['path']).is_file(),x['path'])

if __name__=='__main__':unittest.main()
