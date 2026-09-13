"""Strict versioned boundary. No legacy loader or mechanical defaults."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'designs/pipe_micro/near_wall_v1/machine.json'
OPERATION = ROOT / 'scenarios/pipe_micro/native_resin_transfer_v1.json'


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def strict(value, fields):
    require(isinstance(value, dict) and set(value) == set(fields.split()), 'missing or unknown fields: ' + fields)


def finite_tree(value):
    if isinstance(value, float):
        require(math.isfinite(value), 'nonfinite numeric value')
    elif isinstance(value, dict):
        for v in value.values(): finite_tree(v)
    elif isinstance(value, list):
        for v in value: finite_tree(v)


def positive(*values):
    require(all(type(v) in (int, float) and math.isfinite(v) and v > 0 for v in values), 'expected positive finite quantities')


def load(machine=MODEL, operation=OPERATION):
    m = json.loads(Path(machine).read_text())
    o = json.loads(Path(operation).read_text())
    validate(m, o)
    return m, o


def validate(m, o):
    finite_tree(m); finite_tree(o)
    strict(m, 'schema model_id evidence_class chamber transport shoulders flexure tool specimen fixtures optics environment assumptions')
    require(m['schema'] == 'pipe-micro-machine/v1' and m['model_id'] == 'near_wall_v1', 'unsupported machine identity')
    require(m['evidence_class'] == 'synthetic_contact', 'unqualified model cannot claim calibrated or hardware evidence')
    strict(m['chamber'], 'radius_m half_length_m transfer_port_xyz_m transfer_port_radius_m')
    positive(m['chamber']['radius_m'], m['chamber']['half_length_m'], m['chamber']['transfer_port_radius_m'])
    t=m['transport']; strict(t, 'coupling z_limits_m theta_limits_rad start_z_m start_theta_rad speed_m_s speed_rad_s service_z_limit_m service_theta_limit_rad passing continuous_rotation')
    require(t['coupling']=='shared_ring' and t['passing'] is False and t['continuous_rotation'] is False, 'unsupported transport or service topology')
    positive(t['speed_m_s'], t['speed_rad_s'], t['service_z_limit_m'], t['service_theta_limit_rad'])
    for key,start,service in [('z_limits_m','start_z_m','service_z_limit_m'),('theta_limits_rad','start_theta_rad','service_theta_limit_rad')]:
        lo,hi=t[key]; require(-t[service] <= lo <= t[start] <= hi <= t[service] and lo <= 0 <= hi, 'travel/service bounds')
    require(set(m['shoulders'])=={'donor','receiver'}, 'two-tool study only')
    for p in m['shoulders'].values():
        require(len(p)==3 and math.hypot(*p[:2]) < m['chamber']['radius_m'], 'shoulder outside chamber')
    f=m['flexure']; strict(f, 'model reach_m stroke_m beam_length_m beam_thickness_m beam_width_m stiffness_n_m max_strain tendon_anchor_radius_m max_tension_n pretension_n')
    require(f['model']=='bounded_linear_translation', 'unsupported flexure model')
    positive(*(v for k,v in f.items() if k != 'model'))
    require(3*f['beam_thickness_m']*f['stroke_m']/f['beam_length_m']**2 < f['max_strain'], 'assumed beam strain envelope exceeded')
    require(f['pretension_n']+f['stiffness_n_m']*f['stroke_m'] < f['max_tension_n'], 'tendon envelope exceeded')
    tool=m['tool']; strict(tool,'x_offsets_m pad_half_extents_m finger_length_m finger_half_width_m jaw_open_m jaw_speed_m_s tool_speed_m_s')
    require(set(tool['x_offsets_m'])=={'donor','receiver'}, 'tool frame identity')
    positive(*tool['pad_half_extents_m'],tool['finger_length_m'],tool['finger_half_width_m'],tool['jaw_open_m'],tool['jaw_speed_m_s'],tool['tool_speed_m_s'])
    sp=m['specimen']; strict(sp,'id half_extents_m mass_kg material orientation_rad')
    positive(*sp['half_extents_m'],sp['mass_kg'])
    require(0.0003 <= 2*max(sp['half_extents_m']) <= 0.001, 'specimen outside initial domain')
    require(tool['jaw_open_m']>2*sp['half_extents_m'][2], 'jaw cannot clear specimen')
    require(abs(tool['x_offsets_m']['receiver']-tool['x_offsets_m']['donor'])>2*tool['pad_half_extents_m'][0], 'tool pads collide')
    require(set(m['fixtures'])=={'pickup_xyz_m','destination_xyz_m','half_extents_m'}, 'fixture schema')
    for key in ['pickup_xyz_m','destination_xyz_m']:
        p=m['fixtures'][key]; require(len(p)==3, 'fixture frame')
        for role,s in m['shoulders'].items():
            target=[p[0]+tool['x_offsets_m'][role],p[1],p[2]]
            require(math.dist(s,target) <= f['reach_m'], 'unreachable task pose from shoulder')
            require(abs(p[1]) + o.get('withdrawal_m',1) <= f['stroke_m'], 'task exceeds compliant travel')
        require(math.hypot(*p[:2])+max(sp['half_extents_m']) < m['chamber']['radius_m'], 'task outside chamber')
    opt=m['optics']; strict(opt,'calibration_id source_id sigma_m max_age_ticks camera_xyz_m fov_radius_m observable_axes')
    positive(opt['sigma_m'], opt['max_age_ticks'],opt['fov_radius_m'])
    require(opt['observable_axes']==['x','y','z'], 'only translational operation is implemented; orientation unqualified')
    require(len(opt['camera_xyz_m'])==2 and opt['camera_xyz_m'][0]!=opt['camera_xyz_m'][1], 'independent view geometry required')
    env=m['environment']; strict(env,'temperature_c rh_fraction sealed transfer_chamber closed_during_operation')
    require(0 <= env['rh_fraction'] <= 1 and env['sealed'] is True and env['transfer_chamber'] is True and env['closed_during_operation'] is True, 'environment contract')
    a=m['assumptions']; strict(a,'source status domain resin_process_id resin_batch contact_retention_n pull_off_n gravity_m_s2 rationale')
    require(a['status']=='unmeasured_exploratory_bounds' and a['resin_batch'] is None, 'this candidate has no material measurements')
    for k in ['contact_retention_n','pull_off_n']:
        positive(*a[k]); require(a[k][0] <= a[k][1], 'invalid contact bounds')
    positive(a['gravity_m_s2'])
    require(a['contact_retention_n'][0] > sp['mass_kg']*a['gravity_m_s2'], 'insufficient assumed support')
    strict(o,'schema operation_id model_id evidence_class seed dt_s max_ticks position_tolerance_m separation_m withdrawal_m fault')
    require(o['schema']=='pipe-micro-operation/v1' and o['model_id']==m['model_id'] and o['operation_id']=='native_resin_transfer_v1', 'unsupported operation; legacy aliases are rejected')
    require(o['evidence_class']==m['evidence_class'], 'evidence identity mismatch')
    require(type(o['seed']) is int and type(o['max_ticks']) is int and o['max_ticks']>0, 'integer seed and tick limit required')
    positive(o['dt_s'],o['position_tolerance_m'],o['separation_m'],o['withdrawal_m'])
    require(o['position_tolerance_m']>6*opt['sigma_m'] and o['withdrawal_m']>o['separation_m']>6*opt['sigma_m'], 'uncertainty exceeds operation margin')
    require(o['fault'] in FAULTS, 'unsupported fault')
    require(o['dt_s']*tool['tool_speed_m_s'] <= min(tool['pad_half_extents_m'])/2, 'time step too coarse for tool geometry')
    require(o['dt_s']*tool['jaw_speed_m_s'] <= min(tool['pad_half_extents_m'])/2, 'time step too coarse for jaws')


FAULTS = ('none','stale','wrong_calibration','wrong_identity','occlusion','unobservable','donor_adhesion','receiver_slip','residual_bridge','ambiguous_gap','missing_destination','blocked_jaw','blocked_withdrawal','service_wrap','part_rotation')
