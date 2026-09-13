"""Synthetic measurement boundary, deliberately separate from control.
Part and tool samples have different noise streams. Support evidence remains
synthetic contact evidence, not a claim that cameras can measure retaining force.
"""
import math
from .contracts import digest
from .geometry import envelopes, ray_clear


def noise(seed,tick,identity,axis,sigma):
    h=int(digest([seed,tick,identity,axis])[:8],16)
    return (h/0xffffffff*2-1)*math.sqrt(3)*sigma


def observe(m,o,state,tick):
    opt=m['optics'];fault=o['fault'];geo=envelopes(m,state)
    out={'tick':tick,'calibration_id':opt['calibration_id'],'source_id':opt['source_id'],
         'model_hash':digest(m),'evidence_class':m['evidence_class'],'axes':list(opt['observable_axes']),
         'base_z_m':state['base_z_m'],'base_theta_rad':state['base_theta_rad'],
         'objects':{},'contacts':list(state['supports']),'orientation_status':'constrained_unmeasured',
         'donor_gap_m':None,'invalid_reason':None}
    for identity in ('part','donor','receiver'):
        p=state['part_xyz_m'] if identity=='part' else state['tools'][identity]['xyz_m']
        # Geometry-derived optical feature locations. Tool feature sits on top of palm.
        b=next(b for b in geo if b['id']==('part' if identity=='part' else identity+'/palm'))
        feature=list(b['center_m']);feature[2]+=b['half_extents_m'][2]
        visible=sum(ray_clear(camera,feature,geo,{b['id']}) for camera in opt['camera_xyz_m'])
        in_fov=math.dist(p,m['fixtures']['pickup_xyz_m'])<opt['fov_radius_m']
        out['objects'][identity]={'id':identity,'sample_id':f'{tick}/{identity}',
          'xyz_m':[v+noise(o['seed'],tick,identity,i,opt['sigma_m']) for i,v in enumerate(p)],
          'covariance_m2':[opt['sigma_m']**2]*3,'valid':visible==2 and in_fov,
          'visible_views':visible,'jaw_m':None if identity=='part' else state['tools'][identity]['jaw_m']}
    gap=(state['tools']['donor']['jaw_m']-2*m['specimen']['half_extents_m'][2])/2
    if 'donor' not in state['supports']:out['donor_gap_m']=gap
    if fault=='stale':out['tick']=tick-10
    if fault=='wrong_calibration':out['calibration_id']='unrelated'
    if fault=='wrong_identity':out['objects']['part']['id']='unrelated'
    if fault=='occlusion':out['objects']['part']['valid']=False
    if fault=='unobservable':out['axes']=['x','y']
    if fault=='ambiguous_gap':out['donor_gap_m']=None
    if fault=='missing_destination' and 'destination' in state['supports']:out['objects']['part']['valid']=False
    if state['part_orientation_rad']!=0:out['invalid_reason']='orientation_departure'
    return out
