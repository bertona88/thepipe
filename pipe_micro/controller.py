"""Observation-only executive. No import of plant or access to physical state."""
import math
from .contracts import digest


class Controller:
    def __init__(self,m,o):
        self.m,self.o=m,o;self.phase='BasePositioning';self.phase_tick=0
        self.events=[];self.reason=None;self.completed=False;self.responsibility='fixture'

    def event(self,name,obs):
        self.events.append({'event':name,'tick':obs['tick'],'evidence_ids':[v['sample_id'] for v in obs['objects'].values()],
                            'evidence_class':obs['evidence_class']})

    def hold(self,reason):
        self.reason=reason
        return {'kind':'hold','response':'freeze_motion_preserve_current_tendon_load','reason':reason}

    def advance(self,phase,obs,event=None):
        if event:self.event(event,obs)
        self.phase=phase;self.phase_tick=obs['tick']

    def decide(self,obs,tick):
        m=self.m;o=self.o;opt=m['optics']
        if self.reason:return self.hold(self.reason)
        if (obs['model_hash']!=digest(m) or obs['calibration_id']!=opt['calibration_id'] or obs['source_id']!=opt['source_id'] or obs['evidence_class']!=m['evidence_class']):return self.hold('observation_identity')
        if not 0<=tick-obs['tick']<=opt['max_age_ticks']:return self.hold('stale_observation')
        if obs['axes']!=['x','y','z']:return self.hold('unobservable_required_translation')
        if obs['invalid_reason']:return self.hold(obs['invalid_reason'])
        for name in ('part','donor','receiver'):
            v=obs['objects'].get(name)
            if not v or v['id']!=name or not v['valid']:return self.hold('missing_or_invalid_'+name)
            if len(v['xyz_m'])!=3 or any(not math.isfinite(x) for x in v['xyz_m']):return self.hold('invalid_position')
            if len(v['covariance_m2'])!=3 or any(not math.isfinite(x) or x<0 or math.sqrt(x)*6>=o['position_tolerance_m'] for x in v['covariance_m2']):return self.hold('uncertainty')
        if tick-self.phase_tick>600:return self.hold('phase_timeout_'+self.phase)
        p=obs['objects']['part']['xyz_m'];supports=obs['contacts']
        def move(role,target):return {'kind':'move','role':role,'xyz_m':target}
        def target(role,y):return [m['fixtures']['pickup_xyz_m'][0]+m['tool']['x_offsets_m'][role],y,0.]
        def at(role,t):return math.dist(obs['objects'][role]['xyz_m'],t)<o['position_tolerance_m']
        def jaw(role,opening):return {'kind':'jaw','role':role,'opening_m':opening}
        closed=2*m['specimen']['half_extents_m'][2];opened=m['tool']['jaw_open_m']
        phase=self.phase
        if phase=='BasePositioning':
            if abs(obs['base_z_m'])>1e-12 or abs(obs['base_theta_rad'])>1e-12:return {'kind':'base','z_m':0.,'theta_rad':0.}
            self.advance('PickupApproach',obs,'base_positioned')
        elif phase=='PickupApproach':
            t=target('donor',m['fixtures']['pickup_xyz_m'][1])
            if not at('donor',t):return move('donor',t)
            self.advance('Pickup',obs)
        elif phase=='Pickup':
            if 'donor' not in supports:return jaw('donor',closed)
            self.responsibility='donor';self.advance('Transfer',obs,'donor_retained')
        elif phase=='Transfer':
            if 'donor' not in supports:return self.hold('donor_retention_lost')
            t=target('donor',0.)
            if not at('donor',t):return move('donor',t)
            self.advance('ReceiverApproach',obs)
        elif phase=='ReceiverApproach':
            t=target('receiver',p[1])
            if not at('receiver',t):return move('receiver',t)
            self.advance('ReceiverContact',obs,'receiver_contact_requested')
        elif phase=='ReceiverContact':
            if 'receiver' not in supports:return jaw('receiver',closed)
            self.advance('DualSupported',obs,'dual_supported')
        elif phase=='DualSupported':
            if not {'donor','receiver'}<=set(supports):return self.hold('dual_support_lost')
            self.advance('DonorUnloading',obs,'release_commanded')
        elif phase=='DonorUnloading':
            if 'receiver' not in supports:return self.hold('receiver_retention_lost')
            if obs['objects']['donor']['jaw_m']<opened:return jaw('donor',opened)
            gap=obs['donor_gap_m']
            if 'donor' in supports or gap is None or gap-3*opt['sigma_m']<o['separation_m']:
                return self.hold('separation_unverified')
            self.advance('ReceiverRetained',obs,'separation_observed')
        elif phase=='ReceiverRetained':
            if 'receiver' not in supports:return self.hold('receiver_retention_lost')
            self.responsibility='receiver';self.advance('DonorWithdrawal',obs,'retention_verified')
        elif phase=='DonorWithdrawal':
            t=target('donor',-o['withdrawal_m'])
            if not at('donor',t):return move('donor',t)
            self.advance('DestinationMove',obs,'donor_withdrawn')
        elif phase=='DestinationMove':
            if 'receiver' not in supports:return self.hold('receiver_retention_lost')
            t=target('receiver',m['fixtures']['destination_xyz_m'][1])
            if not at('receiver',t):return move('receiver',t)
            self.advance('DestinationContact',obs)
        elif phase=='DestinationContact':
            # Finish the last tolerance-sized correction before fixture support is asserted.
            if 'destination' not in supports:return move('receiver',target('receiver',obs['objects']['receiver']['xyz_m'][1]+m['fixtures']['destination_xyz_m'][1]-p[1]))
            self.advance('DestinationRelease',obs,'destination_release_commanded')
        elif phase=='DestinationRelease':
            if 'destination' not in supports:return self.hold('destination_support_lost')
            if obs['objects']['receiver']['jaw_m']<opened:return jaw('receiver',opened)
            if 'receiver' in supports:return self.hold('destination_separation_unverified')
            self.advance('ReceiverWithdrawal',obs,'destination_separation_observed')
        elif phase=='ReceiverWithdrawal':
            t=target('receiver',m['fixtures']['destination_xyz_m'][1]+o['withdrawal_m'])
            if not at('receiver',t):return move('receiver',t)
            self.advance('FinalInspection',obs)
        elif phase=='FinalInspection':
            if set(supports)!={'destination'} or math.dist(p,m['fixtures']['destination_xyz_m'])>o['position_tolerance_m']:return self.hold('final_pose_unverified')
            self.responsibility='fixture';self.completed=True;self.event('final_pose_verified',obs)
            return {'kind':'hold','response':'completed_synthetic_operation'}
        # A bounded no-op jaw command advances one tick without implying an observation.
        return jaw('donor',obs['objects']['donor']['jaw_m'])
