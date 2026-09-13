"""Synthetic bounded plant. Only sensor.py may translate physical state to observations.
Contact retention is sampled once from declared bounds, not a second adhesion force.
This model is not a FEM, tendon dynamics solver or validated friction/contact law.
"""
import copy
import math
from .geometry import envelopes, overlap, frame_point


def approach(a,b,step):
    return a+max(-step,min(step,b-a))


class Plant:
    def __init__(self,m,o):
        self.m,self.o=m,o
        self.state={'base_z_m':m['transport']['start_z_m'],'base_theta_rad':m['transport']['start_theta_rad'],
          'tools':{},'part_xyz_m':list(m['fixtures']['pickup_xyz_m']),'supports':['pickup'],
          'environment':copy.deepcopy(m['environment']),'part_orientation_rad':0.,'stopped':False}
        for role,sign in [('donor',-1),('receiver',1)]:
            self.state['tools'][role]={'xyz_m':[m['fixtures']['pickup_xyz_m'][0]+m['tool']['x_offsets_m'][role],sign*.0008,0.],
              'jaw_m':m['tool']['jaw_open_m'],'tension_n':m['flexure']['pretension_n'],'energy_j':0.}
        for tool in self.state['tools'].values():
            tool['xyz_m']=frame_point(tool['xyz_m'],self.state['base_z_m'],self.state['base_theta_rad'])
        self.release_requested=set(); self.fault=o['fault'];self.failure=None
        # Explicit reproducible contact sample; no reliance on platform random state.
        u=((o['seed']*1664525+1013904223)&0xffffffff)/4294967296
        lo,hi=m['assumptions']['contact_retention_n'];self.capacity_n=lo+(hi-lo)*u
        lo,hi=m['assumptions']['pull_off_n'];self.pull_off_n=lo+(hi-lo)*u

    def step(self,command):
        s=self.state;m=self.m;o=self.o;dt=o['dt_s']
        if command['kind']=='hold':
            s['stopped']=True
            return
        if s['stopped']:raise ValueError('motion after stop')
        if command['kind']=='base':
            if self.fault=='service_wrap':self.failure='service_wrap';return
            require_empty=set(s['supports'])=={'pickup'} and all(t['jaw_m']==m['tool']['jaw_open_m'] for t in s['tools'].values())
            if not require_empty:self.failure='base_motion_with_loaded_tools';return
            base_before=copy.deepcopy(s)
            for targetkey,limitkey in [('z_m','z_limits_m'),('theta_rad','theta_limits_rad')]:
                lo,hi=m['transport'][limitkey]
                if not lo<=command[targetkey]<=hi:self.failure='travel_limit';return
            old_z,old_theta=s['base_z_m'],s['base_theta_rad']
            for statekey,targetkey,limitkey,speed in [('base_z_m','z_m','z_limits_m','speed_m_s'),('base_theta_rad','theta_rad','theta_limits_rad','speed_rad_s')]:
                target=command[targetkey];lo,hi=m['transport'][limitkey]
                if not lo<=target<=hi:self.failure='travel_limit';return
                s[statekey]=approach(s[statekey],target,m['transport'][speed]*dt)
            for tool in s['tools'].values():
                local=frame_point(tool['xyz_m'],-old_z,-old_theta)
                tool['xyz_m']=frame_point(local,s['base_z_m'],s['base_theta_rad'])
            if self.collision():
                s.clear();s.update(base_before);self.failure='base_collision'
            return
        if command['kind'] not in ('move','jaw'):raise ValueError('unknown physical command')
        role=command['role'];tool=s['tools'][role];before=copy.deepcopy(s)
        if command['kind']=='move':
            target=command['xyz_m']
            if abs(target[1])>m['flexure']['stroke_m'] or target[0]!=m['fixtures']['pickup_xyz_m'][0]+m['tool']['x_offsets_m'][role] or target[2]!=0:
                self.failure='unsupported_flexure_pose';return
            if self.fault=='blocked_withdrawal' and role=='donor' and role in self.release_requested:
                self.failure='blocked_withdrawal';return
            old=list(tool['xyz_m'])
            tool['xyz_m']=[approach(a,b,m['tool']['tool_speed_m_s']*dt) for a,b in zip(old,target)]
            if role in s['supports']:
                delta=[a-b for a,b in zip(tool['xyz_m'],old)]
                # Moving under dual support is prohibited until one interface separates.
                if any(x in s['supports'] for x in ('donor','receiver') if x!=role):
                    s.clear();s.update(before);self.failure='loaded_dual_support_motion';return
                s['part_xyz_m']=[p+d for p,d in zip(s['part_xyz_m'],delta)]
                s['supports']=[role]
        else:
            target=command['opening_m']
            if not 2*m['specimen']['half_extents_m'][2]<=target<=m['tool']['jaw_open_m']:
                self.failure='jaw_travel_limit';return
            if target>tool['jaw_m']:
                self.release_requested.add(role)
                if self.fault=='blocked_jaw':self.failure='blocked_jaw';return
            tool['jaw_m']=approach(tool['jaw_m'],target,m['tool']['jaw_speed_m_s']*dt)
        q=tool['xyz_m'][1];f=m['flexure'];tool['tension_n']=f['pretension_n']+f['stiffness_n_m']*abs(q)
        tool['energy_j']=.5*f['stiffness_n_m']*q*q
        if tool['tension_n']>f['max_tension_n']:
            s.clear();s.update(before);self.failure='tendon_limit';return
        self.contacts()
        if self.collision():
            s.clear();s.update(before);self.failure='collision';return
        if not s['supports']:self.failure='unsupported_part'

    def contacts(self):
        s=self.state;m=self.m;p=s['part_xyz_m'];half=m['specimen']['half_extents_m']
        for role,t in s['tools'].items():
            contact=abs(t['xyz_m'][1]-p[1])<m['tool']['pad_half_extents_m'][1] and t['jaw_m']<=2*half[2]+1e-12
            capacity=self.capacity_n
            if role=='receiver' and self.fault=='receiver_slip':capacity=0
            if contact and capacity>m['specimen']['mass_kg']*m['assumptions']['gravity_m_s2'] and role not in s['supports']:
                s['supports'].append(role)
            if role in self.release_requested and role in s['supports'] and t['jaw_m']>2*half[2]+self.o['separation_m']:
                stuck=role=='donor' and self.fault in ('donor_adhesion','residual_bridge')
                if not stuck and self.capacity_n>self.pull_off_n:s['supports'].remove(role)
        for fixture in ('pickup','destination'):
            if math.dist(p,m['fixtures'][fixture+'_xyz_m'])<self.o['position_tolerance_m']/2 and fixture not in s['supports']:s['supports'].append(fixture)
            if math.dist(p,m['fixtures'][fixture+'_xyz_m'])>=self.o['position_tolerance_m']/2 and fixture in s['supports']:s['supports'].remove(fixture)
        if self.fault=='part_rotation' and 'receiver' in s['supports']:s['part_orientation_rad']=.1

    def collision(self):
        geo=envelopes(self.m,self.state)
        chamber=self.m['chamber']
        if any(math.hypot(abs(b['center_m'][0])+b['half_extents_m'][0],abs(b['center_m'][1])+b['half_extents_m'][1]) >= chamber['radius_m'] or abs(b['center_m'][2])+b['half_extents_m'][2] >= chamber['half_length_m'] for b in geo):return True
        # Full opposite-tool envelopes; self connected components are not obstacles.
        a=[b for b in geo if b['id'].startswith('donor/')];b=[b for b in geo if b['id'].startswith('receiver/')]
        if any(overlap(x,y) for x in a for y in b):return True
        # Pad/part contact is intended. Fingers/palms/supports/services are never exempt.
        part=next(b for b in geo if b['id']=='part')
        if any(overlap(part,x) for x in a+b if x['role']!='contact_pad'):return True
        # Fixtures use narrow support contact at z=-part_half_height. Jaws touch coupon ends
        # beyond fixture lateral width (fixture geometry selected accordingly).
        fixtures=[x for x in geo if x['role']=='fixture']
        if any(overlap(x,y) for x in a+b for y in fixtures):return True
        return False
