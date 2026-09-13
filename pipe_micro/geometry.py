"""Shared conservative envelopes in metres, common to records and CAD export.
These boxes are reduced geometry, not fabricated solids or a BREP parity claim.
"""
import math


def frame_point(p, z, theta):
    c,s=math.cos(theta),math.sin(theta)
    return [c*p[0]-s*p[1],s*p[0]+c*p[1],p[2]+z]


def box(identity, center, half, role):
    return dict(id=identity, center_m=list(center), half_extents_m=list(half), role=role)


def overlap(a,b,margin=0):
    return all(abs(x-y)<u+v+margin for x,y,u,v in zip(a['center_m'],b['center_m'],a['half_extents_m'],b['half_extents_m']))


def envelopes(m, state):
    result=[]; t=m['tool']; half=t['pad_half_extents_m']
    for role,tool in state['tools'].items():
        p=tool['xyz_m']; sign=-1 if role=='donor' else 1
        for i,direction in enumerate([-1,1]):
            pad=[p[0],p[1],p[2]+direction*(tool['jaw_m']/2+half[2])]
            result.append(box(f'{role}/pad/{i}',pad,half,'contact_pad'))
            finger=[pad[0],pad[1]+sign*(half[1]+t['finger_length_m']/2),pad[2]]
            result.append(box(f'{role}/finger/{i}',finger,[t['finger_half_width_m'],t['finger_length_m']/2,half[2]],'finger'))
        palm=[p[0],p[1]+sign*(half[1]+t['finger_length_m']+half[1]),p[2]]
        result.append(box(f'{role}/palm',palm,[half[0],half[1],t['jaw_open_m']/2+2*half[2]],'palm'))
        shoulder=frame_point(m['shoulders'][role],state['base_z_m'],state['base_theta_rad'])
        # Conservative bounding box includes the compliant swept lateral domain.
        mid=[(a+b)/2 for a,b in zip(shoulder,palm)]
        ext=[abs(a-b)/2+w for a,b,w in zip(shoulder,palm,[half[0],half[1],m['flexure']['beam_thickness_m']/2])]
        result.append(box(f'{role}/flexure',mid,ext,'flexure_envelope'))
        result.append(box(f'{role}/shoulder',shoulder,[.0003,.0003,.0003],'support'))
        # Declared service corridor terminates at the support; does not cross work volume.
        result.append(box(f'{role}/service',[shoulder[0],shoulder[1],-.012],[.00015,.00015,.0115],'service'))
    result.append(box('part',state['part_xyz_m'],m['specimen']['half_extents_m'],'part'))
    for key in ['pickup','destination']:
        p=list(m['fixtures'][key+'_xyz_m']);p[2]-=m['specimen']['half_extents_m'][2]+m['fixtures']['half_extents_m'][2]
        result.append(box(key+'/fixture',p,m['fixtures']['half_extents_m'],'fixture'))
    return result


def ray_clear(start, end, geometry, excluded):
    # Finite segment/AABB slab intersection. Endpoint object deliberately excluded.
    for b in geometry:
        if b['id'] in excluded: continue
        low,high=0.,1.
        for a,v,c,h in zip(start,[y-x for x,y in zip(start,end)],b['center_m'],b['half_extents_m']):
            if abs(v)<1e-15:
                if not c-h <= a <= c+h: low,high=1.,0.;break
            else:
                x,y=sorted(((c-h-a)/v,(c+h-a)/v));low=max(low,x);high=min(high,y)
        if low < high and high>1e-6 and low<1-1e-6: return False
    return True
