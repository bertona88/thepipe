"""Runtime-frame distal candidate; metres at input, millimetres in CAD.

This opt-in path deliberately does not reinterpret the legacy CAD arm's frames.
The candidate uses the machine configuration and runtime's intrinsic Y/X/X/Z
rotations. Boxes are nominal palm/finger/pad bodies, not an actuator mechanism.
"""
from __future__ import annotations

import hashlib
import json
from math import cos, sin, isfinite
from pathlib import Path


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transform(r, v):
    return [sum(r[i][j] * v[j] for j in range(3)) for i in range(3)]


def rotation(axis, angle):
    c, s = cos(angle), sin(angle)
    if axis == 'x':
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if axis == 'y':
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def runtime_frames(config, positions):
    """Independent CAD expression of Rust SerialArm forward kinematics."""
    a = config['arm']
    theta = positions.get('base_theta_rad', 0.0)
    radius = config['carriage']['shoulder_datum_radius_m']
    p = [radius * cos(theta), radius * sin(theta), positions.get('base_z_m', 0.0)]
    # Columns at theta=0: X=-world Y, Y=world Z, Z=-world X.
    r = multiply(rotation('z', theta), [[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    r = multiply(r, multiply(rotation('y', positions.get('shoulder_yaw_rad', 0.0)), rotation('x', positions.get('shoulder_pitch_rad', 0.0))))
    frames = [{'name': 'shoulder', 'position_m': p[:], 'rotation': r}]
    for i, length in enumerate(a['link_lengths_m']):
        delta = transform(r, [0, 0, length])
        p = [p[j] + delta[j] for j in range(3)]
        if i == 0:
            r = multiply(r, rotation('x', positions.get('elbow_pitch_rad', 0.0)))
        elif i == 1:
            r = multiply(r, rotation('z', positions.get('wrist_roll_rad', 0.0)))
        frames.append({'name': ('elbow', 'wrist_origin', 'wrist_endpoint')[i], 'position_m': p[:], 'rotation': r})
    delta = transform(r, [0, 0, a['tool_standoff_m']])
    frames.append({'name': 'tcp', 'position_m': [p[j] + delta[j] for j in range(3)], 'rotation': r})
    return frames


def distal_boxes(config, opening_m):
    """Five boxes, in the same stable order as runtime distal primitives."""
    g = config['gripper']
    d = g['distal_geometry']
    h = g['jaw_half_extents_m']
    if not g['min_opening_m'] <= opening_m <= g['max_opening_m']:
        raise ValueError('opening outside configured limits')
    palm = d['palm_half_extents_m']
    rear = d['palm_center_z_m'] + palm[2]
    front = -h[2]
    if not rear < front or config['arm']['tool_standoff_m'] <= 0:
        raise ValueError('invalid distal attachment span')
    finger_xy = d['finger_half_extents_xy_m']
    if any(not isfinite(x) or x <= 0 for x in [*h, *palm, *finger_xy]):
        raise ValueError('nonphysical box dimensions')
    offset = opening_m / 2 + h[0]
    result = [{'name': 'palm', 'center_m': [0, 0, d['palm_center_z_m']], 'half_extents_m': palm}]
    for side, label in [(-1, 'left'), (1, 'right')]:
        result.append({'name': label + '_finger', 'center_m': [side * offset, 0, (rear + front)/2], 'half_extents_m': [*finger_xy, (front-rear)/2]})
    for side, label in [(-1, 'left'), (1, 'right')]:
        result.append({'name': label + '_pad', 'center_m': [side * offset, 0, 0], 'half_extents_m': h})
    return result


def make_distal_candidate(config, opening_m):
    from build123d import Align, Box, Compound, Pos
    shapes = []
    for primitive in distal_boxes(config, opening_m):
        shape = Pos(*[x * 1000 for x in primitive['center_m']]) * Box(*[x * 2000 for x in primitive['half_extents_m']], align=(Align.CENTER,)*3)
        shape.label = primitive['name']
        shapes.append(shape)
    result = Compound(children=shapes)
    result.label = 'runtime_distal_candidate_tcp_frame'
    return result


def export_candidate(config_path, output, opening_m=None):
    from build123d import export_step, export_stl
    from .export import shape_metadata
    source = Path(config_path).read_bytes()
    config = json.loads(source)
    opening = config['gripper']['max_opening_m'] if opening_m is None else opening_m
    model = make_distal_candidate(config, opening)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    export_step(model, output / 'arm_distal_candidate.step')
    export_stl(model, output / 'arm_distal_candidate.stl', tolerance=0.005)
    metadata = {'schema': 'pipe_arm_distal_candidate_v1', 'configuration_sha256': hashlib.sha256(source).hexdigest(), 'configuration_id': config['id'], 'units': 'mm', 'frame': 'runtime_tcp: jaw closing X; approach Z; wrist endpoint at negative Z', 'opening_m': opening, 'primitives_tcp_m': distal_boxes(config, opening), 'neutral_runtime_frames': runtime_frames(config, {}), 'geometry': shape_metadata(model), 'fidelity': 'Nominal palm, sliding finger and pad solids. Actuator drive, guides, fasteners, material strength and manufacturing tolerances unqualified. Not a build-ready gripper.'}
    (output / 'arm_distal_candidate.metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    return metadata


def make_recorded_fixture(description, frame, body_ids):
    """Exact primitive fixture solids from an authoritative scene sample.

    Requires enabled truth geometry explicitly; this is an evaluation CAD
    export and never represents unavailable estimates as observations.
    """
    from math import acos, degrees, sqrt
    from build123d import Align, Axis, Box, Compound, Cylinder, Pos, Sphere
    truth = frame.get('truth')
    if truth is None:
        raise ValueError('recorded simulation truth required for fixture CAD')
    definitions = {b['id']: b for b in description['rigid_bodies']}
    states = {b['id']: b for b in truth['rigid_bodies']}
    shapes = []
    for body_id in body_ids:
        body = states[body_id]
        if not body['enabled']:
            raise ValueError('disabled fixture component')
        definition = definitions[body_id]
        g = definition['shape']
        if g['kind'] == 'sphere':
            shape = Sphere(1000*g['radius_m'])
        elif g['kind'] == 'box':
            shape = Box(*[2000*x for x in g['half_extents_m']], align=(Align.CENTER,)*3)
        elif g['kind'] == 'capsule':
            radius, half = g['radius_m']*1000, g['half_segment_m']*1000
            shape = Cylinder(radius, 2*half, align=(Align.CENTER,)*3) + [Pos(Z=z)*Sphere(radius) for z in (-half,half)]
        else:
            raise ValueError('unsupported fixture primitive: ' + g['kind'])
        x,y,z,w = body['pose']['rotation_xyzw']
        if abs(x*x+y*y+z*z+w*w-1)>1e-8:
            raise ValueError('invalid fixture quaternion')
        scale = sqrt(x*x+y*y+z*z)
        if scale > 1e-12:
            shape = shape.rotate(Axis((0,0,0),(x/scale,y/scale,z/scale)), degrees(2*acos(max(-1,min(1,w)))))
        shape = Pos(*[1000*v for v in body['pose']['translation_m']]) * shape
        shape.label = f"body_{body_id}_{definition['name']}"
        shapes.append(shape)
    if not shapes:
        raise ValueError('fixture export requires explicit body IDs')
    result = Compound(children=shapes)
    result.label = 'recorded_pickup_return_fixture'
    return result


def verify_recorded_arm_frames(config, frame, tolerance_m=1e-10):
    """Fail closed if an executed articulated sample disagrees with CAD FK."""
    if frame.get('truth') is None:
        raise ValueError('frame has no simulation truth for geometric validation')
    if not frame['truth']['manipulators']:
        raise ValueError('frame contains no articulated arm samples')
    max_error = 0.0
    for arm in frame['truth']['manipulators']:
        q = dict(zip(('shoulder_yaw_rad','shoulder_pitch_rad','elbow_pitch_rad','wrist_roll_rad'), arm['joint_positions_rad']))
        q.update(base_theta_rad=arm['carriage_theta_rad'],base_z_m=arm['carriage_z_m'])
        frames = runtime_frames(config,q)
        for index, pose_name in ((0,'shoulder_pose'),(1,'elbow_pose'),(2,'wrist_pose'),(4,'tool_pose')):
            expected, recorded = frames[index]['position_m'], arm[pose_name]['translation_m']
            error = sum((a-b)**2 for a,b in zip(expected,recorded))**.5
            if not isfinite(error) or error > tolerance_m:
                raise ValueError(f"CAD/runtime frame disagreement at {pose_name}: {error}m")
            max_error = max(max_error,error)
        x,y,z,w = arm['tool_pose']['rotation_xyzw']
        recorded_rotation = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)], [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)], [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
        for i in range(3):
            for j in range(3):
                if abs(recorded_rotation[i][j]-frames[-1]['rotation'][i][j]) > 1e-9:
                    raise ValueError('CAD/runtime TCP orientation disagreement')
    return max_error
