"""Candidate geometry and runtime-frame invariants, independent of legacy CAD."""
import json
from pathlib import Path
import sys
import unittest
from math import pi
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipe_cad.arm_candidate import distal_boxes, runtime_frames, make_distal_candidate


def candidate():
    path = Path(__file__).resolve().parents[2] / 'scenarios/machine_baseline_v1.json'
    c = json.loads(path.read_text())
    c['arm']['tool_standoff_m'] = .005
    c['gripper']['jaw_half_extents_m'] = [.0001,.0011,.0006]
    c['gripper']['distal_geometry'] = {'palm_center_z_m':-.004,'palm_half_extents_m':[.0018,.0013,.001],'finger_half_extents_xy_m':[.0001,.0011]}
    return c


class CandidateTests(unittest.TestCase):
    def test_physical_attachment_and_gear_gap(self):
        c = candidate()
        b = distal_boxes(c, .002)
        self.assertAlmostEqual(b[0]['center_m'][2]-b[0]['half_extents_m'][2], -.005)
        self.assertAlmostEqual(b[1]['center_m'][2]-b[1]['half_extents_m'][2], -.003)
        self.assertAlmostEqual(b[1]['center_m'][2]+b[1]['half_extents_m'][2], -.0006)
        self.assertAlmostEqual(b[3]['center_m'][0]+b[3]['half_extents_m'][0], -.001)
        self.assertAlmostEqual(b[4]['center_m'][0]-b[4]['half_extents_m'][0], .001)
        self.assertGreaterEqual(b[0]['half_extents_m'][0], .0028/2 + 2*.0001)

    def test_neutral_frame_and_roll(self):
        c = candidate()
        f = runtime_frames(c, {})
        self.assertAlmostEqual(f[-2]['position_m'][0], -.005)
        self.assertAlmostEqual(f[-1]['position_m'][0], -.010)
        roll = runtime_frames(c, {'wrist_roll_rad': pi/2})
        for a,b in zip(f[-1]['position_m'],roll[-1]['position_m']):
            self.assertAlmostEqual(a,b)
        self.assertAlmostEqual(roll[-1]['rotation'][2][0],1.)

    def test_base_rotation_and_translation(self):
        c = candidate()
        f = runtime_frames(c, {'base_theta_rad':pi/2, 'base_z_m':.02})
        self.assertAlmostEqual(f[-1]['position_m'][0],0.)
        self.assertAlmostEqual(f[-1]['position_m'][1],-.01)
        self.assertAlmostEqual(f[-1]['position_m'][2],.02)

    def test_build123d_solids_dimensions(self):
        model = make_distal_candidate(candidate(), .0028)
        self.assertTrue(model.is_valid)
        self.assertEqual(len(model.solids()),5)
        box = model.bounding_box()
        self.assertAlmostEqual(box.min.Z,-5.)
        self.assertAlmostEqual(box.max.Z,.6)
        self.assertAlmostEqual(box.size.X,3.6)

    def test_invalid_opening_refused(self):
        with self.assertRaises(ValueError):
            distal_boxes(candidate(), .01)
