from __future__ import annotations

import sys
import unittest
import hashlib
import json
from math import hypot, pi
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pipe_cad.gear_math import involute_gear_outline, transverse_contact_ratio
from pipe_cad.digital_thread import canonical_sha256
from pipe_cad.kinematics import distance, four_axis_arm_frames, four_axis_arm_points
from pipe_cad.params import GEARBOX_INSERTION_SEQUENCE, DesignConfig, default_design


class ParameterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = default_design()

    def test_qualified_cell_baseline(self) -> None:
        self.assertEqual(self.config.validation_errors(), [])
        self.assertEqual(self.config.tube.inner_radius, 80.0)
        self.assertEqual(self.config.tube.usable_length, 320.0)
        self.assertEqual(self.config.tube.rail_count, 4)
        self.assertEqual(self.config.sensing.global_camera_count, 6)
        self.assertEqual(self.config.sensing.simultaneous_macro_view_count, 2)
        self.assertEqual(self.config.sensing.structured_light_projector_count, 1)
        self.assertEqual(self.config.sensing.global_camera_front_radius, 60.0)
        self.assertEqual(
            self.config.sensing.global_camera_end_offsets,
            (-106.0, 106.0),
        )
        self.assertEqual(self.config.sensing.macro_stereo_baseline, 12.0)
        self.assertEqual(
            (
                self.config.sensing.global_image_width_px,
                self.config.sensing.global_image_height_px,
            ),
            (1280, 800),
        )
        self.assertEqual(
            (
                self.config.sensing.macro_image_width_px,
                self.config.sensing.macro_image_height_px,
            ),
            (2048, 1536),
        )
        self.assertEqual(self.config.arm.link_lengths, (32.0, 30.0, 15.0))
        self.assertEqual(self.config.arm.actuator_count, 4)
        self.assertEqual(
            self.config.arm.actuator_axes,
            ("shoulder_yaw", "shoulder_pitch", "elbow_pitch", "wrist_roll"),
        )
        self.assertEqual(self.config.arm.tendon_diameter, 0.20)
        self.assertEqual(self.config.arm.spool_radius, 3.0)
        self.assertEqual(self.config.arm.shoulder_root_radius, 72.0)
        self.assertEqual(len(self.config.arm.default_joint_angles_deg), 4)
        rail_center = (
            self.config.tube.inner_radius
            - self.config.rail.wall_standoff
            - self.config.carriage.radial_depth / 2
        )
        self.assertAlmostEqual(rail_center, 76.9)
        self.assertNotEqual(rail_center, self.config.arm.shoulder_root_radius)

    def test_micro_gear_pitch_diameters_and_ratio(self) -> None:
        g = self.config.gearbox
        pitch_diameters = [
            g.module * g.input_teeth,
            g.module * g.idler_teeth,
            g.module * g.output_teeth,
        ]
        for actual, expected in zip(pitch_diameters, (1.2, 1.8, 2.4), strict=True):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(g.ratio, 2.0)
        for actual, expected in zip(
            (g.input_center_x, g.idler_center_x, g.output_center_x),
            (0.75, 2.25, 4.35),
            strict=True,
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(g.pressure_angle_deg, 25.0)
        self.assertEqual(g.backlash, 0.020)
        self.assertEqual(g.bore_diameter, 0.420)
        self.assertEqual(g.bore_entry_chamfer, 0.025)
        self.assertEqual(g.bore_entry_chamfer_angle_deg, 45.0)
        self.assertEqual(g.shaft_length, 1.55)
        self.assertLessEqual(g.shaft_deburr_chamfer, 0.005)
        self.assertEqual((g.housing_length, g.housing_width), (6.0, 4.0))
        self.assertEqual(g.housing_height, 1.60)
        self.assertEqual(g.housing_floor, 0.25)
        self.assertEqual(g.lid_thickness, 0.20)
        self.assertEqual(g.gear_thickness, 0.35)
        self.assertEqual(g.total_gear_height, 1.30)
        self.assertEqual(g.gear_z, g.housing_floor)
        self.assertAlmostEqual(g.gear_z + g.total_gear_height, 1.55)
        self.assertAlmostEqual(g.cover_under_clearance, 0.05)
        self.assertEqual(g.shaft_seat_diameter, 0.340)
        self.assertEqual(g.shaft_seat_depth, 0.250)
        self.assertEqual(g.input_driver_window_diameter, 0.75)
        self.assertEqual(g.output_observation_window_diameter, 1.00)
        self.assertEqual(g.input_drive_slot_width, 0.10)
        self.assertGreater(
            g.output_phase_mark_long_length,
            g.output_phase_mark_short_length,
        )
        self.assertEqual((g.latch_count, g.datum_count), (2, 3))
        self.assertLessEqual(g.housing_height + g.lid_thickness, 1.85)
        self.assertLessEqual(
            g.housing_height + g.lid_thickness + g.datum_pad_height,
            1.85,
        )
        self.assertEqual(
            GEARBOX_INSERTION_SEQUENCE,
            ("S1", "S2", "S3", "G3", "G2", "G1", "cover"),
        )
        self.assertAlmostEqual(
            g.idler_center_x - g.input_center_x,
            g.input_pitch_radius + g.idler_pitch_radius,
        )
        self.assertAlmostEqual(
            g.output_center_x - g.idler_center_x,
            g.idler_pitch_radius + g.output_pitch_radius,
        )

    def test_involute_outline_radii(self) -> None:
        g = self.config.gearbox
        for teeth in (g.input_teeth, g.idler_teeth, g.output_teeth):
            profile = involute_gear_outline(
                teeth,
                g.module,
                g.pressure_angle_deg,
                g.backlash,
            )
            radii = [hypot(x, y) for x, y in profile.points]
            self.assertAlmostEqual(min(radii), profile.root_radius, places=9)
            self.assertAlmostEqual(max(radii), profile.tip_radius, places=9)
            nominal_tooth_thickness = pi * g.module / 2
            self.assertAlmostEqual(
                profile.pitch_tooth_thickness,
                nominal_tooth_thickness - g.backlash / 2,
                places=12,
            )
            # One mating space plus one tooth must produce the requested
            # per-mesh circular backlash, not twice that value.
            mating_space = pi * g.module - profile.pitch_tooth_thickness
            self.assertAlmostEqual(
                mating_space - profile.pitch_tooth_thickness,
                g.backlash,
                places=12,
            )
            self.assertGreater(len(profile.points), teeth * 10)

        self.assertGreater(
            transverse_contact_ratio(
                g.input_teeth,
                g.idler_teeth,
                g.module,
                g.pressure_angle_deg,
            ),
            1.20,
        )
        self.assertGreater(
            transverse_contact_ratio(
                g.idler_teeth,
                g.output_teeth,
                g.module,
                g.pressure_angle_deg,
            ),
            1.20,
        )

    def test_arm_forward_kinematics(self) -> None:
        a = self.config.arm
        points = four_axis_arm_points(
            a.link_lengths,
            a.default_joint_angles_deg,
        )
        self.assertEqual(len(points), a.link_count + 1)
        for index, (p0, p1) in enumerate(
            zip(points[:-1], points[1:], strict=True)
        ):
            self.assertAlmostEqual(
                distance(p0, p1),
                a.link_lengths[index],
                places=9,
            )
        frames = four_axis_arm_frames(a.link_lengths, a.default_joint_angles_deg)
        self.assertEqual(
            tuple(frame["name"] for frame in frames),
            ("shoulder_yaw", "shoulder_pitch", "elbow_pitch", "wrist_roll"),
        )
        self.assertEqual(frames[0]["direction"], (0.0, 0.0, 1.0))
        self.assertEqual(frames[1]["origin_mm"], points[0])
        self.assertEqual(frames[2]["origin_mm"], points[1])
        self.assertEqual(frames[3]["origin_mm"], points[-1])

    def test_tooling_and_digital_thread_parameters(self) -> None:
        t, g = self.config.tooling, self.config.gearbox
        self.assertLess(t.rotary_blade_width, g.input_drive_slot_width)
        self.assertGreater(t.nest_pocket_depth, 0.0)
        self.assertGreater(t.tray_pocket_depth, 0.0)
        parameters = self.config.to_dict()
        canonical = json.dumps(
            parameters,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")
        expected = hashlib.sha256(canonical).hexdigest()
        self.assertEqual(canonical_sha256(parameters), expected)
        self.assertRegex(expected, r"^[0-9a-f]{64}$")

    def test_bad_qualified_target_is_rejected(self) -> None:
        bad = DesignConfig()
        # Frozen nested dataclasses prevent accidental mutation during export.
        with self.assertRaises((AttributeError, TypeError)):
            bad.tube.length = 100.0  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()
