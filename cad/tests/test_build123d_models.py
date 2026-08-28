from __future__ import annotations

import importlib.util
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


HAS_BUILD123D = importlib.util.find_spec("build123d") is not None


@unittest.skipUnless(HAS_BUILD123D, "build123d is not installed")
class Build123dModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from pipe_cad.params import default_design

        cls.config = default_design()

    def assert_valid_solid(self, shape) -> None:
        self.assertTrue(shape.is_valid)
        self.assertGreater(shape.volume, 0.0)
        self.assertGreater(len(shape.solids()), 0)

    def test_structure_models(self) -> None:
        from pipe_cad.structure import (
            make_rail,
            make_rail_carriage,
            make_theta_bogie,
            make_theta_bogie_body,
            make_theta_track,
            make_theta_track_quadrant,
            make_tube,
        )

        c = self.config
        for shape in (
            make_tube(c.tube),
            make_rail(c.rail, c.tube.usable_length),
            make_rail_carriage(c.carriage, c.rail),
            make_theta_track(c.tube),
            make_theta_track_quadrant(c.tube),
            make_theta_bogie(c.rail),
            make_theta_bogie_body(c.rail),
        ):
            self.assert_valid_solid(shape)

    def test_arm_and_sensing_models(self) -> None:
        from pipe_cad.arm import (
            make_actuator_bank,
            make_actuator_mount,
            make_arm_assembly,
            make_arm_segment,
            make_gripper_assembly,
            make_pitch_axis,
            make_pitch_yoke,
            make_shoulder_yaw_axis,
            make_shoulder_yaw_stage,
            make_wrist_roll_stage,
        )
        from pipe_cad.sensing import (
            make_camera_pod,
            make_camera_shell,
            make_fiducial_plate,
            make_macro_camera_pod,
            make_macro_camera_shell,
            make_macro_stereo_bridge,
            make_projector_pod,
            make_projector_shell,
        )

        c = self.config
        for shape in (
            make_arm_segment(c.arm, c.arm.link_lengths[0]),
            make_arm_segment(c.arm, c.arm.link_lengths[1]),
            make_arm_segment(c.arm, c.arm.link_lengths[2]),
            make_shoulder_yaw_stage(c.arm),
            make_shoulder_yaw_axis(c.arm),
            make_pitch_yoke(c.arm, "shoulder_pitch"),
            make_pitch_axis(c.arm, "shoulder_pitch"),
            make_pitch_yoke(c.arm, "elbow_pitch"),
            make_pitch_axis(c.arm, "elbow_pitch"),
            make_wrist_roll_stage(c.arm),
            make_arm_assembly(c.arm, c.gripper),
            make_gripper_assembly(c.gripper),
            make_actuator_bank(c.arm),
            make_actuator_mount(c.arm),
            make_camera_pod(c.sensing),
            make_camera_shell(c.sensing),
            make_macro_camera_pod(c.sensing),
            make_macro_camera_shell(c.sensing),
            make_macro_stereo_bridge(c.sensing),
            make_projector_pod(c.sensing),
            make_projector_shell(c.sensing),
            make_fiducial_plate(c.sensing, 17),
        ):
            self.assert_valid_solid(shape)
        self.assertIn("axis_Z", make_shoulder_yaw_stage(c.arm).label)
        self.assertIn("axis_Y", make_pitch_yoke(c.arm, "elbow_pitch").label)
        self.assertIn("axis_X", make_wrist_roll_stage(c.arm).label)

    def test_low_cost_tooling_models(self) -> None:
        from pipe_cad.assemblies import tooling_records
        from pipe_cad.tooling import (
            make_calibration_pointer,
            make_compliant_insertion_probe,
            make_gearbox_nest,
            make_parts_tray,
            make_rotary_drive_blade,
            make_vacuum_micro_pick,
        )

        c = self.config
        shapes = (
            make_gearbox_nest(c.tooling, c.gearbox),
            make_parts_tray(c.tooling, c.gearbox),
            make_vacuum_micro_pick(c.tooling),
            make_compliant_insertion_probe(c.tooling),
            make_rotary_drive_blade(c.tooling, c.gearbox),
            make_calibration_pointer(c.tooling),
        )
        for shape in shapes:
            self.assert_valid_solid(shape)
        self.assertEqual(
            [record.name for record in tooling_records(c)],
            [
                "gearbox_nest",
                "gearbox_parts_tray",
                "vacuum_micro_pick",
                "compliant_insertion_probe",
                "rotary_drive_blade",
                "calibration_pointer",
            ],
        )

    def test_probe_has_four_connected_fixed_guided_leaves(self) -> None:
        from build123d import Align, Box

        from pipe_cad.params import (
            PROBE_FLEXURE_LEAF_COUNT,
            nominal_probe_stiffness_n_per_mm,
        )
        from pipe_cad.tooling import make_compliant_insertion_probe

        tool = self.config.tooling
        probe = make_compliant_insertion_probe(tool)
        frame = next(
            child
            for child in probe.children
            if child.label == "insertion_probe_connected_four_leaf_frame"
        )

        # The anchor, all leaves, and the floating platform must form one load
        # path.  At mid-span only the four separate leaves may be present.
        self.assertEqual(len(frame.solids()), 1)
        midspan_slab = Box(
            0.05,
            tool.probe_base_width + 1.0,
            tool.probe_base_height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        self.assertEqual(
            len((frame & midspan_slab).solids()),
            PROBE_FLEXURE_LEAF_COUNT,
        )
        self.assertAlmostEqual(nominal_probe_stiffness_n_per_mm(tool), 0.60)
        self.assertGreaterEqual(0.10 / nominal_probe_stiffness_n_per_mm(tool), 0.10)

    def test_loose_pocket_carrier_is_not_labeled_as_a_restraint(self) -> None:
        from pipe_cad.assemblies import tooling_records
        from pipe_cad.tooling import make_gearbox_nest

        nest = make_gearbox_nest(self.config.tooling, self.config.gearbox)
        self.assertEqual(nest.label, "gearbox_loose_clearance_pocket_carrier")
        record = next(
            item for item in tooling_records(self.config) if item.name == "gearbox_nest"
        )
        joined_notes = " ".join(record.notes).lower()
        self.assertNotIn("3-2-1", joined_notes)
        self.assertIn("clamp", joined_notes)
        self.assertIn("not modeled", joined_notes)

    def test_gearbox_models_and_nominal_envelope(self) -> None:
        from pipe_cad.assemblies import gearbox_records, printable_catalog_records
        from pipe_cad.export import metadata_document
        from pipe_cad.gearbox import (
            make_gearbox_assembly,
            make_gearbox_housing,
            make_gearbox_lid,
            make_shaft,
            make_spur_gear,
        )

        g = self.config.gearbox
        g1 = make_spur_gear(g.input_teeth, g, drive_slot=True)
        g2 = make_spur_gear(g.idler_teeth, g)
        g3 = make_spur_gear(g.output_teeth, g, optical_phase_marks=True)
        for shape in (
            g1,
            g2,
            g3,
            make_gearbox_housing(g),
            make_gearbox_lid(g),
            make_gearbox_assembly(g),
            make_shaft(g),
        ):
            self.assert_valid_solid(shape)
        for gear in (g1, g2, g3):
            self.assertAlmostEqual(gear.bounding_box().size.Z, g.total_gear_height)
        assembly = make_gearbox_assembly(g)
        size = assembly.bounding_box().size
        self.assertLessEqual(size.X, g.housing_length + 1e-6)
        self.assertLessEqual(size.Y, g.housing_width + 1e-6)
        self.assertLessEqual(size.Z, 1.85 + 1e-6)
        records = gearbox_records(self.config)
        insertion_names = [
            record.name
            for record in records
            if record.name != "gearbox_housing"
        ]
        self.assertEqual(
            insertion_names,
            ["S1", "S2", "S3", "G3", "G2", "G1", "cover"],
        )
        metadata = metadata_document(records, self.config, "gearbox")
        self.assertEqual(
            metadata["gearbox_assembly_sequence"],
            ["S1", "S2", "S3", "G3", "G2", "G1", "cover"],
        )
        self.assertEqual(metadata["arm_actuation_bom"]["motor_count"], 16)
        self.assertEqual(metadata["arm_actuation_bom"]["spool_count"], 16)
        self.assertRegex(metadata["parameter_sha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(metadata["geometry_facts_sha256"], r"^[0-9a-f]{64}$")
        self.assertEqual(
            [axis["name"] for axis in metadata["arm_joint_axes_local"]],
            ["shoulder_yaw", "shoulder_pitch", "elbow_pitch", "wrist_roll"],
        )
        self.assertEqual(
            metadata["calibrated_radial_datums_mm"]["shoulder_root_radius"],
            72.0,
        )
        g1_metadata = next(record for record in metadata["records"] if record["name"] == "G1")
        self.assertGreater(g1_metadata["mass_g"], 0.0)
        self.assertEqual(g1_metadata["mass_properties_status"], "homogeneous_nominal_density")
        self.assertIsNotNone(g1_metadata["center_of_mass_mm"])
        self.assertIsNotNone(g1_metadata["inertia_tensor_centroidal_g_mm2"])
        catalog = printable_catalog_records(self.config)
        spool_record = next(record for record in catalog if record.name == "tendon_spool")
        mount_record = next(record for record in catalog if record.name == "actuator_mount")
        self.assertEqual(spool_record.quantity, 16)
        self.assertEqual(mount_record.quantity, 4)
        projector_record = next(
            record for record in catalog if record.name == "projector_pod_shell"
        )
        self.assertEqual(projector_record.quantity, 1)
        for name in (
            "shoulder_yaw_stage",
            "shoulder_pitch_yoke",
            "elbow_pitch_yoke",
            "wrist_roll_stage",
            "gearbox_nest",
            "gearbox_parts_tray",
            "vacuum_micro_pick",
            "compliant_insertion_probe",
            "rotary_drive_blade",
            "calibration_pointer",
            "macro_stereo_bridge",
        ):
            self.assertIn(name, {record.name for record in catalog})

    def test_gear_tolerance_perturbation_guard(self) -> None:
        from pipe_cad.gearbox import make_spur_gear

        g = self.config.gearbox
        nominal = make_spur_gear(g.input_teeth, g)
        loose = make_spur_gear(
            g.input_teeth,
            g,
            bore_delta=g.tolerance_perturbation,
        )
        self.assertGreater(nominal.volume, loose.volume)
        with self.assertRaises(ValueError):
            make_spur_gear(
                g.input_teeth,
                g,
                bore_delta=-(g.bore_diameter - g.shaft_diameter),
            )

    def test_committed_gearbox_metadata_matches_source_exactly(self) -> None:
        from pipe_cad.assemblies import gearbox_records
        from pipe_cad.export import metadata_document

        generated = (
            json.dumps(
                metadata_document(gearbox_records(self.config), self.config, "gearbox"),
                indent=2,
                sort_keys=True,
            )
            + "\n"
        )
        baseline = (
            Path(__file__).resolve().parents[1]
            / "baseline"
            / "gearbox.metadata.json"
        )
        self.assertEqual(generated, baseline.read_text(encoding="utf-8"))

    def test_cell_sensor_layout_is_parameter_locked_and_parked_clear(self) -> None:
        from math import dist

        from pipe_cad.assemblies import cell_records

        records = cell_records(self.config)
        by_name = {record.name: record for record in records}
        cameras = [by_name[f"camera_pod_{index:02d}"] for index in range(6)]
        macros = [by_name[f"macro_camera_{index:02d}"] for index in range(2)]
        macro_bridge = by_name["macro_stereo_bridge_00"]
        self.assertTrue(
            all(
                "front_face_datum_radius_mm=60.000" in record.notes
                for record in cameras
            )
        )
        macro_centers = []
        for record in macros:
            bbox = record.shape.bounding_box()
            macro_centers.append(
                (
                    (bbox.min.X + bbox.max.X) / 2,
                    (bbox.min.Y + bbox.max.Y) / 2,
                    (bbox.min.Z + bbox.max.Z) / 2,
                )
            )
            self.assertIn("locked_baseline_mm=12.000", record.notes)
        self.assertAlmostEqual(dist(*macro_centers), 12.0, places=6)
        parent_arm_box = by_name[
            f"tendon_arm_{self.config.sensing.macro_mount_arm_index:02d}"
        ].shape.bounding_box()
        for macro in macros:
            macro_box = macro.shape.bounding_box()
            self.assertGreater(macro_box.min.Z, parent_arm_box.max.Z)
            self.assertGreater(
                sum(
                    (macro_bridge.shape & solid).volume
                    for solid in macro.shape.solids()
                ),
                1e-6,
                "rigid macro bridge must physically overlap both pod shells",
            )
        self.assertGreater(
            macro_bridge.shape.bounding_box().min.Z,
            parent_arm_box.max.Z,
        )

        fixed_sensors = cameras + [by_name["projector_pod_00"]]
        parked_motion = [
            record
            for record in records
            if record.name.startswith(("tendon_arm_", "actuator_bank_"))
        ]
        for sensor in fixed_sensors:
            for motion in parked_motion:
                sensor_box = sensor.shape.bounding_box()
                motion_box = motion.shape.bounding_box()
                separated = (
                    sensor_box.max.X < motion_box.min.X
                    or motion_box.max.X < sensor_box.min.X
                    or sensor_box.max.Y < motion_box.min.Y
                    or motion_box.max.Y < sensor_box.min.Y
                    or sensor_box.max.Z < motion_box.min.Z
                    or motion_box.max.Z < sensor_box.min.Z
                )
                if not separated:
                    self.assertLess(
                        (sensor.shape & motion.shape).volume,
                        1e-9,
                        f"{sensor.name} intersects {motion.name}",
                    )


if __name__ == "__main__":
    unittest.main()
