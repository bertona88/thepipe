"""Deterministic world placement and printable-part catalogs."""

from __future__ import annotations

from build123d import Pos, Rot

from .arm import (
    make_actuator_bank,
    make_actuator_mount,
    make_arm_assembly,
    make_arm_segment,
    make_gripper_jaw,
    make_gripper_palm,
    make_pitch_yoke,
    make_shoulder_yaw_stage,
    make_tendon_spool,
    make_wrist_roll_stage,
)
from .gearbox import (
    make_gearbox_assembly,
    make_gearbox_housing,
    make_gearbox_lid,
    make_shaft,
    make_spur_gear,
)
from .kinematics import four_axis_arm_points
from .params import DesignConfig
from .records import PartRecord
from .sensing import (
    make_camera_pod,
    make_camera_shell,
    make_fiducial_plate,
    make_macro_camera_pod,
    make_macro_camera_shell,
    make_macro_stereo_bridge,
    make_projector_pod,
    make_projector_shell,
)
from .structure import (
    make_rail,
    make_rail_carriage,
    make_theta_bogie,
    make_theta_bogie_body,
    make_theta_track,
    make_theta_track_quadrant,
    make_tube,
)
from .tooling import (
    make_calibration_pointer,
    make_compliant_insertion_probe,
    make_gearbox_nest,
    make_parts_tray,
    make_rotary_drive_blade,
    make_vacuum_micro_pick,
)


DIY_PRINT = "desktop_MSLA_or_FDM_as_noted"
MICRO_PRINT = "two_photon_polymerization"
PURCHASED = "purchased_or_cut_to_length"


def cell_records(config: DesignConfig) -> list[PartRecord]:
    """Qualified four-arm cell in one snapshot of its independent z/θ axes."""

    t, r, c, a, gp, sp = (
        config.tube,
        config.rail,
        config.carriage,
        config.arm,
        config.gripper,
        config.sensing,
    )
    records = [
        PartRecord(
            "cell_tube",
            "structure",
            make_tube(t),
            t.material,
            PURCHASED,
            printable=False,
            notes=("axis is +Z", "clear tube preferred during optical bring-up"),
        )
    ]
    for name, z in (
        ("theta_track_lower", 1.0),
        ("theta_track_upper", t.length - 4.0),
    ):
        records.append(
            PartRecord(
                name,
                "structure",
                Pos(Z=z) * make_theta_track(t),
                "printed_PETG_track_with_UHMW_wear_strip",
                DIY_PRINT,
                notes=("common circular guide; each rail has its own drive bogie",),
            )
        )
    rail_length = t.length - 2 * t.end_margin
    # Size the radial offset for the wider carriage/bogie envelope rather than
    # only the bare rail so no nominal pose intersects the tube wall.
    rail_center_radius = t.inner_radius - r.wall_standoff - c.radial_depth / 2
    for index in range(t.rail_count):
        angle = 360.0 * index / t.rail_count
        rail_shape = (
            Rot(Z=angle)
            * Pos(rail_center_radius, 0, t.end_margin)
            * make_rail(r, rail_length)
        )
        records.append(
            PartRecord(
                f"rail_{index:02d}",
                "structure",
                rail_shape,
                r.material,
                PURCHASED,
                printable=False,
                notes=(f"azimuth_deg={angle:.3f}",),
            )
        )

        for end_name, z in (("lower", t.end_margin), ("upper", t.length - t.end_margin)):
            bogie = Rot(Z=angle) * Pos(rail_center_radius, 0, z) * make_theta_bogie(r)
            records.append(
                PartRecord(
                    f"theta_bogie_{index:02d}_{end_name}",
                    "motion",
                    bogie,
                    "printed_PETG; acetal_rollers; GT2_belt_drive",
                    DIY_PRINT,
                    notes=(
                        f"rail_axis={index}",
                        "paired end bogies rotate this longitudinal rail independently",
                    ),
                )
            )

    arm_rail_indices = (0, 1, 2, 3)
    # Park the end arms clear of the two global camera triplets in the default
    # exported snapshot. These bases remain independently mobile along Z.
    arm_z = (96.0, 132.0, 198.0, 234.0)
    # Do not conflate the rail's physical center with the calibrated shoulder
    # datum.  The carriage deck spans the 4.9 mm nominal inward offset.
    root_radius = a.shoulder_root_radius
    for arm_index, (rail_index, z) in enumerate(zip(arm_rail_indices, arm_z, strict=True)):
        angle = 360.0 * rail_index / t.rail_count
        carriage_shape = (
            Rot(Z=angle)
            * Pos(rail_center_radius, 0, z)
            * make_rail_carriage(c, r)
        )
        records.append(
            PartRecord(
                f"carriage_{arm_index:02d}",
                "motion",
                carriage_shape,
                c.material,
                DIY_PRINT,
                notes=(
                    "UHMW tape liner",
                    "vision-corrected absolute pose",
                    f"physical_rail_center_radius_mm={rail_center_radius:.3f}",
                    f"shoulder_root_datum_radius_mm={root_radius:.3f}",
                ),
            )
        )
        posed_arm = (
            Rot(Z=angle)
            * Pos(root_radius, 0, z)
            * Rot(Z=180)
            * make_arm_assembly(a, gp, show_tendons=True)
        )
        records.append(
            PartRecord(
                f"tendon_arm_{arm_index:02d}",
                "motion",
                posed_arm,
                f"{a.material}; {a.tendon_material}",
                DIY_PRINT,
                notes=(
                    "four paired differential tendon loops in two bundled lumens",
                    "0.20 mm UHMWPE line; eight line sides but four motors",
                    "1.2 mm steel hinge pins",
                    "physical axes: yaw +Z; pitch/elbow yaw-rotated +Y; wrist final-link +X",
                    f"shoulder_root_datum_radius_mm={root_radius:.3f}",
                ),
            )
        )
        actuator = (
            Rot(Z=angle)
            * Pos(rail_center_radius - 6.0, -1.1, z)
            * make_actuator_bank(a)
        )
        records.append(
            PartRecord(
                f"actuator_bank_{arm_index:02d}",
                "actuation",
                actuator,
                "printed_mount; commodity_6mm_gearmotors",
                PURCHASED,
                printable=False,
                notes=(
                    "four differential capstans; one motor/spool per controlled axis",
                    "paired line shares each capstan; no eight-motor duplication",
                    "current sensing estimates tendon force",
                ),
            )
        )

        if arm_index == sp.macro_mount_arm_index:
            arm_points = four_axis_arm_points(
                a.link_lengths,
                a.default_joint_angles_deg,
            )
            yaw, shoulder_pitch, elbow_pitch, wrist_roll = a.default_joint_angles_deg
            final_pitch = shoulder_pitch + elbow_pitch
            macro_bridge = (
                Rot(Z=angle)
                * Pos(root_radius, 0, z)
                * Rot(Z=180)
                * Pos(*arm_points[-1])
                * Rot(Z=yaw)
                * Rot(Y=-final_pitch)
                * Rot(X=wrist_roll)
                * Pos(0, 0, sp.macro_mount_normal_offset)
                * Rot(Z=180)
                * make_macro_stereo_bridge(sp)
            )
            records.append(
                PartRecord(
                    "macro_stereo_bridge_00",
                    "sensing",
                    macro_bridge,
                    sp.material,
                    DIY_PRINT,
                    notes=(
                        "single crossbar physically overlaps both camera shells",
                        "keyed wrist tongue requires measured coupling transform",
                        f"locked_baseline_mm={sp.macro_stereo_baseline:.3f}",
                    ),
                )
            )
            for macro_index, side in enumerate((-0.5, 0.5)):
                macro = (
                    Rot(Z=angle)
                    * Pos(root_radius, 0, z)
                    * Rot(Z=180)
                    * Pos(*arm_points[-1])
                    * Rot(Z=yaw)
                    * Rot(Y=-final_pitch)
                    * Rot(X=wrist_roll)
                    * Pos(
                        0,
                        side * sp.macro_stereo_baseline,
                        sp.macro_mount_normal_offset,
                    )
                    * Rot(Z=180)
                    * make_macro_camera_pod(sp)
                )
                records.append(
                    PartRecord(
                        f"macro_camera_{macro_index:02d}",
                        "sensing",
                        macro,
                        "printed_case; compact_close_focus_camera",
                        DIY_PRINT,
                        notes=(
                            "one of a rigid two-camera wrist pair",
                            f"locked_baseline_mm={sp.macro_stereo_baseline:.3f}",
                            f"mount_arm_index={sp.macro_mount_arm_index}",
                        ),
                    )
                )

    # Three views at each end keep the registered volume inside two independent
    # 120-degree camera triplets.  The upper triplet is clocked by 60 degrees so
    # arm and rail occlusions do not line up through both end groups.
    camera_angles = tuple(
        angle for triplet in sp.global_camera_triplet_azimuths for angle in triplet
    )
    work_z = t.length / 2
    camera_z = tuple(
        work_z + offset
        for offset in sp.global_camera_end_offsets
        for _ in range(3)
    )
    sensor_front_radius = sp.global_camera_front_radius
    for index, (angle, z) in enumerate(zip(camera_angles, camera_z, strict=True)):
        shape = Rot(Z=angle) * Pos(sensor_front_radius, 0, z) * make_camera_pod(sp)
        records.append(
            PartRecord(
                f"camera_pod_{index:02d}",
                "sensing",
                shape,
                "printed_case; commodity_global_shutter_camera",
                DIY_PRINT,
                notes=(
                    "optical axis points radially inward",
                    "hardware trigger required",
                    f"front_face_datum_radius_mm={sensor_front_radius:.3f}",
                    f"front_face_datum_z_mm={z:.3f}",
                    f"azimuth_deg={angle:.3f}",
                ),
            )
        )

    projector = (
        Rot(Z=sp.projector_azimuth_deg)
        * Pos(sp.projector_front_radius, 0, work_z + sp.projector_z_offset)
        * make_projector_pod(sp)
    )
    records.append(
        PartRecord(
            "projector_pod_00",
            "sensing",
            projector,
            "printed_case; calibrated_coded_projector_or_laser_line",
            DIY_PRINT,
            notes=(
                "shared, replaceable structured-light source",
                f"front_face_datum_radius_mm={sp.projector_front_radius:.3f}",
                f"front_face_datum_z_mm={work_z + sp.projector_z_offset:.3f}",
                f"azimuth_deg={sp.projector_azimuth_deg:.3f}",
            ),
        )
    )

    for index, (angle, z) in enumerate(
        zip((30.0, 150.0, 270.0), (28.0, 164.0, 300.0), strict=True)
    ):
        shape = (
            Rot(Z=angle)
            * Pos(sensor_front_radius - 1.0, 0, z)
            * make_fiducial_plate(sp, marker_id=17 + index)
        )
        records.append(
            PartRecord(
                f"fiducial_{index:02d}",
                "calibration",
                shape,
                sp.material,
                DIY_PRINT,
                notes=("paint relief matte white after printing",),
            )
        )
    return records


def gearbox_records(config: DesignConfig, exploded: bool = False) -> list[PartRecord]:
    """One record per gearbox assembly item, in insertion order."""

    g = config.gearbox
    lift = (0.0, 0.0, 0.0) if not exploded else (0.55, 0.85, 1.15)
    centers = (
        (g.input_center_x, g.center_y),
        (g.idler_center_x, g.center_y),
        (g.output_center_x, g.center_y),
    )
    records = [
        PartRecord(
            "gearbox_housing",
            "gearbox",
            make_gearbox_housing(g),
            g.material,
            MICRO_PRINT,
            notes=(
                "acceptance-article nominal geometry",
                "three 0.340 x 0.250 mm blind split-compliant shaft seats",
                "two cover-latch captures and external three-point datum",
            ),
            density_g_mm3=g.material_density_g_mm3,
        )
    ]
    shaft_start = 0.0
    for index, (x, y) in enumerate(centers):
        records.append(
            PartRecord(
                f"S{index + 1}",
                "gearbox",
                Pos(x, y, shaft_start) * make_shaft(g),
                g.shaft_material,
                PURCHASED,
                printable=False,
                notes=(
                    f"acceptance_article_id=S{index + 1}",
                    "insert into 0.340 x 0.250 mm blind split seat",
                    "end deburr chamfers are <=0.005 mm",
                ),
                density_g_mm3=g.shaft_density_g_mm3,
            )
        )
    specs = (
        (
            "G3",
            centers[2],
            g.output_teeth,
            0.0,
            False,
            True,
            "output_gear_z24",
        ),
        (
            "G2",
            centers[1],
            g.idler_teeth,
            180.0 - 180.0 / g.idler_teeth,
            False,
            False,
            "idler_gear_z18",
        ),
        (
            "G1",
            centers[0],
            g.input_teeth,
            0.0,
            True,
            False,
            "input_gear_z12",
        ),
    )
    for index, (
        name,
        (x, y),
        teeth,
        phase,
        drive_slot,
        phase_marks,
        role,
    ) in enumerate(specs):
        records.append(
            PartRecord(
                name,
                "gearbox",
                Pos(x, y, g.gear_z + lift[index])
                * Rot(Z=phase)
                * make_spur_gear(
                    teeth,
                    g,
                    drive_slot=drive_slot,
                    optical_phase_marks=phase_marks,
                ),
                g.material,
                MICRO_PRINT,
                notes=(
                    f"acceptance_article_id={name}",
                    role,
                    f"phase_deg={phase:.6f}",
                    (
                        "0.10 mm input-driver slot"
                        if drive_slot
                        else "unequal recessed optical phase marks"
                        if phase_marks
                        else "plain idler hub"
                    ),
                    "analytical involute, sampled flank",
                    "0.025 mm x 45 degree bore-entry chamfers at both ends",
                ),
                density_g_mm3=g.material_density_g_mm3,
            )
        )
    lid_z = g.housing_height if not exploded else g.housing_height + 1.75
    records.append(
        PartRecord(
            "cover",
            "gearbox",
            Pos(Z=lid_z) * make_gearbox_lid(g),
            g.material,
            MICRO_PRINT,
            notes=(
                "acceptance_article_id=cover",
                "final vertical insertion and two-latch closure",
                "0.75 mm driver and 1.0 mm observation windows",
            ),
            density_g_mm3=g.material_density_g_mm3,
        )
    )
    return records


def benchmark_records(config: DesignConfig) -> list[PartRecord]:
    """Cell plus the closed gearbox positioned in its central work volume."""

    records = cell_records(config)
    g = config.gearbox
    benchmark = (
        Pos(-g.housing_length / 2, -g.housing_width / 2, config.tube.length / 2)
        * make_gearbox_assembly(g)
    )
    records.append(
        PartRecord(
            "gearbox_benchmark_complete",
            "payload",
            benchmark,
            f"{g.material}; {g.shaft_material}",
            "mixed",
            printable=False,
            notes=("centered in observed volume", "assembled result pose"),
        )
    )
    return records


def tooling_records(config: DesignConfig) -> list[PartRecord]:
    """Origin-centered fixture, kitting tray, and swappable micro-tools."""

    t, g = config.tooling, config.gearbox
    return [
        PartRecord(
            "gearbox_nest",
            "fixture",
            make_gearbox_nest(t, g),
            t.printed_material,
            DIY_PRINT,
            notes=(
                "loose housing clearance pocket; restraint contacts and clamp are not modeled",
                "three underside clearance reliefs for the housing datum pads",
                "two ordinary M2-class bench fastener holes",
            ),
            density_g_mm3=t.printed_density_g_mm3,
        ),
        PartRecord(
            "gearbox_parts_tray",
            "fixture",
            make_parts_tray(t, g),
            t.printed_material,
            DIY_PRINT,
            notes=("pockets ordered S1-S3, G3-G2-G1, housing, cover",),
            density_g_mm3=t.printed_density_g_mm3,
        ),
        PartRecord(
            "vacuum_micro_pick",
            "tooling",
            make_vacuum_micro_pick(t),
            f"{t.printed_material}; replaceable steel tube; soft tip",
            "mixed_shop_assembly",
            printable=False,
            notes=("commodity hypodermic tube in printed holder",),
        ),
        PartRecord(
            "compliant_insertion_probe",
            "tooling",
            make_compliant_insertion_probe(t),
            f"{t.printed_material}; replaceable probe tip",
            "mixed_shop_assembly",
            printable=False,
            notes=("parallel-flexure travel is sensed and force-limited in software",),
        ),
        PartRecord(
            "rotary_drive_blade",
            "tooling",
            make_rotary_drive_blade(t, g),
            t.tool_material,
            "cut_or_EDM_replaceable_tip",
            printable=False,
            notes=(
                f"blade_width_mm={t.rotary_blade_width:.3f}",
                f"G1_slot_width_mm={g.input_drive_slot_width:.3f}",
            ),
        ),
        PartRecord(
            "calibration_pointer",
            "calibration",
            make_calibration_pointer(t),
            f"{t.printed_material}; steel pointer",
            "mixed_shop_assembly",
            printable=False,
            notes=("three-foot datum establishes tool-center-point offset",),
        ),
    ]


def printable_catalog_records(config: DesignConfig) -> list[PartRecord]:
    """One origin-centered definition of each unique fabricated part."""

    c = config
    records = [
        PartRecord("rail_carriage", "motion", make_rail_carriage(c.carriage, c.rail), c.carriage.material, DIY_PRINT),
        PartRecord("theta_end_track_quadrant", "motion", make_theta_track_quadrant(c.tube), "PETG_with_UHMW_strip", DIY_PRINT, quantity=8),
        PartRecord("theta_end_bogie_body", "motion", make_theta_bogie_body(c.rail), "PETG", DIY_PRINT, quantity=8),
        PartRecord("actuator_mount", "actuation", make_actuator_mount(c.arm), c.arm.material, DIY_PRINT, quantity=c.tube.rail_count),
        PartRecord("arm_link_32mm", "motion", make_arm_segment(c.arm, c.arm.link_lengths[0]), c.arm.material, DIY_PRINT, quantity=4),
        PartRecord("arm_link_30mm", "motion", make_arm_segment(c.arm, c.arm.link_lengths[1]), c.arm.material, DIY_PRINT, quantity=4),
        PartRecord("arm_link_15mm", "motion", make_arm_segment(c.arm, c.arm.link_lengths[2]), c.arm.material, DIY_PRINT, quantity=4),
        PartRecord("shoulder_yaw_stage", "motion", make_shoulder_yaw_stage(c.arm), c.arm.material, DIY_PRINT, quantity=4, density_g_mm3=c.manufacturing.nominal_resin_density_g_mm3),
        PartRecord("shoulder_pitch_yoke", "motion", make_pitch_yoke(c.arm, "shoulder_pitch"), c.arm.material, DIY_PRINT, quantity=4, density_g_mm3=c.manufacturing.nominal_resin_density_g_mm3),
        PartRecord("elbow_pitch_yoke", "motion", make_pitch_yoke(c.arm, "elbow_pitch"), c.arm.material, DIY_PRINT, quantity=4, density_g_mm3=c.manufacturing.nominal_resin_density_g_mm3),
        PartRecord("wrist_roll_stage", "motion", make_wrist_roll_stage(c.arm), c.arm.material, DIY_PRINT, quantity=4, density_g_mm3=c.manufacturing.nominal_resin_density_g_mm3),
        PartRecord("tendon_spool", "actuation", make_tendon_spool(c.arm), c.arm.material, DIY_PRINT, quantity=c.arm.actuator_count * c.tube.rail_count),
        PartRecord("gripper_palm", "gripper", make_gripper_palm(c.gripper), c.gripper.material, DIY_PRINT, quantity=4),
        PartRecord("gripper_jaw_left", "gripper", make_gripper_jaw(c.gripper, 1), c.gripper.material, DIY_PRINT, quantity=4),
        PartRecord("gripper_jaw_right", "gripper", make_gripper_jaw(c.gripper, -1), c.gripper.material, DIY_PRINT, quantity=4),
        PartRecord("camera_pod_shell", "sensing", make_camera_shell(c.sensing), c.sensing.material, DIY_PRINT, quantity=6),
        PartRecord("macro_camera_shell", "sensing", make_macro_camera_shell(c.sensing), c.sensing.material, DIY_PRINT, quantity=2),
        PartRecord("macro_stereo_bridge", "sensing", make_macro_stereo_bridge(c.sensing), c.sensing.material, DIY_PRINT),
        PartRecord("projector_pod_shell", "sensing", make_projector_shell(c.sensing), c.sensing.material, DIY_PRINT, quantity=c.sensing.structured_light_projector_count),
        PartRecord("fiducial_plate", "calibration", make_fiducial_plate(c.sensing, 17), c.sensing.material, DIY_PRINT, quantity=3),
        PartRecord("gearbox_housing", "gearbox", make_gearbox_housing(c.gearbox), c.gearbox.material, MICRO_PRINT, density_g_mm3=c.gearbox.material_density_g_mm3),
        PartRecord("G1", "gearbox", make_spur_gear(c.gearbox.input_teeth, c.gearbox, drive_slot=True), c.gearbox.material, MICRO_PRINT, density_g_mm3=c.gearbox.material_density_g_mm3),
        PartRecord("G2", "gearbox", make_spur_gear(c.gearbox.idler_teeth, c.gearbox), c.gearbox.material, MICRO_PRINT, density_g_mm3=c.gearbox.material_density_g_mm3),
        PartRecord("G3", "gearbox", make_spur_gear(c.gearbox.output_teeth, c.gearbox, optical_phase_marks=True), c.gearbox.material, MICRO_PRINT, density_g_mm3=c.gearbox.material_density_g_mm3),
        PartRecord("cover", "gearbox", make_gearbox_lid(c.gearbox), c.gearbox.material, MICRO_PRINT, density_g_mm3=c.gearbox.material_density_g_mm3),
    ]
    records.extend(tooling_records(config))
    return records
