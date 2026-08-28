"""Tendon-driven link, routing, actuator spool, and micro-gripper CAD."""

from __future__ import annotations

from math import atan2, degrees, hypot, radians, sin, cos

from build123d import Align, Box, Compound, Cylinder, Pos, Rot

from .kinematics import four_axis_arm_points
from .params import ArmParams, GripperParams


CENTER3 = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN_Z = (Align.CENTER, Align.CENTER, Align.MIN)


def _x_cylinder(radius: float, length: float):
    return Rot(Y=90) * Cylinder(radius, length, align=MIN_Z)


def _rod_between(start: tuple[float, float, float], end: tuple[float, float, float], radius: float):
    """Cylinder between arbitrary 3-D points, used only as a tendon preview."""

    dx, dy, dz = (
        end[0] - start[0],
        end[1] - start[1],
        end[2] - start[2],
    )
    radial = hypot(dx, dy)
    length = hypot(radial, dz)
    if length <= 0:
        raise ValueError("rod endpoints must be different")
    azimuth = degrees(atan2(dy, dx)) if radial > 1e-12 else 0.0
    elevation = degrees(atan2(dz, radial))
    return (
        Pos(*start)
        * Rot(Z=azimuth)
        * Rot(Y=90.0 - elevation)
        * Cylinder(radius, length, align=MIN_Z)
    )


def make_arm_segment(params: ArmParams, length: float | None = None):
    """One serial link with two hinge bores and dual tendon lumens."""

    link_length = params.link_lengths[0] if length is None else length
    if link_length <= 2 * params.joint_radius:
        raise ValueError("link center distance must exceed two joint radii")
    beam = Pos(X=link_length / 2) * Box(
        link_length,
        params.link_width,
        params.link_thickness,
        align=CENTER3,
    )
    knuckles = [
        Pos(X=x)
        * Cylinder(params.joint_radius, params.link_thickness, align=CENTER3)
        for x in (0.0, link_length)
    ]
    link = beam + knuckles
    hinge_radius = (params.hinge_pin_diameter + params.hinge_bore_clearance) / 2
    hinge_holes = [
        Pos(X=x)
        * Cylinder(hinge_radius, params.link_thickness + 1.0, align=CENTER3)
        for x in (0.0, link_length)
    ]
    channel_radius = params.tendon_channel_diameter / 2
    tendon_channels = [
        Pos(-params.joint_radius - 0.5, y, 0)
        * _x_cylinder(channel_radius, link_length + 2 * params.joint_radius + 1.0)
        for y in (-params.tendon_offset, params.tendon_offset)
    ]
    link = link - [*hinge_holes, *tendon_channels]
    link.label = f"tendon_link_{link_length:g}mm"
    return link


def make_shoulder_yaw_stage(params: ArmParams):
    """Turntable body whose physical rotation axis is local +Z."""

    height = params.shoulder_yaw_stage_height
    body = Cylinder(params.shoulder_yaw_stage_radius, height, align=CENTER3)
    center_bore = Cylinder(
        (params.hinge_pin_diameter + params.hinge_bore_clearance) / 2,
        height + 1.0,
        align=CENTER3,
    )
    cable_pass = Pos(X=-params.shoulder_yaw_stage_radius * 0.55) * Cylinder(
        params.tendon_channel_diameter,
        height + 1.0,
        align=CENTER3,
    )
    stage = body - [center_bore, cable_pass]
    stage.label = "joint_shoulder_yaw_stage_axis_Z"
    return stage


def make_shoulder_yaw_axis(params: ArmParams):
    axis = Cylinder(
        params.hinge_pin_diameter / 2,
        params.shoulder_yaw_stage_height + 1.0,
        align=CENTER3,
    )
    axis.label = "axis_shoulder_yaw_Z"
    return axis


def make_pitch_yoke(params: ArmParams, joint_name: str = "shoulder_pitch"):
    """Printable two-ear yoke; its hinge bore is the local +Y axis."""

    if joint_name not in ("shoulder_pitch", "elbow_pitch"):
        raise ValueError("pitch yoke must be shoulder_pitch or elbow_pitch")
    span = params.pitch_yoke_span
    thickness = params.pitch_yoke_thickness
    ear_height = 5.2
    ear_length = 4.8
    ears = [
        Pos(0, side * (span - thickness) / 2, 0)
        * Box(ear_length, thickness, ear_height, align=CENTER3)
        for side in (-1.0, 1.0)
    ]
    bridge = Pos(0, 0, -(ear_height - thickness) / 2) * Box(
        ear_length,
        span,
        thickness,
        align=CENTER3,
    )
    bore = Rot(X=90) * Cylinder(
        (params.hinge_pin_diameter + params.hinge_bore_clearance) / 2,
        span + 1.0,
        align=CENTER3,
    )
    yoke = (bridge + ears) - bore
    yoke.label = f"joint_{joint_name}_yoke_axis_Y"
    return yoke


def make_pitch_axis(params: ArmParams, joint_name: str = "shoulder_pitch"):
    if joint_name not in ("shoulder_pitch", "elbow_pitch"):
        raise ValueError("pitch axis must be shoulder_pitch or elbow_pitch")
    axis = Rot(X=90) * Cylinder(
        params.hinge_pin_diameter / 2,
        params.pitch_yoke_span,
        align=CENTER3,
    )
    axis.label = f"axis_{joint_name}_Y"
    return axis


def make_wrist_roll_stage(params: ArmParams):
    """Sleeve-and-rotor wrist bearing with its axis along local +X."""

    outer_radius = params.wrist_roll_outer_diameter / 2
    rotor_radius = max(outer_radius - 0.55, params.hinge_pin_diameter)
    half = params.wrist_roll_length / 2
    outer = Pos(X=-half) * _x_cylinder(outer_radius, params.wrist_roll_length)
    inner_cut = Pos(X=-half - 0.25) * _x_cylinder(
        rotor_radius + 0.10,
        params.wrist_roll_length + 0.5,
    )
    sleeve = outer - inner_cut
    rotor = Pos(X=-half) * _x_cylinder(rotor_radius, params.wrist_roll_length)
    witness = Pos(0, 0, outer_radius - 0.05) * Box(
        params.wrist_roll_length * 0.55,
        0.55,
        0.35,
        align=CENTER3,
    )
    sleeve.label = "joint_wrist_roll_sleeve_axis_X"
    rotor.label = "axis_wrist_roll_rotor_X"
    stage = Compound(children=[sleeve, rotor, witness])
    stage.label = "joint_wrist_roll_stage_axis_X"
    return stage


def make_tendon_spool(params: ArmParams):
    """Printable spool body for a commodity N20 motor or micro-servo shaft."""

    core_radius = max(params.spool_radius - 0.7, 0.5)
    core = Cylinder(core_radius, params.spool_width, align=CENTER3)
    flanges = [
        Pos(Z=z) * Cylinder(params.spool_radius, 0.45, align=CENTER3)
        for z in (-(params.spool_width - 0.45) / 2, (params.spool_width - 0.45) / 2)
    ]
    shaft_bore = Cylinder(0.55, params.spool_width + 1.0, align=CENTER3)
    anchor_radius = max(params.tendon_diameter + 0.20, 0.35) / 2
    anchor_holes = [
        Pos(X=side * core_radius * 0.72)
        * Cylinder(
            anchor_radius,
            params.spool_width + 1.0,
            align=CENTER3,
        )
        for side in (-1.0, 1.0)
    ]
    spool = (core + flanges) - [shaft_bore, *anchor_holes]
    spool.label = "paired_line_differential_capstan"
    return spool


def make_actuator_bank(params: ArmParams):
    """Four $5-class 6 mm gearmotors with differential capstan loops.

    Each motor/spool carries both sides of one paired tendon loop.  The four
    channels actuate shoulder yaw, shoulder pitch, elbow pitch, and wrist roll;
    eight motors are neither required nor represented.
    """

    children = [make_actuator_mount(params)]
    for z in _actuator_positions(params):
        motor = Pos(-0.5, 1.1 + params.motor_length / 2, z) * Rot(X=90) * Cylinder(
            params.motor_diameter / 2,
            params.motor_length,
            align=CENTER3,
        )
        gearhead = Pos(
            -0.5,
            1.1 + params.motor_length + params.gearhead_length / 2,
            z,
        ) * Rot(X=90) * Cylinder(
            params.gearhead_diameter / 2,
            params.gearhead_length,
            align=CENTER3,
        )
        spool = Pos(
            -0.5,
            1.1 + params.motor_length + params.gearhead_length + params.spool_width / 2,
            z,
        ) * Rot(X=90) * make_tendon_spool(params)
        children.extend((motor, gearhead, spool))
    bank = Compound(children=children)
    bank.label = "four_channel_differential_capstan_bank"
    return bank


def make_actuator_mount(params: ArmParams):
    """Printable tie-down plate sized from the configured channel count."""

    length_z = (params.actuator_count - 1) * params.actuator_spacing + 8.0
    plate = Box(8.0, 2.2, length_z, align=CENTER3)
    # Two zip-tie passages per motor.  They are intentionally ordinary round
    # holes so this remains printable on both FDM and MSLA machines.
    holes = []
    for motor_z in _actuator_positions(params):
        for x in (-2.3, 2.3):
            holes.append(
                Pos(x, 1.6, motor_z)
                * Rot(X=90)
                * Cylinder(0.85, 3.2, align=MIN_Z)
            )
    mount = plate - holes
    mount.label = f"{params.actuator_count}_motor_actuator_mount"
    return mount


def _actuator_positions(params: ArmParams) -> tuple[float, ...]:
    center = (params.actuator_count - 1) / 2
    return tuple(
        (index - center) * params.actuator_spacing
        for index in range(params.actuator_count)
    )


def make_gripper_palm(params: GripperParams):
    palm = Pos(X=params.palm_length / 2) * Box(
        params.palm_length,
        params.palm_width,
        params.palm_thickness,
        align=CENTER3,
    )
    pivot_holes = [
        Pos(params.palm_length * 0.72, y, 0)
        * Cylinder(params.pivot_diameter / 2, params.palm_thickness + 0.8, align=CENTER3)
        for y in (-params.palm_width * 0.28, params.palm_width * 0.28)
    ]
    return palm - pivot_holes


def make_gripper_jaw(params: GripperParams, side: int = 1):
    """One mirrored jaw. ``side`` is +1 or -1."""

    if side not in (-1, 1):
        raise ValueError("side must be -1 or +1")
    beam = Pos(X=params.jaw_length / 2) * Box(
        params.jaw_length,
        params.jaw_width,
        params.jaw_thickness,
        align=CENTER3,
    )
    tip_y = -side * (params.jaw_width - params.tip_width) / 2
    tip = Pos(params.jaw_length - params.tip_width / 2, tip_y, 0) * Box(
        params.tip_width,
        params.jaw_width * 1.8,
        params.jaw_thickness,
        align=CENTER3,
    )
    pivot = Cylinder(params.pivot_diameter / 2, params.jaw_thickness + 0.8, align=CENTER3)
    tendon = Pos(params.jaw_length * 0.28, 0, 0) * Cylinder(
        params.tendon_channel_diameter / 2,
        params.jaw_thickness + 0.8,
        align=CENTER3,
    )
    jaw = (beam + tip) - [pivot, tendon]
    jaw.label = "gripper_jaw_left" if side > 0 else "gripper_jaw_right"
    return jaw


def make_gripper_assembly(params: GripperParams):
    palm = make_gripper_palm(params)
    pivot_x = params.palm_length * 0.72
    pivot_y = params.palm_width * 0.28
    jaw_angle = 8.0
    left = Pos(pivot_x, pivot_y, 0) * Rot(Z=-jaw_angle) * make_gripper_jaw(params, 1)
    right = Pos(pivot_x, -pivot_y, 0) * Rot(Z=jaw_angle) * make_gripper_jaw(params, -1)
    assembly = Compound(children=[palm, left, right])
    assembly.label = "two_jaw_tendon_gripper"
    return assembly


def make_arm_assembly(
    arm: ArmParams,
    gripper: GripperParams,
    joint_angles_deg: tuple[float, ...] | None = None,
    show_tendons: bool = True,
):
    """Pose the four physical axes and add idealized tendon centerlines.

    The shoulder yaw stage is vertical (+Z).  Its output carries shoulder and
    elbow pitch yokes about the yaw-rotated +Y axis.  The wrist sleeve/rotor is
    coaxial with the final link (+X before pose transforms).  Wrist roll changes
    the gripper pose without inventing a fourth planar bend.
    """

    angles = joint_angles_deg or arm.default_joint_angles_deg
    if len(angles) != 4:
        raise ValueError("arm pose requires yaw, shoulder pitch, elbow pitch, wrist roll")
    yaw, shoulder_pitch, elbow_pitch, wrist_roll = angles
    lengths = arm.link_lengths
    points = four_axis_arm_points(lengths, angles)
    final_pitch = shoulder_pitch + elbow_pitch
    children = [make_shoulder_yaw_stage(arm), make_shoulder_yaw_axis(arm)]

    shoulder_pose = Rot(Z=yaw)
    first_link_pose = Rot(Z=yaw) * Rot(Y=-shoulder_pitch) * Rot(X=90)
    final_link_pose = Rot(Z=yaw) * Rot(Y=-final_pitch) * Rot(X=90)
    children.extend(
        (
            shoulder_pose * make_pitch_yoke(arm, "shoulder_pitch"),
            shoulder_pose * make_pitch_axis(arm, "shoulder_pitch"),
            first_link_pose * make_arm_segment(arm, lengths[0]),
            Pos(*points[1]) * Rot(Z=yaw) * Rot(Y=-shoulder_pitch) * make_pitch_yoke(arm, "elbow_pitch"),
            Pos(*points[1]) * Rot(Z=yaw) * Rot(Y=-shoulder_pitch) * make_pitch_axis(arm, "elbow_pitch"),
            Pos(*points[1]) * final_link_pose * make_arm_segment(arm, lengths[1]),
            Pos(*points[2]) * final_link_pose * make_arm_segment(arm, lengths[2]),
            Pos(*points[-1])
            * Rot(Z=yaw)
            * Rot(Y=-final_pitch)
            * Rot(X=wrist_roll)
            * make_wrist_roll_stage(arm),
            Pos(*points[-1])
            * Rot(Z=yaw)
            * Rot(Y=-final_pitch)
            * Rot(X=wrist_roll)
            * Pos(X=arm.wrist_roll_length / 2)
            * make_gripper_assembly(gripper),
        )
    )

    if show_tendons:
        # Four paired differential loops produce eight line sides.  The four
        # strands on each side share one 0.90 mm routing lumen as a compact
        # bundle.  Exact capstan wrap remains in the multibody model.
        bundle_pitch = arm.tendon_diameter * 1.05
        bundle_center = (arm.actuator_count - 1) / 2
        for channel_index in range(arm.actuator_count):
            bundle_delta = (channel_index - bundle_center) * bundle_pitch
            for side in (-1.0, 1.0):
                route_offset = side * arm.tendon_offset + bundle_delta
                tendon_points = []
                for point in points:
                    # A constant lateral normal is the centerline preview of
                    # the two shared routing lumens.  Bend/wrap contact and
                    # pretension remain multibody-simulator concerns.
                    normal_x = -route_offset * sin(radians(yaw))
                    normal_y = route_offset * cos(radians(yaw))
                    tendon_points.append(
                        (point[0] + normal_x, point[1] + normal_y, point[2])
                    )
                for a, b in zip(tendon_points[:-1], tendon_points[1:], strict=True):
                    children.append(_rod_between(a, b, arm.tendon_diameter / 2))

    assembly = Compound(children=children)
    assembly.label = "posed_four_axis_tendon_arm"
    return assembly
