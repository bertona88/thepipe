"""Small, dependency-free helpers shared by CAD and simulator metadata."""

from __future__ import annotations

from math import cos, hypot, radians, sin


Point3 = tuple[float, float, float]


def planar_chain_points(
    lengths: tuple[float, ...], joint_angles_deg: tuple[float, ...], z: float = 0.0
) -> tuple[Point3, ...]:
    """Forward-kinematic joint centers for relative planar joint angles."""

    if len(lengths) != len(joint_angles_deg):
        raise ValueError("one angle is required per link")
    if any(length <= 0 for length in lengths):
        raise ValueError("link lengths must be positive")
    x = y = cumulative = 0.0
    points: list[Point3] = [(x, y, z)]
    for length, angle in zip(lengths, joint_angles_deg, strict=True):
        cumulative += angle
        x += length * cos(radians(cumulative))
        y += length * sin(radians(cumulative))
        points.append((x, y, z))
    return tuple(points)


def four_axis_arm_points(
    lengths: tuple[float, ...], axis_angles_deg: tuple[float, ...]
) -> tuple[Point3, ...]:
    """Joint centers for the yaw/pitch/elbow/roll serial arm.

    The three link center distances are driven by four physical axes.  Shoulder
    yaw rotates every link about +Z, shoulder pitch elevates link 1, elbow
    pitch changes the elevation of links 2 and 3, and wrist roll changes only
    tool orientation.  Positive pitch is +Z in this kernel-free convention.
    """

    if len(lengths) != 3:
        raise ValueError("the qualified arm requires three link lengths")
    if len(axis_angles_deg) != 4:
        raise ValueError("yaw, shoulder pitch, elbow pitch, and wrist roll are required")
    if any(length <= 0 for length in lengths):
        raise ValueError("link lengths must be positive")
    yaw, shoulder_pitch, elbow_pitch, _wrist_roll = map(radians, axis_angles_deg)
    elevations = (shoulder_pitch, shoulder_pitch + elbow_pitch, shoulder_pitch + elbow_pitch)
    x = y = z = 0.0
    points: list[Point3] = [(x, y, z)]
    for length, elevation in zip(lengths, elevations, strict=True):
        radial_step = length * cos(elevation)
        x += radial_step * cos(yaw)
        y += radial_step * sin(yaw)
        z += length * sin(elevation)
        points.append((x, y, z))
    return tuple(points)


def four_axis_arm_frames(
    lengths: tuple[float, ...], axis_angles_deg: tuple[float, ...]
) -> tuple[dict[str, object], ...]:
    """Named local axis origins/directions for metadata and collision setup."""

    points = four_axis_arm_points(lengths, axis_angles_deg)
    yaw, shoulder_pitch, elbow_pitch, _wrist_roll = map(radians, axis_angles_deg)
    pitch_axis = (-sin(yaw), cos(yaw), 0.0)
    final_elevation = shoulder_pitch + elbow_pitch
    wrist_axis = (
        cos(final_elevation) * cos(yaw),
        cos(final_elevation) * sin(yaw),
        sin(final_elevation),
    )
    return (
        {"name": "shoulder_yaw", "origin_mm": points[0], "direction": (0.0, 0.0, 1.0)},
        {"name": "shoulder_pitch", "origin_mm": points[0], "direction": pitch_axis},
        {"name": "elbow_pitch", "origin_mm": points[1], "direction": pitch_axis},
        {"name": "wrist_roll", "origin_mm": points[-1], "direction": wrist_axis},
    )


def distance(a: Point3, b: Point3) -> float:
    return hypot(hypot(b[0] - a[0], b[1] - a[1]), b[2] - a[2])
