"""Commodity camera, structured-light projector, and fiducial carriers."""

from __future__ import annotations

from build123d import Align, Box, Compound, Cylinder, Pos, Rot

from .params import SensingParams


CENTER3 = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN_X = (Align.MIN, Align.CENTER, Align.CENTER)
MIN_Z = (Align.CENTER, Align.CENTER, Align.MIN)


def _x_cylinder(radius: float, length: float):
    return Rot(Y=90) * Cylinder(radius, length, align=MIN_Z)


def make_camera_pod(params: SensingParams):
    """Open-back enclosure for an OV9281/OV2640-class board camera.

    Local front face is X=0 and the optical axis points toward -X.
    """

    shell = make_camera_shell(params)
    lens = Pos(-2.0, 0, 0) * _x_cylinder(
        params.camera_lens_diameter / 2, 2.0
    )
    board = Pos(params.case_wall + params.camera_board_depth / 2, 0, 0) * Box(
        params.camera_board_depth,
        params.camera_board_width,
        params.camera_board_height,
        align=CENTER3,
    )
    pod = Compound(children=[shell, board, lens])
    pod.label = "camera_pod_OV9281_class"
    return pod


def make_camera_shell(params: SensingParams):
    """Printable global-camera enclosure only."""

    outer = Box(
        params.camera_pod_depth,
        params.camera_pod_width,
        params.camera_pod_height,
        align=MIN_X,
    )
    cavity = Pos(params.case_wall, 0, 0) * Box(
        params.camera_pod_depth,
        params.camera_pod_width - 2 * params.case_wall,
        params.camera_pod_height - 2 * params.case_wall,
        align=MIN_X,
    )
    aperture = Pos(-0.5, 0, 0) * _x_cylinder(
        params.camera_aperture_diameter / 2, params.camera_pod_depth + 1.0
    )
    shell = outer - [cavity, aperture]
    shell.label = "camera_pod_shell"
    return shell


def make_projector_pod(params: SensingParams):
    """Case for a low-cost VCSEL/diffractive structured-light module."""

    shell = make_projector_shell(params)
    emitter = Pos(params.case_wall + 2.0, 0, 0) * Box(4.0, 8.0, 6.0, align=CENTER3)
    pod = Compound(children=[shell, emitter])
    pod.label = "structured_light_projector_pod"
    return pod


def make_projector_shell(params: SensingParams):
    """Printable structured-light enclosure only."""

    outer = Box(
        params.projector_depth,
        params.projector_width,
        params.projector_height,
        align=MIN_X,
    )
    cavity = Pos(params.case_wall, 0, 0) * Box(
        params.projector_depth,
        params.projector_width - 2 * params.case_wall,
        params.projector_height - 2 * params.case_wall,
        align=MIN_X,
    )
    aperture = Pos(-0.5, 0, 0) * _x_cylinder(
        params.projector_aperture_diameter / 2, params.projector_depth + 1.0
    )
    shell = outer - [cavity, aperture]
    shell.label = "structured_light_projector_shell"
    return shell


def make_macro_camera_pod(params: SensingParams):
    """Compact close-focus camera carried by an arm near its wrist."""

    shell = make_macro_camera_shell(params)
    lens = Pos(-1.2, 0, 0) * _x_cylinder(params.macro_lens_diameter / 2, 1.2)
    pod = Compound(children=[shell, lens])
    pod.label = "arm_macro_camera_pod"
    return pod


def make_macro_camera_shell(params: SensingParams):
    """Printable wrist macro-camera enclosure only."""

    outer = Box(
        params.macro_pod_depth,
        params.macro_pod_width,
        params.macro_pod_height,
        align=MIN_X,
    )
    cavity = Pos(params.case_wall, 0, 0) * Box(
        params.macro_pod_depth,
        params.macro_pod_width - 2 * params.case_wall,
        params.macro_pod_height - 2 * params.case_wall,
        align=MIN_X,
    )
    aperture = Pos(-0.3, 0, 0) * _x_cylinder(
        params.macro_aperture_diameter / 2, params.macro_pod_depth + 0.6
    )
    shell = outer - [cavity, aperture]
    shell.label = "arm_macro_camera_shell"
    return shell


def make_macro_stereo_bridge(params: SensingParams):
    """Rigid crossbar and keyed wrist tongue for the two macro-camera pods.

    The crossbar overlaps the top/rear wall of both shells at the locked stereo
    spacing.  The center tongue is deliberately chunky enough for a doweled or
    keyed coupling; its calibrated wrist transform is still a hardware input.
    """

    bridge_depth = 2.0
    bridge_height = 2.0
    bridge_width = params.macro_stereo_baseline + params.macro_pod_width
    bridge_x = params.macro_pod_depth - bridge_depth / 2
    bridge_z = params.macro_pod_height / 2 - bridge_height / 2
    crossbar = Pos(X=bridge_x, Z=bridge_z) * Box(
        bridge_depth,
        bridge_width,
        bridge_height,
        align=CENTER3,
    )
    tongue_length = 4.0
    tongue = Pos(
        X=params.macro_pod_depth + tongue_length / 2 - 0.5,
        Z=bridge_z,
    ) * Box(
        tongue_length,
        4.0,
        bridge_height,
        align=CENTER3,
    )
    bridge = crossbar + tongue
    bridge.label = "macro_stereo_rigid_bridge_and_keyed_wrist_tongue"
    return bridge


def make_fiducial_plate(params: SensingParams, marker_id: int = 0):
    """Raised 6x6 binary tile plate, locally facing -X.

    This is a physical geometry carrier, not a promise that a generated bit
    pattern is a standards-compliant AprilTag.  IDs deterministically alter the
    internal paint/relief mask while preserving a solid border.
    """

    n = params.fiducial_cells
    if n < 4:
        raise ValueError("fiducial needs at least four cells per side")
    plate = Box(
        params.fiducial_thickness,
        params.fiducial_size,
        params.fiducial_size,
        align=MIN_X,
    )
    cell = params.fiducial_size / n
    raised = []
    for row in range(1, n - 1):
        for col in range(1, n - 1):
            bit_index = (row - 1) * (n - 2) + (col - 1)
            bit = (marker_id >> bit_index) & 1
            parity = (row + col + marker_id) & 1
            if bit ^ parity:
                y = -params.fiducial_size / 2 + (col + 0.5) * cell
                z = -params.fiducial_size / 2 + (row + 0.5) * cell
                raised.append(
                    Pos(params.fiducial_thickness, y, z)
                    * Box(
                        params.fiducial_relief,
                        cell * 0.88,
                        cell * 0.88,
                        align=MIN_X,
                    )
                )
    tag = plate + raised
    tag.label = f"calibration_fiducial_{marker_id:03d}"
    return tag
