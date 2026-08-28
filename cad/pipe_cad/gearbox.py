"""Idealized two-photon-polymerized micro gearbox.

The cell-scale hardware is deliberately cheap.  The gearbox is the assembly
benchmark and is therefore modeled at 2PP scale: 0.10 mm module involute gears,
micrometre-class gaps, and separate 0.35 mm dowel shafts.
"""

from __future__ import annotations

from build123d import Align, Box, Compound, Cone, Cylinder, Polygon, Pos, Rot, extrude

from .gear_math import involute_gear_outline
from .params import GearboxParams


CENTER3 = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN3 = (Align.MIN, Align.MIN, Align.MIN)
MIN_Z = (Align.CENTER, Align.CENTER, Align.MIN)


def make_spur_gear(
    teeth: int,
    params: GearboxParams,
    *,
    thickness: float | None = None,
    bore_delta: float = 0.0,
    drive_slot: bool = False,
    optical_phase_marks: bool = False,
):
    """Create one sampled analytical-involute gear around the world origin."""

    height = thickness or params.gear_thickness
    if height > params.total_gear_height:
        raise ValueError("gear face cannot exceed total gear height")
    outline = involute_gear_outline(
        teeth,
        params.module,
        params.pressure_angle_deg,
        params.backlash,
        flank_samples=7,
        arc_samples=4,
    )
    profile = Polygon(*outline.points, align=None)
    gear = extrude(profile, amount=height)
    hub = Cylinder(
        params.hub_diameter / 2,
        params.total_gear_height,
        align=MIN_Z,
    )
    bore_diameter = params.bore_diameter + bore_delta
    if bore_diameter <= params.shaft_diameter:
        raise ValueError("perturbed gear bore no longer clears shaft")
    bore = Pos(Z=-0.1) * Cylinder(
        bore_diameter / 2,
        params.total_gear_height + 0.2,
        align=MIN_Z,
    )
    bore_radius = bore_diameter / 2
    chamfer = params.bore_entry_chamfer
    # At 45 degrees the radial and axial chamfer dimensions are equal.  Two
    # short conical cuts preserve the exact 0.420 mm running bore between them.
    lower_entry = Cone(
        bore_radius + chamfer,
        bore_radius,
        chamfer,
        align=MIN_Z,
    )
    upper_entry = Pos(Z=params.total_gear_height - chamfer) * Cone(
        bore_radius,
        bore_radius + chamfer,
        chamfer,
        align=MIN_Z,
    )
    result = (gear + hub) - [bore, lower_entry, upper_entry]
    if drive_slot:
        slot_depth = 0.20
        drive_feature = Pos(Z=params.total_gear_height - slot_depth / 2) * Box(
            params.hub_diameter + 0.04,
            params.input_drive_slot_width,
            slot_depth,
            align=CENTER3,
        )
        result = result - drive_feature
    if optical_phase_marks:
        mark_depth = params.output_phase_mark_depth
        long_mark = Pos(0.14, 0, params.total_gear_height - mark_depth / 2) * Box(
            params.output_phase_mark_long_length,
            params.output_phase_mark_width,
            mark_depth,
            align=CENTER3,
        )
        short_mark = Rot(Z=135) * Pos(0.13, 0, params.total_gear_height - mark_depth / 2) * Box(
            params.output_phase_mark_short_length,
            params.output_phase_mark_width,
            mark_depth,
            align=CENTER3,
        )
        result = result - [long_mark, short_mark]
    result.label = f"gear_m{params.module:.3f}_z{teeth}"
    return result


def shaft_centers(params: GearboxParams) -> tuple[tuple[float, float], ...]:
    return (
        (params.input_center_x, params.center_y),
        (params.idler_center_x, params.center_y),
        (params.output_center_x, params.center_y),
    )


def make_gearbox_housing(params: GearboxParams, seat_delta: float = 0.0):
    """Open tray with blind split seats, latch captures, and three datums."""

    outer = Box(
        params.housing_length,
        params.housing_width,
        params.housing_height,
        align=MIN3,
    )
    cavity = Pos(params.housing_wall, params.housing_wall, params.housing_floor) * Box(
        params.housing_length - 2 * params.housing_wall,
        params.housing_width - 2 * params.housing_wall,
        params.housing_height,
        align=MIN3,
    )
    tray = outer - cavity

    seat_diameter = params.shaft_seat_diameter + seat_delta
    if seat_diameter <= 0:
        raise ValueError("perturbed shaft seat is non-physical")
    seat_bores = [
        Pos(x, y, -0.01)
        * Cylinder(
            seat_diameter / 2,
            params.shaft_seat_depth + 0.02,
            align=MIN_Z,
        )
        for x, y in shaft_centers(params)
    ]
    seat_splits = [
        Pos(x, y, -0.01)
        * Box(
            seat_diameter / 2 + 0.12,
            params.shaft_seat_split_width,
            params.shaft_seat_depth + 0.02,
            align=(Align.MIN, Align.CENTER, Align.MIN),
        )
        for x, y in shaft_centers(params)
    ]
    tray = tray - [*seat_bores, *seat_splits]

    # A thin local membrane closes each full-depth seat while the radial split
    # leaves a compliant C-section in the 0.25 mm floor.
    seat_caps = [
        Pos(x, y, -params.shaft_seat_cap_thickness)
        * Cylinder(
            seat_diameter / 2 + 0.12,
            params.shaft_seat_cap_thickness,
            align=MIN_Z,
        )
        for x, y in shaft_centers(params)
    ]

    # Two local ledges capture hooks descending from the cover.  They remain
    # inside the nominal 6 x 4 mm XY envelope.
    captures = [
        Pos(1.25, params.housing_wall + 0.055, params.housing_height - 0.17)
        * Box(0.36, 0.12, 0.10, align=CENTER3),
        Pos(
            4.75,
            params.housing_width - params.housing_wall - 0.055,
            params.housing_height - 0.17,
        )
        * Box(0.36, 0.12, 0.10, align=CENTER3),
    ]

    datum_positions = ((0.35, 0.35), (5.65, 0.35), (3.00, 3.65))
    datums = [
        Pos(x, y, -params.datum_pad_height)
        * Cylinder(
            params.datum_pad_diameter / 2,
            params.datum_pad_height,
            align=MIN_Z,
        )
        for x, y in datum_positions
    ]
    tray = tray + [*seat_caps, *captures, *datums]
    tray.label = "micro_gearbox_housing"
    return tray


def make_gearbox_lid(params: GearboxParams):
    """Latched cover with driver/observation windows and an inner lip."""

    plate = Box(
        params.housing_length,
        params.housing_width,
        params.lid_thickness,
        align=MIN3,
    )
    lip_clearance = params.two_photon_running_clearance
    lip_depth = 0.10
    lip_wall = 0.10
    lip_outer_length = params.housing_length - 2 * (params.housing_wall + lip_clearance)
    lip_outer_width = params.housing_width - 2 * (params.housing_wall + lip_clearance)
    outer_lip = Pos(
        params.housing_wall + lip_clearance,
        params.housing_wall + lip_clearance,
        -lip_depth,
    ) * Box(lip_outer_length, lip_outer_width, lip_depth, align=MIN3)
    inner_lip = Pos(
        params.housing_wall + lip_clearance + lip_wall,
        params.housing_wall + lip_clearance + lip_wall,
        -lip_depth - 0.01,
    ) * Box(
        lip_outer_length - 2 * lip_wall,
        lip_outer_width - 2 * lip_wall,
        lip_depth + 0.02,
        align=MIN3,
    )
    lid = plate + (outer_lip - inner_lip)
    idler_x, idler_y = shaft_centers(params)[1]
    idler_relief = Pos(idler_x, idler_y, -0.01) * Cylinder(
        (params.shaft_diameter + params.two_photon_running_clearance) / 2,
        0.09,
        align=MIN_Z,
    )
    input_window = Pos(params.input_center_x, params.center_y, -0.02) * Cylinder(
        params.input_driver_window_diameter / 2,
        params.lid_thickness + 0.04,
        align=MIN_Z,
    )
    output_window = Pos(params.output_center_x, params.center_y, -0.02) * Cylinder(
        params.output_observation_window_diameter / 2,
        params.lid_thickness + 0.04,
        align=MIN_Z,
    )
    lid = lid - [idler_relief, input_window, output_window]

    lower_tab = Pos(1.25, 0.22, -0.135) * Box(
        0.34, 0.08, 0.29, align=CENTER3
    )
    lower_hook = Pos(1.25, 0.16, -0.25) * Box(
        0.34, 0.12, 0.06, align=CENTER3
    )
    upper_tab = Pos(4.75, params.housing_width - 0.22, -0.135) * Box(
        0.34, 0.08, 0.29, align=CENTER3
    )
    upper_hook = Pos(4.75, params.housing_width - 0.16, -0.25) * Box(
        0.34, 0.12, 0.06, align=CENTER3
    )
    lid = lid + [lower_tab, lower_hook, upper_tab, upper_hook]
    lid.label = "micro_gearbox_cover"
    return lid


def make_shaft(params: GearboxParams):
    chamfer = params.shaft_deburr_chamfer
    radius = params.shaft_diameter / 2
    if chamfer <= 0 or chamfer > 0.005 + 1e-12:
        raise ValueError("shaft deburr chamfer must be positive and no larger than 0.005 mm")
    lower = Cone(radius - chamfer, radius, chamfer, align=MIN_Z)
    middle = Pos(Z=chamfer) * Cylinder(
        radius,
        params.shaft_length - 2 * chamfer,
        align=MIN_Z,
    )
    upper = Pos(Z=params.shaft_length - chamfer) * Cone(
        radius,
        radius - chamfer,
        chamfer,
        align=MIN_Z,
    )
    shaft = lower + middle + upper
    shaft.label = "micro_gearbox_shaft"
    return shaft


def make_gearbox_assembly(params: GearboxParams, exploded: bool = False):
    """As-built or Z-exploded gearbox assembly.

    Gear phasing puts an input tooth into an idler valley and an idler valley
    against an output tooth.  The idler only changes spacing and direction; the
    24/12 output ratio is 2:1.
    """

    z_lift = (0.0, 0.0, 0.0) if not exploded else (0.55, 0.85, 1.15)
    centers = shaft_centers(params)
    children = [make_gearbox_housing(params)]

    shaft_start = 0.0
    for x, y in centers:
        children.append(Pos(x, y, shaft_start) * make_shaft(params))

    gear_specs = (
        (centers[2], params.output_teeth, 0.0, False, True),
        (
            centers[1],
            params.idler_teeth,
            180.0 - 180.0 / params.idler_teeth,
            False,
            False,
        ),
        (centers[0], params.input_teeth, 0.0, True, False),
    )
    for index, ((x, y), teeth, phase, drive_slot, phase_marks) in enumerate(gear_specs):
        children.append(
            Pos(x, y, params.gear_z + z_lift[index])
            * Rot(Z=phase)
            * make_spur_gear(
                teeth,
                params,
                drive_slot=drive_slot,
                optical_phase_marks=phase_marks,
            )
        )

    lid_z = params.housing_height if not exploded else params.housing_height + 1.75
    children.append(Pos(Z=lid_z) * make_gearbox_lid(params))
    assembly = Compound(children=children)
    assembly.label = "micro_gearbox_2PP_assembly"
    return assembly
