"""Low-cost assembly fixture and swappable micro-tool CAD.

The tools are deliberately uncomplicated: printed datum blocks and holders,
commodity tubing/needles, and simple replaceable steel tips.  They model the
physical collision and calibration envelopes without pretending to simulate a
specific 2PP fabrication process.
"""

from __future__ import annotations

from build123d import Align, Box, Compound, Cone, Cylinder, Pos

from .gearbox import shaft_centers
from .params import (
    PROBE_FLEXURE_LEAF_COUNT,
    GearboxParams,
    ToolingParams,
)


CENTER3 = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN3 = (Align.MIN, Align.MIN, Align.MIN)
MIN_Z = (Align.CENTER, Align.CENTER, Align.MIN)


def make_gearbox_nest(tool: ToolingParams, gearbox: GearboxParams):
    """Loose clearance-pocket carrier for a pre-fixtured 6 x 4 mm housing.

    The three underside holes clear the housing's external datum pads; they do
    not constitute modeled 3-2-1 contacts.  A clamp and explicit side contacts
    remain required before this carrier can restrain a workpiece.
    """

    base = Box(tool.nest_length, tool.nest_width, tool.nest_height, align=MIN3)
    pocket_length = gearbox.housing_length + 2 * tool.nest_pocket_clearance
    pocket_width = gearbox.housing_width + 2 * tool.nest_pocket_clearance
    pocket_x = (tool.nest_length - pocket_length) / 2
    pocket_y = (tool.nest_width - pocket_width) / 2
    pocket = Pos(
        pocket_x,
        pocket_y,
        tool.nest_height - tool.nest_pocket_depth,
    ) * Box(
        pocket_length,
        pocket_width,
        tool.nest_pocket_depth + 0.1,
        align=MIN3,
    )
    fasteners = [
        Pos(x, tool.nest_width / 2, -0.1)
        * Cylinder(tool.nest_fastener_diameter / 2, tool.nest_height + 0.2, align=MIN_Z)
        for x in (1.5, tool.nest_length - 1.5)
    ]
    datum_local = ((0.35, 0.35), (5.65, 0.35), (3.00, 3.65))
    datum_reliefs = [
        Pos(
            pocket_x + tool.nest_pocket_clearance + x,
            pocket_y + tool.nest_pocket_clearance + y,
            tool.nest_height - tool.nest_pocket_depth - 0.01,
        )
        * Cylinder(
            (gearbox.datum_pad_diameter + 0.06) / 2,
            gearbox.datum_pad_height + 0.05,
            align=MIN_Z,
        )
        for x, y in datum_local
    ]
    nest = base - [pocket, *fasteners, *datum_reliefs]
    nest.label = "gearbox_loose_clearance_pocket_carrier"
    return nest


def make_parts_tray(tool: ToolingParams, gearbox: GearboxParams):
    """Pocketed kitting tray for S1-S3, G3-G2-G1, housing, and cover."""

    tray = Box(tool.tray_length, tool.tray_width, tool.tray_height, align=MIN3)
    z = tool.tray_height - tool.tray_pocket_depth
    tooth_counts = (gearbox.output_teeth, gearbox.idler_teeth, gearbox.input_teeth)
    gear_pockets = []
    for index, teeth in enumerate(tooth_counts):
        outer_radius = gearbox.module * (teeth + 2) / 2
        gear_pockets.append(
            Pos(6.0 + 8.0 * index, 6.0, z)
            * Cylinder(
                outer_radius + tool.tray_gear_clearance,
                tool.tray_pocket_depth + 0.1,
                align=MIN_Z,
            )
        )
    shaft_pockets = [
        Pos(6.0 + 8.0 * index, 12.0, z)
        * Cylinder(
            gearbox.shaft_diameter / 2 + 0.18,
            tool.tray_pocket_depth + 0.1,
            align=MIN_Z,
        )
        for index in range(3)
    ]
    housing_pocket = Pos(3.0, 17.0, z) * Box(
        gearbox.housing_length + 0.30,
        gearbox.housing_width + 0.30,
        tool.tray_pocket_depth + 0.1,
        align=MIN3,
    )
    cover_pocket = Pos(14.0, 17.0, z) * Box(
        gearbox.housing_length + 0.30,
        gearbox.housing_width + 0.30,
        tool.tray_pocket_depth + 0.1,
        align=MIN3,
    )
    tray = tray - [*gear_pockets, *shaft_pockets, housing_pocket, cover_pocket]
    tray.label = "gearbox_insertion_order_parts_tray"
    return tray


def make_vacuum_micro_pick(tool: ToolingParams):
    """Printed holder plus replaceable hypodermic-tube vacuum nozzle."""

    holder = Cylinder(tool.vacuum_holder_diameter / 2, tool.vacuum_holder_length, align=MIN_Z)
    holder_bore = Pos(Z=-0.1) * Cylinder(
        tool.vacuum_nozzle_outer_diameter / 2 + 0.08,
        tool.vacuum_holder_length + 0.2,
        align=MIN_Z,
    )
    holder = holder - holder_bore
    nozzle = Pos(Z=-tool.vacuum_nozzle_length) * Cylinder(
        tool.vacuum_nozzle_outer_diameter / 2,
        tool.vacuum_nozzle_length,
        align=MIN_Z,
    )
    nozzle_bore = Pos(Z=-tool.vacuum_nozzle_length - 0.1) * Cylinder(
        tool.vacuum_nozzle_inner_diameter / 2,
        tool.vacuum_nozzle_length + 0.2,
        align=MIN_Z,
    )
    nozzle = nozzle - nozzle_bore
    tip_length = 1.5
    tip = Pos(Z=-tool.vacuum_nozzle_length - tip_length) * Cone(
        tool.vacuum_tip_outer_diameter / 2,
        tool.vacuum_nozzle_outer_diameter / 2,
        tip_length,
        align=MIN_Z,
    )
    tip_bore = Pos(Z=-tool.vacuum_nozzle_length - tip_length - 0.1) * Cylinder(
        tool.vacuum_tip_inner_diameter / 2,
        tip_length + 0.2,
        align=MIN_Z,
    )
    tip = tip - tip_bore
    holder.label = "vacuum_pick_printed_holder"
    nozzle.label = "vacuum_pick_replaceable_tube"
    tip.label = "vacuum_pick_soft_micro_tip"
    assembly = Compound(children=[holder, nozzle, tip])
    assembly.label = "vacuum_micro_pick"
    return assembly


def make_compliant_insertion_probe(tool: ToolingParams):
    """Connected four-leaf fixed-guided flexure for axial insertion.

    Four leaves span the free length along X and bend in Z.  Both ends overlap
    rigid anchor/platform blocks, so every leaf participates in the load path.
    At the nominal 2 GPa printed-polymer modulus the configured set is about
    0.60 N/mm, giving roughly 0.17 mm travel at the 0.10 N full-scale load.
    """

    free_length = tool.probe_flexure_length
    anchor_center_x = -(free_length + tool.probe_base_length) / 2
    base = Pos(X=anchor_center_x) * Box(
        tool.probe_base_length,
        tool.probe_base_width,
        tool.probe_base_height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    platform_length = 1.0
    platform_center_x = (free_length + platform_length) / 2
    platform = Pos(X=platform_center_x) * Box(
        platform_length,
        tool.probe_base_width * 0.72,
        tool.probe_base_height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    # The stiffness equation uses the clear span between the rigid blocks.
    # Small end overlaps make the boolean union robust without shortening that
    # modeled free length.
    anchor_overlap = 0.10
    leaf_positions = tuple(
        (side_y * tool.probe_base_width * 0.30, level_z * tool.probe_base_height)
        for side_y in (-1.0, 1.0)
        for level_z in (0.25, 0.75)
    )
    if len(leaf_positions) != PROBE_FLEXURE_LEAF_COUNT:
        raise AssertionError("probe leaf layout and stiffness model disagree")
    beams = [
        Pos(0, y, z)
        * Box(
            free_length + 2 * anchor_overlap,
            tool.probe_flexure_width,
            tool.probe_flexure_thickness,
            align=CENTER3,
        )
        for y, z in leaf_positions
    ]
    frame = base + [*beams, platform]
    tip = Pos(X=platform_center_x, Z=tool.probe_base_height) * Cylinder(
        tool.probe_tip_diameter / 2,
        tool.probe_tip_length,
        align=MIN_Z,
    )
    base.label = "insertion_probe_fixed_base"
    frame.label = "insertion_probe_connected_four_leaf_frame"
    platform.label = "insertion_probe_floating_platform"
    tip.label = "insertion_probe_replaceable_tip"
    probe = Compound(children=[frame, tip])
    probe.label = "compliant_insertion_probe"
    return probe


def make_rotary_drive_blade(tool: ToolingParams, gearbox: GearboxParams):
    """Replaceable screwdriver-style blade that clears G1's 0.10 mm slot."""

    if tool.rotary_blade_width >= gearbox.input_drive_slot_width:
        raise ValueError("rotary blade must be narrower than the G1 slot")
    shank = Cylinder(tool.rotary_shank_diameter / 2, tool.rotary_shank_length, align=MIN_Z)
    taper_length = 2.0
    taper_tip_radius = min(0.20, gearbox.input_driver_window_diameter / 2 - 0.02)
    taper = Pos(Z=-taper_length) * Cone(
        taper_tip_radius,
        tool.rotary_shank_diameter / 2,
        taper_length,
        align=MIN_Z,
    )
    blade = Pos(Z=-taper_length - tool.rotary_blade_thickness / 2) * Box(
        tool.rotary_blade_length,
        tool.rotary_blade_width,
        tool.rotary_blade_thickness,
        align=CENTER3,
    )
    shank.label = "rotary_driver_shank"
    blade.label = "rotary_G1_drive_blade"
    driver = Compound(children=[shank, taper, blade])
    driver.label = "rotary_drive_blade_tool"
    return driver


def make_calibration_pointer(tool: ToolingParams):
    """Three-foot printed datum base with a slender metrology pointer."""

    base = Box(
        tool.pointer_base_length,
        tool.pointer_base_width,
        tool.pointer_base_height,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    feet = [
        Pos(x, y, -0.6) * Cone(0.35, 0.18, 0.6, align=MIN_Z)
        for x, y in (
            (-tool.pointer_base_length * 0.36, -tool.pointer_base_width * 0.34),
            (tool.pointer_base_length * 0.36, -tool.pointer_base_width * 0.34),
            (0.0, tool.pointer_base_width * 0.34),
        )
    ]
    rod = Pos(Z=tool.pointer_base_height) * Cylinder(
        tool.pointer_rod_diameter / 2,
        tool.pointer_rod_length,
        align=MIN_Z,
    )
    pointer = Pos(Z=tool.pointer_base_height + tool.pointer_rod_length) * Cone(
        tool.pointer_rod_diameter / 2,
        tool.pointer_tip_diameter / 2,
        tool.pointer_tip_length,
        align=MIN_Z,
    )
    base.label = "calibration_pointer_three_point_base"
    pointer.label = "calibration_pointer_tip"
    assembly = Compound(children=[base, *feet, rod, pointer])
    assembly.label = "three_point_calibration_pointer"
    return assembly
