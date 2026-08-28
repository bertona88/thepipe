"""Tube, longitudinal rails, and low-cost sliding carriages."""

from __future__ import annotations

from build123d import Align, Box, Compound, Cylinder, Pos

from .params import CarriageParams, RailParams, TubeParams


CENTER3 = (Align.CENTER, Align.CENTER, Align.CENTER)
MIN_Z = (Align.CENTER, Align.CENTER, Align.MIN)


def make_tube(params: TubeParams):
    """A straight acrylic/PVC-equivalent tube, axis along +Z."""

    outer = Cylinder(params.outer_radius, params.length, align=MIN_Z)
    inner = Pos(Z=-0.5) * Cylinder(
        params.inner_radius, params.length + 1.0, align=MIN_Z
    )
    tube = outer - inner
    tube.label = "cell_tube"
    return tube


def make_rail(params: RailParams, length: float):
    """Rectangular axial rail centered on X/Y with its lower end at Z=0."""

    rail = Box(
        params.radial_depth,
        params.tangential_width,
        length,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    rail.label = "axial_rail"
    return rail


def make_rail_carriage(carriage: CarriageParams, rail: RailParams):
    """End-loaded sleeve carriage with an inward arm mounting deck.

    The intentionally simple sliding interface can be printed in PETG/resin
    and lined with UHMW tape.  It trades unloaded friction for low cost; the
    vision loop, not the rail, determines absolute position.
    """

    sleeve = Box(
        carriage.radial_depth,
        carriage.tangential_width,
        carriage.axial_length,
        align=CENTER3,
    )
    tunnel = Box(
        rail.radial_depth + 2 * rail.carriage_clearance,
        rail.tangential_width + 2 * rail.carriage_clearance,
        carriage.axial_length + 1.0,
        align=CENTER3,
    )
    sleeve = sleeve - tunnel

    deck_max_x = -carriage.radial_depth / 2 + 0.45
    deck_center_x = deck_max_x - carriage.deck_reach / 2
    deck = Pos(X=deck_center_x) * Box(
        carriage.deck_reach,
        carriage.deck_width,
        carriage.deck_thickness,
        align=CENTER3,
    )
    body = sleeve + deck

    # One pivot hole at the deck tip and two cable-tie/fastener holes inboard.
    pivot_x = deck_max_x - carriage.deck_reach + 1.2
    pivot = Pos(X=pivot_x) * Cylinder(
        carriage.pivot_hole_diameter / 2,
        carriage.deck_thickness + 1.0,
        align=CENTER3,
    )
    tie_holes = [
        Pos(deck_center_x + 1.6, y, 0)
        * Cylinder(
            carriage.fastener_hole_diameter / 2,
            carriage.deck_thickness + 1.0,
            align=CENTER3,
        )
        for y in (-2.2, 2.2)
    ]
    body = body - [pivot, *tie_holes]
    body.label = "rail_carriage"
    return body


def make_theta_track(params: TubeParams, axial_width: float = 3.0, radial_depth: float = 0.8):
    """Printed inner end-track for the rotating longitudinal-rail bogies."""

    if radial_depth <= 0 or radial_depth >= params.inner_radius:
        raise ValueError("theta track radial depth is invalid")
    outer = Cylinder(params.inner_radius - 0.15, axial_width, align=MIN_Z)
    inner = Pos(Z=-0.25) * Cylinder(
        params.inner_radius - 0.15 - radial_depth,
        axial_width + 0.5,
        align=MIN_Z,
    )
    ring = outer - inner
    ring.label = "theta_end_track"
    return ring


def make_theta_track_quadrant(
    params: TubeParams,
    axial_width: float = 3.0,
    radial_depth: float = 0.8,
):
    """One 90-degree printable quadrant; four make each idealized end track."""

    outer = Cylinder(
        params.inner_radius - 0.15,
        axial_width,
        arc_size=90.0,
        align=MIN_Z,
    )
    inner = Pos(Z=-0.25) * Cylinder(
        params.inner_radius - 0.15 - radial_depth,
        axial_width + 0.5,
        arc_size=90.0,
        align=MIN_Z,
    )
    quadrant = outer - inner
    quadrant.label = "theta_end_track_quadrant"
    return quadrant


def make_theta_bogie(rail: RailParams):
    """One cheap belt-driven rail-end bogie assembly, local radial axis +X."""

    body = make_theta_bogie_body(rail)
    rollers = [
        Pos(-2.5, y, 0) * Cylinder(1.4, 1.4, align=CENTER3)
        for y in (-3.6, 3.6)
    ]
    bogie = Compound(children=[body, *rollers])
    bogie.label = "theta_rail_end_bogie"
    return bogie


def make_theta_bogie_body(rail: RailParams):
    """The printable bogie body without purchased acetal rollers."""

    body = Box(4.8, 10.0, 4.0, align=CENTER3)
    rail_socket = Box(
        rail.radial_depth + 0.25,
        rail.tangential_width + 0.25,
        4.8,
        align=CENTER3,
    )
    body = body - rail_socket
    body.label = "theta_rail_end_bogie_body"
    return body
