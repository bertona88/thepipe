"""STEP/STL and deterministic metadata export."""

from __future__ import annotations

import json
import re
from math import isfinite, radians
from pathlib import Path
from typing import Iterable

from build123d import Compound, export_step, export_stl
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from .digital_thread import canonical_sha256
from .kinematics import four_axis_arm_frames
from .params import GEARBOX_INSERTION_SEQUENCE, DesignConfig
from .records import PartRecord


def _safe_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_")


def compound(records: Iterable[PartRecord]):
    result = Compound(children=[record.shape for record in records])
    result.label = "pipe_cell_export"
    return result


def _rounded(value: float, digits: int = 9) -> float:
    if not isfinite(value):
        raise ValueError("non-finite mass property")
    result = round(float(value), digits)
    return 0.0 if result == 0 else result


def shape_metadata(shape, density_g_mm3: float | None = None) -> dict[str, object]:
    bbox = shape.bounding_box()
    volume = float(shape.volume)
    result: dict[str, object] = {
        "bbox_mm": {
            "min": [round(bbox.min.X, 6), round(bbox.min.Y, 6), round(bbox.min.Z, 6)],
            "max": [round(bbox.max.X, 6), round(bbox.max.Y, 6), round(bbox.max.Z, 6)],
            "size": [round(bbox.size.X, 6), round(bbox.size.Y, 6), round(bbox.size.Z, 6)],
        },
        "volume_mm3": round(volume, 9),
        "valid_brep": bool(shape.is_valid),
        "solid_count": len(shape.solids()),
        "density_g_mm3": density_g_mm3,
        "mass_g": None if density_g_mm3 is None else _rounded(volume * density_g_mm3, 12),
        "volume_centroid_mm": None,
        "center_of_mass_mm": None,
        "inertia_tensor_centroidal_mm5": None,
        "inertia_tensor_centroidal_g_mm2": None,
        "mass_properties_basis": "unavailable",
    }
    if not result["valid_brep"]:
        result["mass_properties_status"] = "invalid_brep"
        return result
    if result["solid_count"] == 0:
        result["mass_properties_status"] = "no_closed_solids"
        return result
    if volume <= 0:
        result["mass_properties_status"] = "nonpositive_volume"
        return result
    try:
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape.wrapped, props, True, False, False)
        center = props.CentreOfMass()
        matrix = props.MatrixOfInertia()
        centroid = [_rounded(v, 9) for v in (center.X(), center.Y(), center.Z())]
        inertia = [
            [_rounded(matrix.Value(row, column), 9) for column in range(1, 4)]
            for row in range(1, 4)
        ]
        result["volume_centroid_mm"] = centroid
        result["inertia_tensor_centroidal_mm5"] = inertia
        if density_g_mm3 is None:
            result["mass_properties_status"] = "geometry_only_density_unknown"
            result["mass_properties_basis"] = "uniform_unit_density_geometry_only"
        else:
            result["center_of_mass_mm"] = centroid
            result["inertia_tensor_centroidal_g_mm2"] = [
                [_rounded(value * density_g_mm3, 12) for value in row]
                for row in inertia
            ]
            result["mass_properties_status"] = "homogeneous_nominal_density"
            result["mass_properties_basis"] = "homogeneous_density_times_exact_brep"
    except (RuntimeError, ValueError):
        result["mass_properties_status"] = "kernel_error"
    return result


def metadata_document(
    records: list[PartRecord],
    config: DesignConfig,
    assembly_name: str,
) -> dict[str, object]:
    parameters = config.to_dict()
    record_documents = [
        {
            "name": record.name,
            "category": record.category,
            "material": record.material,
            "process": record.process,
            "printable": record.printable,
            "quantity": record.quantity,
            "notes": list(record.notes),
            **shape_metadata(record.shape, record.density_g_mm3),
        }
        for record in records
    ]
    geometry_facts = [
        {
            key: document[key]
            for key in ("name", "bbox_mm", "volume_mm3", "valid_brep", "solid_count")
        }
        for document in record_documents
    ]
    rail_center_radius = (
        config.tube.inner_radius
        - config.rail.wall_standoff
        - config.carriage.radial_depth / 2
    )
    return {
        "schema": config.schema_version,
        "assembly": assembly_name,
        "units": "mm",
        "coordinate_system": {
            "tube_axis": "+Z",
            "tube_centerline": [0.0, 0.0],
            "gear_shafts": "+Z",
        },
        "parameters": parameters,
        "parameter_sha256": canonical_sha256(parameters),
        "geometry_facts_sha256": canonical_sha256(geometry_facts),
        "calibrated_radial_datums_mm": {
            "physical_rail_center_radius": rail_center_radius,
            "shoulder_root_radius": config.arm.shoulder_root_radius,
            "rail_to_shoulder_offset": rail_center_radius - config.arm.shoulder_root_radius,
        },
        "arm_joint_axes_local": [
            {
                "name": frame["name"],
                "origin_mm": [_rounded(value, 9) for value in frame["origin_mm"]],
                "direction": [_rounded(value, 9) for value in frame["direction"]],
            }
            for frame in four_axis_arm_frames(
                config.arm.link_lengths,
                config.arm.default_joint_angles_deg,
            )
        ],
        "gearbox_assembly_sequence": list(GEARBOX_INSERTION_SEQUENCE),
        "gearbox_assembly_operations": [
            {"action": "insert", "part": "S1"},
            {"action": "insert", "part": "S2"},
            {"action": "insert", "part": "S3"},
            {"action": "place_on_shaft", "part": "G3"},
            {"action": "place_on_shaft", "part": "G2"},
            {"action": "place_on_shaft", "part": "G1"},
            {"action": "close_two_latches", "part": "cover"},
        ],
        "tendon_loop_channels": [
            {
                "axis": axis,
                "motor_count": 1,
                "spool_count": 1,
                "line_sides_on_shared_differential_capstan": 2,
            }
            for axis in config.arm.actuator_axes
        ],
        "arm_actuation_bom": {
            "arm_count": config.tube.rail_count,
            "channels_per_arm": config.arm.actuator_count,
            "motor_count": config.tube.rail_count * config.arm.actuator_count,
            "spool_count": config.tube.rail_count * config.arm.actuator_count,
            "line_diameter_mm": config.arm.tendon_diameter,
            "line_sides_per_shared_capstan": 2,
        },
        "records": record_documents,
    }


def export_assembly(
    records: list[PartRecord],
    config: DesignConfig,
    output_dir: Path,
    assembly_name: str,
    *,
    tolerance: float | None = None,
    individual: bool = False,
) -> dict[str, Path]:
    """Write a closed bundle and return its generated paths."""

    output_dir.mkdir(parents=True, exist_ok=True)
    name = _safe_name(assembly_name)
    model = compound(records)
    linear_tolerance = tolerance or config.manufacturing.stl_linear_tolerance
    angular_tolerance = radians(config.manufacturing.stl_angular_tolerance_deg)
    step_path = output_dir / f"{name}.step"
    stl_path = output_dir / f"{name}.stl"
    metadata_path = output_dir / f"{name}.metadata.json"
    export_step(model, step_path)
    export_stl(
        model,
        stl_path,
        tolerance=linear_tolerance,
        angular_tolerance=angular_tolerance,
    )
    metadata_path.write_text(
        json.dumps(metadata_document(records, config, assembly_name), indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )

    paths: dict[str, Path] = {
        "step": step_path,
        "stl": stl_path,
        "metadata": metadata_path,
    }
    if individual:
        parts_dir = output_dir / f"{name}_parts"
        parts_dir.mkdir(parents=True, exist_ok=True)
        for record in records:
            part_name = _safe_name(record.name)
            part_step = parts_dir / f"{part_name}.step"
            part_stl = parts_dir / f"{part_name}.stl"
            export_step(record.shape, part_step)
            export_stl(
                record.shape,
                part_stl,
                tolerance=linear_tolerance,
                angular_tolerance=angular_tolerance,
            )
        paths["parts"] = parts_dir
    return paths
