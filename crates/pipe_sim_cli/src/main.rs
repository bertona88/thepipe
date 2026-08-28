use std::collections::BTreeSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

use pipe_sim::{ReferenceSimulator, ScenarioSpec};
use serde_json::Value;
use sha2::{Digest, Sha256};

const DEFAULT_MAX_CYCLES: u32 = 12_000;
const CAD_SCHEMA: &str = "pipe-cad/0.1";
const CAD_INSERTION_SEQUENCE: [&str; 7] = ["S1", "S2", "S3", "G3", "G2", "G1", "cover"];
// SHA-256 of canonical JSON for the supported scenario after replacing only
// cad_metadata_path with the relocation sentinel used below. This digest is
// deliberately independent of include_str!: editing the checked-in scenario
// cannot silently redefine the compiled runtime contract.
const COMPILED_SCENARIO_CONTRACT_SHA256: &str =
    "943aa5ff97bcf31f922b7dde245eb382b59950339793d7c4ecb44cdc9d55e5be";

#[derive(Debug, PartialEq, Eq)]
struct Options {
    scenario: String,
    report: Option<PathBuf>,
    max_cycles: u32,
    pretty: bool,
}

fn usage() -> String {
    format!(
        "pipe-sim --scenario <name|path.json> [--report <path|->] [--max-cycles N] [--compact]\n\
         built-ins: {}",
        ScenarioSpec::available().join("|")
    )
}

fn parse_args<I>(args: I) -> Result<Options, String>
where
    I: IntoIterator<Item = String>,
{
    let mut options = Options {
        scenario: "nominal".to_owned(),
        report: None,
        max_cycles: DEFAULT_MAX_CYCLES,
        pretty: true,
    };
    let mut args = args.into_iter();
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--scenario" => {
                options.scenario = args.next().ok_or("--scenario requires a name")?;
            }
            "--report" => {
                options.report = Some(PathBuf::from(
                    args.next().ok_or("--report requires a path or '-'")?,
                ));
            }
            "--max-cycles" => {
                let value = args
                    .next()
                    .ok_or("--max-cycles requires a positive integer")?;
                options.max_cycles = value
                    .parse::<u32>()
                    .map_err(|_| format!("invalid --max-cycles value '{value}'"))?;
                if options.max_cycles == 0 {
                    return Err("--max-cycles must be positive".to_owned());
                }
            }
            "--compact" => options.pretty = false,
            "--help" | "-h" => return Err(usage()),
            other => return Err(format!("unknown argument '{other}'\n{}", usage())),
        }
    }
    resolve_scenario(&options.scenario)?;
    Ok(options)
}

fn resolve_scenario(input: &str) -> Result<ScenarioSpec, String> {
    if let Ok(spec) = ScenarioSpec::named(input) {
        return Ok(spec);
    }
    let path = PathBuf::from(input);
    let source = fs::read_to_string(&path).map_err(|error| {
        format!("scenario '{input}' is neither a built-in name nor a readable JSON file: {error}")
    })?;
    parse_scenario_document(&source, &path)
}

fn parse_scenario_document(source: &str, scenario_path: &Path) -> Result<ScenarioSpec, String> {
    let origin = scenario_path.display().to_string();
    let value: Value = serde_json::from_str(source)
        .map_err(|error| format!("invalid scenario JSON in {origin}: {error}"))?;
    let object = value
        .as_object()
        .ok_or_else(|| format!("invalid scenario JSON in {origin}: root must be an object"))?;
    let schema = object
        .get("schema_version")
        .and_then(Value::as_u64)
        .ok_or_else(|| format!("invalid scenario JSON in {origin}: schema_version must be 1"))?;
    if schema != 1 {
        return Err(format!(
            "unsupported scenario schema_version {schema} in {origin}; expected 1"
        ));
    }
    let name = object
        .get("name")
        .and_then(Value::as_str)
        .ok_or_else(|| format!("invalid scenario JSON in {origin}: missing string name"))?;
    if !matches!(name, "micro_spur_gearbox_v1" | "gearbox_baseline_v1") {
        return Err(format!(
            "unsupported scenario name '{name}' in {origin}; expected gearbox_baseline_v1"
        ));
    }
    // This executable currently constructs a compiled reference model. These
    // checks provide field-specific diagnostics; a whole-document contract
    // comparison below rejects drift in fields the runtime does not consume.
    for (path, expected) in [
        (&["simulation", "fixed_step_s"][..], 0.001),
        (&["simulation", "max_time_s"][..], 240.0),
        (&["tube", "inner_diameter_mm"][..], 160.0),
        (&["tube", "instrumented_length_mm"][..], 320.0),
        (&["tube", "qualified_work_diameter_mm"][..], 80.0),
        (&["tube", "rail_radial_offset_mm"][..], 72.0),
        (&["arms", "count"][..], 4.0),
        (&["arms", "upper_link_mm"][..], 32.0),
        (&["arms", "forearm_mm"][..], 30.0),
        (&["arms", "wrist_tool_offset_mm"][..], 15.0),
        (&["arms", "tendon_pairs"][..], 4.0),
        (&["arms", "tendon_pretension_per_tendon_n"][..], 1.2),
        (&["arms", "tendon_stiffness_n_per_mm"][..], 7.5),
        (&["arms", "spool_radius_mm"][..], 3.0),
        (&["arms", "joint_tendon_moment_arm_mm"][..], 1.65),
        (&["arms", "usable_tendon_payout_mm"][..], 12.0),
        (&["arms", "actuator_backlash_mm"][..], 0.018),
        (&["arms", "actuator_force_limit_n"][..], 4.0),
        (&["arms", "gripper_opening_mm"][..], 2.8),
        (&["arms", "gripper_force_limit_n"][..], 0.15),
        (&["sensing", "camera_count"][..], 6.0),
        (&["sensing", "image_width_px"][..], 1280.0),
        (&["sensing", "image_height_px"][..], 800.0),
        (&["sensing", "horizontal_fov_deg"][..], 68.0),
        (&["sensing", "global_camera_layout", "radius_mm"][..], 60.0),
        (
            &["sensing", "global_camera_layout", "same_end_chord_mm"][..],
            103.923_048_454_132_63,
        ),
        (
            &["sensing", "stereo_baseline_mm"][..],
            103.923_048_454_132_63,
        ),
        (&["sensing", "depth_quantization_mm"][..], 0.00025),
        (&["sensing", "pixel_sigma_px"][..], 0.18),
        (&["sensing", "dropout_probability"][..], 0.002),
        (&["sensing", "minimum_views"][..], 2.0),
        (
            &["sensing", "structured_light_projector_layout", "radius_mm"][..],
            60.0,
        ),
        (
            &[
                "sensing",
                "structured_light_projector_layout",
                "azimuth_deg",
            ][..],
            90.0,
        ),
        (
            &[
                "sensing",
                "structured_light_projector_layout",
                "z_offset_mm",
            ][..],
            0.0,
        ),
        (&["sensing", "local_macro_view", "camera_count"][..], 2.0),
        (
            &["sensing", "local_macro_view", "image_width_px"][..],
            2048.0,
        ),
        (
            &["sensing", "local_macro_view", "image_height_px"][..],
            1536.0,
        ),
        (
            &["sensing", "local_macro_view", "stereo_baseline_mm"][..],
            12.0,
        ),
        (&["sensing", "local_macro_view", "field_width_mm"][..], 4.0),
        (&["sensing", "local_macro_view", "field_height_mm"][..], 3.0),
        (
            &["sensing", "local_macro_view", "pixel_scale_mm"][..],
            0.002,
        ),
        (&["sensing", "local_macro_view", "mount_arm_index"][..], 1.0),
        (
            &["sensing", "local_macro_view", "mount_normal_offset_mm"][..],
            11.0,
        ),
        (&["gearbox", "housing_body_height_mm"][..], 1.60),
        (&["gearbox", "housing_wall_mm"][..], 0.030),
        (&["gearbox", "cover_thickness_mm"][..], 0.20),
        (&["gearbox", "module_mm"][..], 0.10),
        (&["gearbox", "pressure_angle_deg"][..], 25.0),
        (&["gearbox", "shaft_diameter_mm"][..], 0.350),
        (&["gearbox", "shaft_length_mm"][..], 1.55),
        (&["gearbox", "shaft_seated_center_z_mm"][..], 0.775),
        (&["gearbox", "gear_bore_nominal_mm"][..], 0.420),
        (&["gearbox", "face_width_mm"][..], 0.35),
        (&["gearbox", "gear_total_height_mm"][..], 1.30),
        (&["gearbox", "nominal_backlash_mm"][..], 0.020),
        (&["gearbox", "cover_seated_center_z_mm"][..], 1.70),
        (&["guards", "insertion_speed_mm_s"][..], 0.05),
    ] {
        let actual = required_number(&value, path, &origin)?;
        require_near(&origin, &path.join("."), actual, expected)?;
    }
    require_number_array(
        &value,
        &["tube", "macro_qualified_zone_mm"],
        &[12.0, 12.0, 8.0],
        &origin,
    )?;
    require_number_array(
        &value,
        &["arms", "link_collision_radii_mm"],
        &[3.2, 2.8, 1.8],
        &origin,
    )?;
    require_number_array(
        &value,
        &["sensing", "global_camera_layout", "end_z_mm"],
        &[-106.0, 106.0],
        &origin,
    )?;
    require_nested_number_arrays(
        &value,
        &["sensing", "global_camera_layout", "triplet_azimuths_deg"],
        &[&[0.0, 120.0, 240.0], &[60.0, 180.0, 300.0]],
        &origin,
    )?;

    let geometry = required_string(&value, &["part_model", "geometry"], &origin)?;
    if geometry != "ideal_nominal" {
        return Err(format!(
            "unsupported part_model.geometry '{geometry}' in {origin}; expected ideal_nominal"
        ));
    }
    let process = required_string(&value, &["part_model", "eventual_process"], &origin)?;
    if process != "external_2pp_not_simulated" {
        return Err(format!(
            "unsupported part_model.eventual_process '{process}' in {origin}; expected external_2pp_not_simulated"
        ));
    }
    let gears = value
        .pointer("/gearbox/gears")
        .and_then(Value::as_array)
        .ok_or_else(|| {
            format!("invalid scenario JSON in {origin}: gearbox.gears must be an array")
        })?;
    let expected_gears = [
        ("input_gear", 12_u64, "shaft_a"),
        ("idler_gear", 18_u64, "shaft_b"),
        ("output_gear", 24_u64, "shaft_c"),
    ];
    if gears.len() != expected_gears.len() {
        return Err(format!(
            "unsupported gearbox.gears length {} in {origin}; expected 3",
            gears.len()
        ));
    }
    for (index, (gear, (expected_id, expected_teeth, expected_shaft))) in
        gears.iter().zip(expected_gears).enumerate()
    {
        let id = gear.get("id").and_then(Value::as_str);
        let teeth = gear.get("teeth").and_then(Value::as_u64);
        let shaft = gear.get("shaft").and_then(Value::as_str);
        if (id, teeth, shaft)
            != (
                Some(expected_id),
                Some(expected_teeth),
                Some(expected_shaft),
            )
        {
            return Err(format!(
                "unsupported gearbox.gears[{index}] in {origin}; expected {expected_id}/{expected_teeth}/{expected_shaft}"
            ));
        }
        for (field, expected) in [
            ("z_mm", 0.90),
            ("seated_bottom_z_mm", 0.25),
            ("seated_center_z_mm", 0.90),
        ] {
            let actual = gear.get(field).and_then(Value::as_f64).ok_or_else(|| {
                format!("invalid scenario JSON in {origin}: gearbox.gears[{index}].{field} must be numeric")
            })?;
            require_near(
                &origin,
                &format!("gearbox.gears[{index}].{field}"),
                actual,
                expected,
            )?;
        }
    }

    let order = value
        .get("assembly_order")
        .and_then(Value::as_array)
        .ok_or_else(|| {
            format!("invalid scenario JSON in {origin}: assembly_order must be an array")
        })?;
    let actual = order
        .iter()
        .map(Value::as_str)
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| {
            format!("invalid scenario JSON in {origin}: assembly_order entries must be strings")
        })?;
    let expected = [
        "housing",
        "shaft_a",
        "shaft_b",
        "shaft_c",
        "output_gear",
        "idler_gear",
        "input_gear",
        "pre_cover_rotation_test",
        "cover_handoff",
        "cover_closure",
        "post_cover_rotation_test",
    ];
    if actual != expected {
        return Err(format!(
            "unsupported assembly_order in {origin}; expected the canonical S1/S2/S3, G3/G2/G1, cover sequence"
        ));
    }
    if value
        .pointer("/part_model/apply_dimensional_perturbations")
        .and_then(Value::as_bool)
        != Some(false)
    {
        return Err(format!(
            "unsupported or missing part_model.apply_dimensional_perturbations in {origin}; the v1 baseline requires false"
        ));
    }

    validate_locked_scenario_contract(&value, &origin)?;

    let seed_value = object
        .get("seed")
        .ok_or_else(|| format!("invalid scenario JSON in {origin}: missing seed"))?;
    let seed = parse_seed(seed_value).ok_or_else(|| {
        format!("invalid seed in {origin}; expected integer or 0x-prefixed hex string")
    })?;
    if seed != 0x5049_5045_5F47_4258 {
        return Err(format!(
            "unsupported seed 0x{seed:016X} in {origin}; canonical v1 seed is 0x504950455F474258"
        ));
    }
    let validated_cad = load_and_validate_cad_metadata(&value, scenario_path, &origin)?;
    let canonical_scenario = serde_json::to_vec(&value)
        .map_err(|error| format!("failed to canonicalize scenario JSON in {origin}: {error}"))?;
    let mut spec = ScenarioSpec::named("nominal").map_err(|error| error.to_string())?;
    spec.seed = seed;
    spec.scenario_sha256 = Some(sha256_hex(&canonical_scenario));
    spec.cad_parameter_sha256 = Some(validated_cad.parameter_sha256);
    spec.cad_geometry_facts_sha256 = Some(validated_cad.geometry_facts_sha256);
    spec.refresh_configuration_sha256();
    Ok(spec)
}

fn validate_locked_scenario_contract(value: &Value, origin: &str) -> Result<(), String> {
    fn normalized(mut value: Value) -> Value {
        if let Some(object) = value.as_object_mut() {
            object.insert(
                "cad_metadata_path".to_owned(),
                Value::String("<normalized-external-path>".to_owned()),
            );
        }
        value
    }

    let canonical = serde_json::to_vec(&normalized(value.clone())).map_err(|error| {
        format!("failed to canonicalize normalized scenario contract in {origin}: {error}")
    })?;
    let actual_sha256 = sha256_hex(&canonical);
    if actual_sha256 != COMPILED_SCENARIO_CONTRACT_SHA256 {
        return Err(format!(
            "unsupported scenario content in {origin}: the compiled v1 runtime accepts only contract SHA-256 {COMPILED_SCENARIO_CONTRACT_SHA256} (got {actual_sha256}); cad_metadata_path is the only relocatable field"
        ));
    }
    Ok(())
}

#[derive(Debug, PartialEq, Eq)]
struct ValidatedCadMetadata {
    parameter_sha256: String,
    geometry_facts_sha256: String,
}

fn load_and_validate_cad_metadata(
    scenario: &Value,
    scenario_path: &Path,
    scenario_origin: &str,
) -> Result<ValidatedCadMetadata, String> {
    let configured_path = required_string(scenario, &["cad_metadata_path"], scenario_origin)?;
    let configured_path = PathBuf::from(configured_path);
    let metadata_path = if configured_path.is_absolute() {
        configured_path
    } else {
        scenario_path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .join(configured_path)
    };
    let metadata_origin = metadata_path.display().to_string();
    let source = fs::read_to_string(&metadata_path).map_err(|error| {
        format!(
            "failed to read CAD metadata {} referenced by {scenario_origin}: {error}",
            metadata_path.display()
        )
    })?;
    let metadata: Value = serde_json::from_str(&source)
        .map_err(|error| format!("invalid CAD metadata JSON in {metadata_origin}: {error}"))?;
    let object = metadata.as_object().ok_or_else(|| {
        format!("invalid CAD metadata JSON in {metadata_origin}: root must be an object")
    })?;
    let schema = object
        .get("schema")
        .and_then(Value::as_str)
        .ok_or_else(|| {
            format!("invalid CAD metadata in {metadata_origin}: schema must be {CAD_SCHEMA}")
        })?;
    if schema != CAD_SCHEMA {
        return Err(format!(
            "unsupported CAD metadata schema '{schema}' in {metadata_origin}; expected {CAD_SCHEMA}"
        ));
    }
    for (field, expected) in [("assembly", "gearbox"), ("units", "mm")] {
        let actual = object.get(field).and_then(Value::as_str).ok_or_else(|| {
            format!("invalid CAD metadata in {metadata_origin}: {field} must be '{expected}'")
        })?;
        if actual != expected {
            return Err(format!(
                "unsupported CAD metadata {field}='{actual}' in {metadata_origin}; expected '{expected}'"
            ));
        }
    }

    let parameters = object.get("parameters").ok_or_else(|| {
        format!("invalid CAD metadata in {metadata_origin}: missing parameters object")
    })?;
    if !parameters.is_object() {
        return Err(format!(
            "invalid CAD metadata in {metadata_origin}: parameters must be an object"
        ));
    }
    let declared_hash = object
        .get("parameter_sha256")
        .and_then(Value::as_str)
        .ok_or_else(|| {
            format!("invalid CAD metadata in {metadata_origin}: missing lowercase parameter_sha256")
        })?;
    if declared_hash.len() != 64
        || !declared_hash
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(format!(
            "invalid parameter_sha256 in {metadata_origin}; expected 64 lowercase hex digits"
        ));
    }
    let canonical_parameters = serde_json::to_vec(parameters).map_err(|error| {
        format!("failed to canonicalize CAD parameters in {metadata_origin}: {error}")
    })?;
    let computed_hash = sha256_hex(&canonical_parameters);
    if declared_hash != computed_hash {
        return Err(format!(
            "CAD parameter_sha256 mismatch in {metadata_origin}: declared {declared_hash}, computed {computed_hash}"
        ));
    }

    let insertion = object
        .get("gearbox_assembly_sequence")
        .and_then(Value::as_array)
        .ok_or_else(|| {
            format!(
                "invalid CAD metadata in {metadata_origin}: gearbox_assembly_sequence must be an array"
            )
        })?;
    let insertion = insertion
        .iter()
        .map(Value::as_str)
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| {
            format!(
                "invalid CAD metadata in {metadata_origin}: gearbox_assembly_sequence entries must be strings"
            )
        })?;
    if insertion != CAD_INSERTION_SEQUENCE {
        return Err(format!(
            "CAD insertion sequence mismatch in {metadata_origin}; expected S1,S2,S3,G3,G2,G1,cover"
        ));
    }

    let records = object
        .get("records")
        .and_then(Value::as_array)
        .ok_or_else(|| {
            format!("invalid CAD metadata in {metadata_origin}: records must be an array")
        })?;
    let declared_geometry_hash = object
        .get("geometry_facts_sha256")
        .and_then(Value::as_str)
        .ok_or_else(|| {
            format!(
                "invalid CAD metadata in {metadata_origin}: missing lowercase geometry_facts_sha256"
            )
        })?;
    if declared_geometry_hash.len() != 64
        || !declared_geometry_hash
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(format!(
            "invalid geometry_facts_sha256 in {metadata_origin}; expected 64 lowercase hex digits"
        ));
    }
    let geometry_facts = canonical_geometry_facts(records, &metadata_origin)?;
    let computed_geometry_hash =
        sha256_hex(&serde_json::to_vec(&geometry_facts).map_err(|error| {
            format!("failed to canonicalize CAD geometry facts in {metadata_origin}: {error}")
        })?);
    if declared_geometry_hash != computed_geometry_hash {
        return Err(format!(
            "CAD geometry_facts_sha256 mismatch in {metadata_origin}: declared {declared_geometry_hash}, computed {computed_geometry_hash}"
        ));
    }
    let mut names = BTreeSet::new();
    for (index, record) in records.iter().enumerate() {
        let name = record.get("name").and_then(Value::as_str).ok_or_else(|| {
            format!(
                "invalid CAD metadata in {metadata_origin}: records[{index}].name must be a string"
            )
        })?;
        if !names.insert(name) {
            return Err(format!(
                "invalid CAD metadata in {metadata_origin}: duplicate record name '{name}'"
            ));
        }
        if record.get("valid_brep").and_then(Value::as_bool) != Some(true) {
            return Err(format!(
                "invalid BREP for CAD record '{name}' in {metadata_origin}"
            ));
        }
        if record.get("solid_count").and_then(Value::as_u64) == Some(0)
            || record.get("solid_count").and_then(Value::as_u64).is_none()
        {
            return Err(format!(
                "invalid solid_count for CAD record '{name}' in {metadata_origin}"
            ));
        }
    }
    for required in [
        "gearbox_housing",
        "S1",
        "S2",
        "S3",
        "G3",
        "G2",
        "G1",
        "cover",
    ] {
        if !names.contains(required) {
            return Err(format!(
                "CAD metadata {metadata_origin} is missing required record '{required}'"
            ));
        }
    }

    validate_cad_dimensions(scenario, &metadata, scenario_origin, &metadata_origin)?;
    Ok(ValidatedCadMetadata {
        parameter_sha256: declared_hash.to_owned(),
        geometry_facts_sha256: declared_geometry_hash.to_owned(),
    })
}

fn validate_cad_dimensions(
    scenario: &Value,
    metadata: &Value,
    scenario_origin: &str,
    metadata_origin: &str,
) -> Result<(), String> {
    let scenario_number = |path: &[&str]| required_number(scenario, path, scenario_origin);
    let cad_number = |path: &[&str]| required_number(metadata, path, metadata_origin);
    let compare = |label: &str, actual: f64, expected: f64| {
        require_manifest_near(metadata_origin, scenario_origin, label, actual, expected)
    };

    compare(
        "tube.inner_diameter_mm",
        2.0 * cad_number(&["parameters", "tube", "inner_radius"])?,
        scenario_number(&["tube", "inner_diameter_mm"])?,
    )?;
    let tube_length = cad_number(&["parameters", "tube", "length"])?;
    let end_margin = cad_number(&["parameters", "tube", "end_margin"])?;
    compare(
        "tube.instrumented_length_mm",
        tube_length - 2.0 * end_margin,
        scenario_number(&["tube", "instrumented_length_mm"])?,
    )?;
    compare(
        "arms.count",
        cad_number(&["parameters", "tube", "rail_count"])?,
        scenario_number(&["arms", "count"])?,
    )?;
    let link_lengths = required_number_array_value(
        metadata,
        &["parameters", "arm", "link_lengths"],
        metadata_origin,
    )?;
    let expected_links = [
        scenario_number(&["arms", "upper_link_mm"])?,
        scenario_number(&["arms", "forearm_mm"])?,
        scenario_number(&["arms", "wrist_tool_offset_mm"])?,
    ];
    if link_lengths.len() != expected_links.len() {
        return Err(format!(
            "CAD/scenario mismatch for arm.link_lengths: {} has {} entries; {} requires 3",
            metadata_origin,
            link_lengths.len(),
            scenario_origin
        ));
    }
    for (index, (&actual, &expected)) in link_lengths.iter().zip(&expected_links).enumerate() {
        compare(&format!("arm.link_lengths[{index}]"), actual, expected)?;
    }
    for (cad_path, scenario_path, label) in [
        (
            &["parameters", "arm", "spool_radius"][..],
            &["arms", "spool_radius_mm"][..],
            "arm.spool_radius_mm",
        ),
        (
            &["parameters", "arm", "tendon_offset"][..],
            &["arms", "joint_tendon_moment_arm_mm"][..],
            "arm.joint_tendon_moment_arm_mm",
        ),
        (
            &["parameters", "arm", "usable_tendon_payout"][..],
            &["arms", "usable_tendon_payout_mm"][..],
            "arm.usable_tendon_payout_mm",
        ),
        (
            &["parameters", "gripper", "jaw_opening"][..],
            &["arms", "gripper_opening_mm"][..],
            "gripper.max_opening_mm",
        ),
        (
            &["parameters", "sensing", "global_camera_count"][..],
            &["sensing", "camera_count"][..],
            "sensing.global_camera_count",
        ),
        (
            &["parameters", "sensing", "global_image_width_px"][..],
            &["sensing", "image_width_px"][..],
            "sensing.global_image_width_px",
        ),
        (
            &["parameters", "sensing", "global_image_height_px"][..],
            &["sensing", "image_height_px"][..],
            "sensing.global_image_height_px",
        ),
        (
            &["parameters", "sensing", "global_horizontal_fov_deg"][..],
            &["sensing", "horizontal_fov_deg"][..],
            "sensing.global_horizontal_fov_deg",
        ),
        (
            &["parameters", "sensing", "simultaneous_macro_view_count"][..],
            &["sensing", "local_macro_view", "camera_count"][..],
            "sensing.simultaneous_macro_view_count",
        ),
        (
            &["parameters", "sensing", "global_camera_front_radius"][..],
            &["sensing", "global_camera_layout", "radius_mm"][..],
            "sensing.global_camera_front_radius",
        ),
        (
            &["parameters", "sensing", "projector_front_radius"][..],
            &["sensing", "structured_light_projector_layout", "radius_mm"][..],
            "sensing.projector_front_radius",
        ),
        (
            &["parameters", "sensing", "projector_azimuth_deg"][..],
            &[
                "sensing",
                "structured_light_projector_layout",
                "azimuth_deg",
            ][..],
            "sensing.projector_azimuth_deg",
        ),
        (
            &["parameters", "sensing", "projector_z_offset"][..],
            &[
                "sensing",
                "structured_light_projector_layout",
                "z_offset_mm",
            ][..],
            "sensing.projector_z_offset_mm",
        ),
        (
            &["parameters", "sensing", "macro_stereo_baseline"][..],
            &["sensing", "local_macro_view", "stereo_baseline_mm"][..],
            "sensing.macro_stereo_baseline",
        ),
        (
            &["parameters", "sensing", "macro_image_width_px"][..],
            &["sensing", "local_macro_view", "image_width_px"][..],
            "sensing.macro_image_width_px",
        ),
        (
            &["parameters", "sensing", "macro_image_height_px"][..],
            &["sensing", "local_macro_view", "image_height_px"][..],
            "sensing.macro_image_height_px",
        ),
        (
            &["parameters", "sensing", "macro_field_width"][..],
            &["sensing", "local_macro_view", "field_width_mm"][..],
            "sensing.macro_field_width_mm",
        ),
        (
            &["parameters", "sensing", "macro_field_height"][..],
            &["sensing", "local_macro_view", "field_height_mm"][..],
            "sensing.macro_field_height_mm",
        ),
        (
            &["parameters", "sensing", "macro_pixel_scale"][..],
            &["sensing", "local_macro_view", "pixel_scale_mm"][..],
            "sensing.macro_pixel_scale_mm",
        ),
        (
            &["parameters", "sensing", "macro_mount_arm_index"][..],
            &["sensing", "local_macro_view", "mount_arm_index"][..],
            "sensing.macro_mount_arm_index",
        ),
        (
            &["parameters", "sensing", "macro_mount_normal_offset"][..],
            &["sensing", "local_macro_view", "mount_normal_offset_mm"][..],
            "sensing.macro_mount_normal_offset_mm",
        ),
        (
            &["parameters", "sensing", "depth_quantization"][..],
            &["sensing", "depth_quantization_mm"][..],
            "sensing.depth_quantization_mm",
        ),
        (
            &["parameters", "sensing", "pixel_sigma_px"][..],
            &["sensing", "pixel_sigma_px"][..],
            "sensing.pixel_sigma_px",
        ),
        (
            &["parameters", "sensing", "dropout_probability"][..],
            &["sensing", "dropout_probability"][..],
            "sensing.dropout_probability",
        ),
        (
            &["parameters", "gearbox", "module"][..],
            &["gearbox", "module_mm"][..],
            "gearbox.module_mm",
        ),
        (
            &["parameters", "gearbox", "pressure_angle_deg"][..],
            &["gearbox", "pressure_angle_deg"][..],
            "gearbox.pressure_angle_deg",
        ),
        (
            &["parameters", "gearbox", "backlash"][..],
            &["gearbox", "nominal_backlash_mm"][..],
            "gearbox.nominal_backlash_mm",
        ),
        (
            &["parameters", "gearbox", "gear_thickness"][..],
            &["gearbox", "face_width_mm"][..],
            "gearbox.face_width_mm",
        ),
        (
            &["parameters", "gearbox", "total_gear_height"][..],
            &["gearbox", "gear_total_height_mm"][..],
            "gearbox.gear_total_height_mm",
        ),
        (
            &["parameters", "gearbox", "bore_diameter"][..],
            &["gearbox", "gear_bore_nominal_mm"][..],
            "gearbox.gear_bore_nominal_mm",
        ),
        (
            &["parameters", "gearbox", "shaft_diameter"][..],
            &["gearbox", "shaft_diameter_mm"][..],
            "gearbox.shaft_diameter_mm",
        ),
        (
            &["parameters", "gearbox", "shaft_length"][..],
            &["gearbox", "shaft_length_mm"][..],
            "gearbox.shaft_length_mm",
        ),
        (
            &["parameters", "gearbox", "housing_height"][..],
            &["gearbox", "housing_body_height_mm"][..],
            "gearbox.housing_body_height_mm",
        ),
        (
            &["parameters", "gearbox", "housing_wall"][..],
            &["gearbox", "housing_wall_mm"][..],
            "gearbox.housing_wall_mm",
        ),
        (
            &["parameters", "gearbox", "lid_thickness"][..],
            &["gearbox", "cover_thickness_mm"][..],
            "gearbox.cover_thickness_mm",
        ),
    ] {
        compare(
            label,
            cad_number(cad_path)?,
            scenario_number(scenario_path)?,
        )?;
    }
    require_number_array(
        metadata,
        &["parameters", "sensing", "global_camera_end_offsets"],
        &[-106.0, 106.0],
        metadata_origin,
    )?;
    require_nested_number_arrays(
        metadata,
        &["parameters", "sensing", "global_camera_triplet_azimuths"],
        &[&[0.0, 120.0, 240.0], &[60.0, 180.0, 300.0]],
        metadata_origin,
    )?;

    let housing =
        required_number_array_value(scenario, &["gearbox", "housing_outer_mm"], scenario_origin)?;
    if housing.len() != 3 {
        return Err(format!(
            "invalid scenario JSON in {scenario_origin}: gearbox.housing_outer_mm must contain 3 numbers"
        ));
    }
    let housing_length = cad_number(&["parameters", "gearbox", "housing_length"])?;
    let housing_width = cad_number(&["parameters", "gearbox", "housing_width"])?;
    let housing_height = cad_number(&["parameters", "gearbox", "housing_height"])?;
    let lid_thickness = cad_number(&["parameters", "gearbox", "lid_thickness"])?;
    for (index, (actual, expected)) in [
        (housing_length, housing[0]),
        (housing_width, housing[1]),
        (housing_height + lid_thickness, housing[2]),
    ]
    .into_iter()
    .enumerate()
    {
        compare(
            &format!("gearbox.housing_outer_mm[{index}]"),
            actual,
            expected,
        )?;
    }

    let input_teeth = cad_number(&["parameters", "gearbox", "input_teeth"])?;
    let idler_teeth = cad_number(&["parameters", "gearbox", "idler_teeth"])?;
    let output_teeth = cad_number(&["parameters", "gearbox", "output_teeth"])?;
    let scenario_gears = scenario
        .pointer("/gearbox/gears")
        .and_then(Value::as_array)
        .expect("scenario gears validated before CAD metadata");
    for (index, actual) in [input_teeth, idler_teeth, output_teeth]
        .into_iter()
        .enumerate()
    {
        let expected = scenario_gears[index]["teeth"]
            .as_f64()
            .expect("scenario gear teeth validated before CAD metadata");
        compare(&format!("gearbox.gears[{index}].teeth"), actual, expected)?;
    }

    let module = cad_number(&["parameters", "gearbox", "module"])?;
    let input_x = cad_number(&["parameters", "gearbox", "input_center_x"])?;
    let center_y = cad_number(&["parameters", "gearbox", "center_y"])?;
    let idler_x = input_x + module * (input_teeth + idler_teeth) * 0.5;
    let output_x = idler_x + module * (idler_teeth + output_teeth) * 0.5;
    let scenario_centers = scenario
        .pointer("/gearbox/shaft_centers_mm")
        .and_then(Value::as_array)
        .ok_or_else(|| {
            format!("invalid scenario JSON in {scenario_origin}: gearbox.shaft_centers_mm must be an array")
        })?;
    if scenario_centers.len() != 3 {
        return Err(format!(
            "invalid scenario JSON in {scenario_origin}: gearbox.shaft_centers_mm must contain 3 centers"
        ));
    }
    for (index, (actual_x, center)) in [input_x, idler_x, output_x]
        .into_iter()
        .zip(scenario_centers)
        .enumerate()
    {
        let center = center.as_array().ok_or_else(|| {
            format!("invalid scenario JSON in {scenario_origin}: gearbox.shaft_centers_mm[{index}] must be [x,y]")
        })?;
        if center.len() != 2 {
            return Err(format!("invalid scenario JSON in {scenario_origin}: gearbox.shaft_centers_mm[{index}] must be [x,y]"));
        }
        compare(
            &format!("gearbox.shaft_centers_mm[{index}].x"),
            actual_x,
            center[0]
                .as_f64()
                .ok_or_else(|| format!("invalid numeric shaft center in {scenario_origin}"))?,
        )?;
        compare(
            &format!("gearbox.shaft_centers_mm[{index}].y"),
            center_y,
            center[1]
                .as_f64()
                .ok_or_else(|| format!("invalid numeric shaft center in {scenario_origin}"))?,
        )?;
    }

    let housing_floor = cad_number(&["parameters", "gearbox", "housing_floor"])?;
    let seated_center =
        housing_floor + cad_number(&["parameters", "gearbox", "total_gear_height"])? * 0.5;
    compare(
        "gearbox.shaft_seated_center_z_mm",
        cad_number(&["parameters", "gearbox", "shaft_length"])? * 0.5,
        scenario_number(&["gearbox", "shaft_seated_center_z_mm"])?,
    )?;
    compare(
        "gearbox.cover_seated_center_z_mm",
        housing_height + lid_thickness * 0.5,
        scenario_number(&["gearbox", "cover_seated_center_z_mm"])?,
    )?;
    for (index, gear) in scenario_gears.iter().enumerate() {
        compare(
            &format!("gearbox.gears[{index}].seated_bottom_z_mm"),
            housing_floor,
            gear["seated_bottom_z_mm"]
                .as_f64()
                .expect("validated gear bottom z"),
        )?;
        compare(
            &format!("gearbox.gears[{index}].seated_center_z_mm"),
            seated_center,
            gear["seated_center_z_mm"]
                .as_f64()
                .expect("validated gear center z"),
        )?;
    }
    Ok(())
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn canonical_geometry_facts(records: &[Value], origin: &str) -> Result<Value, String> {
    const FIELDS: [&str; 5] = ["name", "bbox_mm", "volume_mm3", "valid_brep", "solid_count"];
    let facts = records
        .iter()
        .enumerate()
        .map(|(index, record)| {
            let mut fact = serde_json::Map::new();
            for field in FIELDS {
                let value = record.get(field).ok_or_else(|| {
                    format!("invalid CAD metadata in {origin}: records[{index}] is missing {field}")
                })?;
                fact.insert(field.to_owned(), value.clone());
            }
            Ok(Value::Object(fact))
        })
        .collect::<Result<Vec<_>, String>>()?;
    Ok(Value::Array(facts))
}

fn nested_number(value: &Value, path: &[&str]) -> Option<f64> {
    path.iter()
        .try_fold(value, |current, key| current.get(*key))
        .and_then(Value::as_f64)
}

fn nested_string<'a>(value: &'a Value, path: &[&str]) -> Option<&'a str> {
    path.iter()
        .try_fold(value, |current, key| current.get(*key))
        .and_then(Value::as_str)
}

fn nested_value<'a>(value: &'a Value, path: &[&str]) -> Option<&'a Value> {
    path.iter()
        .try_fold(value, |current, key| current.get(*key))
}

fn required_number(value: &Value, path: &[&str], origin: &str) -> Result<f64, String> {
    nested_number(value, path)
        .ok_or_else(|| {
            format!(
                "invalid JSON in {origin}: {} must be a finite number",
                path.join(".")
            )
        })
        .and_then(|number| {
            if number.is_finite() {
                Ok(number)
            } else {
                Err(format!(
                    "invalid JSON in {origin}: {} must be finite",
                    path.join(".")
                ))
            }
        })
}

fn required_string<'a>(value: &'a Value, path: &[&str], origin: &str) -> Result<&'a str, String> {
    nested_string(value, path).ok_or_else(|| {
        format!(
            "invalid JSON in {origin}: {} must be a string",
            path.join(".")
        )
    })
}

fn required_number_array_value(
    value: &Value,
    path: &[&str],
    origin: &str,
) -> Result<Vec<f64>, String> {
    let array = nested_value(value, path)
        .and_then(Value::as_array)
        .ok_or_else(|| {
            format!(
                "invalid JSON in {origin}: {} must be an array of numbers",
                path.join(".")
            )
        })?;
    array
        .iter()
        .enumerate()
        .map(|(index, item)| {
            item.as_f64()
                .filter(|number| number.is_finite())
                .ok_or_else(|| {
                    format!(
                        "invalid JSON in {origin}: {}[{index}] must be a finite number",
                        path.join(".")
                    )
                })
        })
        .collect()
}

fn require_number_array(
    value: &Value,
    path: &[&str],
    expected: &[f64],
    origin: &str,
) -> Result<(), String> {
    let actual = required_number_array_value(value, path, origin)?;
    if actual.len() != expected.len() {
        return Err(format!(
            "unsupported {} length {} in {origin}; expected {}",
            path.join("."),
            actual.len(),
            expected.len()
        ));
    }
    for (index, (&actual, &expected)) in actual.iter().zip(expected).enumerate() {
        require_near(
            origin,
            &format!("{}[{index}]", path.join(".")),
            actual,
            expected,
        )?;
    }
    Ok(())
}

fn require_nested_number_arrays(
    value: &Value,
    path: &[&str],
    expected: &[&[f64]],
    origin: &str,
) -> Result<(), String> {
    let rows = nested_value(value, path)
        .and_then(Value::as_array)
        .ok_or_else(|| {
            format!(
                "invalid JSON in {origin}: {} must be an array of numeric arrays",
                path.join(".")
            )
        })?;
    if rows.len() != expected.len() {
        return Err(format!(
            "unsupported {} length {} in {origin}; expected {}",
            path.join("."),
            rows.len(),
            expected.len()
        ));
    }
    for (row_index, (row, expected_row)) in rows.iter().zip(expected).enumerate() {
        let row = row.as_array().ok_or_else(|| {
            format!(
                "invalid JSON in {origin}: {}[{row_index}] must be a numeric array",
                path.join(".")
            )
        })?;
        if row.len() != expected_row.len() {
            return Err(format!(
                "unsupported {}[{row_index}] length {} in {origin}; expected {}",
                path.join("."),
                row.len(),
                expected_row.len()
            ));
        }
        for (column, (actual, &expected)) in row.iter().zip(*expected_row).enumerate() {
            let actual = actual.as_f64().ok_or_else(|| {
                format!(
                    "invalid JSON in {origin}: {}[{row_index}][{column}] must be numeric",
                    path.join(".")
                )
            })?;
            require_near(
                origin,
                &format!("{}[{row_index}][{column}]", path.join(".")),
                actual,
                expected,
            )?;
        }
    }
    Ok(())
}

fn require_near(origin: &str, field: &str, actual: f64, expected: f64) -> Result<(), String> {
    if (actual - expected).abs() > 1.0e-12 {
        Err(format!(
            "unsupported {field}={actual} in {origin}; this build requires {expected}"
        ))
    } else {
        Ok(())
    }
}

fn require_manifest_near(
    metadata_origin: &str,
    scenario_origin: &str,
    field: &str,
    actual: f64,
    expected: f64,
) -> Result<(), String> {
    if (actual - expected).abs() > 1.0e-9 {
        Err(format!(
            "CAD/scenario dimension mismatch for {field}: {metadata_origin} has {actual} mm; {scenario_origin} requires {expected} mm"
        ))
    } else {
        Ok(())
    }
}

fn parse_seed(value: &Value) -> Option<u64> {
    if let Some(number) = value.as_u64() {
        return Some(number);
    }
    let text = value.as_str()?;
    let digits = text
        .strip_prefix("0x")
        .or_else(|| text.strip_prefix("0X"))?;
    u64::from_str_radix(digits, 16).ok()
}

fn run() -> Result<bool, String> {
    let options = parse_args(env::args().skip(1))?;
    let spec = resolve_scenario(&options.scenario)?;
    let mut simulator = ReferenceSimulator::new(spec).map_err(|error| error.to_string())?;
    let report = simulator
        .run_to_completion(options.max_cycles)
        .map_err(|error| error.to_string())?;
    let json = report
        .to_json(options.pretty)
        .map_err(|error| error.to_string())?;
    match options.report.as_deref() {
        Some(path) if path.as_os_str() == "-" => println!("{json}"),
        Some(path) => {
            if let Some(parent) = path
                .parent()
                .filter(|parent| !parent.as_os_str().is_empty())
            {
                fs::create_dir_all(parent).map_err(|error| {
                    format!(
                        "failed to create report directory {}: {error}",
                        parent.display()
                    )
                })?;
            }
            fs::write(path, json)
                .map_err(|error| format!("failed to write report {}: {error}", path.display()))?;
        }
        None => {}
    }
    eprintln!(
        "scenario={} status={} cycles={} physics_s={:.3} components={}/{} retries={} failures={}",
        report.scenario,
        report.status,
        report.control_cycles,
        report.physics_time_s,
        report.metrics.components_completed,
        report.components.len(),
        report.metrics.retries,
        report.failures.len(),
    );
    Ok(report.completed)
}

fn main() {
    match run() {
        Ok(true) => {}
        Ok(false) => std::process::exit(2),
        Err(error) => {
            eprintln!("{error}");
            std::process::exit(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_required_headless_flags() {
        let options = parse_args(
            [
                "--scenario",
                "collision",
                "--report",
                "report.json",
                "--max-cycles",
                "9000",
                "--compact",
            ]
            .into_iter()
            .map(str::to_owned),
        )
        .unwrap();
        assert_eq!(options.scenario, "collision");
        assert_eq!(options.report, Some(PathBuf::from("report.json")));
        assert_eq!(options.max_cycles, 9_000);
        assert!(!options.pretty);
    }

    #[test]
    fn rejects_unknown_scenario() {
        assert!(parse_args(["--scenario".to_owned(), "laser-dragons".to_owned()]).is_err());
    }

    fn valid_manifest() -> Value {
        let mut manifest = serde_json::json!({
            "schema": "pipe-cad/0.1",
            "assembly": "gearbox",
            "units": "mm",
            "parameters": {
                "tube": {"inner_radius": 80.0, "length": 330.0, "end_margin": 5.0, "rail_count": 4},
                "arm": {
                    "link_lengths": [32.0, 30.0, 15.0],
                    "spool_radius": 3.0,
                    "tendon_offset": 1.65,
                    "usable_tendon_payout": 12.0
                },
                "gripper": {"jaw_opening": 2.8},
                "sensing": {
                    "global_camera_count": 6,
                    "simultaneous_macro_view_count": 2,
                    "global_image_width_px": 1280,
                    "global_image_height_px": 800,
                    "global_horizontal_fov_deg": 68.0,
                    "global_camera_front_radius": 60.0,
                    "global_camera_end_offsets": [-106.0, 106.0],
                    "global_camera_triplet_azimuths": [
                        [0.0, 120.0, 240.0],
                        [60.0, 180.0, 300.0]
                    ],
                    "projector_front_radius": 60.0,
                    "projector_azimuth_deg": 90.0,
                    "projector_z_offset": 0.0,
                    "macro_stereo_baseline": 12.0,
                    "macro_mount_arm_index": 1,
                    "macro_mount_normal_offset": 11.0,
                    "macro_image_width_px": 2048,
                    "macro_image_height_px": 1536,
                    "macro_field_width": 4.0,
                    "macro_field_height": 3.0,
                    "macro_pixel_scale": 0.002,
                    "depth_quantization": 0.00025,
                    "pixel_sigma_px": 0.18,
                    "dropout_probability": 0.002
                },
                "gearbox": {
                    "module": 0.10,
                    "pressure_angle_deg": 25.0,
                    "backlash": 0.020,
                    "input_teeth": 12,
                    "idler_teeth": 18,
                    "output_teeth": 24,
                    "gear_thickness": 0.35,
                    "total_gear_height": 1.30,
                    "bore_diameter": 0.420,
                    "shaft_diameter": 0.35,
                    "shaft_length": 1.55,
                    "housing_length": 6.0,
                    "housing_width": 4.0,
                    "housing_height": 1.60,
                    "housing_wall": 0.030,
                    "housing_floor": 0.25,
                    "lid_thickness": 0.20,
                    "input_center_x": 0.75,
                    "center_y": 2.0
                }
            },
            "gearbox_assembly_sequence": ["S1", "S2", "S3", "G3", "G2", "G1", "cover"],
            "records": [
                {"name": "gearbox_housing", "valid_brep": true, "solid_count": 1},
                {"name": "S1", "valid_brep": true, "solid_count": 1},
                {"name": "S2", "valid_brep": true, "solid_count": 1},
                {"name": "S3", "valid_brep": true, "solid_count": 1},
                {"name": "G3", "valid_brep": true, "solid_count": 1},
                {"name": "G2", "valid_brep": true, "solid_count": 1},
                {"name": "G1", "valid_brep": true, "solid_count": 1},
                {"name": "cover", "valid_brep": true, "solid_count": 1}
            ]
        });
        for (index, record) in manifest["records"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .enumerate()
        {
            record["bbox_mm"] = serde_json::json!({
                "min": [index as f64, 0.0, 0.0],
                "max": [index as f64 + 1.0, 1.0, 1.0],
                "size": [1.0, 1.0, 1.0]
            });
            record["volume_mm3"] = serde_json::json!(1.0);
        }
        manifest
    }

    fn write_fixture(label: &str, mutate: impl FnOnce(&mut Value)) -> (PathBuf, PathBuf, String) {
        let directory =
            std::env::temp_dir().join(format!("pipe_sim_cli_{label}_{}", std::process::id()));
        if directory.exists() {
            fs::remove_dir_all(&directory).unwrap();
        }
        fs::create_dir_all(&directory).unwrap();
        let mut manifest = valid_manifest();
        mutate(&mut manifest);
        let canonical = serde_json::to_vec(&manifest["parameters"]).unwrap();
        let parameter_hash = sha256_hex(&canonical);
        manifest["parameter_sha256"] = Value::String(parameter_hash.clone());
        let geometry_facts =
            canonical_geometry_facts(manifest["records"].as_array().unwrap(), "test manifest")
                .unwrap();
        manifest["geometry_facts_sha256"] =
            Value::String(sha256_hex(&serde_json::to_vec(&geometry_facts).unwrap()));
        let manifest_path = directory.join("gearbox.metadata.json");
        fs::write(
            &manifest_path,
            serde_json::to_string_pretty(&manifest).unwrap(),
        )
        .unwrap();
        let scenario_path = directory.join("gearbox_acceptance.json");
        let scenario = include_str!("../../../scenarios/gearbox_acceptance.json").replace(
            "../cad/baseline/gearbox.metadata.json",
            "gearbox.metadata.json",
        );
        fs::write(&scenario_path, scenario).unwrap();
        (directory, scenario_path, parameter_hash)
    }

    #[test]
    fn resolves_relative_validated_cad_manifest_and_exposes_hash() {
        let (directory, scenario_path, parameter_hash) = write_fixture("valid", |_| {});
        let spec = resolve_scenario(scenario_path.to_str().unwrap()).unwrap();
        assert_eq!(spec.name, "gearbox_baseline_v1");
        assert_eq!(spec.seed, 0x5049_5045_5F47_4258);
        assert_eq!(
            spec.cad_parameter_sha256.as_deref(),
            Some(parameter_hash.as_str())
        );
        let manifest: Value = serde_json::from_str(
            &fs::read_to_string(directory.join("gearbox.metadata.json")).unwrap(),
        )
        .unwrap();
        assert_eq!(
            spec.cad_geometry_facts_sha256.as_deref(),
            manifest["geometry_facts_sha256"].as_str()
        );
        let scenario: Value =
            serde_json::from_str(&fs::read_to_string(&scenario_path).unwrap()).unwrap();
        let expected_scenario_hash = sha256_hex(&serde_json::to_vec(&scenario).unwrap());
        assert_eq!(
            spec.scenario_sha256.as_deref(),
            Some(expected_scenario_hash.as_str())
        );
        assert_ne!(spec.configuration_sha256, parameter_hash);
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn checked_in_scenario_resolves_against_canonical_manifest() {
        let scenario_path =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/gearbox_acceptance.json");
        let spec = resolve_scenario(scenario_path.to_str().unwrap()).unwrap();
        assert_eq!(
            spec.cad_parameter_sha256.as_deref(),
            Some("23e1fbcbdb795ad262ae09f9da75272a0955566c41d4ed8d16d399cbdfeab40c")
        );
        assert_eq!(
            spec.cad_geometry_facts_sha256.as_deref(),
            Some("44c1a7054ab49b4421c69557aab21dded9aa0543d378b13ec8a7ff79cf18632d")
        );
        assert_eq!(spec.scenario_sha256.as_deref().map(str::len), Some(64));
        assert_eq!(spec.configuration_sha256.len(), 64);
    }

    #[test]
    fn rejects_manifest_with_invalid_brep() {
        let (directory, scenario_path, _) = write_fixture("invalid_brep", |manifest| {
            manifest["records"][4]["valid_brep"] = Value::Bool(false);
        });
        let error = resolve_scenario(scenario_path.to_str().unwrap()).unwrap_err();
        assert!(error.contains("invalid BREP"));
        assert!(error.contains("G3"));
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn rejects_manifest_dimension_drift() {
        let (directory, scenario_path, _) = write_fixture("dimension_drift", |manifest| {
            manifest["parameters"]["gearbox"]["module"] = serde_json::json!(0.11);
        });
        let error = resolve_scenario(scenario_path.to_str().unwrap()).unwrap_err();
        assert!(error.contains("CAD/scenario dimension mismatch"));
        assert!(error.contains("gearbox.module_mm"));
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn rejects_manifest_sensor_extrinsic_drift() {
        let (directory, scenario_path, _) = write_fixture("sensor_extrinsic_drift", |manifest| {
            manifest["parameters"]["sensing"]["projector_front_radius"] = serde_json::json!(61.0);
        });
        let error = resolve_scenario(scenario_path.to_str().unwrap()).unwrap_err();
        assert!(error.contains("CAD/scenario dimension mismatch"));
        assert!(error.contains("sensing.projector_front_radius"));
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn rejects_manifest_insertion_sequence_drift() {
        let (directory, scenario_path, _) = write_fixture("sequence_drift", |manifest| {
            manifest["gearbox_assembly_sequence"] =
                serde_json::json!(["S1", "S2", "S3", "G1", "G2", "G3", "cover"]);
        });
        let error = resolve_scenario(scenario_path.to_str().unwrap()).unwrap_err();
        assert!(error.contains("CAD insertion sequence mismatch"));
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn rejects_manifest_parameter_hash_tampering() {
        let (directory, scenario_path, _) = write_fixture("hash_tampering", |_| {});
        let manifest_path = directory.join("gearbox.metadata.json");
        let mut manifest: Value =
            serde_json::from_str(&fs::read_to_string(&manifest_path).unwrap()).unwrap();
        manifest["parameter_sha256"] = Value::String("0".repeat(64));
        fs::write(
            &manifest_path,
            serde_json::to_string_pretty(&manifest).unwrap(),
        )
        .unwrap();
        let error = resolve_scenario(scenario_path.to_str().unwrap()).unwrap_err();
        assert!(error.contains("parameter_sha256 mismatch"));
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn rejects_json_with_incompatible_fixed_step() {
        let error = parse_scenario_document(
            r#"{
                "schema_version": 1,
                "name": "gearbox_baseline_v1",
                "seed": "0x504950455F474258",
                "simulation": { "fixed_step_s": 0.01 }
            }"#,
            Path::new("inline-test.json"),
        )
        .unwrap_err();
        assert!(error.contains("simulation.fixed_step_s"));
        assert!(error.contains("requires 0.001"));
    }

    #[test]
    fn rejects_json_with_incompatible_minimum_view_count() {
        let mut document: Value =
            serde_json::from_str(include_str!("../../../scenarios/gearbox_acceptance.json"))
                .unwrap();
        document["sensing"]["minimum_views"] = serde_json::json!(1);
        let error = parse_scenario_document(
            &serde_json::to_string(&document).unwrap(),
            Path::new("inline-test.json"),
        )
        .unwrap_err();
        assert!(error.contains("sensing.minimum_views"));
        assert!(error.contains("requires 2"));
    }

    #[test]
    fn rejects_drift_in_nonruntime_scenario_claims() {
        let mut document: Value =
            serde_json::from_str(include_str!("../../../scenarios/gearbox_acceptance.json"))
                .unwrap();
        document["simulation"]["gravity_m_s2"] = serde_json::json!([0.0, 0.0, 0.0]);
        let error = parse_scenario_document(
            &serde_json::to_string(&document).unwrap(),
            Path::new("inline-test.json"),
        )
        .unwrap_err();
        assert!(error.contains("compiled v1 runtime accepts only"));
    }
}
