//! Articulated design-state experiment, not an executed or completed assembly.
//! The machine's own FK supplies link/jaw occlusion at every candidate state.
use crate::{
    machine_config,
    metrology::{machine_optical_scene, optical_pose, MachineMetrology},
    sha256_hex, SimError,
};
use pipe_optics::{
    metrology::{synthetic::*, *},
    *,
};
use pipe_sim_core::{ManipulatorMotionState, SerialArmInstance};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GearStudyConfig {
    pub repeats: u32,
    pub pick_world_m: Vec3,
    pub insertion_world_m: Vec3,
    pub transfer_world_m: Vec3,
    pub approach_distance_m: f64,
    pub gear_radius_m: f64,
    pub gear_half_thickness_m: f64,
    pub bore_radius_m: f64,
    pub shaft_radius_m: f64,
    pub shaft_length_m: f64,
    pub marker_offset_candidates_m: Vec<f64>,
    pub camera_axial_candidates_m: Vec<f64>,
    pub projector_angle_candidates_rad: Vec<f64>,
    pub relation_rms_candidates_m: Vec<f64>,
    pub relation_axis_rms_rad: f64,
    pub maximum_alignment_rms_m: f64,
    pub seed: u64,
}
impl Default for GearStudyConfig {
    fn default() -> Self {
        Self {
            repeats: 4,
            pick_world_m: Vec3::new(-0.004, 0.004, 0.),
            insertion_world_m: Vec3::new(0.004, 0., 0.),
            transfer_world_m: Vec3::new(0., 0.004, 0.003),
            approach_distance_m: 0.003,
            gear_radius_m: 0.001,
            gear_half_thickness_m: 0.0004,
            bore_radius_m: 0.00026,
            shaft_radius_m: 0.00025,
            shaft_length_m: 0.002,
            marker_offset_candidates_m: vec![0.003, 0.006],
            camera_axial_candidates_m: vec![0.010, 0.025],
            projector_angle_candidates_rad: vec![
                -std::f64::consts::FRAC_PI_4,
                3. * std::f64::consts::FRAC_PI_4,
            ],
            relation_rms_candidates_m: vec![1e-6, 8e-6],
            relation_axis_rms_rad: 0.0002,
            maximum_alignment_rms_m: 5e-6,
            seed: 73,
        }
    }
}
#[derive(Clone, Debug, Serialize)]
pub struct ModeResult {
    pub estimate: Option<ObservedPoint>,
    pub rejection: Option<String>,
    pub visibility: Vec<VisibilityAttempt>,
}
#[derive(Clone, Debug, Serialize)]
pub struct GearSample {
    pub state: String,
    pub truth_world_from_tool: RigidTransform,
    pub truth_world_from_part: RigidTransform,
    pub truth_world_from_shaft: RigidTransform,
    pub repeat: u32,
    pub design_state_index: usize,
    pub synthetic_acquisition_start_s: f64,
    pub tool_pose: Option<ObservedToolPose>,
    pub marker_pose: Option<ObservedToolPose>,
    pub inferred_bore: Option<InferredPartFeature>,
    pub shaft_datum: Option<InferredPartFeature>,
    pub tool_rejection: Option<String>,
    pub marker_rejection: Option<String>,
    pub shaft_rejection: Option<String>,
    pub tool_visibility: Vec<VisibilityAttempt>,
    pub marker_visibility: Vec<VisibilityAttempt>,
    pub health: MetrologyHealth,
    pub direct_transparent_passive: ModeResult,
    pub direct_transparent_structured: ModeResult,
    pub opaque_witness_structured: ModeResult,
    pub bore_error_against_independent_truth_m: Option<f64>,
    pub bore_depth_error_world_z_m: Option<f64>,
    pub bore_axis_error_rad: Option<f64>,
    pub alignment_error_against_independent_truth_m: Option<f64>,
    /// Sum of marginal RMS bounds. Valid without asserting independence or
    /// cancellation of shared calibration errors. Not an exact covariance.
    pub alignment_rms_upper_bound_m: Option<f64>,
    pub precision_rejections: Vec<String>,
    pub needs_stop_or_reposition: bool,
    pub occluder_count: usize,
    pub static_contact_pairs: Vec<(u32, u32, f64)>,
}
#[derive(Clone, Debug, Serialize)]
pub struct CandidateResult {
    pub camera_axial_m: f64,
    pub projector_angle_rad: f64,
    pub marker_offset_m: f64,
    pub relation_rms_m: f64,
    pub optical_config: MetrologyConfig,
    pub config_sha256: String,
    pub rejected_design_states: Vec<String>,
    pub samples: Vec<GearSample>,
    pub state_summaries: Vec<StateSummary>,
    pub stop_or_reposition_count: usize,
    pub accepted_alignment_count: usize,
    pub accepted_alignment_3d_rms_m: Option<f64>,
    pub available_estimate_alignment_3d_rms_m: Option<f64>,
    pub all_states_optically_supported: bool,
    pub static_states_with_contact: usize,
}
#[derive(Clone, Debug, Serialize)]
pub struct GearStudyReport {
    pub schema_version: u32,
    pub source_revision: String,
    pub machine_config_sha256: String,
    pub config_sha256: String,
    pub configuration: GearStudyConfig,
    pub candidates: Vec<CandidateResult>,
    pub execution_probe: ExecutionProbe,
    pub fidelity: Vec<String>,
}
fn err(e: impl std::fmt::Debug) -> SimError {
    SimError::InvalidScenario(format!("gear observability: {e:?}"))
}
fn result(a: Result<SyntheticAcquisition, MetrologyError>, c: &MetrologyConfig) -> ModeResult {
    match a {
        Ok(a) => match reconstruct_point(c, &a.observations) {
            Ok(p) => ModeResult {
                estimate: Some(p),
                rejection: None,
                visibility: a.visibility,
            },
            Err(e) => ModeResult {
                estimate: None,
                rejection: Some(e.to_string()),
                visibility: a.visibility,
            },
        },
        Err(e) => ModeResult {
            estimate: None,
            rejection: Some(e.to_string()),
            visibility: vec![],
        },
    }
}
fn structured(
    c: &MetrologyConfig,
    s: &Scene,
    f: &TruthFeature,
    t: &AcquisitionTiming,
) -> ModeResult {
    let projector = c
        .devices
        .iter()
        .find(|d| d.kind == DeviceKind::Projector)
        .unwrap()
        .id;
    let mut a = SyntheticAcquisition::default();
    let mut reasons = BTreeMap::new();
    for d in c.devices.iter().filter(|d| d.kind == DeviceKind::Camera) {
        match acquire_structured(c, s, f, d.id, projector, t) {
            Ok(v) => {
                a.observations.extend(v.observations);
                a.visibility.extend(v.visibility);
            }
            Err(e) => {
                *reasons.entry(e.to_string()).or_insert(0) += 1;
                a.visibility.push(VisibilityAttempt {
                    object_id: f.object_id,
                    feature_id: f.feature_id,
                    sensor_id: d.id,
                    rejection: Some(e),
                });
            }
        }
    }
    let mut r = result(Ok(a), c);
    if r.estimate.is_none() && !reasons.is_empty() {
        r.rejection = Some(format!("{:?}", reasons));
    }
    r
}
/// Faceted annulus: a real aperture, unlike a solid cylinder hiding the bore.
/// Transparent bulk is treated conservatively as an opaque occluder; no ray
/// through it is accepted as an undistorted direct view.
fn annulus(s: &mut Scene, p: RigidTransform, c: &GearStudyConfig) {
    for i in 0..48 {
        let a = i as f64 * std::f64::consts::TAU / 48.;
        let b = (i + 1) as f64 * std::f64::consts::TAU / 48.;
        let v = |r: f64, z: f64, t: f64| p.transform_point(Vec3::new(r * t.cos(), r * t.sin(), z));
        let h = c.gear_half_thickness_m;
        let quads = [
            [
                v(c.bore_radius_m, h, a),
                v(c.gear_radius_m, h, a),
                v(c.gear_radius_m, h, b),
                v(c.bore_radius_m, h, b),
            ],
            [
                v(c.bore_radius_m, -h, a),
                v(c.gear_radius_m, -h, a),
                v(c.gear_radius_m, -h, b),
                v(c.bore_radius_m, -h, b),
            ],
            [
                v(c.gear_radius_m, -h, a),
                v(c.gear_radius_m, h, a),
                v(c.gear_radius_m, h, b),
                v(c.gear_radius_m, -h, b),
            ],
            [
                v(c.bore_radius_m, -h, a),
                v(c.bore_radius_m, h, a),
                v(c.bore_radius_m, h, b),
                v(c.bore_radius_m, -h, b),
            ],
        ];
        for q in quads {
            for [a, b, c] in [[0, 1, 2], [0, 2, 3]] {
                s.push(Primitive::new(
                    Geometry::Triangle(Triangle {
                        a: q[a],
                        b: q[b],
                        c: q[c],
                        double_sided: true,
                    }),
                    Material::default(),
                    700,
                ));
            }
        }
    }
}
fn rms(values: impl Iterator<Item = f64>) -> Option<f64> {
    let v: Vec<_> = values.collect();
    (!v.is_empty()).then(|| (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt())
}

pub fn run_gear_study(c: GearStudyConfig, revision: &str) -> Result<GearStudyReport, SimError> {
    if revision.len() != 40
        || !revision.bytes().all(|b| b.is_ascii_hexdigit())
        || ![c.pick_world_m, c.insertion_world_m, c.transfer_world_m]
            .iter()
            .all(|v| v.is_finite())
        || !c.approach_distance_m.is_finite()
        || c.approach_distance_m <= 0.
        || c.repeats == 0
        || c.repeats > 100
        || c.marker_offset_candidates_m.is_empty()
        || c.camera_axial_candidates_m.is_empty()
        || c.projector_angle_candidates_rad.is_empty()
        || !c
            .projector_angle_candidates_rad
            .iter()
            .all(|v| v.is_finite())
        || c.relation_rms_candidates_m.is_empty()
        || c.marker_offset_candidates_m.len()
            * c.camera_axial_candidates_m.len()
            * c.relation_rms_candidates_m.len()
            * c.projector_angle_candidates_rad.len()
            > 64
        || ![
            c.gear_radius_m,
            c.gear_half_thickness_m,
            c.bore_radius_m,
            c.shaft_radius_m,
            c.shaft_length_m,
            c.relation_axis_rms_rad,
            c.maximum_alignment_rms_m,
        ]
        .iter()
        .chain(c.marker_offset_candidates_m.iter())
        .chain(c.camera_axial_candidates_m.iter())
        .chain(c.relation_rms_candidates_m.iter())
        .all(|x| x.is_finite() && *x > 0.)
        || c.shaft_length_m <= 2. * c.shaft_radius_m
        || c.shaft_radius_m >= c.bore_radius_m
        || c.bore_radius_m >= c.gear_radius_m
    {
        return Err(err("invalid study configuration"));
    }
    let loaded = machine_config::load_baseline_machine_config()?;
    let fixture_machine = machine_config::build_baseline_machine(&loaded)?;
    let fixed_pick_pose = datum_pose(&fixture_machine, c.pick_world_m)?;
    let fixed_shaft_pose = datum_pose(&fixture_machine, c.insertion_world_m)?;
    let execution_probe = execution_probe(&c)?;
    if 2. * c.gear_radius_m > loaded.cell.gripper.max_opening_m {
        return Err(err("gear exceeds gripper opening"));
    }
    let mut candidates = Vec::new();
    // These are design waypoints, not commanded/executed assembly progress.
    let states = [
        (
            "approach",
            c.pick_world_m + Vec3::Z * c.approach_distance_m,
            false,
        ),
        ("grasp", c.pick_world_m, true),
        ("reorient", c.transfer_world_m, true),
        ("precontact", c.insertion_world_m + Vec3::Z * 0.001, true),
        ("insertion_geometry", c.insertion_world_m, true),
    ];
    for &projector_angle in &c.projector_angle_candidates_rad {
        for &axial in &c.camera_axial_candidates_m {
            for &offset in &c.marker_offset_candidates_m {
                for &relation_rms in &c.relation_rms_candidates_m {
                    let mut config = MachineMetrology::for_machine(loaded.cell)
                        .map_err(err)?
                        .config;
                    config.seed = c.seed;
                    for d in config
                        .devices
                        .iter_mut()
                        .filter(|d| d.kind == DeviceKind::Camera)
                    {
                        let mut eye = d.model.center_world();
                        eye.z = axial * eye.z.signum();
                        let distance = (eye - config.precision_volume.center_world_m).norm();
                        let scale = distance / d.focus_working_distance_m;
                        d.model.world_from_camera =
                            look_at(eye, config.precision_volume.center_world_m);
                        d.model.intrinsics.fx_px *= scale;
                        d.model.intrinsics.fy_px *= scale;
                        d.effective_focal_length_m *= scale;
                        d.focus_working_distance_m = distance;
                        d.nominal_lens_focal_length_m =
                            1. / (1. / d.effective_focal_length_m + 1. / distance);
                    }
                    for d in config
                        .devices
                        .iter_mut()
                        .filter(|d| d.kind == DeviceKind::Projector)
                    {
                        let old = d.model.center_world();
                        let radius = old.x.hypot(old.y);
                        d.model.world_from_camera = look_at(
                            Vec3::new(
                                radius * projector_angle.cos(),
                                radius * projector_angle.sin(),
                                old.z,
                            ),
                            config.precision_volume.center_world_m,
                        );
                    }
                    config.validate().map_err(err)?;
                    let config_hash =
                        sha256_hex(serde_json::to_string(&config).map_err(err)?.as_bytes());
                    let mut samples = Vec::new();
                    let mut rejected = Vec::new();
                    for (index, (name, target, closed)) in states.iter().enumerate() {
                        let mut machine = machine_config::build_baseline_machine(&loaded)?;
                        let old = &machine.serial_arms[0];
                        let solution = match old.arm.solve_tool_position(
                            pipe_sim_core::Vec3::new(target.x, target.y, target.z),
                            old.arm.positions,
                        ) {
                            Ok(v) => v,
                            Err(e) => {
                                rejected.push(format!("{name}: {e:?}"));
                                continue;
                            }
                        };
                        let mut arm = old.arm.clone();
                        arm.set_positions(solution.positions).map_err(err)?;
                        let mut instance =
                            SerialArmInstance::new(old.id, arm, old.gripper_config).map_err(err)?;
                        instance.motion =
                            ManipulatorMotionState::from_positions(solution.positions);
                        if *closed {
                            instance.gripper.opening_m = 2. * c.gear_radius_m;
                            instance.gripper.command_opening_m = 2. * c.gear_radius_m;
                        }
                        machine.serial_arms[0] = instance;
                        let tool_truth = optical_pose(machine.serial_arms[0].tool_pose());
                        let gear_truth = if *closed {
                            tool_truth
                        } else {
                            optical_pose(fixed_pick_pose)
                        };
                        let shaft_truth = optical_pose(fixed_shaft_pose);
                        let mut scene = machine_optical_scene(&machine);
                        annulus(&mut scene, gear_truth, &c);
                        scene.push(Primitive::new(
                            Geometry::Cylinder(Cylinder {
                                start: shaft_truth.transform_point(Vec3::Z * (-c.shaft_radius_m)),
                                end: shaft_truth.transform_point(
                                    Vec3::Z * (-c.shaft_length_m + c.shaft_radius_m),
                                ),
                                radius_m: c.shaft_radius_m,
                                capped: true,
                            }),
                            Material::default(),
                            701,
                        ));
                        for z in [-c.shaft_radius_m, -c.shaft_length_m + c.shaft_radius_m] {
                            scene.push(Primitive::new(
                                Geometry::Sphere(Sphere {
                                    center: shaft_truth.transform_point(Vec3::Z * z),
                                    radius_m: c.shaft_radius_m,
                                }),
                                Material::default(),
                                701,
                            ));
                        }
                        // Static physical envelope checks are separate from optical
                        // validity; never certify a path from isolated endpoints.
                        use pipe_sim_core::{BodyId, GearGeometry, MotionType, RigidBody, Shape};
                        let core_gear = if *closed {
                            machine.serial_arms[0].tool_pose()
                        } else {
                            fixed_pick_pose
                        };
                        machine
                            .add_body(RigidBody::new(
                                BodyId(700),
                                Shape::Gear(GearGeometry::uniform_spur(
                                    18,
                                    c.gear_radius_m / 10.,
                                    20f64.to_radians(),
                                    2. * c.gear_half_thickness_m,
                                    c.bore_radius_m,
                                )),
                                core_gear,
                                MotionType::Static,
                            ))
                            .map_err(err)?;
                        machine
                            .add_body(RigidBody::new(
                                BodyId(701),
                                Shape::Capsule {
                                    radius_m: c.shaft_radius_m,
                                    half_segment_m: c.shaft_length_m / 2. - c.shaft_radius_m,
                                },
                                pipe_sim_core::Pose::new(
                                    fixed_shaft_pose.transform_point(pipe_sim_core::Vec3::new(
                                        0.,
                                        0.,
                                        -c.shaft_length_m / 2.,
                                    )),
                                    fixed_shaft_pose.rotation,
                                ),
                                MotionType::Static,
                            ))
                            .map_err(err)?;
                        let static_contacts: Vec<_> = machine
                            .query_collisions_with_arms(machine.config.collision)
                            .contacts
                            .iter()
                            .filter(|p| p.penetration_depth_m > 1e-9)
                            .map(|p| (p.body_a.0, p.body_b.0, p.penetration_depth_m))
                            .collect();
                        let tool = RigidFiducial::baseline(1);
                        let mut marker = RigidFiducial::baseline(700);
                        marker.arm_id = None;
                        marker.tool_type = "part_marker".into();
                        marker.tcp_from_fiducial =
                            RigidTransform::new(Mat3::IDENTITY, Vec3::new(0., offset, 0.));
                        let mut shaft_marker = marker.clone();
                        shaft_marker.object_id = 701;
                        shaft_marker.identity_code = "shaft-reference".into();
                        shaft_marker.tcp_from_fiducial.translation = Vec3::new(0., -offset, 0.);
                        let relation = PartFeatureRelation {
                            feature_id: 10,
                            center_fiducial_m: Vec3::new(0., -offset, 0.),
                            axis_fiducial: Vec3::Z,
                            calibration_id: "independent_part_characterization_candidate".into(),
                            provenance: RelationProvenance::SyntheticIndependentCharacterization,
                            characterization_rms_m: relation_rms,
                            axis_characterization_rms_rad: c.relation_axis_rms_rad,
                        };
                        let shaft_relation = PartFeatureRelation {
                            center_fiducial_m: Vec3::new(0., offset, 0.),
                            ..relation.clone()
                        };
                        for repeat in 0..c.repeats {
                            let t = timing(
                                &config,
                                (index as u64) * 1000 + repeat as u64 + 1,
                                1. + index as f64 + repeat as f64 * 0.1,
                                0.,
                            );
                            let ta = acquire_tool(
                                &config,
                                &scene,
                                &tool,
                                tool_truth,
                                MeasurementMode::Precision,
                                &t,
                            )
                            .map_err(err)?;
                            let ma = acquire_tool(
                                &config,
                                &scene,
                                &marker,
                                gear_truth,
                                MeasurementMode::Precision,
                                &t,
                            )
                            .map_err(err)?;
                            let sa = acquire_tool(
                                &config,
                                &scene,
                                &shaft_marker,
                                shaft_truth,
                                MeasurementMode::Precision,
                                &t,
                            )
                            .map_err(err)?;
                            let tp = estimate_tool_pose(&config, &tool, &ta.observations);
                            let mp = estimate_tool_pose(&config, &marker, &ma.observations);
                            let sp = estimate_tool_pose(&config, &shaft_marker, &sa.observations);
                            let inferred = mp
                                .as_ref()
                                .ok()
                                .and_then(|p| infer_part_feature(p, &relation).ok());
                            let shaft = sp
                                .as_ref()
                                .ok()
                                .and_then(|p| infer_part_feature(p, &shaft_relation).ok());
                            // Fixed independent manufacturing offsets: do not resample them
                            // between exposures or hide them in repeatability statistics.
                            let truth_bore = gear_truth.transform_point(Vec3::X * relation_rms);
                            let truth_shaft = shaft_truth.transform_point(Vec3::Y * relation_rms);
                            let truth_axis = gear_truth.transform_vector(
                                Mat3::from_axis_angle(Vec3::X * c.relation_axis_rms_rad) * Vec3::Z,
                            );
                            let refs = acquire_references(&config, &scene, &t).map_err(err)?;
                            let now = t.available_s + config.acquisition.max_trigger_skew_s;
                            let health = evaluate_health(&config, &refs, now);
                            let mut reasons = Vec::new();
                            if let (Ok(tool_pose), Some(bore), Some(shaft)) =
                                (&tp, &inferred, &shaft)
                            {
                                for (label, p) in [("bore", bore), ("shaft", shaft)] {
                                    if let Err(e) = require_inferred_precision(
                                        &config,
                                        &PrecisionContract::default(),
                                        tool_pose,
                                        p,
                                        &health,
                                        now,
                                    ) {
                                        reasons.push(format!("{label}: {e:?}"));
                                    }
                                }
                                if bore.center.predicted_3d_rms_m()
                                    + shaft.center.predicted_3d_rms_m()
                                    > c.maximum_alignment_rms_m
                                {
                                    reasons.push("relative_alignment_uncertainty".into());
                                }
                                if (bore.axis_rms_rad + shaft.axis_rms_rad)
                                    * c.gear_half_thickness_m
                                    > c.bore_radius_m - c.shaft_radius_m
                                {
                                    reasons.push("axis_uncertainty_exceeds_clearance".into());
                                }
                            } else {
                                reasons.push("missing_tool_part_or_shaft_observation".into());
                            }
                            let f = TruthFeature {
                                object_id: 700,
                                feature_id: 20,
                                point_world_m: gear_truth.transform_point(Vec3::new(
                                    c.gear_radius_m * 0.7,
                                    0.,
                                    c.gear_half_thickness_m,
                                )),
                                normal_world: Some(gear_truth.transform_vector(Vec3::Z)),
                                velocity_world_m_s: Vec3::ZERO,
                                surface: SurfaceResponse {
                                    translucent_fraction: 0.9,
                                    texture_contrast: 0.,
                                    ..Default::default()
                                },
                            };
                            let passive = result(
                                acquire_passive(
                                    &config,
                                    &scene,
                                    &f,
                                    MeasurementMode::Precision,
                                    MeasurementClass::CriticalFeature,
                                    &t,
                                ),
                                &config,
                            );
                            let sl = structured(&config, &scene, &f, &t);
                            let witness = TruthFeature {
                                surface: SurfaceResponse::default(),
                                ..f
                            };
                            let witness_sl = structured(&config, &scene, &witness, &t);
                            let alignment = inferred.as_ref().zip(shaft.as_ref()).map(|(b, s)| {
                                ((b.center.position_world_m - s.center.position_world_m)
                                    - (truth_bore - truth_shaft))
                                    .norm()
                            });
                            let bound = inferred.as_ref().zip(shaft.as_ref()).map(|(b, s)| {
                                b.center.predicted_3d_rms_m() + s.center.predicted_3d_rms_m()
                            });
                            samples.push(GearSample {
                                state: name.to_string(),
                                truth_world_from_tool: tool_truth,
                                truth_world_from_part: gear_truth,
                                truth_world_from_shaft: shaft_truth,
                                repeat,
                                design_state_index: index,
                                synthetic_acquisition_start_s: t.exposure_start_s,
                                bore_error_against_independent_truth_m: inferred
                                    .as_ref()
                                    .map(|p| (p.center.position_world_m - truth_bore).norm()),
                                bore_depth_error_world_z_m: inferred
                                    .as_ref()
                                    .map(|p| p.center.position_world_m.z - truth_bore.z),
                                bore_axis_error_rad: inferred
                                    .as_ref()
                                    .map(|p| p.axis_world.dot(truth_axis).clamp(-1., 1.).acos()),
                                alignment_error_against_independent_truth_m: alignment,
                                alignment_rms_upper_bound_m: bound,
                                tool_rejection: tp.as_ref().err().map(|e| e.to_string()),
                                marker_rejection: mp.as_ref().err().map(|e| e.to_string()),
                                shaft_rejection: sp.as_ref().err().map(|e| e.to_string()),
                                tool_pose: tp.ok(),
                                marker_pose: mp.ok(),
                                inferred_bore: inferred,
                                shaft_datum: shaft,
                                tool_visibility: ta.visibility,
                                marker_visibility: ma.visibility,
                                health,
                                direct_transparent_passive: passive,
                                direct_transparent_structured: sl,
                                opaque_witness_structured: witness_sl,
                                needs_stop_or_reposition: !reasons.is_empty(),
                                precision_rejections: reasons,
                                occluder_count: scene.primitives.len(),
                                static_contact_pairs: static_contacts.clone(),
                            });
                        }
                    }
                    let accepted = samples
                        .iter()
                        .filter(|s| !s.needs_stop_or_reposition)
                        .count();
                    candidates.push(CandidateResult {
                        camera_axial_m: axial,
                        projector_angle_rad: projector_angle,
                        marker_offset_m: offset,
                        relation_rms_m: relation_rms,
                        optical_config: config,
                        config_sha256: config_hash,
                        accepted_alignment_3d_rms_m: rms(samples
                            .iter()
                            .filter(|s| !s.needs_stop_or_reposition)
                            .filter_map(|s| s.alignment_error_against_independent_truth_m)),
                        available_estimate_alignment_3d_rms_m: rms(samples
                            .iter()
                            .filter_map(|s| s.alignment_error_against_independent_truth_m)),
                        all_states_optically_supported: rejected.is_empty()
                            && accepted == states.len() * c.repeats as usize,
                        stop_or_reposition_count: samples.len() - accepted,
                        accepted_alignment_count: accepted,
                        static_states_with_contact: samples
                            .iter()
                            .filter(|s| !s.static_contact_pairs.is_empty())
                            .map(|s| s.design_state_index)
                            .collect::<std::collections::BTreeSet<_>>()
                            .len(),
                        state_summaries: summarize_states(&samples),
                        samples,
                        rejected_design_states: rejected,
                    });
                }
            }
        }
    }
    Ok(GearStudyReport{schema_version:1,source_revision:revision.into(),machine_config_sha256:loaded.source_sha256,
        config_sha256:sha256_hex(serde_json::to_string(&c).map_err(err)?.as_bytes()),configuration:c,candidates,execution_probe,
        fidelity:["Articulated static design states from authoritative runtime FK, not executed motion, grasp, contact, or assembly completion. Acquisition times are synthetic stop-and-measure exposures, not machine elapsed time.",
        "Legacy machine is 160 mm ID. Camera candidates adapt explicitly to that geometry. Other arms remain at runtime home poses.",
        "2 mm gear is an annular envelope without teeth; 0.52 mm bore and 0.50 mm shaft. Transparent bulk blocks rays conservatively; refraction, internal reflections and silhouette-edge extraction are not implemented.",
        "Transparent direct measurements fail under the reduced surface model. Opaque witness uses identical geometry to isolate material observability; success is not a direct transparent-bore measurement.",
        "Part and shaft markers are nominal rigid handling-tab constellations. Tab supports, camera housings and rails are absent; mounting clearance and marker manufacturability are unvalidated.",
        "Relation errors are fixed independent dimensional offsets and axis perturbations, excluded from pose fitting. Marginal alignment RMS bounds sum conservatively because shared calibration covariance is unavailable.",
        "Precision admission here evaluates optical support only. Static contact pairs are reported against shared arm/part envelopes; they are not collision-certified trajectories. Guarded contact is required for seating, but is not executed by this study.",
        "Extracted-feature simulation assumes known marker identities and correspondences; no raster images or identity-tag decoder. Diffuse marker illumination and structured patterns are modeled; transparent silhouette extraction is not.",
        "No physical micron-accuracy claim. No automatic camera-layout selection from one sampled task; rejected states count against complete coverage."]
        .iter().map(|s|s.to_string()).collect()})
}

#[derive(Clone, Debug, Serialize)]
pub struct ExecutionEvent {
    pub phase: String,
    pub machine_tick: u64,
    pub time_s: f64,
    pub command_sequence: u64,
    pub truth_tool_pose: RigidTransform,
    pub held_body_id: Option<u32>,
}
#[derive(Clone, Debug, Serialize)]
pub struct ExecutionProbe {
    pub fidelity: String,
    pub events: Vec<ExecutionEvent>,
    pub stopped_reason: String,
    pub assembly_completed: bool,
}
/// Actual plant commands retain collision preflight and bounds. This probe
/// never issues insertion: a contact-capable optical controller must authorize it.
fn execution_probe(c: &GearStudyConfig) -> Result<ExecutionProbe, SimError> {
    use pipe_sim_core::*;
    let loaded = machine_config::load_baseline_machine_config()?;
    let mut s = machine_config::build_baseline_machine(&loaded)?;
    let pick = Vec3::new(c.pick_world_m.x, c.pick_world_m.y, c.pick_world_m.z);
    let insert = Vec3::new(
        c.insertion_world_m.x,
        c.insertion_world_m.y,
        c.insertion_world_m.z,
    );
    let mut nominal = s.serial_arms[0].arm.clone();
    let solution = nominal
        .solve_tool_position(pick, nominal.positions)
        .map_err(err)?;
    nominal.set_positions(solution.positions).map_err(err)?;
    let geometry = GearGeometry::uniform_spur(
        18,
        c.gear_radius_m / 10.,
        20f64.to_radians(),
        2. * c.gear_half_thickness_m,
        c.bore_radius_m,
    );
    s.add_body(RigidBody::new(
        BodyId(700),
        Shape::Gear(geometry),
        nominal.forward_kinematics().tool_pose,
        MotionType::Dynamic,
    ))
    .map_err(err)?;
    let final_pose = datum_pose(&s, c.insertion_world_m)?;
    s.add_body(RigidBody::new(
        BodyId(701),
        Shape::Capsule {
            radius_m: c.shaft_radius_m,
            half_segment_m: c.shaft_length_m / 2. - c.shaft_radius_m,
        },
        Pose::new(
            final_pose.transform_point(Vec3::new(0., 0., -c.shaft_length_m / 2.)),
            final_pose.rotation,
        ),
        MotionType::Static,
    ))
    .map_err(err)?;
    let mut events = Vec::new();
    let outcome = (|| -> Result<(), String> {
        let commands = [
            (
                "approach",
                MachineCommand::SetToolPoseTarget {
                    manipulator: ManipulatorId(1),
                    target_position_world_m: pick + Vec3::Z * c.approach_distance_m,
                },
            ),
            (
                "pick",
                MachineCommand::SetToolPoseTarget {
                    manipulator: ManipulatorId(1),
                    target_position_world_m: pick,
                },
            ),
            (
                "close",
                MachineCommand::SetGripperOpening {
                    manipulator: ManipulatorId(1),
                    target_opening_m: 2. * c.gear_radius_m - 5e-6,
                },
            ),
            (
                "reorient",
                MachineCommand::SetToolPoseTarget {
                    manipulator: ManipulatorId(1),
                    target_position_world_m: Vec3::new(
                        c.transfer_world_m.x,
                        c.transfer_world_m.y,
                        c.transfer_world_m.z,
                    ),
                },
            ),
            (
                "precontact",
                MachineCommand::SetToolPoseTarget {
                    manipulator: ManipulatorId(1),
                    target_position_world_m: insert + Vec3::Z * 0.001,
                },
            ),
        ];
        for (phase, command) in commands {
            s.submit_machine_command(command)
                .map_err(|e| format!("{phase}: command rejected: {e:?}"))?;
            let mut settled = false;
            for _ in 0..20000 {
                s.step().map_err(|e| format!("{phase}: {e:?}"))?;
                let collision = s.query_collisions_with_arms(s.config.collision);
                if let Some(contact) = collision
                    .contacts
                    .iter()
                    .find(|c| c.penetration_depth_m > 1e-9)
                {
                    return Err(format!(
                        "{phase}: unplanned contact {} / {}",
                        contact.body_a.0, contact.body_b.0
                    ));
                }
                let arm = &s.serial_arms[0];
                if !arm
                    .motion
                    .tool_motion
                    .is_some_and(|p| p.status == ToolMotionStatus::Active)
                    && (arm.gripper.opening_m - arm.gripper.command_opening_m).abs() < 1e-12
                {
                    settled = true;
                    break;
                }
            }
            if !settled {
                return Err(format!("{phase}: timeout"));
            }
            if phase == "close" {
                s.grasp_body_serial(ArmId(1), BodyId(700))
                    .map_err(|e| format!("grasp rejected: {e:?}"))?;
            }
            events.push(ExecutionEvent {
                phase: phase.into(),
                machine_tick: s.step_index,
                time_s: s.time_s,
                command_sequence: s.machine_command_sequence,
                truth_tool_pose: optical_pose(s.serial_arms[0].tool_pose()),
                held_body_id: s.serial_arms[0].gripper.held_body.map(|b| b.0),
            });
        }
        Ok(())
    })();
    s.submit_machine_command(MachineCommand::Stop {
        manipulator: Some(ManipulatorId(1)),
    })
    .map_err(err)?;
    events.push(ExecutionEvent {
        phase: "stop_requested".into(),
        machine_tick: s.step_index,
        time_s: s.time_s,
        command_sequence: s.machine_command_sequence,
        truth_tool_pose: optical_pose(s.serial_arms[0].tool_pose()),
        held_body_id: s.serial_arms[0].gripper.held_body.map(|b| b.0),
    });
    Ok(ExecutionProbe{events,assembly_completed:false,stopped_reason:outcome.err().unwrap_or_else(||"precontact reached; insertion requires observed alignment and guarded contact controller".into()),
        fidelity:"Actual bounded plant commands and collision checks; nominal targets, no optical servo or contact insertion. Gear annular collision envelope, capsule shaft, rigid grasp model. Stop request is recorded; deceleration is not executed after abort.".into()})
}

#[derive(Clone, Debug, Serialize)]
pub struct StateSummary {
    pub state: String,
    pub attempts: usize,
    pub tool_pose_count: usize,
    pub part_pose_count: usize,
    pub shaft_pose_count: usize,
    pub minimum_part_camera_count: Option<usize>,
    pub marker_occlusion_fraction: f64,
    pub bore_3d_rms_error_m: Option<f64>,
    /// sqrt(mean squared displacement from repeat mean), fixed design state.
    pub bore_repeatability_3d_rms_m: Option<f64>,
    pub alignment_3d_rms_error_m: Option<f64>,
    pub maximum_alignment_rms_bound_m: Option<f64>,
    pub transparent_passive_count: usize,
    pub transparent_structured_count: usize,
    pub opaque_witness_structured_count: usize,
    pub rejection_counts: BTreeMap<String, usize>,
    pub static_contact_pairs: Vec<(u32, u32, f64)>,
}
fn summarize_states(samples: &[GearSample]) -> Vec<StateSummary> {
    let mut groups: BTreeMap<&str, Vec<&GearSample>> = BTreeMap::new();
    for s in samples {
        groups.entry(&s.state).or_default().push(s);
    }
    groups
        .into_iter()
        .map(|(name, s)| {
            let mut rejections = BTreeMap::new();
            for row in &s {
                for r in &row.precision_rejections {
                    *rejections.entry(r.clone()).or_default() += 1;
                }
            }
            let points: Vec<_> = s
                .iter()
                .filter_map(|s| s.inferred_bore.as_ref().map(|p| p.center.position_world_m))
                .collect();
            let repeatability = if points.len() > 1 {
                let mean = points.iter().fold(Vec3::ZERO, |a, p| a + *p) / points.len() as f64;
                rms(points.iter().map(|p| (*p - mean).norm()))
            } else {
                None
            };
            let attempts = s.iter().map(|s| s.marker_visibility.len()).sum::<usize>();
            let blocked = s
                .iter()
                .flat_map(|s| &s.marker_visibility)
                .filter(|v| v.rejection == Some(MetrologyError::Occluded))
                .count();
            StateSummary {
                state: name.into(),
                attempts: s.len(),
                tool_pose_count: s.iter().filter(|s| s.tool_pose.is_some()).count(),
                part_pose_count: points.len(),
                shaft_pose_count: s.iter().filter(|s| s.shaft_datum.is_some()).count(),
                minimum_part_camera_count: s
                    .iter()
                    .filter_map(|s| {
                        s.marker_pose
                            .as_ref()
                            .map(|p| p.quality.usable_camera_count)
                    })
                    .min(),
                marker_occlusion_fraction: if attempts > 0 {
                    blocked as f64 / attempts as f64
                } else {
                    0.
                },
                bore_3d_rms_error_m: rms(s
                    .iter()
                    .filter_map(|s| s.bore_error_against_independent_truth_m)),
                bore_repeatability_3d_rms_m: repeatability,
                alignment_3d_rms_error_m: rms(s
                    .iter()
                    .filter_map(|s| s.alignment_error_against_independent_truth_m)),
                maximum_alignment_rms_bound_m: s
                    .iter()
                    .filter_map(|s| s.alignment_rms_upper_bound_m)
                    .reduce(f64::max),
                transparent_passive_count: s
                    .iter()
                    .filter(|s| s.direct_transparent_passive.estimate.is_some())
                    .count(),
                transparent_structured_count: s
                    .iter()
                    .filter(|s| s.direct_transparent_structured.estimate.is_some())
                    .count(),
                opaque_witness_structured_count: s
                    .iter()
                    .filter(|s| s.opaque_witness_structured.estimate.is_some())
                    .count(),
                rejection_counts: rejections,
                static_contact_pairs: s[0].static_contact_pairs.clone(),
            }
        })
        .collect()
}

fn datum_pose(
    machine: &pipe_sim_core::Simulation,
    target: Vec3,
) -> Result<pipe_sim_core::Pose, SimError> {
    let mut arm = machine.serial_arms[0].arm.clone();
    let solution = arm
        .solve_tool_position(
            pipe_sim_core::Vec3::new(target.x, target.y, target.z),
            arm.positions,
        )
        .map_err(err)?;
    arm.set_positions(solution.positions).map_err(err)?;
    Ok(arm.forward_kinematics().tool_pose)
}
