//! Optical adapter over the SAME Simulation used by machine commands. No
//! second arm model: live collision primitives and jaw poses cast the shadows.
use crate::{sha256_hex, SimError};
use pipe_optics::metrology::{synthetic::*, verification::*, *};
use pipe_optics::{
    Cylinder, Geometry, Mat3, Material, Primitive, RigidTransform, Scene, Sphere, Triangle, Vec3,
};
use pipe_sim_core::{PipeCellConfig, Pose, Shape, Simulation, ToolMotionStatus};
use serde::{Deserialize, Serialize};

pub fn baseline_config() -> MetrologyConfig {
    MetrologyConfig::baseline()
}

pub fn configuration_json() -> Result<String, MetrologyError> {
    serde_json::to_string_pretty(&baseline_config())
        .map_err(|e| MetrologyError::InvalidConfiguration(e.to_string()))
}

pub fn verification_json(
    config_json: &str,
    source_revision: &str,
    repeats: u32,
) -> Result<String, MetrologyError> {
    let config = serde_json::from_str(config_json)
        .map_err(|e| MetrologyError::InvalidConfiguration(e.to_string()))?;
    let report = run_verification(config, source_revision, repeats)?;
    serde_json::to_string(&report).map_err(|e| MetrologyError::InvalidConfiguration(e.to_string()))
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RegisteredPartFeature {
    pub entity_id: u32,
    pub body_id: u32,
    pub feature_id: u32,
    pub point_body_m: Vec3,
    pub normal_body: Option<Vec3>,
    pub surface: SurfaceResponse,
}
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct MachineMetrologyFrame {
    pub schema_version: u32,
    pub machine_tick: u64,
    pub time_s: f64,
    pub config_sha256: String,
    pub world: ObservedWorld,
    pub visibility: Vec<VisibilityAttempt>,
    pub rejected_entities: Vec<(u32, MetrologyError)>,
    pub occluder_count: usize,
    pub fidelity: Vec<String>,
}
pub struct MachineMetrology {
    pub config: MetrologyConfig,
    pub tools: Vec<RigidFiducial>,
    pub part_features: Vec<RegisteredPartFeature>,
    pub extra_occluders: Scene,
    pub world: ObservedWorld,
    captured_command_sequence: Option<u64>,
    captured_configuration: Option<String>,
}
impl MachineMetrology {
    pub fn new(
        config: MetrologyConfig,
        tools: Vec<RigidFiducial>,
        part_features: Vec<RegisteredPartFeature>,
    ) -> Result<Self, MetrologyError> {
        config.validate()?;
        let mut ids = std::collections::BTreeSet::new();
        for tool in &tools {
            tool.validate()?;
            if !ids.insert(tool.object_id) || tool.arm_id.is_none() {
                return Err(MetrologyError::InvalidConfiguration(
                    "duplicate entity or missing arm attachment".into(),
                ));
            }
        }
        for part in &part_features {
            if !ids.insert(part.entity_id) || !part.point_body_m.is_finite() {
                return Err(MetrologyError::InvalidConfiguration(
                    "invalid part entity".into(),
                ));
            }
        }
        Ok(Self {
            config,
            tools,
            part_features,
            extra_occluders: Scene::default(),
            world: ObservedWorld::default(),
            captured_command_sequence: None,
            captured_configuration: None,
        })
    }

    fn configuration_stamp(&self) -> Result<String, SimError> {
        serde_json::to_string(&(&self.config, &self.tools, &self.part_features))
            .map(|s| sha256_hex(s.as_bytes()))
            .map_err(|e| SimError::InvalidScenario(e.to_string()))
    }

    pub(crate) fn require_current_capture(&self, command_sequence: u64) -> Result<(), SimError> {
        if self.captured_command_sequence != Some(command_sequence)
            || self.captured_configuration.as_ref() != Some(&self.configuration_stamp()?)
        {
            return Err(SimError::InvalidScenario(
                "precision observation predates a machine command or configuration change".into(),
            ));
        }
        Ok(())
    }

    pub(crate) fn consume_capture(&mut self) {
        self.world.begin_acquisition(self.world.observation_epoch);
        self.captured_command_sequence = None;
        self.captured_configuration = None;
    }
    /// Explicitly adapt optical mounts to an existing machine's tube. The
    /// standalone 100 mm candidate is never silently imposed on the 160 mm
    /// legacy runtime. Sampling is preserved by updating effective focal length.
    pub fn for_machine(cell: PipeCellConfig) -> Result<Self, MetrologyError> {
        let mut c = MetrologyConfig::baseline();
        c.id = "metrology_adapted_to_machine_geometry_v1".into();
        c.tube_id_m = 2.0 * cell.tube.inner_radius_m;
        c.tube_working_length_m = cell.tube.working_length_m;
        for d in &mut c.devices {
            let p = d.model.center_world();
            let scale = cell.tube.inner_radius_m / p.x.hypot(p.y);
            let eye = Vec3::new(p.x * scale, p.y * scale, p.z);
            let distance = (eye - c.precision_volume.center_world_m).norm();
            let focal_scale = distance / d.focus_working_distance_m;
            d.model.world_from_camera = look_at(eye, c.precision_volume.center_world_m);
            d.model.intrinsics.fx_px *= focal_scale;
            d.model.intrinsics.fy_px *= focal_scale;
            d.effective_focal_length_m *= focal_scale;
            d.focus_working_distance_m = distance;
            d.nominal_lens_focal_length_m =
                1.0 / (1.0 / d.effective_focal_length_m + 1.0 / distance);
        }
        let tools = (1..=cell.manipulator_count as u32)
            .map(RigidFiducial::baseline)
            .collect();
        Self::new(c, tools, Vec::new())
    }
    /// Precision acquisition requires a stationary machine. Physical stopped
    /// evidence will come from an independent stop/interlock input; this adapter
    /// checks the deterministic plant. Time advances through exposure/latency.
    pub(crate) fn acquire(
        &mut self,
        mechanics: &mut Simulation,
        cell: PipeCellConfig,
    ) -> Result<MachineMetrologyFrame, SimError> {
        self.consume_capture();
        self.world.begin_acquisition(mechanics.step_index + 1);
        self.config.validate().map_err(metrology_error)?;
        if (self.config.tube_id_m - 2.0 * cell.tube.inner_radius_m).abs() > 1e-9
            || (self.config.tube_working_length_m - cell.tube.working_length_m).abs() > 1e-9
        {
            return Err(metrology_error(MetrologyError::InvalidConfiguration(
                "optical tube and authoritative machine disagree".into(),
            )));
        }
        if !stationary(mechanics) {
            return Err(metrology_error(MetrologyError::MotionDuringSequence));
        }
        let mut scene = machine_optical_scene(mechanics);
        scene
            .primitives
            .extend(self.extra_occluders.primitives.iter().copied());
        let t = timing(
            &self.config,
            mechanics.step_index + 1,
            mechanics.time_s + mechanics.config.fixed_dt_s,
            0.0,
        );
        let mut visibility = Vec::new();
        let mut rejected = Vec::new();
        for tool in &self.tools {
            let arm = mechanics
                .serial_arms
                .iter()
                .find(|a| Some(a.id.0) == tool.arm_id)
                .ok_or_else(|| metrology_error(MetrologyError::InvalidObservation))?;
            let acquisition = acquire_tool(
                &self.config,
                &scene,
                tool,
                optical_pose(arm.tool_pose()),
                MeasurementMode::Precision,
                &t,
            )
            .map_err(metrology_error)?;
            visibility.extend(acquisition.visibility);
            match estimate_tool_pose(&self.config, tool, &acquisition.observations) {
                Ok(pose) => {
                    self.world.entities.insert(
                        tool.object_id,
                        ObservedEntity {
                            kind: ObservedEntityKind::Tool,
                            point: None,
                            pose: Some(pose),
                        },
                    );
                }
                Err(e) => rejected.push((tool.object_id, e)),
            }
        }
        for feature in &self.part_features {
            let body = mechanics
                .bodies
                .iter()
                .find(|b| b.id.0 == feature.body_id)
                .ok_or_else(|| metrology_error(MetrologyError::InvalidObservation))?;
            let pose = optical_pose(body.pose);
            let truth = TruthFeature {
                object_id: feature.entity_id,
                feature_id: feature.feature_id,
                point_world_m: pose.transform_point(feature.point_body_m),
                normal_world: feature.normal_body.map(|n| pose.transform_vector(n)),
                velocity_world_m_s: Vec3::ZERO,
                surface: feature.surface.clone(),
            };
            let a = acquire_passive(
                &self.config,
                &scene,
                &truth,
                MeasurementMode::Precision,
                MeasurementClass::CriticalFeature,
                &t,
            )
            .map_err(metrology_error)?;
            visibility.extend(a.visibility);
            match reconstruct_point(&self.config, &a.observations) {
                Ok(point) => {
                    self.world.entities.insert(
                        feature.entity_id,
                        ObservedEntity {
                            kind: ObservedEntityKind::CriticalFeature,
                            point: Some(point),
                            pose: None,
                        },
                    );
                }
                Err(e) => rejected.push((feature.entity_id, e)),
            }
        }
        let references = acquire_references(&self.config, &scene, &t).map_err(metrology_error)?;
        while mechanics.time_s < t.available_s + self.config.acquisition.max_trigger_skew_s {
            mechanics
                .step()
                .map_err(|e| SimError::Mechanics(format!("{e:?}")))?;
        }
        if !stationary(mechanics) {
            self.world.begin_acquisition(mechanics.step_index);
            return Err(metrology_error(MetrologyError::MotionDuringSequence));
        }
        self.world.health = Some(evaluate_health(&self.config, &references, mechanics.time_s));
        self.captured_command_sequence = Some(mechanics.machine_command_sequence);
        self.captured_configuration = Some(self.configuration_stamp()?);
        let config_json = serde_json::to_string(&self.config)
            .map_err(|e| SimError::InvalidScenario(e.to_string()))?;
        Ok(MachineMetrologyFrame {
            schema_version: 1,
            machine_tick: mechanics.step_index,
            time_s: mechanics.time_s,
            config_sha256: sha256_hex(config_json.as_bytes()),
            world: self.world.clone(),
            visibility,
            rejected_entities: rejected,
            occluder_count: scene.primitives.len(),
            fidelity: machine_fidelity(),
        })
    }
}
fn metrology_error(e: MetrologyError) -> SimError {
    SimError::InvalidScenario(format!("optical metrology: {e}"))
}
fn stationary(s: &Simulation) -> bool {
    s.serial_arms.iter().all(|a| {
        let m = &a.motion;
        !m.tool_motion
            .is_some_and(|p| p.status == ToolMotionStatus::Active)
            && m.carriage.z_velocity_m_s.abs() < 1e-12
            && m.carriage.theta_velocity_rad_s.abs() < 1e-12
            && m.joint_velocities_rad_s.iter().all(|v| v.abs() < 1e-12)
            && (m.stopped
                || ((m.carriage.z_m - m.carriage_target.z_m).abs() < 1e-12
                    && (m.carriage.theta_rad - m.carriage_target.theta_rad).abs() < 1e-12
                    && m.joint_positions_rad
                        .iter()
                        .zip(m.joint_targets_rad)
                        .all(|(a, b)| (a - b).abs() < 1e-12)))
            && (a.gripper.opening_m - a.gripper.command_opening_m).abs() < 1e-12
    }) && s.bodies.iter().all(|b| {
        b.linear_velocity_m_s.length() < 1e-12 && b.angular_velocity_rad_s.length() < 1e-12
    })
}
pub fn machine_fidelity() -> Vec<String> {
    ["Live link collision capsules, gripper jaw boxes and rigid bodies supply ray occlusion; gear cylinders conservatively fill tooth gaps and bores.",
     "Rail solids, tendons, camera housings and reference supports are absent from the current machine runtime; supply CAD triangle meshes via extra_occluders before scoring a hardware layout.",
     "Distal marker constellation is a configurable nominal design attachment; manufacturing and TCP calibration errors are simulated. Physical mounting and rigidity are unvalidated.",
     "Stopped synthetic acquisition only in this adapter. Existing M1e/M1f controller remains on its separately documented reduced sensor pipeline."]
        .iter().map(|s|s.to_string()).collect()
}
pub fn machine_optical_scene(s: &Simulation) -> Scene {
    let mut scene = Scene::default();
    for body in s.bodies.iter().filter(|b| b.enabled) {
        push_shape(&mut scene, optical_pose(body.pose), body.shape, body.id.0);
    }
    for arm in &s.serial_arms {
        for (i, (pose, shape)) in arm.kinematics.collision_capsules.iter().enumerate() {
            push_shape(
                &mut scene,
                optical_pose(*pose),
                *shape,
                100_000 + arm.id.0 * 100 + i as u32,
            );
        }
        for (i, pose) in arm
            .gripper
            .jaw_poses(arm.tool_pose(), arm.gripper_config)
            .iter()
            .enumerate()
        {
            push_shape(
                &mut scene,
                optical_pose(*pose),
                Shape::Box {
                    half_extents_m: arm.gripper_config.jaw_half_extents_m,
                },
                200_000 + arm.id.0 * 100 + i as u32,
            );
        }
    }
    scene
}
fn optical_vec(v: pipe_sim_core::Vec3) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}
pub(crate) fn optical_pose(p: Pose) -> RigidTransform {
    let x = optical_vec(p.transform_vector(pipe_sim_core::Vec3::X));
    let y = optical_vec(p.transform_vector(pipe_sim_core::Vec3::Y));
    let z = optical_vec(p.transform_vector(pipe_sim_core::Vec3::Z));
    RigidTransform::new(
        Mat3::new([[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]]),
        optical_vec(p.translation),
    )
}
fn push_shape(scene: &mut Scene, pose: RigidTransform, shape: Shape, tag: u32) {
    let mat = Material::default();
    let cylinder = |scene: &mut Scene, radius_m: f64, half: f64, offset: f64| {
        scene.push(Primitive::new(
            Geometry::Cylinder(Cylinder {
                start: pose.transform_point(Vec3::Z * (offset - half)),
                end: pose.transform_point(Vec3::Z * (offset + half)),
                radius_m,
                capped: true,
            }),
            mat,
            tag,
        ));
    };
    match shape {
        Shape::Sphere { radius_m } => scene.push(Primitive::new(
            Geometry::Sphere(Sphere {
                center: pose.translation,
                radius_m,
            }),
            mat,
            tag,
        )),
        Shape::Capsule {
            radius_m,
            half_segment_m,
        } => {
            cylinder(scene, radius_m, half_segment_m, 0.0);
            for z in [-half_segment_m, half_segment_m] {
                scene.push(Primitive::new(
                    Geometry::Sphere(Sphere {
                        center: pose.transform_point(Vec3::Z * z),
                        radius_m,
                    }),
                    mat,
                    tag,
                ));
            }
        }
        Shape::Box { half_extents_m } => {
            let h = optical_vec(half_extents_m);
            let mut v = Vec::new();
            for z in [-1.0, 1.0] {
                for y in [-1.0, 1.0] {
                    for x in [-1.0, 1.0] {
                        v.push(pose.transform_point(h.component_mul(Vec3::new(x, y, z))));
                    }
                }
            }
            for [a, b, c] in [
                [0, 1, 3],
                [0, 3, 2],
                [4, 6, 7],
                [4, 7, 5],
                [0, 4, 5],
                [0, 5, 1],
                [2, 3, 7],
                [2, 7, 6],
                [0, 2, 6],
                [0, 6, 4],
                [1, 5, 7],
                [1, 7, 3],
            ] {
                scene.push(Primitive::new(
                    Geometry::Triangle(Triangle {
                        a: v[a],
                        b: v[b],
                        c: v[c],
                        double_sided: true,
                    }),
                    mat,
                    tag,
                ));
            }
        }
        Shape::Gear(g) => {
            cylinder(
                scene,
                g.tip_radius_m,
                g.half_thickness_m,
                g.tooth_center_offset_m,
            );
            cylinder(scene, g.hub_radius_m, g.half_total_height_m, 0.0);
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct MetrologyVerificationReport {
    pub schema_version: u32,
    pub source_revision: String,
    pub config_sha256: String,
    pub configuration: MetrologyConfig,
    pub accuracy_claim: String,
    pub passive: VerificationSummary,
    pub structured: VerificationSummary,
    pub tool: ToolVerification,
    pub sensitivity: Vec<VerificationSummary>,
    pub fidelity: Vec<String>,
}
pub fn run_verification(
    config: MetrologyConfig,
    source_revision: &str,
    repeats: u32,
) -> Result<MetrologyVerificationReport, MetrologyError> {
    if source_revision.len() != 40 || !source_revision.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(MetrologyError::InvalidConfiguration(
            "full source commit SHA required".into(),
        ));
    }
    let scene = Scene::default();
    let passive = verify_volume(&config, &scene, repeats, false)?;
    let structured = verify_volume(&config, &scene, repeats, true)?;
    let tool = verify_tool(&config, &scene, repeats * 2)?;
    let mut sensitivity = Vec::new();
    for (name, noise, delta_k, calibration) in [
        ("localization_noise_x10", 10.0, 0.0, 1.0),
        ("structure_plus_5K", 1.0, 5.0, 1.0),
        ("camera_calibration_x10", 1.0, 0.0, 10.0),
    ] {
        let mut c = config.clone();
        c.errors.localization_sigma_px *= noise;
        c.thermal.structure_temperature_k += delta_k;
        c.errors.camera_calibration_rms_m *= calibration;
        let mut r = verify_volume(&c, &scene, repeats, false)?;
        r.condition = name.into();
        sensitivity.push(r);
    }
    let json = serde_json::to_string(&config)
        .map_err(|e| MetrologyError::InvalidConfiguration(e.to_string()))?;
    Ok(MetrologyVerificationReport {schema_version:1,source_revision:source_revision.into(),config_sha256:sha256_hex(json.as_bytes()),configuration:config,
        accuracy_claim:"Synthetic independent-geometry verification only; no physical accuracy or hardware qualification claim.".into(),passive,structured,tool,sensitivity,
        fidelity:["Verification artifact coordinates are excluded from all calibration fits. Calibration is imported nominal data; this report does not fit it.",
            "Image-feature and pattern-intensity simulation, with seeded noise and physical projection perturbations; no raster rendering, measured lens PSF/MTF or full BRDF.",
            "The supplied depth-of-field interval is a provisional design bound; blur, diffraction and mounting aperture clipping require measured optical characterization.",
            "Telecentric lenses are representable candidates but rejected as unsupported by this perspective solver.",
            "Common RMS priors are retained once after solving. These priors are not achieved accuracy; reported error is computed separately against independent geometry.",
            "Volume verification uses isolated feature geometry to measure sensor performance. Machine occlusion is a separate live-runtime test; no full-machine visibility or dense-surface precision claim."]
            .iter().map(|s|s.to_string()).collect()})
}
