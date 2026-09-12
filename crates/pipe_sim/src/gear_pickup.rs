//! One observed pickup-and-return experiment on the authoritative machine.
//! Sensor truth is confined to MachineMetrology; controller targets use estimates.
use crate::{
    build_optics, machine_config,
    metrology::{MachineMetrology, MachineMetrologyFrame, RegisteredBodyFiducial},
    scene, sha256_hex, SceneDescription, SceneFrame, SimError,
};
use pipe_optics::{metrology::*, Mat3, RigidTransform, Vec3 as OVec};
use pipe_sim_core::{
    ArmId, BodyId, GearGeometry, MachineCommand, ManipulatorId, MotionType, PipeCellConfig, Pose,
    RigidBody, Shape, Simulation, ToolMotionStatus, Vec3,
};
use serde::Serialize;

const GEAR: BodyId = BodyId(700);
const SUPPORTS: [BodyId; 3] = [BodyId(710), BodyId(711), BodyId(712)];
const MAX_STEPS: usize = 60000;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PickupFault {
    #[default]
    None,
    MissingPartObservation,
    StaleObservation,
    MotionTimeout,
    UnsupportedRelease,
}
#[derive(Clone, Debug, Serialize)]
pub struct GearPickupSample {
    pub tick: u64,
    pub time_s: f64,
    pub phase: String,
    pub command_sequence: u64,
    pub scene_frame: SceneFrame,
    pub metrology: Option<MachineMetrologyFrame>,
    pub held_part_body_id: Option<u32>,
    pub command: Option<String>,
    pub support_gaps_m: Vec<(u32, f64)>,
    pub insertion_precision_rejection: Option<String>,
}
#[derive(Clone, Debug, Serialize)]
pub struct GearPickupReport {
    pub schema_version: u32,
    pub source_revision: String,
    pub source_tree_sha256: String,
    pub machine_config_sha256: String,
    pub configuration_sha256: String,
    pub status: String,
    pub refusal: Option<String>,
    pub fault: PickupFault,
    pub completed_phases: Vec<String>,
    pub scene_description: SceneDescription,
    pub samples: Vec<GearPickupSample>,
    pub fidelity: Vec<String>,
    pub gear_outer_diameter_m: f64,
    pub gear_thickness_m: f64,
    pub optical_config: MetrologyConfig,
    pub insertion_alignment_evaluated: bool,
    pub insertion_precision_limit_m: f64,
}
fn err(e: impl std::fmt::Debug) -> SimError {
    SimError::Mechanics(format!("gear pickup: {e:?}"))
}
fn ov(v: Vec3) -> OVec {
    OVec::new(v.x, v.y, v.z)
}
fn cv(v: OVec) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}
fn marker(id: u32, arm: Option<u32>, points: &[(OVec, OVec)]) -> RigidFiducial {
    RigidFiducial {
        object_id: id,
        arm_id: arm,
        tool_type: "direct_printed_surface_marks".into(),
        identity_code: format!("pickup-{id}"),
        geometry_calibration_id: "synthetic_calibration_v1".into(),
        features: points
            .iter()
            .enumerate()
            .map(|(i, (p, n))| MetrologyFeature {
                id: i as u32 + 1,
                point_fiducial_m: *p,
                normal_fiducial: Some(*n),
                diameter_m: 0.0001,
            })
            .collect(),
        tcp_from_fiducial: RigidTransform::new(Mat3::IDENTITY, OVec::ZERO),
    }
}

fn prepare_gear_pickup(
    machine_json: &str,
    source_revision: &str,
    source_tree_sha256: &str,
    fault: PickupFault,
) -> Result<Operation, SimError> {
    let loaded = machine_config::load_machine_config(machine_json)?;
    if (loaded.cell.tube.inner_radius_m - 0.080).abs() > 1e-12 {
        return Err(err("pickup requires the 160mm machine"));
    }
    let mut sim = machine_config::build_baseline_machine(&loaded)?;
    let mut design = sim.serial_arms[0].arm.clone();
    let pick = Vec3::new(-0.004, 0.004, 0.);
    let solution = design
        .solve_tool_position(pick, design.positions)
        .map_err(err)?;
    design.set_positions(solution.positions).map_err(err)?;
    let nest = design.forward_kinematics().tool_pose;
    let gear = GearGeometry::uniform_spur(18, 0.0001, 20f64.to_radians(), 0.0008, 0.00026);
    if (2. * gear.tip_radius_m - 0.002).abs() > 1e-12 {
        return Err(err("actual GearGeometry OD disagrees with 2mm coupon"));
    }
    sim.add_body(RigidBody::new(
        GEAR,
        Shape::Gear(gear),
        nest,
        MotionType::Dynamic,
    ))
    .map_err(err)?;
    for (i, support_id) in SUPPORTS.iter().enumerate() {
        let angle = i as f64 * std::f64::consts::TAU / 3.;
        let xy = Vec3::new(0.0006 * angle.cos(), 0.0006 * angle.sin(), 0.);
        sim.add_body(RigidBody::new(
            *support_id,
            Shape::Sphere { radius_m: 0.00015 },
            Pose::new(nest.transform_point(xy + Vec3::Z * 0.00055), nest.rotation),
            MotionType::Static,
        ))
        .map_err(err)?;
        sim.add_body(RigidBody::new(
            BodyId(720 + i as u32),
            Shape::Capsule {
                radius_m: 0.0001,
                half_segment_m: 0.0003,
            },
            Pose::new(nest.transform_point(xy + Vec3::Z * 0.001), nest.rotation),
            MotionType::Static,
        ))
        .map_err(err)?;
    }
    sim.add_body(RigidBody::new(
        BodyId(730),
        Shape::Box {
            half_extents_m: Vec3::new(0.0009, 0.0009, 0.00015),
        },
        Pose::new(nest.transform_point(Vec3::Z * 0.00145), nest.rotation),
        MotionType::Static,
    ))
    .map_err(err)?;
    sim.serial_arms[0].support_body_ids = SUPPORTS.to_vec();
    let mut optics = MachineMetrology::for_machine(loaded.cell).map_err(err)?;
    let mut tool_points = Vec::new();
    let mut part_points = Vec::new();
    for x in [-1., 1.] {
        for y in [-1., 1.] {
            tool_points.push((
                OVec::new(x * 0.00165, y * 0.00115, -0.003),
                OVec::new(0., 0., 1.),
            ));
            part_points.push((
                OVec::new(x * 0.0005, y * 0.0004, -0.0004),
                OVec::new(0., 0., -1.),
            ));
        }
    }
    for x in [-1., 1.] {
        for y in [-1., 1.] {
            tool_points.push((
                OVec::new(x * 0.0018, y * 0.0008, -0.0033),
                OVec::new(x, 0., 0.),
            ));
        }
    }
    for degrees in [80_f64, 90., 100.] {
        for z in [-0.00025, 0.00025] {
            let a = degrees.to_radians();
            part_points.push((
                OVec::new(0.001 * a.cos(), 0.001 * a.sin(), z),
                OVec::new(a.cos(), a.sin(), 0.),
            ));
        }
    }
    optics.tools = vec![marker(1, Some(1), &tool_points)];
    optics.surface_attached_fiducials = [1, 700].into_iter().collect();
    optics.body_fiducials = vec![RegisteredBodyFiducial {
        body_id: GEAR.0,
        fiducial: marker(GEAR.0, None, &part_points),
        relation: PartFeatureRelation {
            feature_id: 1,
            center_fiducial_m: OVec::ZERO,
            axis_fiducial: OVec::new(0., 0., 1.),
            calibration_id: "pickup_bore_relation_unqualified_synthetic_v1".into(),
            provenance: RelationProvenance::SyntheticIndependentCharacterization,
            characterization_rms_m: 1e-6,
            axis_characterization_rms_rad: 0.0002,
        },
    }];
    let mut desc = scene::build_scene_description(
        &loaded.id,
        &loaded.source_sha256,
        loaded.cell,
        &sim,
        &build_optics(0),
        &[(700, "2mm gear pickup coupon".into())],
    );
    // These are the actual adapted metrology mounts, not the legacy 8-camera rig.
    desc.sensors = optics
        .config
        .devices
        .iter()
        .map(|d| scene::SensorDescription {
            id: d.id,
            kind: "metrology_camera_or_projector",
            nominal_translation_m: [
                d.model.center_world().x,
                d.model.center_world().y,
                d.model.center_world().z,
            ],
            nominal_rotation_world_from_sensor: d.model.world_from_camera.rotation.m,
        })
        .collect();
    let config_hash = sha256_hex(
        serde_json::to_string(&(
            &loaded.source_sha256,
            &optics.config,
            &optics.tools,
            &optics.body_fiducials,
            &optics.surface_attached_fiducials,
            fault,
            "gear_pickup_v1",
            source_tree_sha256,
        ))
        .map_err(err)?
        .as_bytes(),
    );
    let report = GearPickupReport {
        schema_version: 1,
        source_revision: source_revision.into(),
        source_tree_sha256: source_tree_sha256.into(),
        machine_config_sha256: loaded.source_sha256,
        configuration_sha256: config_hash,
        status: "running".into(),

        refusal: None,
        fault,
        completed_phases: Vec::new(),
        scene_description: desc,
        samples: Vec::new(),
        fidelity: vec![
            "Authoritative 1ms deterministic Simulation; actual arm, distal tool, support and part collision geometry. All other arms remain parked in the 160mm tube.".into(),
            "Synthetic noisy image features, ray occlusion and independently estimated gear markers; no controller truth fallback. Surface markers use substrate-constrained physical endpoints with tangential registration/manufacturing errors; camera error and feature calibration uncertainty remain modeled; normal substrate form error is not simulated. Rim marks lie on the cylindrical gear collision envelope, not validated printable tooth surfaces: this is a marked gear-envelope coupon, not a qualified toothed-gear marker layout. Printed marks and their nominal calibrated relation are not hardware qualified.".into(),
            "Pickup-only admission uses conservative 3-sigma relative uncertainty against 100um engagement margin; this is not the unchanged 5um insertion admission or assembly success.".into(),
            "Rigid attachment, zero gravity and compliance-derived grip force proxy. Supported-release admission permits up to20um positive gap to three supports. Zero gravity supplies no physical settling or load transfer; the exact gaps are reported, not called measured contact.".into(),
            "Support stems and plate are fixed fixture components; their internal overlap is intentional. The return nest datum is nominal design geometry, not an independently calibrated fixture observation. No tool or moving part collision masks are suppressed.".into(),
            "Replay physical poses are simulation truth; optical estimates are stored separately and never substituted. Active tool-plan Stop is an instantaneous idealized simulation hold, not actuator braking or a calibrated stopping distance. Final stopped state and velocities are sampled. Motion samples every 20 ticks; acquisition and phase transitions additionally sampled, with no display interpolation.".into(),
        ],
        gear_outer_diameter_m: 2.*gear.tip_radius_m,
        gear_thickness_m: 2.*gear.half_total_height_m,
        optical_config: optics.config.clone(),
        insertion_alignment_evaluated: false,
        insertion_precision_limit_m: 5e-6
    };
    Ok(Operation {
        sim,
        optics,
        cell: loaded.cell,
        nest,
        report,
        phase: "initial".into(),
        last_frame: None,
        insertion_rejection: None,
    })
}

pub fn run_gear_pickup(
    machine_json: &str,
    source_revision: &str,
    source_tree_sha256: &str,
    fault: PickupFault,
) -> Result<GearPickupReport, SimError> {
    let mut op = prepare_gear_pickup(machine_json, source_revision, source_tree_sha256, fault)?;
    op.record(None);
    let outcome = op.execute();
    if let Err(reason) = outcome {
        op.report.refusal = Some(reason);
        op.phase = "controlled_stop".into();
        let stopped = op.command(MachineCommand::Stop { manipulator: None });
        op.report.status = if stopped.is_ok() {
            "controlled_refusal"
        } else {
            "stop_failed"
        }
        .into();
        if let Err(e) = stopped {
            op.report.refusal = Some(format!(
                "{}; stop:{e}",
                op.report.refusal.unwrap_or_default()
            ));
        }
        op.record(None);
    } else {
        op.report.status = "completed_pickup_return".into();
        op.phase = "complete".into();
        op.record(None);
    }
    Ok(op.report)
}

struct Operation {
    sim: Simulation,
    optics: MachineMetrology,
    cell: PipeCellConfig,
    nest: Pose,
    report: GearPickupReport,
    phase: String,
    last_frame: Option<MachineMetrologyFrame>,
    insertion_rejection: Option<String>,
}
impl Operation {
    fn record(&mut self, command: Option<String>) {
        let c = self
            .sim
            .query_collisions_with_arms(self.sim.config.collision);
        self.report.samples.push(GearPickupSample {
            tick: self.sim.step_index,
            time_s: self.sim.time_s,
            phase: self.phase.clone(),
            command_sequence: self.sim.machine_command_sequence,
            scene_frame: scene::build_scene_frame(&self.sim, &c.contacts),
            metrology: self.last_frame.take(),
            held_part_body_id: self.sim.serial_arms[0].gripper.held_body.map(|b| b.0),
            command,
            support_gaps_m: SUPPORTS
                .iter()
                .filter_map(|id| {
                    let a = self.sim.body(GEAR)?;
                    let b = self.sim.body(*id)?;
                    pipe_sim_core::query_pair(a, b).map(|p| (id.0, p.signed_distance_m))
                })
                .collect(),
            insertion_precision_rejection: self.insertion_rejection.take(),
        });
    }
    fn settled(&self) -> bool {
        self.sim.serial_arms.iter().all(|a| {
            !a.motion
                .tool_motion
                .is_some_and(|p| p.status == ToolMotionStatus::Active)
                && a.motion
                    .joint_velocities_rad_s
                    .iter()
                    .all(|v| v.abs() < 1e-12)
                && a.motion.carriage.z_velocity_m_s.abs() < 1e-12
                && a.motion.carriage.theta_velocity_rad_s.abs() < 1e-12
                && (a.gripper.opening_m - a.gripper.command_opening_m).abs() < 1e-12
        })
    }
    fn command(&mut self, c: MachineCommand) -> Result<(), String> {
        self.optics.consume_capture();
        self.sim.submit_machine_command(c).map_err(|e| {
            format!(
                "{}: command rejected:{e:?}; diagnostic:{:?}",
                self.phase, self.sim.last_tool_path_collision
            )
        })?;
        self.record(Some(format!("{c:?}")));
        let stopping = matches!(c, MachineCommand::Stop { .. });
        let limit = if self.report.fault == PickupFault::MotionTimeout && !stopping {
            1
        } else {
            MAX_STEPS
        };
        for _ in 0..limit {
            self.sim.step().map_err(|e| format!("step:{e:?}"))?;
            if !stopping {
                self.sim
                    .validate_serial_arm_current_clearance(ArmId(1))
                    .map_err(|e| {
                        format!(
                            "{}:current clearance:{e:?}; diagnostic:{:?}",
                            self.phase, self.sim.last_tool_path_collision
                        )
                    })?;
            }
            if !stopping
                && matches!(self.phase.as_str(), "observed_lift" | "observed_return")
                && self.sim.serial_arms[0].gripper.held_body != Some(GEAR)
            {
                return Err("held-body retention lost during motion".into());
            }
            if self.sim.step_index % 20 == 0 {
                self.record(None);
            }
            if self.settled() {
                self.record(None);
                return Ok(());
            }
        }
        Err(format!("{}:motion timeout", self.phase))
    }
    fn observe(&mut self) -> Result<(ObservedToolPose, InferredPartFeature), String> {
        let mut frame = self
            .optics
            .acquire(&mut self.sim, self.cell)
            .map_err(|e| format!("{}:observation:{e}", self.phase))?;
        if self.report.fault == PickupFault::MissingPartObservation {
            frame.inferred_features.clear();
            frame.world.entities.remove(&GEAR.0);
            self.optics.world.entities.remove(&GEAR.0);
        }
        let result = (|| {
            let tool = frame
                .world
                .entities
                .get(&1)
                .and_then(|e| e.pose.clone())
                .ok_or_else(|| {
                    format!(
                        "{}:tool_unobserved:{:?}",
                        self.phase, frame.rejected_entities
                    )
                })?;
            let part = frame
                .inferred_features
                .get(&GEAR.0)
                .cloned()
                .ok_or_else(|| {
                    format!(
                        "{}:part_unobserved:{:?}",
                        self.phase, frame.rejected_entities
                    )
                })?;
            let health = frame.world.health.as_ref().ok_or("missing health")?;
            self.insertion_rejection = require_precision(
                &self.optics.config,
                &PrecisionContract::default(),
                &tool,
                &part.center,
                health,
                self.sim.time_s,
            )
            .err()
            .map(|e| format!("{e:?}"));
            let policy = PrecisionContract {
                maximum_tool_3d_rms_m: 20e-6,
                maximum_target_3d_rms_m: 20e-6,
                maximum_orientation_rms_rad: 0.02,
                ..PrecisionContract::default()
            };
            let now = if self.report.fault == PickupFault::StaleObservation {
                self.sim.time_s + 1.
            } else {
                self.sim.time_s
            };
            require_precision(
                &self.optics.config,
                &policy,
                &tool,
                &part.center,
                health,
                now,
            )
            .map_err(|e| format!("{}:pickup_precision:{e:?}", self.phase))?;
            let bound = 3.
                * (tool.predicted_tcp_3d_rms_m()
                    + part.center.predicted_3d_rms_m()
                    + 0.001 * (tool.orientation_rms_rad() + part.axis_rms_rad));
            if !bound.is_finite() || bound > 100e-6 {
                return Err(format!(
                    "{}:engagement_uncertainty_bound_m={bound:e}>100um",
                    self.phase
                ));
            }
            Ok((tool, part))
        })();
        self.last_frame = Some(frame);
        self.record(None);
        result
    }
    fn move_to(&mut self, target: Vec3, axis: Vec3) -> Result<(), String> {
        self.command(MachineCommand::SetToolAxisTarget {
            manipulator: ManipulatorId(1),
            target_position_world_m: target,
            target_axis_world: axis,
        })
    }
    fn finish(&mut self) {
        self.report.completed_phases.push(self.phase.clone());
        self.record(None);
    }
    fn execute(&mut self) -> Result<(), String> {
        let axis = self.nest.rotation.rotate_vec3(Vec3::Z);
        self.phase = "open_gripper".into();
        self.command(MachineCommand::SetGripperOpening {
            manipulator: ManipulatorId(1),
            target_opening_m: self.cell.gripper.max_opening_m,
        })?;
        self.finish();
        self.phase = "coarse_fixture_approach".into();
        self.move_to(self.nest.translation - axis * 0.010, axis)?;
        self.move_to(self.nest.translation - axis * 0.003, axis)?;
        self.finish();
        self.phase = "observed_pick_approach".into();
        self.sim.serial_arms[0].grasp_target_body_id = Some(GEAR);
        for _ in 0..12 {
            let (tool, part) = self.observe()?;
            let delta = part.center.position_world_m - tool.world_from_tcp.translation;
            let distance = delta.norm();
            if distance < 8e-6 {
                break;
            }
            let step = delta * (0.0004 / distance).min(1.);
            self.optics
                .require_current_capture(self.sim.machine_command_sequence)
                .map_err(|e| e.to_string())?;
            self.move_to(
                cv(tool.world_from_tcp.translation + step),
                cv(part.axis_world),
            )?;
        }
        let (tool, part) = self.observe()?;
        if (part.center.position_world_m - tool.world_from_tcp.translation).norm() > 8e-6 {
            return Err("pick residual remains >8um".into());
        }
        self.finish();
        self.phase = "close_and_grasp".into();
        self.command(MachineCommand::SetGripperOpening {
            manipulator: ManipulatorId(1),
            target_opening_m: 0.001995,
        })?;
        self.sim
            .grasp_body_serial(ArmId(1), GEAR)
            .map_err(|e| format!("grasp:{e:?}"))?;
        self.finish();
        self.phase = "observed_lift".into();
        let (tool, part) = self.observe()?;
        let relation = part.center.position_world_m - tool.world_from_tcp.translation;
        let lift_target = tool.world_from_tcp.translation - ov(axis) * 0.001;
        self.move_to(cv(lift_target), cv(part.axis_world))?;
        let (tool, part) = self.observe()?;
        if (self.sim.serial_arms[0].gripper.held_body != Some(GEAR))
            || (part.center.position_world_m - tool.world_from_tcp.translation - relation).norm()
                > 20e-6
        {
            return Err("retention verification failed".into());
        }
        self.finish();
        self.phase = "hold_for_observation".into();
        for _ in 0..3 {
            let (tool, part) = self.observe()?;
            if (part.center.position_world_m - tool.world_from_tcp.translation - relation).norm()
                > 20e-6
                || self.sim.serial_arms[0].gripper.held_body != Some(GEAR)
            {
                return Err("hold retention verification failed".into());
            }
        }
        self.finish();
        self.phase = "observed_return".into();
        for _ in 0..8 {
            let (tool, part) = self.observe()?;
            let delta = ov(self.nest.translation) - part.center.position_world_m;
            let distance = delta.norm();
            if distance < 8e-6 {
                break;
            }
            self.move_to(
                cv(tool.world_from_tcp.translation + delta * (0.0004 / distance).min(1.)),
                axis,
            )?;
        }
        let (_, part) = self.observe()?;
        if (part.center.position_world_m - ov(self.nest.translation)).norm() > 20e-6
            || part.axis_world.dot(ov(axis)).clamp(-1., 1.).acos() > 0.003
        {
            return Err("observed supported placement pose rejected".into());
        }
        self.finish();
        self.phase = "supported_release".into();
        let support_ids = if self.report.fault == PickupFault::UnsupportedRelease {
            vec![BodyId(999999)]
        } else {
            SUPPORTS.to_vec()
        };
        self.sim
            .release_body_serial_on_support(ArmId(1), &support_ids, 20e-6)
            .map_err(|e| format!("support release:{e:?}"))?;
        self.command(MachineCommand::SetGripperOpening {
            manipulator: ManipulatorId(1),
            target_opening_m: self.cell.gripper.max_opening_m,
        })?;
        self.observe()?;
        self.finish();
        self.phase = "withdraw_and_verify".into();
        self.move_to(self.nest.translation - axis * 0.003, axis)?;
        let (_, part) = self.observe()?;
        if self.sim.serial_arms[0].gripper.held_body.is_some()
            || (part.center.position_world_m - ov(self.nest.translation)).norm() > 20e-6
        {
            return Err("release verification failed".into());
        }
        self.finish();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const MACHINE: &str = include_str!("../../../scenarios/machine_gear_pickup_v1.json");
    #[test]
    fn surface_marker_layout_observes_tool_and_gear_at_static_precontact() {
        // Static design check only: this does not contribute execution evidence.
        let mut op =
            prepare_gear_pickup(MACHINE, "test", &"1".repeat(64), PickupFault::None).unwrap();
        let axis = op.nest.transform_vector(Vec3::Z);
        let arm = &mut op.sim.serial_arms[0];
        let solution = arm
            .arm
            .solve_tool_axis(op.nest.translation - axis * 0.003, axis, arm.arm.positions)
            .unwrap();
        arm.arm.set_positions(solution.positions).unwrap();
        arm.motion = pipe_sim_core::ManipulatorMotionState::from_positions(solution.positions);
        arm.kinematics = arm.arm.forward_kinematics();
        op.sim
            .validate_serial_arm_current_clearance(ArmId(1))
            .unwrap();
        // Every printed center must lie on its intended substrate and outside
        // all other modeled physical geometry. No buried or floating markers.
        let mut bodies = op.sim.bodies.clone();
        bodies.extend(op.sim.serial_arm_collision_bodies());
        for model in op
            .optics
            .tools
            .iter()
            .chain(op.optics.body_fiducials.iter().map(|b| &b.fiducial))
        {
            let pose = if model.object_id == 1 {
                op.sim.serial_arms[0].tool_pose()
            } else {
                op.sim.body(GEAR).unwrap().pose
            };
            for feature in &model.features {
                let witness = RigidBody::new(
                    BodyId(12345),
                    Shape::Sphere { radius_m: 1e-9 },
                    Pose::from_translation(pose.transform_point(cv(feature.point_fiducial_m))),
                    MotionType::Static,
                );
                let mut on_surface = false;
                for body in &bodies {
                    if let Some(p) = pipe_sim_core::query_pair(&witness, body) {
                        assert!(
                            p.signed_distance_m >= -0.1e-6,
                            "marker {}/{} buried in body {} by {}m",
                            model.object_id,
                            feature.id,
                            body.id.0,
                            p.signed_distance_m
                        );
                        on_surface |= p.signed_distance_m.abs() < 0.1e-6;
                    }
                }
                assert!(
                    on_surface,
                    "marker {}/{} lacks modeled substrate",
                    model.object_id, feature.id
                );
            }
        }
        let result = op.observe();
        assert!(result.is_ok(), "{result:?}");
    }
    #[test]
    fn observation_faults_refuse_without_returning_a_truth_estimate() {
        for fault in [
            PickupFault::MissingPartObservation,
            PickupFault::StaleObservation,
        ] {
            let mut op = prepare_gear_pickup(MACHINE, "test", &"1".repeat(64), fault).unwrap();
            let axis = op.nest.transform_vector(Vec3::Z);
            let arm = &mut op.sim.serial_arms[0];
            let solution = arm
                .arm
                .solve_tool_axis(op.nest.translation - axis * 0.003, axis, arm.arm.positions)
                .unwrap();
            arm.arm.set_positions(solution.positions).unwrap();
            arm.motion = pipe_sim_core::ManipulatorMotionState::from_positions(solution.positions);
            arm.kinematics = arm.arm.forward_kinematics();
            let rejection = op.observe().unwrap_err();
            assert!(
                rejection.contains(if fault == PickupFault::MissingPartObservation {
                    "part_unobserved"
                } else {
                    "Stale"
                }),
                "{rejection}"
            );
            assert_eq!(op.sim.machine_command_sequence, 0);
        }
    }
    #[test]
    fn closed_grasp_static_layout_preserves_both_observations() {
        let mut op =
            prepare_gear_pickup(MACHINE, "test", &"1".repeat(64), PickupFault::None).unwrap();
        let axis = op.nest.transform_vector(Vec3::Z);
        let arm = &mut op.sim.serial_arms[0];
        let solution = arm
            .arm
            .solve_tool_axis(op.nest.translation, axis, arm.arm.positions)
            .unwrap();
        arm.arm.set_positions(solution.positions).unwrap();
        arm.motion = pipe_sim_core::ManipulatorMotionState::from_positions(solution.positions);
        arm.kinematics = arm.arm.forward_kinematics();
        arm.grasp_target_body_id = Some(GEAR);
        arm.gripper.opening_m = 0.001995;
        arm.gripper.command_opening_m = 0.001995;
        op.sim
            .validate_serial_arm_current_clearance(ArmId(1))
            .unwrap();
        op.sim.grasp_body_serial(ArmId(1), GEAR).unwrap();
        op.sim
            .submit_machine_command(MachineCommand::SetToolAxisTarget {
                manipulator: ManipulatorId(1),
                target_position_world_m: op.nest.translation - axis * 0.001,
                target_axis_world: axis,
            })
            .unwrap();
        // Admission only; the separate complete run supplies execution evidence.
        op.sim
            .submit_machine_command(MachineCommand::Stop { manipulator: None })
            .unwrap();
        let observation = op.observe();
        assert!(
            observation.is_ok(),
            "{observation:?}; visibility:{:?}",
            op.report
                .samples
                .last()
                .unwrap()
                .metrology
                .as_ref()
                .unwrap()
                .visibility
        );
    }
    #[test]
    fn timeout_refuses_and_executes_terminal_hold_with_truth_and_estimate_separate() {
        let r =
            run_gear_pickup(MACHINE, "test", &"1".repeat(64), PickupFault::MotionTimeout).unwrap();
        assert_eq!(r.status, "controlled_refusal");
        assert!(r.refusal.as_ref().unwrap().contains("motion timeout"));
        let last = r.samples.last().unwrap();
        assert_eq!(last.phase, "controlled_stop");
        assert!(last.scene_frame.estimate.is_none());
        assert!(last
            .scene_frame
            .commanded
            .manipulators
            .iter()
            .all(|a| a.stopped));
        for arm in &last.scene_frame.truth.as_ref().unwrap().manipulators {
            assert!(arm.joint_velocities_rad_s.iter().all(|v| v.abs() < 1e-12));
            assert!(arm.carriage_z_velocity_m_s.abs() < 1e-12);
            assert!(arm.carriage_theta_velocity_rad_s.abs() < 1e-12);
        }
        assert_eq!(last.support_gaps_m.len(), 3);
        assert_eq!(r.gear_outer_diameter_m, 0.002);
        assert_eq!(r.gear_thickness_m, 0.0008);
        assert!(!r.completed_phases.iter().any(|p| p == "close_and_grasp"));
    }
}
