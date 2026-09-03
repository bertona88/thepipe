//! M1c deterministic single-arm calibration-peg manipulation runtime.

use pipe_optics::StructuredLightRig;
use pipe_sim_core::{
    query_pair, ArmId, BodyId, CollisionFilter, MachineCommand, ManipulatorId, MotionType,
    PipeCellConfig, Pose, RigidBody, Shape, Simulation, ToolMotionStatus, Vec3,
};
use serde::Serialize;

use crate::{
    build_optics, machine_config, scene, serialize_json, SceneDescription, SceneFrame, SimError,
};

pub const SIMPLE_MANIPULATION_REPORT_SCHEMA_VERSION: u32 = 1;
pub const CALIBRATION_PEG_BODY_ID: u32 = 10_001;
pub const CALIBRATION_PICK_APPROACH_WORLD_M: [f64; 3] = [22.0e-3, 0.0, -6.0e-3];
pub const CALIBRATION_PICK_WORLD_M: [f64; 3] = [20.0e-3, 0.0, -6.0e-3];
pub const CALIBRATION_INSERT_WORLD_M: [f64; 3] = [20.0e-3, 0.0, 6.0e-3];
pub const CALIBRATION_INSERT_APPROACH_DISTANCE_M: f64 = 2.0e-3;

const ACTIVE_ARM_ID: ArmId = ArmId(1);
const ACTIVE_MANIPULATOR_ID: ManipulatorId = ManipulatorId(1);
const PEG_RADIUS_M: f64 = 0.200e-3;
const PEG_HALF_SEGMENT_M: f64 = 0.350e-3;
const GRIP_COMPRESSION_M: f64 = 12.0e-6;
const SOCKET_INNER_HALF_WIDTH_M: f64 = 0.325e-3;
const SOCKET_WALL_THICKNESS_M: f64 = 0.200e-3;
const SOCKET_HALF_DEPTH_M: f64 = 0.800e-3;
const PEG_COLLISION_GROUP: u32 = 0b0010;
const SOCKET_COLLISION_GROUP: u32 = 0b0100;
const SOCKET_BODY_IDS: [BodyId; 4] = [
    BodyId(10_002),
    BodyId(10_003),
    BodyId(10_004),
    BodyId(10_005),
];
const ACTION_SEQUENCE: [&str; 10] = [
    "open_gripper",
    "pick_approach",
    "pick",
    "close_gripper",
    "grasp",
    "retract",
    "transfer",
    "insert",
    "release",
    "retreat",
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ManipulationPhase {
    Ready,
    OpenGripper,
    PickApproach,
    Pick,
    CloseGripper,
    Grasp,
    Retract,
    Transfer,
    Insert,
    Release,
    Retreat,
    Complete,
    Aborted,
}

impl ManipulationPhase {
    const fn name(self) -> &'static str {
        match self {
            Self::Ready => "ready",
            Self::OpenGripper => "open_gripper",
            Self::PickApproach => "pick_approach",
            Self::Pick => "pick",
            Self::CloseGripper => "close_gripper",
            Self::Grasp => "grasp",
            Self::Retract => "retract",
            Self::Transfer => "transfer",
            Self::Insert => "insert",
            Self::Release => "release",
            Self::Retreat => "retreat",
            Self::Complete => "complete",
            Self::Aborted => "aborted",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ManipulationTraceRecord {
    pub index: u32,
    pub phase: &'static str,
    pub tick: u64,
    pub time_s: f64,
    pub command_sequence: u64,
    pub target_position_world_m: Option<[f64; 3]>,
    pub tool_position_world_m: [f64; 3],
    pub tool_position_error_m: Option<f64>,
    pub peg_position_world_m: [f64; 3],
    pub gripper_opening_m: f64,
    pub estimated_grip_force_n: f64,
    pub held_body_id: Option<u32>,
    pub maximum_unplanned_penetration_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SimpleManipulationReport {
    pub schema_version: u32,
    pub fidelity: &'static str,
    pub machine_config_id: String,
    pub machine_config_sha256: String,
    pub status: &'static str,
    pub phase: &'static str,
    pub failure_reason: Option<String>,
    pub sequence: [&'static str; 10],
    pub manipulator_id: u32,
    pub peg_body_id: u32,
    pub held_part_pose_source: &'static str,
    pub truth_available: bool,
    pub estimate_available: bool,
    pub grasp_observed: bool,
    pub release_observed: bool,
    pub peak_estimated_grip_force_n: f64,
    pub maximum_unplanned_penetration_m: f64,
    pub maximum_planned_socket_penetration_m: f64,
    pub final_peg_pose: scene::PoseSnapshot,
    pub socket_target_pose: scene::PoseSnapshot,
    pub final_socket_lateral_error_m: f64,
    pub final_socket_axial_error_m: f64,
    pub final_socket_axis_error_rad: f64,
    pub final_socket_minimum_clearance_m: f64,
    pub trace: Vec<ManipulationTraceRecord>,
}

impl SimpleManipulationReport {
    pub fn to_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(self, pretty)
    }
}

#[derive(Clone, Copy, Debug)]
struct InsertionMetrics {
    lateral_error_m: f64,
    axial_error_m: f64,
    axis_error_rad: f64,
    minimum_clearance_m: f64,
}

/// Standalone M1c plant. It deliberately uses configured target datums rather
/// than an estimator; `SceneFrame::estimate` remains empty until M3.
pub struct SimpleManipulationRuntime {
    machine_config_id: String,
    machine_config_sha256: String,
    cell_config: PipeCellConfig,
    mechanics: Simulation,
    optics: StructuredLightRig,
    body_names: Vec<(u32, String)>,
    socket_target_pose: Pose,
    phase: ManipulationPhase,
    current_target: Option<Vec3>,
    failure_reason: Option<String>,
    trace: Vec<ManipulationTraceRecord>,
    peak_estimated_grip_force_n: f64,
    maximum_unplanned_penetration_m: f64,
    maximum_planned_socket_penetration_m: f64,
    grasp_observed: bool,
    release_observed: bool,
}

impl SimpleManipulationRuntime {
    pub fn new() -> Result<Self, SimError> {
        let loaded = machine_config::load_baseline_machine_config()?;
        let mut mechanics = machine_config::build_baseline_machine(&loaded)?;
        let pick_target = array_vec3(CALIBRATION_PICK_WORLD_M);
        let insert_target = array_vec3(CALIBRATION_INSERT_WORLD_M);
        let pick_tool_pose = solved_tool_pose(&mechanics, pick_target)?;
        let socket_target_pose = solved_tool_pose(&mechanics, insert_target)?;

        let mut peg = RigidBody::new(
            BodyId(CALIBRATION_PEG_BODY_ID),
            Shape::Capsule {
                radius_m: PEG_RADIUS_M,
                half_segment_m: PEG_HALF_SEGMENT_M,
            },
            Pose::new(pick_target, pick_tool_pose.rotation),
            MotionType::Dynamic,
        );
        peg.collision_filter = CollisionFilter {
            group: PEG_COLLISION_GROUP,
            mask: SOCKET_COLLISION_GROUP,
        };
        mechanics
            .add_body(peg)
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
        add_socket_coupon(&mut mechanics, socket_target_pose)?;

        let mut runtime = Self {
            machine_config_id: loaded.id,
            machine_config_sha256: loaded.source_sha256,
            cell_config: loaded.cell,
            mechanics,
            optics: build_optics(0),
            body_names: vec![
                (CALIBRATION_PEG_BODY_ID, "calibration_peg".to_owned()),
                (SOCKET_BODY_IDS[0].0, "calibration_socket_left".to_owned()),
                (SOCKET_BODY_IDS[1].0, "calibration_socket_right".to_owned()),
                (SOCKET_BODY_IDS[2].0, "calibration_socket_lower".to_owned()),
                (SOCKET_BODY_IDS[3].0, "calibration_socket_upper".to_owned()),
            ],
            socket_target_pose,
            phase: ManipulationPhase::Ready,
            current_target: None,
            failure_reason: None,
            trace: Vec::new(),
            peak_estimated_grip_force_n: 0.0,
            maximum_unplanned_penetration_m: 0.0,
            maximum_planned_socket_penetration_m: 0.0,
            grasp_observed: false,
            release_observed: false,
        };
        runtime.record(ManipulationPhase::Ready, None);
        Ok(runtime)
    }

    /// Execute the bounded M1c action sequence. Expected plant failures return
    /// an `aborted` report rather than an incomplete success or a panic.
    pub fn run_cycle(
        &mut self,
        max_steps_per_action: u32,
    ) -> Result<SimpleManipulationReport, SimError> {
        if self.is_terminal() {
            return Ok(self.report());
        }
        let outcome = if max_steps_per_action == 0 {
            Err("max_steps_per_action_must_be_positive".to_owned())
        } else {
            self.execute_cycle(max_steps_per_action)
        };
        if let Err(reason) = outcome {
            self.failure_reason = Some(reason);
            self.phase = ManipulationPhase::Aborted;
            self.record(ManipulationPhase::Aborted, self.current_target);
        }
        Ok(self.report())
    }

    fn execute_cycle(&mut self, max_steps: u32) -> Result<(), String> {
        self.command_gripper(self.cell_config.gripper.max_opening_m, max_steps)?;
        self.finish_phase(ManipulationPhase::OpenGripper, None);

        self.move_tool(array_vec3(CALIBRATION_PICK_APPROACH_WORLD_M), max_steps)?;
        self.finish_phase(
            ManipulationPhase::PickApproach,
            Some(array_vec3(CALIBRATION_PICK_APPROACH_WORLD_M)),
        );

        self.move_tool(array_vec3(CALIBRATION_PICK_WORLD_M), max_steps)?;
        self.finish_phase(
            ManipulationPhase::Pick,
            Some(array_vec3(CALIBRATION_PICK_WORLD_M)),
        );

        self.command_gripper(2.0 * PEG_RADIUS_M - GRIP_COMPRESSION_M, max_steps)?;
        self.finish_phase(ManipulationPhase::CloseGripper, None);

        self.mechanics
            .grasp_body_serial(ACTIVE_ARM_ID, BodyId(CALIBRATION_PEG_BODY_ID))
            .map_err(|error| format!("grasp_rejected:{error:?}"))?;
        let grip_force_n = self.active_arm().gripper.estimated_grip_force_n;
        if grip_force_n <= 0.0 || grip_force_n > self.cell_config.gripper.max_grip_force_n {
            return Err("grasp_force_out_of_bounds".to_owned());
        }
        self.peak_estimated_grip_force_n = self.peak_estimated_grip_force_n.max(grip_force_n);
        self.grasp_observed = true;
        self.finish_phase(
            ManipulationPhase::Grasp,
            Some(array_vec3(CALIBRATION_PICK_WORLD_M)),
        );

        self.move_tool(array_vec3(CALIBRATION_PICK_APPROACH_WORLD_M), max_steps)?;
        self.require_peg_held()?;
        self.finish_phase(
            ManipulationPhase::Retract,
            Some(array_vec3(CALIBRATION_PICK_APPROACH_WORLD_M)),
        );

        // Follow the socket's local -Z axis. A world-axis offset would approach
        // this tilted coupon laterally and correctly fail carried-body
        // collision preflight before entering the opening.
        let insert_approach = self.insert_approach_target();
        self.move_tool(insert_approach, max_steps)?;
        self.require_peg_held()?;
        self.finish_phase(ManipulationPhase::Transfer, Some(insert_approach));

        self.move_tool(array_vec3(CALIBRATION_INSERT_WORLD_M), max_steps)?;
        self.require_peg_held()?;
        self.validate_inserted_pose()?;
        self.finish_phase(
            ManipulationPhase::Insert,
            Some(array_vec3(CALIBRATION_INSERT_WORLD_M)),
        );

        self.command_gripper(self.cell_config.gripper.max_opening_m, max_steps)?;
        if self.active_arm().gripper.held_body.is_some() {
            return Err("release_not_observed_after_opening".to_owned());
        }
        self.release_observed = true;
        self.finish_phase(ManipulationPhase::Release, None);

        self.move_tool(insert_approach, max_steps)?;
        self.validate_inserted_pose()?;
        self.finish_phase(ManipulationPhase::Retreat, Some(insert_approach));

        self.phase = ManipulationPhase::Complete;
        self.current_target = None;
        self.record(ManipulationPhase::Complete, None);
        Ok(())
    }

    fn move_tool(&mut self, target: Vec3, max_steps: u32) -> Result<(), String> {
        self.current_target = Some(target);
        self.mechanics
            .submit_machine_command(MachineCommand::SetToolPoseTarget {
                manipulator: ACTIVE_MANIPULATOR_ID,
                target_position_world_m: target,
            })
            .map_err(|error| format!("tool_command_rejected:{error:?}"))?;
        for _ in 0..max_steps {
            if !self.tool_motion_active() {
                break;
            }
            self.step_checked()?;
        }
        let arm = self.active_arm();
        let Some(plan) = arm.motion.tool_motion else {
            return Err("tool_plan_missing".to_owned());
        };
        if plan.status != ToolMotionStatus::Complete {
            return Err(format!("tool_motion_timeout_after_{max_steps}_steps"));
        }
        let error_m = (arm.tool_pose().translation - target).length();
        if error_m > 1.0e-9 {
            return Err(format!("tool_position_error_m={error_m:.9e}"));
        }
        Ok(())
    }

    fn command_gripper(&mut self, target_opening_m: f64, max_steps: u32) -> Result<(), String> {
        self.current_target = None;
        self.mechanics
            .submit_machine_command(MachineCommand::SetGripperOpening {
                manipulator: ACTIVE_MANIPULATOR_ID,
                target_opening_m,
            })
            .map_err(|error| format!("gripper_command_rejected:{error:?}"))?;
        for _ in 0..max_steps {
            let gripper = self.active_arm().gripper;
            if (gripper.opening_m - target_opening_m).abs() <= 1.0e-12
                && gripper.opening_velocity_m_s.abs() <= 1.0e-12
            {
                return Ok(());
            }
            self.step_checked()?;
        }
        Err(format!("gripper_motion_timeout_after_{max_steps}_steps"))
    }

    fn step_checked(&mut self) -> Result<(), String> {
        self.mechanics
            .step()
            .map_err(|error| format!("mechanics_step_failed:{error:?}"))?;
        let collision = self
            .mechanics
            .query_collisions_with_arms(self.mechanics.config.collision);
        for contact in collision.contacts {
            if is_planned_socket_pair(contact.body_a, contact.body_b) {
                self.maximum_planned_socket_penetration_m = self
                    .maximum_planned_socket_penetration_m
                    .max(contact.penetration_depth_m);
            } else {
                self.maximum_unplanned_penetration_m = self
                    .maximum_unplanned_penetration_m
                    .max(contact.penetration_depth_m);
                return Err(format!(
                    "unplanned_contact_between_{}_{}",
                    contact.body_a.0, contact.body_b.0
                ));
            }
        }
        self.peak_estimated_grip_force_n = self
            .peak_estimated_grip_force_n
            .max(self.active_arm().gripper.estimated_grip_force_n);
        Ok(())
    }

    fn require_peg_held(&self) -> Result<(), String> {
        if self.active_arm().gripper.held_body == Some(BodyId(CALIBRATION_PEG_BODY_ID)) {
            Ok(())
        } else {
            Err("peg_ownership_lost".to_owned())
        }
    }

    fn validate_inserted_pose(&self) -> Result<(), String> {
        let metrics = self.insertion_metrics();
        if metrics.lateral_error_m > 1.0e-9
            || metrics.axial_error_m > 1.0e-9
            || metrics.axis_error_rad > 1.0e-9
        {
            return Err(format!(
                "insert_pose_out_of_tolerance:lateral={:.9e},axial={:.9e},axis={:.9e}",
                metrics.lateral_error_m, metrics.axial_error_m, metrics.axis_error_rad
            ));
        }
        if metrics.minimum_clearance_m <= self.mechanics.config.collision.clearance_threshold_m {
            return Err(format!(
                "socket_clearance_m={:.9e}_does_not_exceed_preflight_threshold_m={:.9e}",
                metrics.minimum_clearance_m, self.mechanics.config.collision.clearance_threshold_m
            ));
        }
        Ok(())
    }

    fn finish_phase(&mut self, phase: ManipulationPhase, target: Option<Vec3>) {
        self.phase = phase;
        self.current_target = target;
        self.record(phase, target);
    }

    fn record(&mut self, phase: ManipulationPhase, target: Option<Vec3>) {
        let arm = self.active_arm();
        let tool_position = arm.tool_pose().translation;
        let peg_position = self.peg_body().pose.translation;
        let record = ManipulationTraceRecord {
            index: self.trace.len() as u32,
            phase: phase.name(),
            tick: self.mechanics.step_index,
            time_s: self.mechanics.time_s,
            command_sequence: self.mechanics.machine_command_sequence,
            target_position_world_m: target.map(vec3),
            tool_position_world_m: vec3(tool_position),
            tool_position_error_m: target.map(|target| (tool_position - target).length()),
            peg_position_world_m: vec3(peg_position),
            gripper_opening_m: arm.gripper.opening_m,
            estimated_grip_force_n: arm.gripper.estimated_grip_force_n,
            held_body_id: arm.gripper.held_body.map(|body_id| body_id.0),
            maximum_unplanned_penetration_m: self.maximum_unplanned_penetration_m,
        };
        self.trace.push(record);
    }

    fn insertion_metrics(&self) -> InsertionMetrics {
        let peg = self.peg_body();
        let local_position = self
            .socket_target_pose
            .inverse_transform_point(peg.pose.translation);
        let peg_axis = peg.pose.transform_vector(Vec3::Z).normalized_or(Vec3::Z);
        let socket_axis = self
            .socket_target_pose
            .transform_vector(Vec3::Z)
            .normalized_or(Vec3::Z);
        let axis_error_rad = peg_axis
            .cross(socket_axis)
            .length()
            .atan2(peg_axis.dot(socket_axis).clamp(-1.0, 1.0));
        let minimum_clearance_m = SOCKET_BODY_IDS
            .iter()
            .filter_map(|body_id| {
                self.mechanics
                    .body(*body_id)
                    .and_then(|wall| query_pair(peg, wall))
                    .map(|proximity| proximity.signed_distance_m)
            })
            .fold(f64::INFINITY, f64::min);
        InsertionMetrics {
            lateral_error_m: local_position.x.hypot(local_position.y),
            axial_error_m: local_position.z.abs(),
            axis_error_rad,
            minimum_clearance_m,
        }
    }

    fn insert_approach_target(&self) -> Vec3 {
        self.socket_target_pose.translation
            - self.socket_target_pose.transform_vector(Vec3::Z)
                * CALIBRATION_INSERT_APPROACH_DISTANCE_M
    }

    fn active_arm(&self) -> &pipe_sim_core::SerialArmInstance {
        self.mechanics
            .serial_arm(ACTIVE_ARM_ID)
            .expect("M1c runtime owns active arm 1")
    }

    fn peg_body(&self) -> &RigidBody {
        self.mechanics
            .body(BodyId(CALIBRATION_PEG_BODY_ID))
            .expect("M1c runtime owns the calibration peg")
    }

    fn tool_motion_active(&self) -> bool {
        self.active_arm()
            .motion
            .tool_motion
            .is_some_and(|plan| plan.status == ToolMotionStatus::Active)
    }

    pub fn is_terminal(&self) -> bool {
        matches!(
            self.phase,
            ManipulationPhase::Complete | ManipulationPhase::Aborted
        )
    }

    pub fn is_completed(&self) -> bool {
        self.phase == ManipulationPhase::Complete
    }

    pub fn scene_description(&self) -> SceneDescription {
        scene::build_scene_description(
            &self.machine_config_id,
            &self.machine_config_sha256,
            self.cell_config,
            &self.mechanics,
            &self.optics,
            &self.body_names,
        )
    }

    pub fn scene_frame(&self) -> SceneFrame {
        let collision = self
            .mechanics
            .query_collisions_with_arms(self.mechanics.config.collision);
        scene::build_scene_frame(&self.mechanics, &collision.contacts)
    }

    pub fn scene_description_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(&self.scene_description(), pretty)
    }

    pub fn scene_frame_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(&self.scene_frame(), pretty)
    }

    pub fn report(&self) -> SimpleManipulationReport {
        let metrics = self.insertion_metrics();
        let status = match self.phase {
            ManipulationPhase::Ready => "ready",
            ManipulationPhase::Complete => "complete",
            ManipulationPhase::Aborted => "aborted",
            _ => "running",
        };
        let frame = self.scene_frame();
        SimpleManipulationReport {
            schema_version: SIMPLE_MANIPULATION_REPORT_SCHEMA_VERSION,
            fidelity: "F0_geometry_M1c_simulation_baseline_not_hardware_qualified",
            machine_config_id: self.machine_config_id.clone(),
            machine_config_sha256: self.machine_config_sha256.clone(),
            status,
            phase: self.phase.name(),
            failure_reason: self.failure_reason.clone(),
            sequence: ACTION_SEQUENCE,
            manipulator_id: ACTIVE_MANIPULATOR_ID.0,
            peg_body_id: CALIBRATION_PEG_BODY_ID,
            held_part_pose_source: "plant_tool_attachment",
            truth_available: frame.truth.is_some(),
            estimate_available: frame.estimate.is_some(),
            grasp_observed: self.grasp_observed,
            release_observed: self.release_observed,
            peak_estimated_grip_force_n: self.peak_estimated_grip_force_n,
            maximum_unplanned_penetration_m: self.maximum_unplanned_penetration_m,
            maximum_planned_socket_penetration_m: self.maximum_planned_socket_penetration_m,
            final_peg_pose: self.peg_body().pose.into(),
            socket_target_pose: self.socket_target_pose.into(),
            final_socket_lateral_error_m: metrics.lateral_error_m,
            final_socket_axial_error_m: metrics.axial_error_m,
            final_socket_axis_error_rad: metrics.axis_error_rad,
            final_socket_minimum_clearance_m: metrics.minimum_clearance_m,
            trace: self.trace.clone(),
        }
    }
}

impl Default for SimpleManipulationRuntime {
    fn default() -> Self {
        Self::new().expect("embedded M1c machine and coupon configuration is valid")
    }
}

fn solved_tool_pose(mechanics: &Simulation, target: Vec3) -> Result<Pose, SimError> {
    let instance = mechanics
        .serial_arm(ACTIVE_ARM_ID)
        .ok_or_else(|| SimError::Mechanics("active arm 1 missing".to_owned()))?;
    let solution = instance
        .arm
        .solve_tool_position(target, instance.motion.positions())
        .map_err(|error| SimError::Mechanics(format!("M1c target IK failed: {error:?}")))?;
    let mut candidate = instance.arm.clone();
    candidate
        .set_positions(solution.positions)
        .map_err(|error| SimError::Mechanics(format!("M1c target FK failed: {error:?}")))?;
    Ok(candidate.forward_kinematics().tool_pose)
}

fn add_socket_coupon(mechanics: &mut Simulation, socket_pose: Pose) -> Result<(), SimError> {
    let outer_half_width_m = SOCKET_INNER_HALF_WIDTH_M + SOCKET_WALL_THICKNESS_M;
    let wall_center_m = SOCKET_INNER_HALF_WIDTH_M + 0.5 * SOCKET_WALL_THICKNESS_M;
    let definitions = [
        (
            SOCKET_BODY_IDS[0],
            Vec3::new(-wall_center_m, 0.0, 0.0),
            Vec3::new(
                0.5 * SOCKET_WALL_THICKNESS_M,
                outer_half_width_m,
                SOCKET_HALF_DEPTH_M,
            ),
        ),
        (
            SOCKET_BODY_IDS[1],
            Vec3::new(wall_center_m, 0.0, 0.0),
            Vec3::new(
                0.5 * SOCKET_WALL_THICKNESS_M,
                outer_half_width_m,
                SOCKET_HALF_DEPTH_M,
            ),
        ),
        (
            SOCKET_BODY_IDS[2],
            Vec3::new(0.0, -wall_center_m, 0.0),
            Vec3::new(
                SOCKET_INNER_HALF_WIDTH_M,
                0.5 * SOCKET_WALL_THICKNESS_M,
                SOCKET_HALF_DEPTH_M,
            ),
        ),
        (
            SOCKET_BODY_IDS[3],
            Vec3::new(0.0, wall_center_m, 0.0),
            Vec3::new(
                SOCKET_INNER_HALF_WIDTH_M,
                0.5 * SOCKET_WALL_THICKNESS_M,
                SOCKET_HALF_DEPTH_M,
            ),
        ),
    ];
    for (body_id, local_center, half_extents_m) in definitions {
        let mut wall = RigidBody::new(
            body_id,
            Shape::Box { half_extents_m },
            socket_pose * Pose::from_translation(local_center),
            MotionType::Static,
        );
        wall.collision_filter = CollisionFilter {
            group: SOCKET_COLLISION_GROUP,
            mask: PEG_COLLISION_GROUP,
        };
        mechanics
            .add_body(wall)
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
    }
    Ok(())
}

fn is_planned_socket_pair(body_a: BodyId, body_b: BodyId) -> bool {
    let (a, b) = if body_a <= body_b {
        (body_a, body_b)
    } else {
        (body_b, body_a)
    };
    a == BodyId(CALIBRATION_PEG_BODY_ID) && SOCKET_BODY_IDS.contains(&b)
}

fn array_vec3(value: [f64; 3]) -> Vec3 {
    Vec3::new(value[0], value[1], value[2])
}

fn vec3(value: Vec3) -> [f64; 3] {
    [value.x, value.y, value.z]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn m1c_cycle_grasps_carries_inserts_releases_and_retreats() {
        let mut runtime = SimpleManipulationRuntime::new().unwrap();
        let report = runtime.run_cycle(20_000).unwrap();

        assert_eq!(report.status, "complete", "{report:#?}");
        assert_eq!(report.phase, "complete");
        assert_eq!(report.failure_reason, None);
        assert!(report.grasp_observed);
        assert!(report.release_observed);
        assert!(report.peak_estimated_grip_force_n > 0.0);
        assert!(report.peak_estimated_grip_force_n <= runtime.cell_config.gripper.max_grip_force_n);
        assert_eq!(report.maximum_unplanned_penetration_m, 0.0);
        assert_eq!(report.maximum_planned_socket_penetration_m, 0.0);
        assert!(report.final_socket_lateral_error_m < 1.0e-9);
        assert!(report.final_socket_axial_error_m < 1.0e-9);
        assert!(report.final_socket_axis_error_rad < 1.0e-9);
        assert!(
            report.final_socket_minimum_clearance_m
                > runtime.mechanics.config.collision.clearance_threshold_m
        );
        assert!(!report.estimate_available);
        assert_eq!(runtime.active_arm().gripper.held_body, None);
        assert_eq!(report.trace.first().unwrap().phase, "ready");
        assert_eq!(report.trace.last().unwrap().phase, "complete");
        assert!(report.trace.iter().any(|record| record.phase == "grasp"
            && record.held_body_id == Some(CALIBRATION_PEG_BODY_ID)));
        assert!(report
            .trace
            .iter()
            .any(|record| record.phase == "release" && record.held_body_id.is_none()));
    }

    #[test]
    fn m1c_cycle_is_deterministically_replayable() {
        let mut first = SimpleManipulationRuntime::new().unwrap();
        let first = first.run_cycle(20_000).unwrap();
        let mut second = SimpleManipulationRuntime::new().unwrap();
        let second = second.run_cycle(20_000).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn off_center_peg_aborts_without_claiming_a_grasp() {
        let mut runtime = SimpleManipulationRuntime::new().unwrap();
        let peg = runtime
            .mechanics
            .body_mut(BodyId(CALIBRATION_PEG_BODY_ID))
            .unwrap();
        peg.pose.translation += peg.pose.transform_vector(Vec3::X) * 50.0e-6;

        let report = runtime.run_cycle(20_000).unwrap();
        assert_eq!(report.status, "aborted");
        assert!(report
            .failure_reason
            .as_deref()
            .is_some_and(|reason| reason.starts_with("grasp_rejected:")));
        assert!(!report.grasp_observed);
        assert!(!report.release_observed);
        assert_eq!(runtime.active_arm().gripper.held_body, None);
        assert_eq!(report.trace.last().unwrap().phase, "aborted");
    }

    #[test]
    fn scene_names_coupon_bodies_and_keeps_estimate_empty() {
        let runtime = SimpleManipulationRuntime::new().unwrap();
        let description = runtime.scene_description();
        assert!(description
            .rigid_bodies
            .iter()
            .any(|body| body.id == CALIBRATION_PEG_BODY_ID && body.name == "calibration_peg"));
        let frame = runtime.scene_frame();
        assert!(frame.truth.is_some());
        assert!(frame.estimate.is_none());
    }

    #[test]
    fn insertion_approach_is_axial_in_the_socket_frame() {
        let runtime = SimpleManipulationRuntime::new().unwrap();
        let local_approach = runtime
            .socket_target_pose
            .inverse_transform_point(runtime.insert_approach_target());

        assert!(local_approach.x.abs() < 1.0e-12);
        assert!(local_approach.y.abs() < 1.0e-12);
        assert!((local_approach.z + CALIBRATION_INSERT_APPROACH_DISTANCE_M).abs() < 1.0e-12);
    }
}
