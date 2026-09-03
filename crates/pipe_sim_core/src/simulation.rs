//! Fixed-step deterministic simulation state and simple impulse solver.

use crate::arm::{ArmError, ArmKinematics, ContinuumArm};
use crate::collision::{query_pair, Clearance, CollisionReport, CollisionSettings, Contact};
use crate::geometry::{BodyId, MotionType, RigidBody};
use crate::gripper::{GripperConfig, GripperState};
use crate::machine::{
    wrap_angle_pi, CarriageConfig, MachineBackend, MachineCommand, MachineCommandError,
    MachineCommandEvent, ManipulatorMotionConfig, ManipulatorMotionState, ToolMotionPlan,
    ToolMotionStatus,
};
use crate::math::{Pose, Quat, Vec3};
use crate::serial_arm::{
    SerialArm, SerialArmError, SerialArmKinematics, ToolPositionIkError,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ArmId(pub u32);

/// IDs at or above this value are reserved for ephemeral serial-arm link
/// colliders returned by [`Simulation::serial_arm_collision_bodies`].
pub const SERIAL_ARM_COLLISION_BODY_ID_BASE: u32 = 0xC000_0000;
const MAX_SERIAL_ARM_COLLISION_ID: u32 = 0x0FFF_FFFF;
const SERIAL_ARM_LINK_COUNT: u8 = 3;
const TOOL_PATH_MAX_ANGULAR_STEP_RAD: f64 = core::f64::consts::PI / 180.0;
const TOOL_PATH_MAX_LINEAR_STEP_M: f64 = 0.25e-3;
const TOOL_PATH_MAX_SAMPLE_COUNT: usize = 4_096;
const SMOOTHSTEP_MAX_SLOPE: f64 = 1.5;

/// Stable mapping from a serial arm and physical link index to its reserved
/// collision ID. Link indices 0, 1, and 2 identify upper arm, forearm, and
/// wrist respectively.
pub fn serial_arm_link_body_id(arm_id: ArmId, link_index: u8) -> Option<BodyId> {
    if arm_id.0 > MAX_SERIAL_ARM_COLLISION_ID || link_index >= SERIAL_ARM_LINK_COUNT {
        return None;
    }
    Some(BodyId(
        SERIAL_ARM_COLLISION_BODY_ID_BASE | (arm_id.0 << 2) | link_index as u32,
    ))
}

fn serial_arm_link_key(body_id: BodyId) -> Option<(ArmId, u8)> {
    if body_id.0 < SERIAL_ARM_COLLISION_BODY_ID_BASE {
        return None;
    }
    let encoded = body_id.0 - SERIAL_ARM_COLLISION_BODY_ID_BASE;
    let link_index = (encoded & 0b11) as u8;
    if link_index >= SERIAL_ARM_LINK_COUNT {
        return None;
    }
    Some((ArmId(encoded >> 2), link_index))
}

#[derive(Clone, Debug, PartialEq)]
pub struct ArmInstance {
    pub id: ArmId,
    pub arm: ContinuumArm,
    pub gripper_config: GripperConfig,
    pub gripper: GripperState,
    pub external_tendon_loads_n: Vec<[f64; 3]>,
    pub kinematics: ArmKinematics,
    /// Tool-frame pose of the held body's origin.
    pub held_body_local_pose: Option<Pose>,
}

impl ArmInstance {
    pub fn new(
        id: ArmId,
        arm: ContinuumArm,
        gripper_config: GripperConfig,
    ) -> Result<Self, SimulationError> {
        if !gripper_config.is_valid() {
            return Err(SimulationError::InvalidGripperConfig);
        }
        let kinematics = arm.forward_kinematics();
        let external_tendon_loads_n = vec![[0.0; 3]; arm.segments.len()];
        Ok(Self {
            id,
            arm,
            gripper_config,
            gripper: GripperState::new(gripper_config.max_opening_m, gripper_config),
            external_tendon_loads_n,
            kinematics,
            held_body_local_pose: None,
        })
    }

    pub fn tool_pose(&self) -> Pose {
        self.kinematics.tip_pose
    }

    fn step(&mut self, dt_s: f64) -> Result<(), ArmError> {
        self.arm
            .step_actuators(dt_s, &self.external_tendon_loads_n)?;
        self.gripper.step(dt_s, self.gripper_config);
        self.kinematics = self.arm.forward_kinematics();
        Ok(())
    }
}

/// Fixed-step wrapper for the reference rigid serial arm. Tendon/rail state is
/// set through [`SerialArm`]; a simulation step updates its gripper and caches
/// forward kinematics before collision and grasp attachment processing.
#[derive(Clone, Debug, PartialEq)]
pub struct SerialArmInstance {
    pub id: ArmId,
    pub arm: SerialArm,
    pub gripper_config: GripperConfig,
    pub gripper: GripperState,
    pub kinematics: SerialArmKinematics,
    pub held_body_local_pose: Option<Pose>,
    /// Authoritative carriage and bounded joint motion. `arm.positions` is the
    /// FK/tendon projection of this state, retained for the local arm model.
    pub motion: ManipulatorMotionState,
    pub carriage_config: CarriageConfig,
    pub motion_config: ManipulatorMotionConfig,
    /// Scale applied when time-parameterizing Cartesian plans. Direct axis
    /// commands retain their configured limits.
    pub tool_motion_speed_scale: f64,
}

impl SerialArmInstance {
    pub fn new(
        id: ArmId,
        arm: SerialArm,
        gripper_config: GripperConfig,
    ) -> Result<Self, SimulationError> {
        if !gripper_config.is_valid() {
            return Err(SimulationError::InvalidGripperConfig);
        }
        let kinematics = arm.forward_kinematics();
        let motion = ManipulatorMotionState::from_positions(arm.positions);
        let carriage_config = CarriageConfig::from_serial_arm(arm.config);
        Ok(Self {
            id,
            arm,
            gripper_config,
            gripper: GripperState::new(gripper_config.max_opening_m, gripper_config),
            kinematics,
            held_body_local_pose: None,
            motion,
            carriage_config,
            motion_config: ManipulatorMotionConfig::default(),
            tool_motion_speed_scale: 1.0,
        })
    }

    pub fn tool_pose(&self) -> Pose {
        self.kinematics.tool_pose
    }

    fn step(&mut self, dt_s: f64) -> Result<(), SerialArmError> {
        self.motion.step(
            dt_s,
            self.carriage_config,
            self.arm.config,
            self.motion_config,
        );
        self.arm.set_positions(self.motion.positions())?;
        self.gripper.step(dt_s, self.gripper_config);
        self.kinematics = self.arm.forward_kinematics();
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimulationConfig {
    pub fixed_dt_s: f64,
    pub gravity_m_s2: Vec3,
    pub linear_damping_per_s: f64,
    pub angular_damping_per_s: f64,
    pub solver_iterations: u16,
    pub penetration_slop_m: f64,
    pub positional_correction_fraction: f64,
    pub collision: CollisionSettings,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            fixed_dt_s: 0.0005,
            gravity_m_s2: Vec3::new(0.0, 0.0, -9.80665),
            linear_damping_per_s: 1.5,
            angular_damping_per_s: 2.0,
            solver_iterations: 6,
            penetration_slop_m: 0.5e-6,
            positional_correction_fraction: 0.7,
            collision: CollisionSettings::default(),
        }
    }
}

impl SimulationConfig {
    pub fn is_valid(self) -> bool {
        self.fixed_dt_s > 0.0
            && self.fixed_dt_s.is_finite()
            && self.gravity_m_s2.is_finite()
            && self.linear_damping_per_s >= 0.0
            && self.angular_damping_per_s >= 0.0
            && self.solver_iterations > 0
            && self.penetration_slop_m >= 0.0
            && (0.0..=1.0).contains(&self.positional_correction_fraction)
            && self.collision.is_valid()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SimulationError {
    InvalidConfig,
    InvalidBody,
    ReservedBodyId,
    DuplicateBodyId,
    BodyNotFound,
    DuplicateArmId,
    ArmIdOutOfRange,
    ArmNotFound,
    InvalidGripperConfig,
    Arm(ArmError),
    SerialArm(SerialArmError),
    InvalidMachineCommand(MachineCommandError),
    BodyAlreadyHeld,
    GraspRejected,
    InvalidElapsedTime,
}

impl From<ArmError> for SimulationError {
    fn from(value: ArmError) -> Self {
        Self::Arm(value)
    }
}

impl From<SerialArmError> for SimulationError {
    fn from(value: SerialArmError) -> Self {
        Self::SerialArm(value)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct StepReport {
    pub step_index: u64,
    pub time_s: f64,
    pub contacts: Vec<Contact>,
    pub clearances: Vec<Clearance>,
    pub maximum_penetration_m: f64,
    pub dynamic_kinetic_energy_j: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToolMotionTraceSample {
    pub tick: u64,
    pub time_s: f64,
    pub manipulator: ArmId,
    pub target_position_world_m: Vec3,
    pub actual_position_world_m: Vec3,
    pub position_error_m: f64,
    pub progress: f64,
    pub positions: crate::serial_arm::SerialJointPositions,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Simulation {
    pub config: SimulationConfig,
    pub time_s: f64,
    pub step_index: u64,
    pub bodies: Vec<RigidBody>,
    pub arms: Vec<ArmInstance>,
    /// Reference rigid-link arms. `arms` above contains only the optional
    /// reduced-order continuum model for backward-compatible adapters.
    pub serial_arms: Vec<SerialArmInstance>,
    pub machine_command_sequence: u64,
    pub machine_command_log: Vec<MachineCommandEvent>,
    /// Deterministic, fixed-step evidence for Cartesian point motions. Samples
    /// are appended only while a tool plan advances, including its terminal
    /// sample, and are never synthesized by presentation adapters.
    pub tool_motion_trace: Vec<ToolMotionTraceSample>,
    /// Remainder used only by [`Simulation::advance_by`]. Calling `step`
    /// directly leaves this unchanged.
    pub accumulator_s: f64,
}

impl Simulation {
    pub fn new(config: SimulationConfig) -> Result<Self, SimulationError> {
        if !config.is_valid() {
            return Err(SimulationError::InvalidConfig);
        }
        Ok(Self {
            config,
            time_s: 0.0,
            step_index: 0,
            bodies: Vec::new(),
            arms: Vec::new(),
            serial_arms: Vec::new(),
            machine_command_sequence: 0,
            machine_command_log: Vec::new(),
            tool_motion_trace: Vec::new(),
            accumulator_s: 0.0,
        })
    }

    pub fn add_body(&mut self, body: RigidBody) -> Result<(), SimulationError> {
        if body.id.0 >= SERIAL_ARM_COLLISION_BODY_ID_BASE {
            return Err(SimulationError::ReservedBodyId);
        }
        if !body.shape.is_valid()
            || !body.pose.translation.is_finite()
            || !body.pose.rotation.is_finite()
            || (body.motion == MotionType::Dynamic
                && (!body.mass_kg.is_finite() || body.mass_kg <= 0.0))
        {
            return Err(SimulationError::InvalidBody);
        }
        if self.bodies.iter().any(|existing| existing.id == body.id) {
            return Err(SimulationError::DuplicateBodyId);
        }
        self.bodies.push(body);
        self.bodies.sort_by_key(|body| body.id);
        Ok(())
    }

    pub fn remove_body(&mut self, id: BodyId) -> Result<RigidBody, SimulationError> {
        let index = self
            .bodies
            .binary_search_by_key(&id, |body| body.id)
            .map_err(|_| SimulationError::BodyNotFound)?;
        for arm in &mut self.arms {
            if arm.gripper.held_body == Some(id) {
                arm.gripper.release();
                arm.held_body_local_pose = None;
            }
        }
        for arm in &mut self.serial_arms {
            if arm.gripper.held_body == Some(id) {
                arm.gripper.release();
                arm.held_body_local_pose = None;
            }
        }
        Ok(self.bodies.remove(index))
    }

    pub fn body(&self, id: BodyId) -> Option<&RigidBody> {
        self.bodies
            .binary_search_by_key(&id, |body| body.id)
            .ok()
            .map(|index| &self.bodies[index])
    }

    pub fn body_mut(&mut self, id: BodyId) -> Option<&mut RigidBody> {
        self.bodies
            .binary_search_by_key(&id, |body| body.id)
            .ok()
            .map(|index| &mut self.bodies[index])
    }

    pub fn add_arm(&mut self, arm: ArmInstance) -> Result<(), SimulationError> {
        if self.arms.iter().any(|existing| existing.id == arm.id)
            || self
                .serial_arms
                .iter()
                .any(|existing| existing.id == arm.id)
        {
            return Err(SimulationError::DuplicateArmId);
        }
        self.arms.push(arm);
        self.arms.sort_by_key(|arm| arm.id);
        Ok(())
    }

    pub fn add_serial_arm(&mut self, arm: SerialArmInstance) -> Result<(), SimulationError> {
        if arm.id.0 > MAX_SERIAL_ARM_COLLISION_ID {
            return Err(SimulationError::ArmIdOutOfRange);
        }
        if self.arms.iter().any(|existing| existing.id == arm.id)
            || self
                .serial_arms
                .iter()
                .any(|existing| existing.id == arm.id)
        {
            return Err(SimulationError::DuplicateArmId);
        }
        self.serial_arms.push(arm);
        self.serial_arms.sort_by_key(|arm| arm.id);
        Ok(())
    }

    pub fn arm(&self, id: ArmId) -> Option<&ArmInstance> {
        self.arms
            .binary_search_by_key(&id, |arm| arm.id)
            .ok()
            .map(|index| &self.arms[index])
    }

    pub fn arm_mut(&mut self, id: ArmId) -> Option<&mut ArmInstance> {
        self.arms
            .binary_search_by_key(&id, |arm| arm.id)
            .ok()
            .map(|index| &mut self.arms[index])
    }

    pub fn serial_arm(&self, id: ArmId) -> Option<&SerialArmInstance> {
        self.serial_arms
            .binary_search_by_key(&id, |arm| arm.id)
            .ok()
            .map(|index| &self.serial_arms[index])
    }

    pub fn serial_arm_mut(&mut self, id: ArmId) -> Option<&mut SerialArmInstance> {
        self.serial_arms
            .binary_search_by_key(&id, |arm| arm.id)
            .ok()
            .map(|index| &mut self.serial_arms[index])
    }

    /// Validate and record a plant command before changing any target state.
    /// A controlled stop addressed to `None` atomically holds every arm.
    pub fn submit_machine_command(
        &mut self,
        command: MachineCommand,
    ) -> Result<u64, SimulationError> {
        if let MachineCommand::Stop { manipulator: None } = command {
            for arm in &mut self.serial_arms {
                arm.motion.stop();
                arm.gripper.stop();
            }
        } else {
            let manipulator = command
                .manipulator()
                .expect("only the all-arm stop omits a manipulator");
            let arm_id = ArmId(manipulator.0);
            let arm_index = self
                .serial_arms
                .binary_search_by_key(&arm_id, |arm| arm.id)
                .map_err(|_| SimulationError::ArmNotFound)?;
            let arm = &self.serial_arms[arm_index];
            ManipulatorMotionState::validate_command(
                command,
                arm.carriage_config,
                arm.arm.config,
                arm.gripper_config,
            )
            .map_err(SimulationError::InvalidMachineCommand)?;

            let tool_plan = if let MachineCommand::SetToolPoseTarget {
                target_position_world_m,
                ..
            } = command
            {
                let solution = arm
                    .arm
                    .solve_tool_position(target_position_world_m, arm.motion.positions())
                    .map_err(|error| {
                        SimulationError::InvalidMachineCommand(match error {
                            ToolPositionIkError::NonFiniteTarget => {
                                MachineCommandError::NonFiniteTarget
                            }
                            ToolPositionIkError::Unreachable => {
                                MachineCommandError::ToolTargetUnreachable
                            }
                            ToolPositionIkError::JointLimits => {
                                MachineCommandError::ToolTargetJointLimits
                            }
                        })
                    })?;
                let plan = ToolMotionPlan::new(
                    target_position_world_m,
                    arm.motion.positions(),
                    solution.positions,
                    arm.carriage_config,
                    arm.motion_config,
                    arm.tool_motion_speed_scale,
                );
                self.validate_tool_motion_path(arm_id, plan)?;
                Some(plan)
            } else {
                None
            };

            let arm = &mut self.serial_arms[arm_index];
            if let Some(plan) = tool_plan {
                arm.motion.start_tool_motion(plan);
            } else {
                arm.motion.apply_command(command);
                match command {
                    MachineCommand::SetGripperOpening {
                        target_opening_m, ..
                    } => arm
                        .gripper
                        .set_command(target_opening_m, arm.gripper_config),
                    MachineCommand::Stop { .. } => arm.gripper.stop(),
                    _ => {}
                }
            }
        }

        self.machine_command_sequence += 1;
        self.machine_command_log.push(MachineCommandEvent {
            sequence: self.machine_command_sequence,
            issued_at_tick: self.step_index,
            command,
        });
        Ok(self.machine_command_sequence)
    }

    fn validate_tool_motion_path(
        &self,
        arm_id: ArmId,
        plan: ToolMotionPlan,
    ) -> Result<(), SimulationError> {
        let moving_arm = self
            .serial_arm(arm_id)
            .ok_or(SimulationError::ArmNotFound)?;
        let obstacle_bodies = self
            .bodies
            .iter()
            .filter(|body| body.enabled)
            .cloned()
            .chain(
                self.serial_arms
                    .iter()
                    .filter(|arm| arm.id != arm_id)
                    .flat_map(|arm| {
                        arm.kinematics
                            .collision_capsules
                            .iter()
                            .enumerate()
                            .filter_map(|(index, (pose, shape))| {
                                Some(RigidBody::new(
                                    serial_arm_link_body_id(arm.id, index as u8)?,
                                    *shape,
                                    *pose,
                                    MotionType::Kinematic,
                                ))
                            })
                    }),
            )
            .collect::<Vec<_>>();
        let sample_count = tool_path_sample_count(plan)?;
        let mut candidate = moving_arm.arm.clone();
        for sample in 0..=sample_count {
            let progress = sample as f64 / sample_count as f64;
            candidate
                .set_positions(plan.sample(progress))
                .map_err(SimulationError::SerialArm)?;
            let candidate_links = candidate.forward_kinematics().collision_capsules;
            for (link_index, (pose, shape)) in candidate_links.iter().enumerate() {
                let moving_body = RigidBody::new(
                    serial_arm_link_body_id(arm_id, link_index as u8)
                        .expect("serial arm has three physical links"),
                    *shape,
                    *pose,
                    MotionType::Kinematic,
                );
                if obstacle_bodies.iter().any(|obstacle| {
                    moving_body.collision_filter.allows(obstacle.collision_filter)
                        && query_pair(&moving_body, obstacle).is_some_and(|proximity| {
                            proximity.signed_distance_m
                                <= self.config.collision.clearance_threshold_m
                        })
                }) {
                    return Err(SimulationError::InvalidMachineCommand(
                        MachineCommandError::ToolPathCollision,
                    ));
                }
            }
            if let (Some(first), Some(last)) = (candidate_links.first(), candidate_links.last()) {
                let first_body = RigidBody::new(
                    serial_arm_link_body_id(arm_id, 0).expect("valid link"),
                    first.1,
                    first.0,
                    MotionType::Kinematic,
                );
                let last_body = RigidBody::new(
                    serial_arm_link_body_id(arm_id, 2).expect("valid link"),
                    last.1,
                    last.0,
                    MotionType::Kinematic,
                );
                if query_pair(&first_body, &last_body).is_some_and(|proximity| {
                    proximity.signed_distance_m <= self.config.collision.clearance_threshold_m
                }) {
                    return Err(SimulationError::InvalidMachineCommand(
                        MachineCommandError::ToolPathCollision,
                    ));
                }
            }
        }
        Ok(())
    }

    /// Materialize current serial-arm link capsules as kinematic rigid bodies.
    ///
    /// This is a read-only export: it recomputes forward kinematics directly
    /// from each arm's current joint state and never mutates `self.bodies` or
    /// the cached step state. Callers may alter the returned collision filters
    /// before performing custom queries.
    pub fn serial_arm_collision_bodies(&self) -> Vec<RigidBody> {
        let mut result =
            Vec::with_capacity(self.serial_arms.len() * SERIAL_ARM_LINK_COUNT as usize);
        for instance in &self.serial_arms {
            let kinematics = instance.arm.forward_kinematics();
            for (link_index, (pose, shape)) in kinematics.collision_capsules.into_iter().enumerate()
            {
                let Some(body_id) = serial_arm_link_body_id(instance.id, link_index as u8) else {
                    continue;
                };
                result.push(RigidBody::new(body_id, shape, pose, MotionType::Kinematic));
            }
        }
        result.sort_by_key(|body| body.id);
        result
    }

    /// Query authoritative parts together with current serial-arm link
    /// capsules without changing the fixed-step scene. Adjacent links belonging
    /// to the same arm are excluded; non-adjacent self-collision and all
    /// inter-arm/arm-part pairs remain enabled.
    pub fn query_collisions_with_arms(&self, settings: CollisionSettings) -> CollisionReport {
        let mut scene = self.bodies.clone();
        scene.extend(self.serial_arm_collision_bodies());
        scene.sort_by_key(|body| body.id);

        let mut report = CollisionReport::default();
        for index_a in 0..scene.len() {
            for index_b in (index_a + 1)..scene.len() {
                let a = &scene[index_a];
                let b = &scene[index_b];
                if !a.enabled
                    || !b.enabled
                    || !a.shape.is_valid()
                    || !b.shape.is_valid()
                    || !a.collision_filter.allows(b.collision_filter)
                {
                    continue;
                }
                if let (Some((arm_a, link_a)), Some((arm_b, link_b))) =
                    (serial_arm_link_key(a.id), serial_arm_link_key(b.id))
                {
                    if arm_a == arm_b && link_a.abs_diff(link_b) <= 1 {
                        continue;
                    }
                }
                if a.aabb().distance(b.aabb()) > settings.clearance_threshold_m.max(0.0) {
                    continue;
                }
                report.broad_phase_pairs.push((a.id, b.id));
                let Some(proximity) = query_pair(a, b) else {
                    continue;
                };
                if proximity.signed_distance_m <= settings.contact_offset_m {
                    report.contacts.push(Contact::from(proximity));
                } else if proximity.signed_distance_m <= settings.clearance_threshold_m {
                    report.clearances.push(Clearance {
                        body_a: proximity.body_a,
                        body_b: proximity.body_b,
                        distance_m: proximity.signed_distance_m,
                        point_a_world_m: proximity.point_a_world_m,
                        point_b_world_m: proximity.point_b_world_m,
                        kind: proximity.kind,
                    });
                }
            }
        }
        report
    }

    pub fn grasp_body(&mut self, arm_id: ArmId, body_id: BodyId) -> Result<(), SimulationError> {
        if self
            .arms
            .iter()
            .any(|arm| arm.gripper.held_body == Some(body_id))
            || self
                .serial_arms
                .iter()
                .any(|arm| arm.gripper.held_body == Some(body_id))
        {
            return Err(SimulationError::BodyAlreadyHeld);
        }
        let body = self
            .body(body_id)
            .ok_or(SimulationError::BodyNotFound)?
            .clone();
        let arm_index = self
            .arms
            .binary_search_by_key(&arm_id, |arm| arm.id)
            .map_err(|_| SimulationError::ArmNotFound)?;
        let arm = &mut self.arms[arm_index];
        let tool_pose = arm.tool_pose();
        let candidate = arm
            .gripper
            .evaluate_candidate(tool_pose, &body, arm.gripper_config);
        if !arm.gripper.try_grasp(candidate, arm.gripper_config) {
            return Err(SimulationError::GraspRejected);
        }
        arm.held_body_local_pose = Some(tool_pose.inverse() * body.pose);
        Ok(())
    }

    pub fn grasp_body_serial(
        &mut self,
        arm_id: ArmId,
        body_id: BodyId,
    ) -> Result<(), SimulationError> {
        if self
            .arms
            .iter()
            .any(|arm| arm.gripper.held_body == Some(body_id))
            || self
                .serial_arms
                .iter()
                .any(|arm| arm.gripper.held_body == Some(body_id))
        {
            return Err(SimulationError::BodyAlreadyHeld);
        }
        let body = self
            .body(body_id)
            .ok_or(SimulationError::BodyNotFound)?
            .clone();
        let arm_index = self
            .serial_arms
            .binary_search_by_key(&arm_id, |arm| arm.id)
            .map_err(|_| SimulationError::ArmNotFound)?;
        let arm = &mut self.serial_arms[arm_index];
        let tool_pose = arm.tool_pose();
        let candidate = arm
            .gripper
            .evaluate_candidate(tool_pose, &body, arm.gripper_config);
        if !arm.gripper.try_grasp(candidate, arm.gripper_config) {
            return Err(SimulationError::GraspRejected);
        }
        arm.held_body_local_pose = Some(tool_pose.inverse() * body.pose);
        Ok(())
    }

    pub fn release_body(&mut self, arm_id: ArmId) -> Result<Option<BodyId>, SimulationError> {
        let arm = self.arm_mut(arm_id).ok_or(SimulationError::ArmNotFound)?;
        arm.held_body_local_pose = None;
        Ok(arm.gripper.release())
    }

    pub fn release_body_serial(
        &mut self,
        arm_id: ArmId,
    ) -> Result<Option<BodyId>, SimulationError> {
        let arm = self
            .serial_arm_mut(arm_id)
            .ok_or(SimulationError::ArmNotFound)?;
        arm.held_body_local_pose = None;
        Ok(arm.gripper.release())
    }

    pub fn step(&mut self) -> Result<StepReport, SimulationError> {
        if !self.config.is_valid() {
            return Err(SimulationError::InvalidConfig);
        }
        let dt_s = self.config.fixed_dt_s;

        for arm in &mut self.arms {
            arm.step(dt_s)?;
        }
        let active_tool_motions = self
            .serial_arms
            .iter()
            .filter_map(|arm| {
                matches!(
                    arm.motion.tool_motion.map(|plan| plan.status),
                    Some(ToolMotionStatus::Active)
                )
                .then_some(arm.id)
            })
            .collect::<Vec<_>>();
        for arm in &mut self.serial_arms {
            arm.step(dt_s)?;
        }

        for arm_id in active_tool_motions {
            let sample = {
                let arm = self
                    .serial_arm(arm_id)
                    .expect("active tool-motion arm remains present");
                let plan = arm
                    .motion
                    .tool_motion
                    .expect("active tool-motion arm retains its plan");
                let actual_position_world_m = arm.kinematics.tool_pose.translation;
                ToolMotionTraceSample {
                    tick: self.step_index + 1,
                    time_s: (self.step_index + 1) as f64 * dt_s,
                    manipulator: arm_id,
                    target_position_world_m: plan.target_position_world_m,
                    actual_position_world_m,
                    position_error_m: (actual_position_world_m - plan.target_position_world_m)
                        .length(),
                    progress: plan.progress(),
                    positions: arm.motion.positions(),
                }
            };
            self.tool_motion_trace.push(sample);
        }

        // Kinematically attach grasped parts after arm motion.
        let mut attachments = self
            .arms
            .iter()
            .filter_map(|arm| {
                Some((
                    arm.gripper.held_body?,
                    arm.tool_pose() * arm.held_body_local_pose?,
                ))
            })
            .collect::<Vec<_>>();
        attachments.extend(self.serial_arms.iter().filter_map(|arm| {
            Some((
                arm.gripper.held_body?,
                arm.tool_pose() * arm.held_body_local_pose?,
            ))
        }));
        for (body_id, pose) in &attachments {
            if let Some(body) = self.body_mut(*body_id) {
                body.pose = *pose;
                body.linear_velocity_m_s = Vec3::ZERO;
                body.angular_velocity_rad_s = Vec3::ZERO;
            }
        }

        for body in &mut self.bodies {
            if body.motion != MotionType::Dynamic
                || attachments.iter().any(|(held, _)| *held == body.id)
            {
                body.clear_forces();
                continue;
            }
            let inverse_mass = body.inverse_mass();
            let acceleration = self.config.gravity_m_s2 + body.accumulated_force_n * inverse_mass;
            body.linear_velocity_m_s += acceleration * dt_s;
            body.linear_velocity_m_s =
                body.linear_velocity_m_s * (-self.config.linear_damping_per_s * dt_s).exp();
            body.pose.translation += body.linear_velocity_m_s * dt_s;

            // Isotropic bounding-sphere inertia is a stable low-cost proxy.
            let radius = body.shape.local_bounding_radius_m();
            let inertia = (0.4 * body.mass_kg * radius * radius).max(1.0e-18);
            body.angular_velocity_rad_s += body.accumulated_torque_nm * (dt_s / inertia);
            body.angular_velocity_rad_s =
                body.angular_velocity_rad_s * (-self.config.angular_damping_per_s * dt_s).exp();
            let delta_rotation = Quat::from_scaled_axis(body.angular_velocity_rad_s * dt_s);
            body.pose.rotation = (delta_rotation * body.pose.rotation).normalized();
            body.clear_forces();
        }

        for _ in 0..self.config.solver_iterations {
            let contacts = CollisionReport::query(&self.bodies, self.config.collision).contacts;
            if contacts.is_empty() {
                break;
            }
            for contact in contacts {
                self.resolve_contact(contact, &attachments);
            }
        }

        self.step_index += 1;
        self.time_s = self.step_index as f64 * dt_s;
        let collision = CollisionReport::query(&self.bodies, self.config.collision);
        let maximum_penetration_m = collision
            .contacts
            .iter()
            .map(|contact| contact.penetration_depth_m)
            .fold(0.0, f64::max);
        let dynamic_kinetic_energy_j = self
            .bodies
            .iter()
            .filter(|body| body.motion == MotionType::Dynamic)
            .map(|body| 0.5 * body.mass_kg * body.linear_velocity_m_s.length_squared())
            .sum();
        Ok(StepReport {
            step_index: self.step_index,
            time_s: self.time_s,
            contacts: collision.contacts,
            clearances: collision.clearances,
            maximum_penetration_m,
            dynamic_kinetic_energy_j,
        })
    }

    fn resolve_contact(&mut self, contact: Contact, attachments: &[(BodyId, Pose)]) {
        let Ok(index_a) = self
            .bodies
            .binary_search_by_key(&contact.body_a, |body| body.id)
        else {
            return;
        };
        let Ok(index_b) = self
            .bodies
            .binary_search_by_key(&contact.body_b, |body| body.id)
        else {
            return;
        };
        let (a, b) = two_mut(&mut self.bodies, index_a, index_b);
        // A grasped body is a kinematic extension of the tool for this reduced
        // solver. It may push dynamic parts, but contacts must not detach it
        // from the tool pose between fixed steps.
        let inverse_mass_a = if attachments.iter().any(|(id, _)| *id == a.id) {
            0.0
        } else {
            a.inverse_mass()
        };
        let inverse_mass_b = if attachments.iter().any(|(id, _)| *id == b.id) {
            0.0
        } else {
            b.inverse_mass()
        };
        let inverse_mass_sum = inverse_mass_a + inverse_mass_b;
        if inverse_mass_sum <= 0.0 {
            return;
        }

        let penetration = (contact.penetration_depth_m - self.config.penetration_slop_m).max(0.0);
        if penetration > 0.0 {
            let correction = contact.normal_a_to_b
                * (penetration * self.config.positional_correction_fraction / inverse_mass_sum);
            a.pose.translation -= correction * inverse_mass_a;
            b.pose.translation += correction * inverse_mass_b;
        }

        let relative_velocity = b.linear_velocity_m_s - a.linear_velocity_m_s;
        let normal_speed = relative_velocity.dot(contact.normal_a_to_b);
        if normal_speed >= 0.0 {
            return;
        }
        let restitution = a
            .material
            .restitution
            .min(b.material.restitution)
            .clamp(0.0, 1.0);
        let normal_impulse_magnitude = -(1.0 + restitution) * normal_speed / inverse_mass_sum;
        let normal_impulse = contact.normal_a_to_b * normal_impulse_magnitude;
        a.linear_velocity_m_s -= normal_impulse * inverse_mass_a;
        b.linear_velocity_m_s += normal_impulse * inverse_mass_b;

        let post_relative = b.linear_velocity_m_s - a.linear_velocity_m_s;
        let tangent_velocity =
            post_relative - contact.normal_a_to_b * post_relative.dot(contact.normal_a_to_b);
        let tangent_speed = tangent_velocity.length();
        if tangent_speed > 1.0e-12 {
            let friction = (a.material.friction * b.material.friction).max(0.0).sqrt();
            let desired = tangent_speed / inverse_mass_sum;
            let friction_impulse_magnitude = desired.min(friction * normal_impulse_magnitude);
            let friction_impulse = tangent_velocity / tangent_speed * friction_impulse_magnitude;
            a.linear_velocity_m_s += friction_impulse * inverse_mass_a;
            b.linear_velocity_m_s -= friction_impulse * inverse_mass_b;
        }
    }

    /// Consume real/host time using only complete fixed steps. `max_steps`
    /// prevents a stalled UI tab from causing an unbounded catch-up loop.
    pub fn advance_by(
        &mut self,
        elapsed_s: f64,
        max_steps: usize,
    ) -> Result<Vec<StepReport>, SimulationError> {
        if elapsed_s < 0.0 || !elapsed_s.is_finite() {
            return Err(SimulationError::InvalidElapsedTime);
        }
        self.accumulator_s += elapsed_s;
        let mut reports = Vec::new();
        while self.accumulator_s + 1.0e-15 >= self.config.fixed_dt_s && reports.len() < max_steps {
            self.accumulator_s -= self.config.fixed_dt_s;
            reports.push(self.step()?);
        }
        Ok(reports)
    }
}

fn tool_path_sample_count(plan: ToolMotionPlan) -> Result<usize, SimulationError> {
    let joint_delta = plan
        .start
        .tendon_joint_angles()
        .iter()
        .zip(plan.goal.tendon_joint_angles())
        .map(|(start, goal)| (goal - start).abs())
        .fold(0.0, f64::max);
    let theta_delta = wrap_angle_pi(plan.goal.base_theta_rad - plan.start.base_theta_rad).abs();
    let z_delta = (plan.goal.base_z_m - plan.start.base_z_m).abs();
    // ToolMotionPlan uses cubic smoothstep, whose maximum slope is 1.5.
    // Account for that slope so consecutive configuration samples honor the
    // advertised angular and linear bounds over the complete path.
    let required = (SMOOTHSTEP_MAX_SLOPE * joint_delta.max(theta_delta)
        / TOOL_PATH_MAX_ANGULAR_STEP_RAD)
        .max(SMOOTHSTEP_MAX_SLOPE * z_delta / TOOL_PATH_MAX_LINEAR_STEP_M)
        .ceil()
        .max(2.0);
    if !required.is_finite() || required > TOOL_PATH_MAX_SAMPLE_COUNT as f64 {
        return Err(SimulationError::InvalidMachineCommand(
            MachineCommandError::ToolPathSamplingLimit,
        ));
    }
    Ok(required as usize)
}

impl MachineBackend for Simulation {
    type Error = SimulationError;

    fn submit_command(&mut self, command: MachineCommand) -> Result<u64, Self::Error> {
        self.submit_machine_command(command)
    }

    fn advance_fixed_step(&mut self) -> Result<(), Self::Error> {
        self.step().map(|_| ())
    }
}

fn two_mut<T>(slice: &mut [T], a: usize, b: usize) -> (&mut T, &mut T) {
    assert_ne!(a, b);
    if a < b {
        let (left, right) = slice.split_at_mut(b);
        (&mut left[a], &mut right[0])
    } else {
        let (left, right) = slice.split_at_mut(a);
        (&mut right[0], &mut left[b])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Material, Shape};
    use crate::machine::ManipulatorId;

    #[test]
    fn fixed_step_clock_does_not_accumulate_addition_drift() {
        let mut simulation = Simulation::new(SimulationConfig::default()).unwrap();
        for _ in 0..10_000 {
            simulation.step().unwrap();
        }
        assert_eq!(
            simulation.time_s,
            simulation.step_index as f64 * simulation.config.fixed_dt_s
        );
    }

    #[test]
    fn dynamic_sphere_falls_and_static_sphere_does_not() {
        let mut simulation = Simulation::new(SimulationConfig::default()).unwrap();
        simulation
            .add_body(RigidBody::new(
                BodyId(1),
                Shape::Sphere { radius_m: 0.1e-3 },
                Pose::from_translation(Vec3::Z),
                MotionType::Dynamic,
            ))
            .unwrap();
        simulation
            .add_body(RigidBody::new(
                BodyId(2),
                Shape::Sphere { radius_m: 0.1e-3 },
                Pose::from_translation(Vec3::Z * 2.0),
                MotionType::Static,
            ))
            .unwrap();
        simulation.step().unwrap();
        assert!(simulation.body(BodyId(1)).unwrap().pose.translation.z < 1.0);
        assert_eq!(simulation.body(BodyId(2)).unwrap().pose.translation.z, 2.0);
    }

    #[test]
    fn solver_separates_overlapping_dynamic_and_static_spheres() {
        let config = SimulationConfig {
            gravity_m_s2: Vec3::ZERO,
            ..SimulationConfig::default()
        };
        let mut simulation = Simulation::new(config).unwrap();
        simulation
            .add_body(RigidBody::new(
                BodyId(1),
                Shape::Sphere { radius_m: 1.0 },
                Pose::IDENTITY,
                MotionType::Static,
            ))
            .unwrap();
        simulation
            .add_body(
                RigidBody::new(
                    BodyId(2),
                    Shape::Sphere { radius_m: 1.0 },
                    Pose::from_translation(Vec3::X * 1.5),
                    MotionType::Dynamic,
                )
                .with_material(Material {
                    density_kg_m3: 1.0,
                    friction: 0.0,
                    restitution: 0.0,
                }),
            )
            .unwrap();
        let before = simulation.body(BodyId(2)).unwrap().pose.translation.x;
        let report = simulation.step().unwrap();
        let after = simulation.body(BodyId(2)).unwrap().pose.translation.x;
        assert!(after > before);
        assert!(report.maximum_penetration_m < 0.01);
    }

    #[test]
    fn advance_by_executes_only_whole_steps() {
        let mut simulation = Simulation::new(SimulationConfig::default()).unwrap();
        let dt = simulation.config.fixed_dt_s;
        assert!(simulation.advance_by(0.5 * dt, 10).unwrap().is_empty());
        assert_eq!(simulation.advance_by(2.0 * dt, 10).unwrap().len(), 2);
        assert!((simulation.accumulator_s - 0.5 * dt).abs() < 1e-15);
    }

    fn baseline_serial_instance(id: u32) -> SerialArmInstance {
        let arm = SerialArm::new(crate::serial_arm::SerialArmConfig::default()).unwrap();
        SerialArmInstance::new(ArmId(id), arm, GripperConfig::default()).unwrap()
    }

    #[test]
    fn arm_collision_query_detects_part_without_mutating_body_scene() {
        let mut simulation = Simulation::new(SimulationConfig::default()).unwrap();
        simulation
            .add_serial_arm(baseline_serial_instance(1))
            .unwrap();
        let arm_config = crate::serial_arm::SerialArmConfig::default();
        let upper_link_midpoint =
            Vec3::X * (arm_config.rail_radius_m - 0.5 * arm_config.upper_arm_length_m);
        simulation
            .add_body(RigidBody::new(
                BodyId(7),
                Shape::Sphere { radius_m: 0.5e-3 },
                Pose::from_translation(upper_link_midpoint),
                MotionType::Static,
            ))
            .unwrap();

        let body_count = simulation.bodies.len();
        let report = simulation.query_collisions_with_arms(CollisionSettings::default());
        assert_eq!(simulation.bodies.len(), body_count);
        let upper_id = serial_arm_link_body_id(ArmId(1), 0).unwrap();
        assert!(report.contacts.iter().any(|contact| {
            (contact.body_a == BodyId(7) && contact.body_b == upper_id)
                || (contact.body_a == upper_id && contact.body_b == BodyId(7))
        }));
    }

    #[test]
    fn arm_collision_query_detects_inter_arm_and_excludes_adjacent_self_links() {
        let mut one_arm = Simulation::new(SimulationConfig::default()).unwrap();
        one_arm.add_serial_arm(baseline_serial_instance(1)).unwrap();
        let self_report = one_arm.query_collisions_with_arms(CollisionSettings::default());
        assert!(self_report.contacts.iter().all(|contact| {
            match (
                serial_arm_link_key(contact.body_a),
                serial_arm_link_key(contact.body_b),
            ) {
                (Some((arm_a, link_a)), Some((arm_b, link_b))) => {
                    arm_a != arm_b || link_a.abs_diff(link_b) > 1
                }
                _ => true,
            }
        }));

        one_arm.add_serial_arm(baseline_serial_instance(2)).unwrap();
        let report = one_arm.query_collisions_with_arms(CollisionSettings::default());
        assert!(report.contacts.iter().any(|contact| {
            matches!(
                (
                    serial_arm_link_key(contact.body_a),
                    serial_arm_link_key(contact.body_b),
                ),
                (Some((ArmId(1), _)), Some((ArmId(2), _)))
            )
        }));
    }

    #[test]
    fn authoritative_body_ids_cannot_enter_reserved_arm_range() {
        let mut simulation = Simulation::new(SimulationConfig::default()).unwrap();
        let result = simulation.add_body(RigidBody::new(
            BodyId(SERIAL_ARM_COLLISION_BODY_ID_BASE),
            Shape::Sphere { radius_m: 1.0e-3 },
            Pose::IDENTITY,
            MotionType::Static,
        ));
        assert_eq!(result, Err(SimulationError::ReservedBodyId));
    }

    #[test]
    fn machine_commands_drive_bounded_authoritative_arm_state() {
        let mut simulation = Simulation::new(SimulationConfig {
            gravity_m_s2: Vec3::ZERO,
            ..SimulationConfig::default()
        })
        .unwrap();
        simulation
            .add_serial_arm(baseline_serial_instance(1))
            .unwrap();
        simulation
            .submit_machine_command(MachineCommand::MoveCarriageZ {
                manipulator: ManipulatorId(1),
                target_z_m: 25.0e-3,
            })
            .unwrap();
        simulation
            .submit_machine_command(MachineCommand::SetJointTargets {
                manipulator: ManipulatorId(1),
                target_rad: [0.20, -0.15, 0.30, 0.10],
            })
            .unwrap();
        for _ in 0..4_000 {
            simulation.step().unwrap();
        }
        let arm = simulation.serial_arm(ArmId(1)).unwrap();
        assert!((arm.motion.carriage.z_m - 25.0e-3).abs() < 1.0e-12);
        assert_eq!(arm.arm.positions, arm.motion.positions());
        assert_eq!(simulation.machine_command_sequence, 2);
        assert_eq!(simulation.machine_command_log.len(), 2);
    }

    #[test]
    fn cartesian_tool_command_executes_a_deterministic_bounded_trace() {
        let mut simulation = Simulation::new(SimulationConfig {
            fixed_dt_s: 0.001,
            gravity_m_s2: Vec3::ZERO,
            ..SimulationConfig::default()
        })
        .unwrap();
        simulation
            .add_serial_arm(baseline_serial_instance(1))
            .unwrap();
        let target = Vec3::new(20.0e-3, 0.0, 10.0e-3);
        simulation
            .submit_machine_command(MachineCommand::SetToolPoseTarget {
                manipulator: ManipulatorId(1),
                target_position_world_m: target,
            })
            .unwrap();
        for _ in 0..10_000 {
            simulation.step().unwrap();
            if simulation.serial_arm(ArmId(1)).unwrap().motion.tool_motion
                .is_some_and(|plan| plan.status == ToolMotionStatus::Complete)
            {
                break;
            }
        }
        let arm = simulation.serial_arm(ArmId(1)).unwrap();
        let plan = arm.motion.tool_motion.expect("tool plan remains inspectable");
        assert_eq!(plan.status, ToolMotionStatus::Complete);
        assert!((arm.tool_pose().translation - target).length() < 1.0e-9);
        assert!(!simulation.tool_motion_trace.is_empty());
        let last = simulation.tool_motion_trace.last().unwrap();
        assert_eq!(last.progress, 1.0);
        assert!(last.position_error_m < 1.0e-9);
        assert_eq!(last.tick, simulation.step_index);
    }

    #[test]
    fn cartesian_tool_command_rejects_unreachable_and_colliding_paths_atomically() {
        let mut unreachable = Simulation::new(SimulationConfig::default()).unwrap();
        unreachable
            .add_serial_arm(baseline_serial_instance(1))
            .unwrap();
        let before = unreachable.serial_arm(ArmId(1)).unwrap().motion;
        let result = unreachable.submit_machine_command(MachineCommand::SetToolPoseTarget {
            manipulator: ManipulatorId(1),
            target_position_world_m: Vec3::new(90.0e-3, 0.0, 0.0),
        });
        assert_eq!(
            result,
            Err(SimulationError::InvalidMachineCommand(
                MachineCommandError::ToolTargetUnreachable
            ))
        );
        assert_eq!(unreachable.serial_arm(ArmId(1)).unwrap().motion, before);
        assert_eq!(unreachable.machine_command_sequence, 0);

        let mut colliding = Simulation::new(SimulationConfig::default()).unwrap();
        colliding
            .add_serial_arm(baseline_serial_instance(1))
            .unwrap();
        colliding
            .add_body(RigidBody::new(
                BodyId(7),
                Shape::Sphere { radius_m: 8.0e-3 },
                Pose::from_translation(Vec3::new(20.0e-3, 0.0, 0.0)),
                MotionType::Static,
            ))
            .unwrap();
        let result = colliding.submit_machine_command(MachineCommand::SetToolPoseTarget {
            manipulator: ManipulatorId(1),
            target_position_world_m: Vec3::new(20.0e-3, 0.0, 0.0),
        });
        assert_eq!(
            result,
            Err(SimulationError::InvalidMachineCommand(
                MachineCommandError::ToolPathCollision
            ))
        );
        assert_eq!(colliding.machine_command_sequence, 0);
        assert!(colliding.tool_motion_trace.is_empty());
    }

    #[test]
    fn tool_path_sampling_honors_documented_configuration_step_bounds() {
        let start = crate::serial_arm::SerialJointPositions {
            base_z_m: -150.0e-3,
            ..crate::serial_arm::SerialJointPositions::default()
        };
        let goal = crate::serial_arm::SerialJointPositions {
            base_z_m: 150.0e-3,
            base_theta_rad: core::f64::consts::PI,
            shoulder_yaw_rad: 100.0_f64.to_radians(),
            shoulder_pitch_rad: 120.0_f64.to_radians(),
            elbow_pitch_rad: 155.0_f64.to_radians(),
            wrist_roll_rad: core::f64::consts::PI,
        };
        let plan = ToolMotionPlan::new(
            Vec3::ZERO,
            start,
            goal,
            CarriageConfig::default(),
            ManipulatorMotionConfig::default(),
            1.0,
        );
        let sample_count = tool_path_sample_count(plan).unwrap();
        let mut previous = plan.sample(0.0);
        for sample in 1..=sample_count {
            let current = plan.sample(sample as f64 / sample_count as f64);
            assert!(
                (current.base_z_m - previous.base_z_m).abs()
                    <= TOOL_PATH_MAX_LINEAR_STEP_M + f64::EPSILON
            );
            assert!(
                wrap_angle_pi(current.base_theta_rad - previous.base_theta_rad).abs()
                    <= TOOL_PATH_MAX_ANGULAR_STEP_RAD + f64::EPSILON
            );
            for (current, previous) in current
                .tendon_joint_angles()
                .iter()
                .zip(previous.tendon_joint_angles())
            {
                assert!(
                    (current - previous).abs()
                        <= TOOL_PATH_MAX_ANGULAR_STEP_RAD + f64::EPSILON
                );
            }
            previous = current;
        }
    }

    #[test]
    fn duplicate_ids_are_rejected() {
        let mut simulation = Simulation::new(SimulationConfig::default()).unwrap();
        let make = || {
            RigidBody::new(
                BodyId(1),
                Shape::Sphere { radius_m: 1.0 },
                Pose::IDENTITY,
                MotionType::Static,
            )
        };
        simulation.add_body(make()).unwrap();
        assert_eq!(
            simulation.add_body(make()),
            Err(SimulationError::DuplicateBodyId)
        );
    }
}
