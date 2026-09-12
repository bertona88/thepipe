//! Fixed-step deterministic simulation state and simple impulse solver.

use crate::arm::{ArmError, ArmKinematics, ContinuumArm};
use crate::collision::{query_pair, Clearance, CollisionReport, CollisionSettings, Contact};
use crate::geometry::{BodyId, MotionType, RigidBody, Shape};
use crate::gripper::{GripperConfig, GripperState};
use crate::machine::{
    wrap_angle_pi, CarriageConfig, MachineBackend, MachineCommand, MachineCommandError,
    MachineCommandEvent, ManipulatorMotionConfig, ManipulatorMotionState, ToolMotionPlan,
    ToolMotionStatus,
};
use crate::math::{Pose, Quat, Vec3};
use crate::serial_arm::{SerialArm, SerialArmError, SerialArmKinematics, ToolPositionIkError};

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
const DISTAL_TOOL_PATH_MAX_SAMPLE_COUNT: usize = 32_768;
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

/// Tool IDs preserve existing link identifiers.
pub fn serial_arm_tool_body_id(arm_id: ArmId, index: u8) -> Option<BodyId> {
    (arm_id.0 <= 0x01ff_ffff && index < 5)
        .then_some(BodyId(0xB000_0000 | (arm_id.0 << 3) | index as u32))
}
fn serial_arm_tool_key(id: BodyId) -> Option<(ArmId, u8)> {
    (id.0 >= 0xB000_0000 && id.0 < 0xC000_0000 && (id.0 & 7) < 5)
        .then_some((ArmId((id.0 & 0x0fff_ffff) >> 3), (id.0 & 7) as u8))
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToolPathCollisionDiagnostic {
    pub arm_id: ArmId,
    pub sample_index: usize,
    pub progress: f64,
    pub moving_body_id: BodyId,
    pub obstacle_body_id: BodyId,
    pub signed_distance_m: f64,
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

fn intended_robot_interface(a: BodyId, b: BodyId) -> bool {
    if let (Some((aa, la)), Some((ab, lb))) = (serial_arm_link_key(a), serial_arm_link_key(b)) {
        return aa == ab && la.abs_diff(lb) <= 1;
    }
    if let (Some((aa, ta)), Some((ab, tb))) = (serial_arm_tool_key(a), serial_arm_tool_key(b)) {
        return aa == ab
            && ((ta == 0 && (tb == 1 || tb == 2))
                || (tb == 0 && (ta == 1 || ta == 2))
                || ta.abs_diff(tb) == 2 && ta.min(tb) >= 1 && ta.min(tb) <= 2);
    }
    let pair = serial_arm_link_key(a)
        .zip(serial_arm_tool_key(b))
        .or_else(|| serial_arm_link_key(b).zip(serial_arm_tool_key(a)));
    pair.is_some_and(|((aa, l), (ab, t))| aa == ab && l == 2 && t == 0)
}

/// Carried parts always participate in collision checks against arm links.
/// World-body filters can intentionally narrow process contact (for example,
/// a peg against its socket), but they must not suppress robot self- or
/// inter-arm collision preflight.
fn carried_body_collision_enabled(carried: &RigidBody, obstacle: &RigidBody) -> bool {
    serial_arm_link_key(obstacle.id).is_some()
        || serial_arm_tool_key(obstacle.id).is_some()
        || carried.collision_filter.allows(obstacle.collision_filter)
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
    /// Explicit static support interface for the carried part, at most 2um overlap.
    pub support_body_ids: Vec<BodyId>,
    /// Explicit acquisition target; only pad contacts within compliance are intended.
    pub grasp_target_body_id: Option<BodyId>,
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
            support_body_ids: Vec::new(),
            grasp_target_body_id: None,
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
    pub last_tool_path_collision: Option<ToolPathCollisionDiagnostic>,
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
            last_tool_path_collision: None,
            accumulator_s: 0.0,
        })
    }

    pub fn add_body(&mut self, body: RigidBody) -> Result<(), SimulationError> {
        if body.id.0 >= 0xB000_0000 {
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
        if arm.id.0 > MAX_SERIAL_ARM_COLLISION_ID
            || (arm.gripper_config.distal_geometry.is_some()
                && serial_arm_tool_body_id(arm.id, 0).is_none())
        {
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

            let tool_target = match command {
                MachineCommand::SetToolPoseTarget {
                    target_position_world_m,
                    ..
                } => Some((target_position_world_m, None)),
                MachineCommand::SetToolAxisTarget {
                    target_position_world_m,
                    target_axis_world,
                    ..
                } => Some((target_position_world_m, Some(target_axis_world))),
                _ => None,
            };
            let tool_plan = if let Some((target_position_world_m, target_axis_world)) = tool_target
            {
                let positions = match target_axis_world {
                    Some(axis) => arm
                        .arm
                        .solve_tool_axis(target_position_world_m, axis, arm.motion.positions())
                        .map(|solution| solution.positions),
                    None => arm
                        .arm
                        .solve_tool_position(target_position_world_m, arm.motion.positions())
                        .map(|solution| solution.positions),
                }
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
                let mut plan = ToolMotionPlan::new(
                    target_position_world_m,
                    arm.motion.positions(),
                    positions,
                    arm.carriage_config,
                    arm.motion_config,
                    arm.tool_motion_speed_scale,
                );
                plan.target_axis_world = target_axis_world;
                self.validate_tool_motion_path(arm_id, plan)?;
                Some(plan)
            } else {
                None
            };

            if let MachineCommand::SetGripperOpening {
                target_opening_m, ..
            } = command
            {
                if self.serial_arms[arm_index]
                    .gripper_config
                    .distal_geometry
                    .is_some()
                {
                    self.validate_gripper_motion_path(arm_id, target_opening_m)?;
                }
            }
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

    /// Read the actual current geometry through the same path/contact policy.
    /// Does not command motion or advance time; useful after each plant step.
    pub fn validate_serial_arm_current_clearance(
        &mut self,
        arm_id: ArmId,
    ) -> Result<(), SimulationError> {
        let arm = self
            .serial_arm(arm_id)
            .ok_or(SimulationError::ArmNotFound)?;
        let plan = ToolMotionPlan::new(
            arm.tool_pose().translation,
            arm.motion.positions(),
            arm.motion.positions(),
            arm.carriage_config,
            arm.motion_config,
            arm.tool_motion_speed_scale,
        );
        self.validate_tool_motion_path(arm_id, plan)
    }

    fn validate_gripper_motion_path(
        &mut self,
        arm_id: ArmId,
        target: f64,
    ) -> Result<(), SimulationError> {
        let arm = self
            .serial_arm(arm_id)
            .ok_or(SimulationError::ArmNotFound)?
            .clone();
        let count = ((target - arm.gripper.opening_m).abs() / 10.0e-6)
            .ceil()
            .max(1.0) as usize;
        let mut probe = self.clone();
        let plan = ToolMotionPlan::new(
            arm.tool_pose().translation,
            arm.motion.positions(),
            arm.motion.positions(),
            arm.carriage_config,
            arm.motion_config,
            arm.tool_motion_speed_scale,
        );
        for i in 0..=count {
            probe.serial_arm_mut(arm_id).expect("arm").gripper.opening_m =
                arm.gripper.opening_m + (target - arm.gripper.opening_m) * i as f64 / count as f64;
            if let Err(error) = probe.validate_tool_motion_path(arm_id, plan) {
                self.last_tool_path_collision = probe.last_tool_path_collision;
                return Err(error);
            }
        }
        Ok(())
    }

    fn validate_tool_motion_path(
        &mut self,
        arm_id: ArmId,
        plan: ToolMotionPlan,
    ) -> Result<(), SimulationError> {
        self.last_tool_path_collision = None;
        let arm = self
            .serial_arm(arm_id)
            .ok_or(SimulationError::ArmNotFound)?
            .clone();
        let held = match (arm.gripper.held_body, arm.held_body_local_pose) {
            (Some(id), Some(local)) => Some((
                self.body(id).ok_or(SimulationError::BodyNotFound)?.clone(),
                local,
            )),
            (None, None) => None,
            _ => return Err(SimulationError::GraspRejected),
        };
        let mut obstacles: Vec<_> = self
            .bodies
            .iter()
            .filter(|b| b.enabled && Some(b.id) != arm.gripper.held_body)
            .cloned()
            .collect();
        obstacles.extend(self.serial_arm_collision_bodies().into_iter().filter(|b| {
            serial_arm_link_key(b.id).map(|k| k.0) != Some(arm_id)
                && serial_arm_tool_key(b.id).map(|k| k.0) != Some(arm_id)
        }));
        let mut samples = tool_path_sample_count(plan)?;
        if arm.gripper_config.distal_geometry.is_some() {
            // Bound nominal distal displacement per sample to 50um using the
            // sum of all rotating-axis contributions, not only TCP translation.
            let angular_sum: f64 = plan
                .start
                .tendon_joint_angles()
                .iter()
                .zip(plan.goal.tendon_joint_angles())
                .map(|(a, b)| (b - a).abs())
                .sum();
            let travel_bound = angular_sum * arm.arm.config.maximum_reach_m()
                + wrap_angle_pi(plan.goal.base_theta_rad - plan.start.base_theta_rad).abs()
                    * (arm.arm.config.rail_radius_m + arm.arm.config.maximum_reach_m())
                + (plan.goal.base_z_m - plan.start.base_z_m).abs();
            let required = (SMOOTHSTEP_MAX_SLOPE * travel_bound / 50.0e-6).ceil();
            if !required.is_finite() || required > DISTAL_TOOL_PATH_MAX_SAMPLE_COUNT as f64 {
                return Err(SimulationError::InvalidMachineCommand(
                    MachineCommandError::ToolPathSamplingLimit,
                ));
            }
            samples = samples.max(required as usize);
        }
        let mut candidate = arm.arm.clone();
        for sample in 0..=samples {
            let progress = sample as f64 / samples as f64;
            candidate
                .set_positions(plan.sample(progress))
                .map_err(SimulationError::SerialArm)?;
            let fk = candidate.forward_kinematics();
            let mut robot: Vec<_> = fk
                .collision_capsules
                .iter()
                .enumerate()
                .map(|(i, (pose, shape))| {
                    RigidBody::new(
                        serial_arm_link_body_id(arm_id, i as u8).expect("valid arm"),
                        *shape,
                        *pose,
                        MotionType::Kinematic,
                    )
                })
                .collect();
            robot.extend(
                arm.gripper
                    .tool_collision_primitives(fk.tool_pose, arm.gripper_config)
                    .into_iter()
                    .map(|p| {
                        RigidBody::new(
                            serial_arm_tool_body_id(arm_id, p.component_index)
                                .expect("valid tool id"),
                            p.shape,
                            p.pose,
                            MotionType::Kinematic,
                        )
                    }),
            );
            let mut collision = None;
            for body in &robot {
                for obstacle in &obstacles {
                    if !body.collision_filter.allows(obstacle.collision_filter) {
                        continue;
                    }
                    if let Some(p) = query_pair(body, obstacle) {
                        let intended_pad = serial_arm_tool_key(body.id)
                            .is_some_and(|k| k.0 == arm_id && k.1 >= 3)
                            && arm.grasp_target_body_id == Some(obstacle.id)
                            && p.signed_distance_m >= -2.0 * arm.gripper_config.pad_compliance_m
                            && arm
                                .gripper
                                .evaluate_candidate(fk.tool_pose, obstacle, arm.gripper_config)
                                .is_reachable(arm.gripper_config);
                        if p.signed_distance_m <= self.config.collision.clearance_threshold_m
                            && !intended_pad
                        {
                            collision = Some(p);
                            break;
                        }
                    }
                }
                if collision.is_some() {
                    break;
                }
            }
            if collision.is_none() {
                if let Some((body, local)) = &held {
                    let mut carried = body.clone();
                    carried.pose = fk.tool_pose * *local;
                    for obstacle in obstacles.iter().chain(robot.iter()) {
                        let own_link = serial_arm_link_key(obstacle.id).filter(|k| k.0 == arm_id);
                        let own_tool = serial_arm_tool_key(obstacle.id).filter(|k| k.0 == arm_id);
                        if arm.gripper_config.distal_geometry.is_none()
                            && own_link.is_some_and(|k| k.1 == 2)
                        {
                            continue;
                        }
                        if !carried_body_collision_enabled(&carried, obstacle) && own_tool.is_none()
                        {
                            continue;
                        }
                        let mut checked_carried = carried.clone();
                        let mut checked_obstacle = obstacle.clone();
                        if serial_arm_link_key(obstacle.id).is_some()
                            || serial_arm_tool_key(obstacle.id).is_some()
                        {
                            checked_carried.collision_filter =
                                crate::geometry::CollisionFilter::default();
                            checked_obstacle.collision_filter =
                                crate::geometry::CollisionFilter::default();
                        }
                        if let Some(p) = query_pair(&checked_carried, &checked_obstacle) {
                            let pad_contact = own_tool.is_some_and(|k| k.1 >= 3)
                                && p.signed_distance_m
                                    >= -2.0 * arm.gripper_config.pad_compliance_m
                                && arm
                                    .gripper
                                    .evaluate_held_candidate(
                                        fk.tool_pose,
                                        &carried,
                                        arm.gripper_config,
                                    )
                                    .is_reachable(arm.gripper_config);
                            let support_contact = arm.support_body_ids.contains(&obstacle.id)
                                && obstacle.motion == MotionType::Static
                                && matches!(obstacle.shape, Shape::Sphere { .. })
                                && p.signed_distance_m >= -2.0e-6;
                            if p.signed_distance_m <= self.config.collision.clearance_threshold_m
                                && !pad_contact
                                && !support_contact
                            {
                                collision = Some(p);
                                break;
                            }
                        }
                    }
                }
            }
            if collision.is_none() {
                // Nonadjacent links and upstream/tool interference. Wrist/palm
                // and tool internal interfaces are intentional physical mounts.
                for i in 0..robot.len() {
                    for j in i + 1..robot.len() {
                        if intended_robot_interface(robot[i].id, robot[j].id) {
                            continue;
                        }
                        if let Some(p) = query_pair(&robot[i], &robot[j]) {
                            if p.signed_distance_m <= self.config.collision.clearance_threshold_m {
                                collision = Some(p);
                                break;
                            }
                        }
                    }
                    if collision.is_some() {
                        break;
                    }
                }
            }
            if let Some(p) = collision {
                let a_moves = robot.iter().any(|b| b.id == p.body_a)
                    || arm.gripper.held_body == Some(p.body_a);
                self.last_tool_path_collision = Some(ToolPathCollisionDiagnostic {
                    arm_id,
                    sample_index: sample,
                    progress,
                    moving_body_id: if a_moves { p.body_a } else { p.body_b },
                    obstacle_body_id: if a_moves { p.body_b } else { p.body_a },
                    signed_distance_m: p.signed_distance_m,
                });
                return Err(SimulationError::InvalidMachineCommand(
                    MachineCommandError::ToolPathCollision,
                ));
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
        for instance in &self.serial_arms {
            for p in instance.gripper.tool_collision_primitives(
                instance.arm.forward_kinematics().tool_pose,
                instance.gripper_config,
            ) {
                if let Some(id) = serial_arm_tool_body_id(instance.id, p.component_index) {
                    result.push(RigidBody::new(id, p.shape, p.pose, MotionType::Kinematic));
                }
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
                if intended_robot_interface(a.id, b.id) {
                    continue;
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
        if !body.enabled || body.motion == MotionType::Static {
            return Err(SimulationError::GraspRejected);
        }
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
        if !body.enabled || body.motion == MotionType::Static {
            return Err(SimulationError::GraspRejected);
        }
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

    /// Acquire a body with the explicit partial axial-overlap gripper model.
    ///
    /// The legacy [`Self::grasp_body_serial`] path continues to require full
    /// axial containment. This entry point is intended for a shaft or capsule
    /// whose center is axially offset from the tool while enough material
    /// remains between both pads. The gripper retains the supplied overlap
    /// threshold and reapplies it on every held-contact refresh.
    pub fn grasp_body_serial_with_partial_axial_overlap(
        &mut self,
        arm_id: ArmId,
        body_id: BodyId,
        minimum_axial_overlap_m: f64,
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
        if !body.enabled || body.motion == MotionType::Static {
            return Err(SimulationError::GraspRejected);
        }
        let arm_index = self
            .serial_arms
            .binary_search_by_key(&arm_id, |arm| arm.id)
            .map_err(|_| SimulationError::ArmNotFound)?;
        let arm = &mut self.serial_arms[arm_index];
        let tool_pose = arm.tool_pose();
        let candidate = arm.gripper.evaluate_partial_axial_overlap_candidate(
            tool_pose,
            &body,
            arm.gripper_config,
            minimum_axial_overlap_m,
        );
        if !arm.gripper.try_grasp(candidate, arm.gripper_config) {
            return Err(SimulationError::GraspRejected);
        }
        arm.held_body_local_pose = Some(tool_pose.inverse() * body.pose);
        Ok(())
    }

    /// Atomic stationary ownership transfer after an observed controller authorizes
    /// handoff. This is a kinematic reduced-model transaction, not a dual-grasp
    /// force solver. Every fallible check precedes mutation; rejection retains
    /// the donor and the exact body pose. Collision admission belongs to the
    /// caller's handoff corridor, just as for standalone grasp admission.
    pub fn handoff_body_serial(
        &mut self,
        donor_id: ArmId,
        receiver_id: ArmId,
        body_id: BodyId,
        minimum_axial_overlap_m: f64,
    ) -> Result<(), SimulationError> {
        if donor_id == receiver_id {
            return Err(SimulationError::GraspRejected);
        }
        let donor_index = self
            .serial_arms
            .binary_search_by_key(&donor_id, |arm| arm.id)
            .map_err(|_| SimulationError::ArmNotFound)?;
        let receiver_index = self
            .serial_arms
            .binary_search_by_key(&receiver_id, |arm| arm.id)
            .map_err(|_| SimulationError::ArmNotFound)?;
        let donor = &self.serial_arms[donor_index];
        let receiver = &self.serial_arms[receiver_index];
        if donor.gripper.held_body != Some(body_id)
            || donor.held_body_local_pose.is_none()
            || receiver.gripper.held_body.is_some()
            || receiver.held_body_local_pose.is_some()
            || self
                .arms
                .iter()
                .any(|arm| arm.gripper.held_body == Some(body_id))
            || self
                .serial_arms
                .iter()
                .filter(|arm| arm.gripper.held_body == Some(body_id))
                .count()
                != 1
        {
            return Err(SimulationError::GraspRejected);
        }
        for arm in [donor, receiver] {
            if !arm.gripper.opening_m.is_finite()
                || !arm.gripper.command_opening_m.is_finite()
                || !arm.motion.carriage.z_m.is_finite()
                || !arm.motion.carriage.theta_rad.is_finite()
                || !arm.motion.carriage_target.z_m.is_finite()
                || !arm.motion.carriage_target.theta_rad.is_finite()
                || arm
                    .motion
                    .joint_targets_rad
                    .iter()
                    .zip(arm.motion.joint_positions_rad)
                    .any(|(target, position)| {
                        !target.is_finite()
                            || !position.is_finite()
                            || (target - position).abs() > 1.0e-12
                    })
                || (arm.motion.carriage_target.z_m - arm.motion.carriage.z_m).abs() > 1.0e-12
                || (arm.motion.carriage_target.theta_rad - arm.motion.carriage.theta_rad).abs()
                    > 1.0e-12
                || arm
                    .motion
                    .tool_motion
                    .is_some_and(|plan| plan.status == ToolMotionStatus::Active)
                || arm
                    .motion
                    .joint_velocities_rad_s
                    .iter()
                    .any(|v| !v.is_finite() || v.abs() > 1.0e-12)
                || !arm.motion.carriage.z_velocity_m_s.is_finite()
                || arm.motion.carriage.z_velocity_m_s.abs() > 1.0e-12
                || !arm.motion.carriage.theta_velocity_rad_s.is_finite()
                || arm.motion.carriage.theta_velocity_rad_s.abs() > 1.0e-12
                || !arm.gripper.opening_velocity_m_s.is_finite()
                || arm.gripper.opening_velocity_m_s.abs() > 1.0e-12
                || (arm.gripper.command_opening_m - arm.gripper.opening_m).abs() > 1.0e-12
            {
                return Err(SimulationError::GraspRejected);
            }
        }
        let body = self.body(body_id).ok_or(SimulationError::BodyNotFound)?;
        if !body.enabled || body.motion == MotionType::Static {
            return Err(SimulationError::GraspRejected);
        }
        // Recheck donor retention as well as receiver acquisition at this tick.
        // Preserve the donor's acquisition policy. The receiver's requested
        // overlap must not weaken the retention gate of the existing grasp.
        let donor_candidate =
            donor
                .gripper
                .evaluate_held_candidate(donor.tool_pose(), body, donor.gripper_config);
        let mut donor_check = donor.gripper;
        donor_check.release();
        if !donor_check.try_grasp(donor_candidate, donor.gripper_config) {
            return Err(SimulationError::GraspRejected);
        }
        let receiver_pose = receiver.tool_pose();
        let candidate = receiver.gripper.evaluate_partial_axial_overlap_candidate(
            receiver_pose,
            body,
            receiver.gripper_config,
            minimum_axial_overlap_m,
        );
        let mut receiver_gripper = receiver.gripper;
        if !receiver_gripper.try_grasp(candidate, receiver.gripper_config) {
            return Err(SimulationError::GraspRejected);
        }
        let local_pose = receiver_pose.inverse() * body.pose;
        self.serial_arms[donor_index].gripper.release();
        self.serial_arms[donor_index].held_body_local_pose = None;
        self.serial_arms[receiver_index].gripper = receiver_gripper;
        self.serial_arms[receiver_index].held_body_local_pose = Some(local_pose);
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

    /// Reduced-model release onto three registered static spherical supports.
    /// Checks geometry and stopped motion, not qualified contact force.
    pub fn release_body_serial_on_support(
        &mut self,
        arm_id: ArmId,
        support_ids: &[BodyId],
        max_gap_m: f64,
    ) -> Result<Option<BodyId>, SimulationError> {
        let arm = self
            .serial_arm(arm_id)
            .ok_or(SimulationError::ArmNotFound)?;
        let stationary = |v: f64| v.is_finite() && v.abs() <= 1.0e-12;
        if support_ids.len() != 3
            || !max_gap_m.is_finite()
            || !(0.0..=20.0e-6).contains(&max_gap_m)
            || !arm.motion.positions().is_finite()
            || !arm.motion.carriage_target.z_m.is_finite()
            || !arm.motion.carriage_target.theta_rad.is_finite()
            || !stationary(arm.motion.carriage_target.z_m - arm.motion.carriage.z_m)
            || !stationary(wrap_angle_pi(
                arm.motion.carriage_target.theta_rad - arm.motion.carriage.theta_rad,
            ))
            || arm
                .motion
                .joint_targets_rad
                .iter()
                .zip(arm.motion.joint_positions_rad)
                .any(|(a, b)| !a.is_finite() || !stationary(a - b))
            || arm
                .motion
                .joint_velocities_rad_s
                .iter()
                .any(|v| !stationary(*v))
            || !stationary(arm.motion.carriage.z_velocity_m_s)
            || !stationary(arm.motion.carriage.theta_velocity_rad_s)
            || !arm.gripper.opening_m.is_finite()
            || !arm.gripper.command_opening_m.is_finite()
            || !stationary(arm.gripper.command_opening_m - arm.gripper.opening_m)
            || !stationary(arm.gripper.opening_velocity_m_s)
            || arm
                .motion
                .tool_motion
                .is_some_and(|p| p.status == ToolMotionStatus::Active)
        {
            return Err(SimulationError::GraspRejected);
        }
        let body = self
            .body(
                arm.gripper
                    .held_body
                    .ok_or(SimulationError::GraspRejected)?,
            )
            .ok_or(SimulationError::BodyNotFound)?;
        if !body.enabled
            || body.motion == MotionType::Static
            || !body.pose.translation.is_finite()
            || !body.pose.rotation.is_finite()
            || arm.held_body_local_pose.is_none()
        {
            return Err(SimulationError::GraspRejected);
        }
        let mut local = Vec::new();
        for (i, id) in support_ids.iter().enumerate() {
            if support_ids[..i].contains(id) || !arm.support_body_ids.contains(id) {
                return Err(SimulationError::GraspRejected);
            }
            let support = self.body(*id).ok_or(SimulationError::BodyNotFound)?;
            if support.motion != MotionType::Static
                || !support.enabled
                || !matches!(support.shape, Shape::Sphere { .. })
            {
                return Err(SimulationError::GraspRejected);
            }
            let gap = query_pair(body, support)
                .ok_or(SimulationError::GraspRejected)?
                .signed_distance_m;
            if !gap.is_finite() || gap < -2.0e-6 || gap > max_gap_m {
                return Err(SimulationError::GraspRejected);
            }
            local.push(
                body.pose
                    .inverse()
                    .transform_point(support.pose.translation),
            );
        }
        // The coupon mating axis is body-local Z. Supports must bracket the
        // centre on the positive face; duplicate/collinear contacts fail.
        if local.iter().any(|p| p.z <= 0.0) {
            return Err(SimulationError::GraspRejected);
        }
        let crosses: Vec<_> = (0..3)
            .map(|i| {
                let a = local[i];
                let b = local[(i + 1) % 3];
                a.x * b.y - a.y * b.x
            })
            .collect();
        if !(crosses.iter().all(|c| *c > 1.0e-12) || crosses.iter().all(|c| *c < -1.0e-12)) {
            return Err(SimulationError::GraspRejected);
        }
        self.release_body_serial(arm_id)
    }

    fn refresh_grasp_contacts(&mut self) {
        for arm in &mut self.arms {
            let Some(body_id) = arm.gripper.held_body else {
                continue;
            };
            let mut body = self
                .bodies
                .binary_search_by_key(&body_id, |body| body.id)
                .ok()
                .map(|index| self.bodies[index].clone());
            let retained = if let (Some(body), Some(local_pose)) =
                (body.as_mut(), arm.held_body_local_pose)
            {
                body.pose = arm.tool_pose() * local_pose;
                let candidate =
                    arm.gripper
                        .evaluate_held_candidate(arm.tool_pose(), body, arm.gripper_config);
                body.enabled
                    && body.motion != MotionType::Static
                    && arm
                        .gripper
                        .update_held_contact(candidate, arm.gripper_config)
            } else {
                false
            };
            if !retained {
                arm.gripper.release();
                arm.held_body_local_pose = None;
            }
        }
        for arm in &mut self.serial_arms {
            let Some(body_id) = arm.gripper.held_body else {
                continue;
            };
            let mut body = self
                .bodies
                .binary_search_by_key(&body_id, |body| body.id)
                .ok()
                .map(|index| self.bodies[index].clone());
            let retained = if let (Some(body), Some(local_pose)) =
                (body.as_mut(), arm.held_body_local_pose)
            {
                body.pose = arm.tool_pose() * local_pose;
                let candidate =
                    arm.gripper
                        .evaluate_held_candidate(arm.tool_pose(), body, arm.gripper_config);
                body.enabled
                    && body.motion != MotionType::Static
                    && arm
                        .gripper
                        .update_held_contact(candidate, arm.gripper_config)
            } else {
                false
            };
            if !retained {
                arm.gripper.release();
                arm.held_body_local_pose = None;
            }
        }
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
        self.refresh_grasp_contacts();

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
    use crate::geometry::{CollisionFilter, Material, Shape};
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
    fn axis_command_executes_through_tendons_and_preflights_before_commit() {
        let mut simulation = Simulation::new(SimulationConfig {
            fixed_dt_s: 0.001,
            gravity_m_s2: Vec3::ZERO,
            ..SimulationConfig::default()
        })
        .unwrap();
        let mut instance = baseline_serial_instance(1);
        let start = instance
            .arm
            .solve_tool_position(Vec3::new(0.020, 0.0, 0.0), instance.arm.positions)
            .unwrap()
            .positions;
        instance.arm.set_positions(start).unwrap();
        instance.motion = ManipulatorMotionState::from_positions(start);
        instance.kinematics = instance.arm.forward_kinematics();
        let mut reference = instance.arm.clone();
        reference
            .set_positions(crate::SerialJointPositions {
                shoulder_yaw_rad: 0.02,
                shoulder_pitch_rad: start.shoulder_pitch_rad + 0.01,
                ..start
            })
            .unwrap();
        let target = reference.forward_kinematics().tool_pose;
        simulation.add_serial_arm(instance).unwrap();
        let command = MachineCommand::SetToolAxisTarget {
            manipulator: ManipulatorId(1),
            target_position_world_m: target.translation,
            target_axis_world: target.transform_vector(Vec3::Z),
        };
        let mut collision = simulation.clone();
        collision
            .add_body(RigidBody::new(
                BodyId(987),
                Shape::Sphere { radius_m: 0.001 },
                Pose::from_translation(target.translation),
                MotionType::Static,
            ))
            .unwrap();
        assert!(collision.submit_machine_command(command).is_err());
        assert_eq!(collision.machine_command_sequence, 0);
        assert!(collision.machine_command_log.is_empty());
        assert!(collision
            .serial_arm(ArmId(1))
            .unwrap()
            .motion
            .tool_motion
            .is_none());
        simulation.submit_machine_command(command).unwrap();
        let mut replay = simulation.clone();
        for _ in 0..20_000 {
            simulation.step().unwrap();
            replay.step().unwrap();
            let arm = simulation.serial_arm(ArmId(1)).unwrap();
            if arm.motion.tool_motion.unwrap().status == ToolMotionStatus::Complete {
                break;
            }
        }
        let arm = simulation.serial_arm(ArmId(1)).unwrap();
        assert_eq!(
            arm.motion.tool_motion.unwrap().status,
            ToolMotionStatus::Complete
        );
        assert_eq!(arm.arm.positions, arm.motion.positions());
        assert!((arm.tool_pose().translation - target.translation).length() < 1.0e-9);
        assert!(
            (arm.tool_pose().transform_vector(Vec3::Z) - target.transform_vector(Vec3::Z)).length()
                < 1.0e-7
        );
        assert_eq!(simulation.tool_motion_trace, replay.tool_motion_trace);
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
            if simulation
                .serial_arm(ArmId(1))
                .unwrap()
                .motion
                .tool_motion
                .is_some_and(|plan| plan.status == ToolMotionStatus::Complete)
            {
                break;
            }
        }
        let arm = simulation.serial_arm(ArmId(1)).unwrap();
        let plan = arm
            .motion
            .tool_motion
            .expect("tool plan remains inspectable");
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
    fn cartesian_preflight_sweeps_the_attached_body() {
        const HELD_GROUP: u32 = 0b0010;
        const OBSTACLE_GROUP: u32 = 0b0100;

        let mut simulation = Simulation::new(SimulationConfig {
            gravity_m_s2: Vec3::ZERO,
            ..SimulationConfig::default()
        })
        .unwrap();
        simulation
            .add_serial_arm(baseline_serial_instance(1))
            .unwrap();
        let tool_pose = simulation.serial_arm(ArmId(1)).unwrap().tool_pose();
        let mut held = RigidBody::new(
            BodyId(7),
            Shape::Sphere { radius_m: 0.2e-3 },
            tool_pose,
            MotionType::Dynamic,
        );
        held.collision_filter = CollisionFilter {
            group: HELD_GROUP,
            mask: OBSTACLE_GROUP,
        };
        simulation.add_body(held).unwrap();
        simulation
            .serial_arm_mut(ArmId(1))
            .unwrap()
            .gripper
            .opening_m = 0.39e-3;
        simulation.grasp_body_serial(ArmId(1), BodyId(7)).unwrap();

        let target = Vec3::new(20.0e-3, 0.0, 0.0);
        let mut obstacle = RigidBody::new(
            BodyId(8),
            Shape::Sphere { radius_m: 0.3e-3 },
            Pose::from_translation(target),
            MotionType::Static,
        );
        obstacle.collision_filter = CollisionFilter {
            group: OBSTACLE_GROUP,
            mask: HELD_GROUP,
        };
        simulation.add_body(obstacle).unwrap();

        let result = simulation.submit_machine_command(MachineCommand::SetToolPoseTarget {
            manipulator: ManipulatorId(1),
            target_position_world_m: target,
        });
        assert_eq!(
            result,
            Err(SimulationError::InvalidMachineCommand(
                MachineCommandError::ToolPathCollision
            ))
        );
        assert_eq!(simulation.machine_command_sequence, 0);
        assert_eq!(
            simulation.serial_arm(ArmId(1)).unwrap().gripper.held_body,
            Some(BodyId(7))
        );
    }

    #[test]
    fn serial_partial_axial_grasp_is_opt_in_and_persists_across_steps() {
        let gripper_config = GripperConfig {
            jaw_half_extents_m: Vec3::new(100.0e-6, 250.0e-6, 200.0e-6),
            ..GripperConfig::default()
        };
        let arm = SerialArm::new(crate::serial_arm::SerialArmConfig::default()).unwrap();
        let arm = SerialArmInstance::new(ArmId(1), arm, gripper_config).unwrap();
        let mut simulation = Simulation::new(SimulationConfig {
            gravity_m_s2: Vec3::ZERO,
            ..SimulationConfig::default()
        })
        .unwrap();
        simulation.add_serial_arm(arm).unwrap();
        let tool_pose = simulation.serial_arm(ArmId(1)).unwrap().tool_pose();
        let body_local_pose = Pose::new(Vec3::Z * 0.75e-3, Quat::from_axis_angle(Vec3::Y, 0.015));
        simulation
            .add_body(RigidBody::new(
                BodyId(7),
                Shape::Capsule {
                    radius_m: 0.2e-3,
                    half_segment_m: 0.7e-3,
                },
                tool_pose * body_local_pose,
                MotionType::Dynamic,
            ))
            .unwrap();
        let gripper = &mut simulation.serial_arm_mut(ArmId(1)).unwrap().gripper;
        gripper.opening_m = 0.38e-3;
        gripper.command_opening_m = 0.38e-3;

        assert_eq!(
            simulation.grasp_body_serial(ArmId(1), BodyId(7)),
            Err(SimulationError::GraspRejected)
        );
        simulation
            .grasp_body_serial_with_partial_axial_overlap(
                ArmId(1),
                BodyId(7),
                crate::gripper::MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M,
            )
            .unwrap();
        assert_eq!(
            simulation
                .serial_arm(ArmId(1))
                .unwrap()
                .gripper
                .held_minimum_axial_overlap_m,
            Some(crate::gripper::MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M)
        );

        simulation.step().unwrap();
        assert_eq!(
            simulation.serial_arm(ArmId(1)).unwrap().gripper.held_body,
            Some(BodyId(7))
        );
        assert_eq!(
            simulation.release_body_serial(ArmId(1)).unwrap(),
            Some(BodyId(7))
        );
        assert_eq!(
            simulation
                .serial_arm(ArmId(1))
                .unwrap()
                .gripper
                .held_minimum_axial_overlap_m,
            None
        );
    }

    #[test]
    fn carried_body_filters_cannot_disable_arm_link_preflight() {
        const PROCESS_GROUP: u32 = 0b0010;
        const PROCESS_MASK: u32 = 0b0100;

        let mut carried = RigidBody::new(
            BodyId(7),
            Shape::Sphere { radius_m: 0.2e-3 },
            Pose::IDENTITY,
            MotionType::Dynamic,
        );
        carried.collision_filter = CollisionFilter {
            group: PROCESS_GROUP,
            mask: PROCESS_MASK,
        };
        let link = RigidBody::new(
            serial_arm_link_body_id(ArmId(1), 0).unwrap(),
            Shape::Sphere { radius_m: 0.2e-3 },
            Pose::IDENTITY,
            MotionType::Kinematic,
        );
        let ordinary_body = RigidBody::new(
            BodyId(8),
            Shape::Sphere { radius_m: 0.2e-3 },
            Pose::IDENTITY,
            MotionType::Static,
        );

        assert!(!carried.collision_filter.allows(link.collision_filter));
        assert!(carried_body_collision_enabled(&carried, &link));
        assert!(!carried_body_collision_enabled(&carried, &ordinary_body));
    }

    #[test]
    fn static_and_disabled_bodies_cannot_be_grasped() {
        let mut simulation = Simulation::new(SimulationConfig::default()).unwrap();
        simulation
            .add_serial_arm(baseline_serial_instance(1))
            .unwrap();
        let tool_pose = simulation.serial_arm(ArmId(1)).unwrap().tool_pose();
        simulation
            .add_body(RigidBody::new(
                BodyId(7),
                Shape::Sphere { radius_m: 0.2e-3 },
                tool_pose,
                MotionType::Static,
            ))
            .unwrap();
        let mut disabled = RigidBody::new(
            BodyId(8),
            Shape::Sphere { radius_m: 0.2e-3 },
            tool_pose,
            MotionType::Dynamic,
        );
        disabled.enabled = false;
        simulation.add_body(disabled).unwrap();
        simulation
            .serial_arm_mut(ArmId(1))
            .unwrap()
            .gripper
            .opening_m = 0.4e-3;

        assert_eq!(
            simulation.grasp_body_serial(ArmId(1), BodyId(7)),
            Err(SimulationError::GraspRejected)
        );
        assert_eq!(
            simulation.grasp_body_serial(ArmId(1), BodyId(8)),
            Err(SimulationError::GraspRejected)
        );
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
                    (current - previous).abs() <= TOOL_PATH_MAX_ANGULAR_STEP_RAD + f64::EPSILON
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
    fn distal_candidate() -> Simulation {
        let mut sim = Simulation::new(SimulationConfig {
            gravity_m_s2: Vec3::ZERO,
            ..SimulationConfig::default()
        })
        .unwrap();
        let mut arm = baseline_serial_instance(1);
        arm.arm.config.tool_standoff_m = 0.005;
        arm.gripper_config.jaw_half_extents_m = Vec3::new(0.0001, 0.0011, 0.0006);
        arm.gripper_config.distal_geometry = Some(crate::gripper::DistalToolGeometry {
            palm_center_z_m: -0.004,
            palm_half_extents_m: Vec3::new(0.0018, 0.0013, 0.001),
            finger_half_extents_xy_m: [0.0001, 0.0011],
        });
        arm.kinematics = arm.arm.forward_kinematics();
        sim.add_serial_arm(arm).unwrap();
        sim
    }
    #[test]
    fn physical_tool_collision_export_and_jaw_sweep_refuse_obstacles() {
        let mut sim = distal_candidate();
        assert_eq!(sim.serial_arm_collision_bodies().len(), 8);
        let tool = sim.serial_arm(ArmId(1)).unwrap().tool_pose();
        sim.add_body(RigidBody::new(
            BodyId(12),
            Shape::Sphere { radius_m: 0.00005 },
            tool * Pose::from_translation(Vec3::new(0.0012, 0.0, 0.0)),
            MotionType::Static,
        ))
        .unwrap();
        let before = sim.serial_arm(ArmId(1)).unwrap().gripper;
        assert!(sim
            .submit_machine_command(MachineCommand::SetGripperOpening {
                manipulator: ManipulatorId(1),
                target_opening_m: 0.002
            })
            .is_err());
        assert_eq!(sim.serial_arm(ArmId(1)).unwrap().gripper, before);
        let d = sim.last_tool_path_collision.unwrap();
        assert_eq!(d.obstacle_body_id, BodyId(12));
        assert!(serial_arm_tool_key(d.moving_body_id).is_some());
    }
    #[test]
    fn intended_target_allows_only_pad_contact_and_not_palm_interference() {
        let mut sim = distal_candidate();
        let tool = sim.serial_arm(ArmId(1)).unwrap().tool_pose();
        sim.add_body(RigidBody::new(
            BodyId(12),
            Shape::Box {
                half_extents_m: Vec3::new(0.001, 0.001, 0.0004),
            },
            tool,
            MotionType::Kinematic,
        ))
        .unwrap();
        sim.serial_arm_mut(ArmId(1)).unwrap().grasp_target_body_id = Some(BodyId(12));
        sim.submit_machine_command(MachineCommand::SetGripperOpening {
            manipulator: ManipulatorId(1),
            target_opening_m: 0.001995,
        })
        .unwrap();
        sim.body_mut(BodyId(12)).unwrap().pose =
            tool * Pose::from_translation(Vec3::new(0.0, 0.0, -0.004));
        assert!(sim
            .submit_machine_command(MachineCommand::SetGripperOpening {
                manipulator: ManipulatorId(1),
                target_opening_m: 0.00199
            })
            .is_err());
    }
    #[test]
    fn support_release_requires_three_registered_contacts_and_stopped_motion() {
        let mut sim = distal_candidate();
        let tool = sim.serial_arm(ArmId(1)).unwrap().tool_pose();
        sim.add_body(RigidBody::new(
            BodyId(12),
            Shape::Box {
                half_extents_m: Vec3::new(0.001, 0.001, 0.0004),
            },
            tool,
            MotionType::Kinematic,
        ))
        .unwrap();
        {
            let arm = sim.serial_arm_mut(ArmId(1)).unwrap();
            arm.gripper.opening_m = 0.001995;
            arm.gripper.command_opening_m = 0.001995;
        }
        sim.grasp_body_serial(ArmId(1), BodyId(12)).unwrap();
        let ids = [BodyId(20), BodyId(21), BodyId(22)];
        for (i, id) in ids.iter().enumerate() {
            let theta = i as f64 * core::f64::consts::TAU / 3.0;
            sim.add_body(RigidBody::new(
                *id,
                Shape::Sphere { radius_m: 0.00015 },
                tool * Pose::from_translation(Vec3::new(
                    0.0006 * theta.cos(),
                    0.0006 * theta.sin(),
                    0.00055,
                )),
                MotionType::Static,
            ))
            .unwrap();
        }
        assert!(sim
            .release_body_serial_on_support(ArmId(1), &ids, 2e-6)
            .is_err());
        sim.serial_arm_mut(ArmId(1)).unwrap().support_body_ids = ids.to_vec();
        assert!(sim
            .release_body_serial_on_support(ArmId(1), &[ids[0], ids[0], ids[2]], 2e-6)
            .is_err());
        sim.serial_arm_mut(ArmId(1))
            .unwrap()
            .motion
            .carriage
            .z_velocity_m_s = 0.001;
        assert!(sim
            .release_body_serial_on_support(ArmId(1), &ids, 2e-6)
            .is_err());
        sim.serial_arm_mut(ArmId(1))
            .unwrap()
            .motion
            .carriage
            .z_velocity_m_s = 0.0;
        sim.serial_arm_mut(ArmId(1))
            .unwrap()
            .motion
            .joint_targets_rad[0] += 0.01;
        assert!(sim
            .release_body_serial_on_support(ArmId(1), &ids, 2e-6)
            .is_err());
        sim.serial_arm_mut(ArmId(1))
            .unwrap()
            .motion
            .joint_targets_rad[0] -= 0.01;
        sim.serial_arm_mut(ArmId(1))
            .unwrap()
            .gripper
            .opening_velocity_m_s = f64::NAN;
        assert!(sim
            .release_body_serial_on_support(ArmId(1), &ids, 2e-6)
            .is_err());
        sim.serial_arm_mut(ArmId(1))
            .unwrap()
            .gripper
            .opening_velocity_m_s = 0.0;
        sim.serial_arm_mut(ArmId(1))
            .unwrap()
            .gripper
            .command_opening_m += 0.0001;
        assert!(sim
            .release_body_serial_on_support(ArmId(1), &ids, 2e-6)
            .is_err());
        sim.serial_arm_mut(ArmId(1))
            .unwrap()
            .gripper
            .command_opening_m -= 0.0001;
        assert_eq!(
            sim.release_body_serial_on_support(ArmId(1), &ids, 2e-6)
                .unwrap(),
            Some(BodyId(12))
        );
    }
    #[test]
    fn distal_ids_and_forced_tool_contact_do_not_depend_on_part_masks() {
        let mut sim = distal_candidate();
        let mut arm = sim.serial_arm(ArmId(1)).unwrap().clone();
        arm.id = ArmId(0x0200_0000);
        assert_eq!(
            sim.add_serial_arm(arm),
            Err(SimulationError::ArmIdOutOfRange)
        );
        let tool = sim.serial_arm(ArmId(1)).unwrap().tool_pose();
        let mut body = RigidBody::new(
            BodyId(12),
            Shape::Box {
                half_extents_m: Vec3::new(0.001, 0.001, 0.0004),
            },
            tool,
            MotionType::Kinematic,
        );
        body.collision_filter = CollisionFilter { group: 0, mask: 0 };
        sim.add_body(body).unwrap();
        {
            let a = sim.serial_arm_mut(ArmId(1)).unwrap();
            a.gripper.opening_m = 0.001995;
            a.gripper.command_opening_m = 0.001995;
        }
        sim.grasp_body_serial(ArmId(1), BodyId(12)).unwrap();
        // Deliberately corrupted attachment reaches the palm. No world filter
        // may hide held-part interference with physical robot geometry.
        sim.serial_arm_mut(ArmId(1)).unwrap().held_body_local_pose =
            Some(Pose::from_translation(Vec3::new(0.0, 0.0, -0.004)));
        assert!(sim.validate_serial_arm_current_clearance(ArmId(1)).is_err());
    }

    #[test]
    fn physical_tool_sampling_remains_computationally_bounded() {
        let mut sim = distal_candidate();
        let arm = sim.serial_arm(ArmId(1)).unwrap();
        let mut goal = arm.motion.positions();
        goal.shoulder_yaw_rad += 20.0;
        let plan = ToolMotionPlan::new(
            arm.tool_pose().translation,
            arm.motion.positions(),
            goal,
            arm.carriage_config,
            arm.motion_config,
            1.0,
        );
        assert_eq!(
            sim.validate_tool_motion_path(ArmId(1), plan),
            Err(SimulationError::InvalidMachineCommand(
                MachineCommandError::ToolPathSamplingLimit
            ))
        );
    }
}
