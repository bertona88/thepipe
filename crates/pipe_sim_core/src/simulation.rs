//! Fixed-step deterministic simulation state and simple impulse solver.

use crate::arm::{ArmError, ArmKinematics, ContinuumArm};
use crate::collision::{query_pair, Clearance, CollisionReport, CollisionSettings, Contact};
use crate::geometry::{BodyId, MotionType, RigidBody};
use crate::gripper::{GripperConfig, GripperState};
use crate::math::{Pose, Quat, Vec3};
use crate::serial_arm::{SerialArm, SerialArmKinematics};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ArmId(pub u32);

/// IDs at or above this value are reserved for ephemeral serial-arm link
/// colliders returned by [`Simulation::serial_arm_collision_bodies`].
pub const SERIAL_ARM_COLLISION_BODY_ID_BASE: u32 = 0xC000_0000;
const MAX_SERIAL_ARM_COLLISION_ID: u32 = 0x0FFF_FFFF;
const SERIAL_ARM_LINK_COUNT: u8 = 3;

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
        Ok(Self {
            id,
            arm,
            gripper_config,
            gripper: GripperState::new(gripper_config.max_opening_m, gripper_config),
            kinematics,
            held_body_local_pose: None,
        })
    }

    pub fn tool_pose(&self) -> Pose {
        self.kinematics.tool_pose
    }

    fn step(&mut self, dt_s: f64) {
        self.gripper.step(dt_s, self.gripper_config);
        self.kinematics = self.arm.forward_kinematics();
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
    BodyAlreadyHeld,
    GraspRejected,
    InvalidElapsedTime,
}

impl From<ArmError> for SimulationError {
    fn from(value: ArmError) -> Self {
        Self::Arm(value)
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
        for arm in &mut self.serial_arms {
            arm.step(dt_s);
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
