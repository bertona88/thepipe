//! Rapier-backed rigid-body dynamics for the Pipe assembly cell.
//!
//! All public lengths, velocities, and accelerations are SI `f64` values. The
//! default characteristic length is one millimetre so Rapier's normalized
//! tolerances remain useful for 0.1 mm-module gears.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

use pipe_sim_core::{BodyId, Pose, Quat, Vec3};
use rapier3d_f64::{
    dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
        RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
    },
    geometry::{
        BroadPhaseBvh, ColliderBuilder, ColliderHandle, ColliderSet, Group, InteractionGroups,
        InteractionTestMode, NarrowPhase, SharedShape,
    },
    math::{Pose as RapierPose, Rotation, Vector},
    parry::query,
    pipeline::PhysicsPipeline,
};

pub const DEFAULT_GEAR_MODULE_M: f64 = 0.1e-3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicsConfig {
    pub fixed_dt_s: f64,
    pub gravity_m_s2: Vec3,
    /// Characteristic object length used to scale Rapier's solver tolerances.
    pub length_unit_m: f64,
    pub solver_iterations: usize,
    pub max_ccd_substeps: usize,
    /// Contacts up to this positive separation are included in step reports.
    pub contact_prediction_m: f64,
    /// Non-contacting pairs no farther apart than this are reported.
    pub clearance_query_m: f64,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            fixed_dt_s: 0.5e-3,
            gravity_m_s2: Vec3::new(0.0, 0.0, -9.80665),
            length_unit_m: 1.0e-3,
            solver_iterations: 8,
            max_ccd_substeps: 8,
            contact_prediction_m: 2.0e-6,
            clearance_query_m: 25.0e-6,
        }
    }
}

impl PhysicsConfig {
    pub fn is_valid(self) -> bool {
        self.fixed_dt_s.is_finite()
            && self.fixed_dt_s > 0.0
            && self.gravity_m_s2.is_finite()
            && self.length_unit_m.is_finite()
            && self.length_unit_m > 0.0
            && self.solver_iterations > 0
            && self.max_ccd_substeps > 0
            && self.contact_prediction_m.is_finite()
            && self.contact_prediction_m >= 0.0
            && self.clearance_query_m.is_finite()
            && self.clearance_query_m >= self.contact_prediction_m
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyMotion {
    Fixed,
    Dynamic,
    KinematicPosition,
    KinematicVelocity,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CollisionGroups {
    pub memberships: u32,
    pub filter: u32,
}

impl CollisionGroups {
    pub const ALL: Self = Self {
        memberships: u32::MAX,
        filter: u32::MAX,
    };

    pub const fn new(memberships: u32, filter: u32) -> Self {
        Self {
            memberships,
            filter,
        }
    }

    fn permits(self, other: Self) -> bool {
        self.memberships & other.filter != 0 && other.memberships & self.filter != 0
    }

    fn rapier(self) -> InteractionGroups {
        InteractionGroups::new(
            Group::from_bits_truncate(self.memberships),
            Group::from_bits_truncate(self.filter),
            InteractionTestMode::And,
        )
    }
}

impl Default for CollisionGroups {
    fn default() -> Self {
        Self::ALL
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum PrimitiveShape {
    Sphere {
        radius_m: f64,
    },
    /// Segment endpoints are on the local Z axis.
    CapsuleZ {
        half_segment_m: f64,
        radius_m: f64,
    },
    Cuboid {
        half_extents_m: Vec3,
    },
    /// Cylinder axis is the local Y axis, matching Rapier/Parry.
    CylinderY {
        half_height_m: f64,
        radius_m: f64,
    },
    /// Conservative solid-cylinder envelope for early gearbox planning.
    GearEnvelope {
        tip_radius_m: f64,
        half_thickness_m: f64,
    },
}

impl PrimitiveShape {
    pub fn is_valid(&self) -> bool {
        match *self {
            Self::Sphere { radius_m } => positive(radius_m),
            Self::CapsuleZ {
                half_segment_m,
                radius_m,
            } => nonnegative(half_segment_m) && positive(radius_m),
            Self::Cuboid { half_extents_m } => {
                half_extents_m.is_finite()
                    && positive(half_extents_m.x)
                    && positive(half_extents_m.y)
                    && positive(half_extents_m.z)
            }
            Self::CylinderY {
                half_height_m,
                radius_m,
            }
            | Self::GearEnvelope {
                tip_radius_m: radius_m,
                half_thickness_m: half_height_m,
            } => positive(half_height_m) && positive(radius_m),
        }
    }

    fn shared_shape(&self) -> SharedShape {
        match *self {
            Self::Sphere { radius_m } => SharedShape::ball(radius_m),
            Self::CapsuleZ {
                half_segment_m,
                radius_m,
            } => SharedShape::capsule(
                Vector::new(0.0, 0.0, -half_segment_m),
                Vector::new(0.0, 0.0, half_segment_m),
                radius_m,
            ),
            Self::Cuboid { half_extents_m } => {
                SharedShape::cuboid(half_extents_m.x, half_extents_m.y, half_extents_m.z)
            }
            Self::CylinderY {
                half_height_m,
                radius_m,
            }
            | Self::GearEnvelope {
                tip_radius_m: radius_m,
                half_thickness_m: half_height_m,
            } => SharedShape::cylinder(half_height_m, radius_m),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ColliderPart {
    pub local_pose: Pose,
    pub shape: PrimitiveShape,
}

impl ColliderPart {
    pub fn new(shape: PrimitiveShape) -> Self {
        Self {
            local_pose: Pose::IDENTITY,
            shape,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum ColliderGeometry {
    Primitive(PrimitiveShape),
    Compound(Vec<ColliderPart>),
}

impl ColliderGeometry {
    pub fn is_valid(&self) -> bool {
        match self {
            Self::Primitive(shape) => shape.is_valid(),
            Self::Compound(parts) => {
                !parts.is_empty()
                    && parts.iter().all(|part| {
                        part.shape.is_valid()
                            && part.local_pose.translation.is_finite()
                            && part.local_pose.rotation.is_finite()
                    })
            }
        }
    }

    fn shared_shape(&self) -> SharedShape {
        match self {
            Self::Primitive(shape) => shape.shared_shape(),
            Self::Compound(parts) => SharedShape::compound(
                parts
                    .iter()
                    .map(|part| (to_iso(part.local_pose), part.shape.shared_shape()))
                    .collect(),
            ),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ColliderSpec {
    pub local_pose: Pose,
    pub geometry: ColliderGeometry,
    pub density_kg_m3: f64,
    pub friction: f64,
    pub restitution: f64,
    pub groups: CollisionGroups,
}

impl ColliderSpec {
    pub fn new(geometry: ColliderGeometry) -> Self {
        Self {
            local_pose: Pose::IDENTITY,
            geometry,
            density_kg_m3: 1_150.0,
            friction: 0.35,
            restitution: 0.03,
            groups: CollisionGroups::ALL,
        }
    }

    fn is_valid(&self) -> bool {
        self.geometry.is_valid()
            && positive(self.density_kg_m3)
            && nonnegative(self.friction)
            && self.restitution.is_finite()
            && (0.0..=1.0).contains(&self.restitution)
            && self.groups.memberships != 0
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BodySpec {
    pub id: BodyId,
    pub motion: BodyMotion,
    pub pose: Pose,
    pub linear_velocity_m_s: Vec3,
    pub angular_velocity_rad_s: Vec3,
    pub ccd_enabled: bool,
    pub colliders: Vec<ColliderSpec>,
}

impl BodySpec {
    pub fn new(id: BodyId, motion: BodyMotion, pose: Pose, collider: ColliderSpec) -> Self {
        Self {
            id,
            motion,
            pose,
            linear_velocity_m_s: Vec3::ZERO,
            angular_velocity_rad_s: Vec3::ZERO,
            ccd_enabled: false,
            colliders: vec![collider],
        }
    }

    fn is_valid(&self) -> bool {
        self.pose.translation.is_finite()
            && self.pose.rotation.is_finite()
            && self.linear_velocity_m_s.is_finite()
            && self.angular_velocity_rad_s.is_finite()
            && !self.colliders.is_empty()
            && self.colliders.iter().all(ColliderSpec::is_valid)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct BodyPair(BodyId, BodyId);

impl BodyPair {
    fn new(a: BodyId, b: BodyId) -> Self {
        if a <= b {
            Self(a, b)
        } else {
            Self(b, a)
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyState {
    pub id: BodyId,
    pub pose: Pose,
    pub linear_velocity_m_s: Vec3,
    pub angular_velocity_rad_s: Vec3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactReport {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub point_a_m: Vec3,
    pub point_b_m: Vec3,
    pub normal_a_to_b: Vec3,
    /// Negative when penetrating, positive for a speculative contact.
    pub separation_m: f64,
    pub penetration_m: f64,
    pub allowed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClearanceReport {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub clearance_m: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StepReport {
    pub step_index: u64,
    pub time_s: f64,
    pub contacts: Vec<ContactReport>,
    pub clearances: Vec<ClearanceReport>,
    pub maximum_penetration_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhysicsError {
    InvalidConfig,
    InvalidBody,
    DuplicateBodyId,
    BodyNotFound,
}

#[derive(Debug)]
struct ColliderEntry {
    handle: ColliderHandle,
    groups: CollisionGroups,
}

#[derive(Debug)]
struct BodyEntry {
    handle: RigidBodyHandle,
    colliders: Vec<ColliderEntry>,
}

/// A single deterministic, fixed-step Rapier world.
pub struct PhysicsWorld {
    config: PhysicsConfig,
    integration: IntegrationParameters,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    entries: BTreeMap<BodyId, BodyEntry>,
    allowed_contacts: BTreeSet<BodyPair>,
    step_index: u64,
    time_s: f64,
}

impl PhysicsWorld {
    pub fn new(config: PhysicsConfig) -> Result<Self, PhysicsError> {
        if !config.is_valid() {
            return Err(PhysicsError::InvalidConfig);
        }
        let integration = IntegrationParameters {
            dt: config.fixed_dt_s,
            length_unit: config.length_unit_m,
            num_solver_iterations: config.solver_iterations,
            max_ccd_substeps: config.max_ccd_substeps,
            ..IntegrationParameters::default()
        };
        Ok(Self {
            config,
            integration,
            pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            entries: BTreeMap::new(),
            allowed_contacts: BTreeSet::new(),
            step_index: 0,
            time_s: 0.0,
        })
    }

    pub fn config(&self) -> PhysicsConfig {
        self.config
    }

    pub fn body_ids(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.entries.keys().copied()
    }

    pub fn add_body(&mut self, spec: BodySpec) -> Result<(), PhysicsError> {
        if !spec.is_valid() {
            return Err(PhysicsError::InvalidBody);
        }
        if self.entries.contains_key(&spec.id) {
            return Err(PhysicsError::DuplicateBodyId);
        }
        let builder = match spec.motion {
            BodyMotion::Fixed => RigidBodyBuilder::fixed(),
            BodyMotion::Dynamic => RigidBodyBuilder::dynamic(),
            BodyMotion::KinematicPosition => RigidBodyBuilder::kinematic_position_based(),
            BodyMotion::KinematicVelocity => RigidBodyBuilder::kinematic_velocity_based(),
        };
        let rigid_body = builder
            .pose(to_iso(spec.pose))
            .linvel(to_vector(spec.linear_velocity_m_s))
            .angvel(to_vector(spec.angular_velocity_rad_s))
            .ccd_enabled(spec.ccd_enabled)
            .user_data(spec.id.0 as u128)
            .build();
        let body_handle = self.bodies.insert(rigid_body);
        let mut collider_entries = Vec::with_capacity(spec.colliders.len());
        for collider_spec in spec.colliders {
            let groups = collider_spec.groups;
            let collider = ColliderBuilder::new(collider_spec.geometry.shared_shape())
                .position(to_iso(collider_spec.local_pose))
                .density(collider_spec.density_kg_m3)
                .friction(collider_spec.friction)
                .restitution(collider_spec.restitution)
                .collision_groups(groups.rapier())
                .user_data(spec.id.0 as u128)
                .build();
            let handle = self
                .colliders
                .insert_with_parent(collider, body_handle, &mut self.bodies);
            collider_entries.push(ColliderEntry { handle, groups });
        }
        self.entries.insert(
            spec.id,
            BodyEntry {
                handle: body_handle,
                colliders: collider_entries,
            },
        );
        Ok(())
    }

    /// Mark a contact as expected by the assembly plan. It remains physical,
    /// but reports distinguish it from an unexpected collision.
    pub fn set_contact_allowed(&mut self, a: BodyId, b: BodyId, allowed: bool) {
        let pair = BodyPair::new(a, b);
        if allowed {
            self.allowed_contacts.insert(pair);
        } else {
            self.allowed_contacts.remove(&pair);
        }
    }

    pub fn set_kinematic_target(&mut self, id: BodyId, pose: Pose) -> Result<(), PhysicsError> {
        let entry = self.entries.get(&id).ok_or(PhysicsError::BodyNotFound)?;
        self.bodies[entry.handle].set_next_kinematic_position(to_iso(pose));
        Ok(())
    }

    pub fn body_state(&self, id: BodyId) -> Option<BodyState> {
        let entry = self.entries.get(&id)?;
        let body = &self.bodies[entry.handle];
        Some(BodyState {
            id,
            pose: from_iso(body.position()),
            linear_velocity_m_s: from_vector(&body.linvel()),
            angular_velocity_rad_s: from_vector(&body.angvel()),
        })
    }

    pub fn states(&self) -> Vec<BodyState> {
        self.body_ids()
            .filter_map(|id| self.body_state(id))
            .collect()
    }

    pub fn step(&mut self) -> StepReport {
        let gravity = to_vector(self.config.gravity_m_s2);
        self.pipeline.step(
            gravity,
            &self.integration,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            &(),
            &(),
        );
        self.step_index += 1;
        self.time_s = self.step_index as f64 * self.config.fixed_dt_s;
        self.proximity_report()
    }

    fn proximity_report(&self) -> StepReport {
        let mut contacts = Vec::new();
        let mut clearances = Vec::new();
        let ids: Vec<_> = self.body_ids().collect();
        for (i, &body_a) in ids.iter().enumerate() {
            for &body_b in &ids[i + 1..] {
                let pair = BodyPair::new(body_a, body_b);
                let a = &self.entries[&body_a];
                let b = &self.entries[&body_b];
                let mut minimum_clearance = f64::INFINITY;
                for collider_a in &a.colliders {
                    for collider_b in &b.colliders {
                        if !collider_a.groups.permits(collider_b.groups) {
                            continue;
                        }
                        let ca = &self.colliders[collider_a.handle];
                        let cb = &self.colliders[collider_b.handle];
                        if let Ok(Some(contact)) = query::contact(
                            ca.position(),
                            ca.shape(),
                            cb.position(),
                            cb.shape(),
                            self.config.contact_prediction_m,
                        ) {
                            contacts.push(ContactReport {
                                body_a,
                                body_b,
                                point_a_m: from_point(&contact.point1),
                                point_b_m: from_point(&contact.point2),
                                normal_a_to_b: from_vector(&contact.normal1),
                                separation_m: contact.dist,
                                penetration_m: (-contact.dist).max(0.0),
                                allowed: self.allowed_contacts.contains(&pair),
                            });
                        } else if let Ok(distance) =
                            query::distance(ca.position(), ca.shape(), cb.position(), cb.shape())
                        {
                            minimum_clearance = minimum_clearance.min(distance);
                        }
                    }
                }
                if minimum_clearance <= self.config.clearance_query_m {
                    clearances.push(ClearanceReport {
                        body_a,
                        body_b,
                        clearance_m: minimum_clearance,
                    });
                }
            }
        }
        contacts.sort_by(|a, b| {
            (a.body_a, a.body_b)
                .cmp(&(b.body_a, b.body_b))
                .then_with(|| a.separation_m.total_cmp(&b.separation_m))
        });
        clearances.sort_by_key(|c| (c.body_a, c.body_b));
        let maximum_penetration_m = contacts
            .iter()
            .map(|contact| contact.penetration_m)
            .fold(0.0, f64::max);
        StepReport {
            step_index: self.step_index,
            time_s: self.time_s,
            contacts,
            clearances,
            maximum_penetration_m,
        }
    }
}

fn positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn nonnegative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

fn to_vector(value: Vec3) -> Vector {
    Vector::new(value.x, value.y, value.z)
}

fn from_vector(value: &Vector) -> Vec3 {
    Vec3::new(value.x, value.y, value.z)
}

fn from_point(value: &Vector) -> Vec3 {
    Vec3::new(value.x, value.y, value.z)
}

fn to_iso(pose: Pose) -> RapierPose {
    let q = pose.rotation.normalized();
    RapierPose {
        rotation: Rotation::from_xyzw(q.x, q.y, q.z, q.w).normalize(),
        translation: Vector::new(pose.translation.x, pose.translation.y, pose.translation.z),
    }
}

fn from_iso(pose: &RapierPose) -> Pose {
    let q = pose.rotation;
    Pose::new(
        Vec3::new(pose.translation.x, pose.translation.y, pose.translation.z),
        Quat::new(q.w, q.x, q.y, q.z),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sphere(id: u32, z_m: f64, radius_m: f64) -> BodySpec {
        BodySpec::new(
            BodyId(id),
            BodyMotion::Dynamic,
            Pose::from_translation(Vec3::new(0.0, 0.0, z_m)),
            ColliderSpec::new(ColliderGeometry::Primitive(PrimitiveShape::Sphere {
                radius_m,
            })),
        )
    }

    #[test]
    fn free_fall_uses_si_gravity() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default()).unwrap();
        world.add_body(sphere(1, 1.0e-3, 50.0e-6)).unwrap();
        for _ in 0..20 {
            world.step();
        }
        let state = world.body_state(BodyId(1)).unwrap();
        assert!(state.pose.translation.z < 1.0e-3);
        assert!(state.linear_velocity_m_s.z < -0.05);
    }

    #[test]
    fn ccd_stops_a_fast_micro_part_at_a_thin_wall() {
        let config = PhysicsConfig {
            fixed_dt_s: 2.0e-3,
            gravity_m_s2: Vec3::ZERO,
            ..PhysicsConfig::default()
        };
        let mut world = PhysicsWorld::new(config).unwrap();
        let wall = BodySpec::new(
            BodyId(1),
            BodyMotion::Fixed,
            Pose::IDENTITY,
            ColliderSpec::new(ColliderGeometry::Primitive(PrimitiveShape::Cuboid {
                half_extents_m: Vec3::new(0.5e-3, 0.5e-3, 5.0e-6),
            })),
        );
        let mut part = sphere(2, 0.5e-3, 20.0e-6);
        part.linear_velocity_m_s.z = -1.0;
        part.ccd_enabled = true;
        world.add_body(wall).unwrap();
        world.add_body(part).unwrap();
        world.step();
        assert!(world.body_state(BodyId(2)).unwrap().pose.translation.z > -20.0e-6);
    }

    #[test]
    fn incompatible_collision_groups_do_not_contact() {
        let config = PhysicsConfig {
            gravity_m_s2: Vec3::ZERO,
            ..PhysicsConfig::default()
        };
        let mut world = PhysicsWorld::new(config).unwrap();
        let mut a = sphere(1, 0.0, 0.1e-3);
        let mut b = sphere(2, 0.1e-3, 0.1e-3);
        a.colliders[0].groups = CollisionGroups::new(0b01, 0b01);
        b.colliders[0].groups = CollisionGroups::new(0b10, 0b10);
        world.add_body(a).unwrap();
        world.add_body(b).unwrap();
        assert!(world.step().contacts.is_empty());
    }

    #[test]
    fn replay_is_bitwise_deterministic_and_ids_are_sorted() {
        fn run() -> Vec<BodyState> {
            let mut world = PhysicsWorld::new(PhysicsConfig::default()).unwrap();
            world.add_body(sphere(9, 1.0e-3, 40.0e-6)).unwrap();
            world.add_body(sphere(2, 2.0e-3, 40.0e-6)).unwrap();
            for _ in 0..64 {
                world.step();
            }
            world.states()
        }
        let first = run();
        let second = run();
        assert_eq!(first, second);
        assert_eq!(
            first.iter().map(|state| state.id).collect::<Vec<_>>(),
            vec![BodyId(2), BodyId(9)]
        );
    }
}
