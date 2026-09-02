//! Simple parallel-jaw micro-gripper.

use crate::geometry::{BodyId, RigidBody, Shape};
use crate::math::{Pose, Vec3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GripperConfig {
    pub min_opening_m: f64,
    pub max_opening_m: f64,
    pub max_speed_m_s: f64,
    pub jaw_half_extents_m: Vec3,
    pub pad_compliance_m: f64,
    pub max_grip_force_n: f64,
}

impl Default for GripperConfig {
    fn default() -> Self {
        Self {
            min_opening_m: 80.0e-6,
            max_opening_m: 2.8e-3,
            max_speed_m_s: 4.0e-3,
            jaw_half_extents_m: Vec3::new(100.0e-6, 250.0e-6, 600.0e-6),
            pad_compliance_m: 12.0e-6,
            max_grip_force_n: 0.15,
        }
    }
}

impl GripperConfig {
    pub fn is_valid(self) -> bool {
        self.min_opening_m >= 0.0
            && self.max_opening_m >= self.min_opening_m
            && self.max_speed_m_s > 0.0
            && self.jaw_half_extents_m.x > 0.0
            && self.jaw_half_extents_m.y > 0.0
            && self.jaw_half_extents_m.z > 0.0
            && self.pad_compliance_m >= 0.0
            && self.max_grip_force_n > 0.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraspCandidate {
    pub body_id: BodyId,
    pub required_opening_m: f64,
    pub center_error_m: Vec3,
    /// Positive means free space; negative means geometric interference.
    pub jaw_clearance_m: f64,
    pub within_finger_depth: bool,
}

impl GraspCandidate {
    pub fn is_reachable(self, config: GripperConfig) -> bool {
        self.required_opening_m <= config.max_opening_m + 2.0 * config.pad_compliance_m
            && self.required_opening_m >= config.min_opening_m - 2.0 * config.pad_compliance_m
            && self.within_finger_depth
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GripperState {
    pub command_opening_m: f64,
    pub opening_m: f64,
    pub opening_velocity_m_s: f64,
    pub held_body: Option<BodyId>,
    pub estimated_grip_force_n: f64,
}

impl GripperState {
    pub fn new(opening_m: f64, config: GripperConfig) -> Self {
        let opening_m = opening_m.clamp(config.min_opening_m, config.max_opening_m);
        Self {
            command_opening_m: opening_m,
            opening_m,
            opening_velocity_m_s: 0.0,
            held_body: None,
            estimated_grip_force_n: 0.0,
        }
    }

    pub fn set_command(&mut self, opening_m: f64, config: GripperConfig) {
        self.command_opening_m = opening_m.clamp(config.min_opening_m, config.max_opening_m);
    }

    /// Hold the current jaw opening without dropping an attached body.
    pub fn stop(&mut self) {
        self.command_opening_m = self.opening_m;
        self.opening_velocity_m_s = 0.0;
    }

    pub fn step(&mut self, dt_s: f64, config: GripperConfig) {
        if dt_s <= 0.0 || !dt_s.is_finite() || !config.is_valid() {
            return;
        }
        let error = self.command_opening_m - self.opening_m;
        let delta = error.signum() * error.abs().min(config.max_speed_m_s * dt_s);
        self.opening_m += delta;
        self.opening_velocity_m_s = delta / dt_s;
        if self.held_body.is_none() {
            self.estimated_grip_force_n = 0.0;
        }
    }

    pub fn jaw_poses(self, tool_pose: Pose, config: GripperConfig) -> [Pose; 2] {
        let offset = self.opening_m * 0.5 + config.jaw_half_extents_m.x;
        [
            tool_pose * Pose::from_translation(Vec3::new(-offset, 0.0, 0.0)),
            tool_pose * Pose::from_translation(Vec3::new(offset, 0.0, 0.0)),
        ]
    }

    pub fn jaw_shapes(config: GripperConfig) -> [Shape; 2] {
        let shape = Shape::Box {
            half_extents_m: config.jaw_half_extents_m,
        };
        [shape, shape]
    }

    /// Conservative candidate test using the body's world AABB transformed at
    /// all eight corners into the tool frame.
    pub fn evaluate_candidate(
        self,
        tool_pose: Pose,
        body: &RigidBody,
        config: GripperConfig,
    ) -> GraspCandidate {
        let aabb = body.aabb();
        let mut local_min = Vec3::splat(f64::INFINITY);
        let mut local_max = Vec3::splat(f64::NEG_INFINITY);
        for x in [aabb.min.x, aabb.max.x] {
            for y in [aabb.min.y, aabb.max.y] {
                for z in [aabb.min.z, aabb.max.z] {
                    let local = tool_pose.inverse_transform_point(Vec3::new(x, y, z));
                    local_min = local_min.min(local);
                    local_max = local_max.max(local);
                }
            }
        }
        let size = local_max - local_min;
        let center = (local_min + local_max) * 0.5;
        let depth = config.jaw_half_extents_m;
        let within_finger_depth = center.y.abs() + size.y * 0.5 <= depth.y * 2.0
            && center.z.abs() + size.z * 0.5 <= depth.z;
        GraspCandidate {
            body_id: body.id,
            required_opening_m: size.x,
            center_error_m: center,
            jaw_clearance_m: self.opening_m - size.x,
            within_finger_depth,
        }
    }

    pub fn try_grasp(&mut self, candidate: GraspCandidate, config: GripperConfig) -> bool {
        if !candidate.is_reachable(config)
            || candidate.jaw_clearance_m.abs() > config.pad_compliance_m * 2.0
            || self.held_body.is_some()
        {
            return false;
        }
        self.held_body = Some(candidate.body_id);
        let compression = (-candidate.jaw_clearance_m).max(0.0);
        let fraction = if config.pad_compliance_m > 0.0 {
            (compression / (2.0 * config.pad_compliance_m)).clamp(0.0, 1.0)
        } else {
            1.0
        };
        self.estimated_grip_force_n = config.max_grip_force_n * fraction;
        true
    }

    pub fn release(&mut self) -> Option<BodyId> {
        self.estimated_grip_force_n = 0.0;
        self.held_body.take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{MotionType, Shape};

    #[test]
    fn jaws_move_symmetrically() {
        let config = GripperConfig::default();
        let state = GripperState::new(1.0e-3, config);
        let poses = state.jaw_poses(Pose::IDENTITY, config);
        assert!((poses[0].translation.x + poses[1].translation.x).abs() < 1e-15);
    }

    #[test]
    fn stop_holds_current_opening_and_preserves_grasp_state() {
        let config = GripperConfig::default();
        let mut state = GripperState::new(config.max_opening_m, config);
        state.set_command(config.min_opening_m, config);
        state.step(0.1, config);
        state.held_body = Some(BodyId(9));
        let opening = state.opening_m;
        state.stop();
        state.step(0.1, config);
        assert_eq!(state.opening_m, opening);
        assert_eq!(state.command_opening_m, opening);
        assert_eq!(state.opening_velocity_m_s, 0.0);
        assert_eq!(state.held_body, Some(BodyId(9)));
    }

    #[test]
    fn baseline_opening_and_force_limit_match_cad() {
        let config = GripperConfig::default();
        assert_eq!(config.max_opening_m, 2.8e-3);
        assert_eq!(config.max_grip_force_n, 0.15);
        assert!(config.max_grip_force_n <= 0.15);
    }

    #[test]
    fn centered_sphere_is_a_grasp_candidate() {
        let config = GripperConfig::default();
        let mut state = GripperState::new(0.4e-3, config);
        let body = RigidBody::new(
            BodyId(7),
            Shape::Sphere { radius_m: 0.2e-3 },
            Pose::IDENTITY,
            MotionType::Dynamic,
        );
        let candidate = state.evaluate_candidate(Pose::IDENTITY, &body, config);
        assert!(candidate.is_reachable(config));
        assert!(state.try_grasp(candidate, config));
        assert_eq!(state.held_body, Some(BodyId(7)));
        assert_eq!(state.release(), Some(BodyId(7)));
    }

    #[test]
    fn far_object_is_rejected_by_finger_depth() {
        let config = GripperConfig::default();
        let state = GripperState::new(0.4e-3, config);
        let body = RigidBody::new(
            BodyId(1),
            Shape::Sphere { radius_m: 0.1e-3 },
            Pose::from_translation(Vec3::new(0.0, 0.0, 2.0e-3)),
            MotionType::Dynamic,
        );
        assert!(
            !state
                .evaluate_candidate(Pose::IDENTITY, &body, config)
                .within_finger_depth
        );
    }
}
