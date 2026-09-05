//! Tendon-driven rigid serial arm used by the baseline Pipe cell.
//!
//! The mobile base has two rail coordinates: translation along the tube's
//! global Z axis and azimuth around its inner wall. Four antagonistic tendon
//! pairs actuate shoulder yaw, shoulder pitch, elbow pitch, and wrist roll.
//! The continuum model in [`crate::arm`] remains available as a reduced-order
//! prototype, while this module is the reference arm for gearbox assembly.

use crate::geometry::Shape;
use crate::math::{Pose, Quat, Vec3};

pub const TENDON_JOINT_COUNT: usize = 4;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TendonJointConfig {
    /// Effective tendon moment arm at the joint.
    pub routing_radius_m: f64,
    /// Elastic stiffness of one tendon path.
    pub tendon_stiffness_n_m: f64,
    /// Tension in each antagonistic tendon at neutral angle.
    pub pretension_n: f64,
    /// Differential lost cable travel before torque changes sign.
    pub differential_backlash_m: f64,
    pub max_tension_n: f64,
}

impl TendonJointConfig {
    pub fn is_valid(self) -> bool {
        self.routing_radius_m > 0.0
            && self.routing_radius_m.is_finite()
            && self.tendon_stiffness_n_m > 0.0
            && self.tendon_stiffness_n_m.is_finite()
            && self.pretension_n >= 0.0
            && self.pretension_n.is_finite()
            && self.differential_backlash_m >= 0.0
            && self.differential_backlash_m.is_finite()
            && self.max_tension_n > self.pretension_n
            && self.max_tension_n.is_finite()
    }

    /// Cable offsets for an ideal antagonistic pair. Positive joint angle
    /// lengthens tendon 0 and shortens tendon 1.
    pub fn offsets_for_angle(self, angle_rad: f64) -> [f64; 2] {
        let travel = self.routing_radius_m * angle_rad;
        [travel, -travel]
    }

    fn geometric_offsets_for_angle(self, angle_rad: f64) -> [f64; 2] {
        self.offsets_for_angle(angle_rad)
    }

    /// Apply differential backlash while preserving common-mode cable travel.
    fn transmitted_offsets(self, tendon_offsets_m: [f64; 2]) -> [f64; 2] {
        let common = 0.5 * (tendon_offsets_m[0] + tendon_offsets_m[1]);
        let differential = tendon_offsets_m[0] - tendon_offsets_m[1];
        let transmitted =
            differential.signum() * (differential.abs() - self.differential_backlash_m).max(0.0);
        [common + 0.5 * transmitted, common - 0.5 * transmitted]
    }

    pub fn torsional_stiffness_nm_rad(self) -> f64 {
        2.0 * self.tendon_stiffness_n_m * self.routing_radius_m.powi(2)
    }

    /// Solve joint angle from measured tendon offsets and external torque.
    /// Positive external torque increases the solved angle.
    pub fn solve_angle(self, tendon_offsets_m: [f64; 2], external_torque_nm: f64) -> f64 {
        let differential = tendon_offsets_m[0] - tendon_offsets_m[1];
        let transmitted =
            differential.signum() * (differential.abs() - self.differential_backlash_m).max(0.0);
        let k_theta = self.torsional_stiffness_nm_rad();
        transmitted / (2.0 * self.routing_radius_m) + external_torque_nm / k_theta
    }

    pub fn telemetry(
        self,
        tendon_offsets_m: [f64; 2],
        external_torque_nm: f64,
    ) -> TendonJointTelemetry {
        let joint_angle_rad = self.solve_angle(tendon_offsets_m, external_torque_nm);
        self.telemetry_at_angle(tendon_offsets_m, joint_angle_rad)
    }

    /// Re-evaluate cable stretch at an externally constrained joint angle. This
    /// is used at hard stops, where the free tendon/load solution may lie beyond
    /// the configured joint limit.
    fn telemetry_at_angle(
        self,
        tendon_offsets_m: [f64; 2],
        joint_angle_rad: f64,
    ) -> TendonJointTelemetry {
        let geometric = self.geometric_offsets_for_angle(joint_angle_rad);
        let paid_out = self.transmitted_offsets(tendon_offsets_m);
        let stretch = [geometric[0] - paid_out[0], geometric[1] - paid_out[1]];
        let no_load_angle = (paid_out[0] - paid_out[1]) / (2.0 * self.routing_radius_m);
        TendonJointTelemetry {
            joint_angle_rad,
            tendon_tensions_n: [
                (self.pretension_n + self.tendon_stiffness_n_m * stretch[0])
                    .clamp(0.0, self.max_tension_n),
                (self.pretension_n + self.tendon_stiffness_n_m * stretch[1])
                    .clamp(0.0, self.max_tension_n),
            ],
            torsional_stiffness_nm_rad: self.torsional_stiffness_nm_rad(),
            elastic_deflection_rad: joint_angle_rad - no_load_angle,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TendonJointTelemetry {
    pub joint_angle_rad: f64,
    pub tendon_tensions_n: [f64; 2],
    pub torsional_stiffness_nm_rad: f64,
    pub elastic_deflection_rad: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SerialJointPositions {
    /// Longitudinal mobile-base coordinate in tube/world coordinates.
    pub base_z_m: f64,
    /// Azimuth of the mobile base around the tube inner wall.
    pub base_theta_rad: f64,
    pub shoulder_yaw_rad: f64,
    pub shoulder_pitch_rad: f64,
    pub elbow_pitch_rad: f64,
    pub wrist_roll_rad: f64,
}

impl Default for SerialJointPositions {
    fn default() -> Self {
        Self {
            base_z_m: 0.0,
            base_theta_rad: 0.0,
            shoulder_yaw_rad: 0.0,
            shoulder_pitch_rad: 0.0,
            elbow_pitch_rad: 0.0,
            wrist_roll_rad: 0.0,
        }
    }
}

impl SerialJointPositions {
    pub fn tendon_joint_angles(self) -> [f64; TENDON_JOINT_COUNT] {
        [
            self.shoulder_yaw_rad,
            self.shoulder_pitch_rad,
            self.elbow_pitch_rad,
            self.wrist_roll_rad,
        ]
    }

    fn with_tendon_joint_angles(mut self, angles: [f64; TENDON_JOINT_COUNT]) -> Self {
        self.shoulder_yaw_rad = angles[0];
        self.shoulder_pitch_rad = angles[1];
        self.elbow_pitch_rad = angles[2];
        self.wrist_roll_rad = angles[3];
        self
    }

    pub fn is_finite(self) -> bool {
        self.base_z_m.is_finite()
            && self.base_theta_rad.is_finite()
            && self.shoulder_yaw_rad.is_finite()
            && self.shoulder_pitch_rad.is_finite()
            && self.elbow_pitch_rad.is_finite()
            && self.wrist_roll_rad.is_finite()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SerialArmConfig {
    /// Radius from tube axis to the shoulder pivot.
    pub rail_radius_m: f64,
    pub base_z_limits_m: [f64; 2],
    pub upper_arm_length_m: f64,
    pub forearm_length_m: f64,
    pub wrist_length_m: f64,
    pub link_collision_radii_m: [f64; 3],
    /// Yaw, shoulder pitch, elbow pitch, wrist roll limits.
    pub joint_limits_rad: [[f64; 2]; TENDON_JOINT_COUNT],
    /// Yaw, shoulder pitch, elbow pitch, wrist roll transmissions.
    pub tendon_joints: [TendonJointConfig; TENDON_JOINT_COUNT],
}

impl Default for SerialArmConfig {
    fn default() -> Self {
        let tendon_joint = TendonJointConfig {
            routing_radius_m: 1.65e-3,
            tendon_stiffness_n_m: 7_500.0,
            pretension_n: 1.2,
            differential_backlash_m: 18.0e-6,
            max_tension_n: 4.0,
        };
        Self {
            rail_radius_m: 72.0e-3,
            base_z_limits_m: [-150.0e-3, 150.0e-3],
            upper_arm_length_m: 32.0e-3,
            forearm_length_m: 30.0e-3,
            wrist_length_m: 15.0e-3,
            link_collision_radii_m: [3.2e-3, 2.8e-3, 1.8e-3],
            joint_limits_rad: [
                [-100.0_f64.to_radians(), 100.0_f64.to_radians()],
                [-70.0_f64.to_radians(), 120.0_f64.to_radians()],
                [-10.0_f64.to_radians(), 155.0_f64.to_radians()],
                [-core::f64::consts::PI, core::f64::consts::PI],
            ],
            tendon_joints: [tendon_joint; TENDON_JOINT_COUNT],
        }
    }
}

impl SerialArmConfig {
    pub fn is_valid(self) -> bool {
        self.rail_radius_m > 0.0
            && self.rail_radius_m.is_finite()
            && self.base_z_limits_m[0].is_finite()
            && self.base_z_limits_m[1].is_finite()
            && self.base_z_limits_m[0] <= self.base_z_limits_m[1]
            && self.upper_arm_length_m > 0.0
            && self.forearm_length_m > 0.0
            && self.wrist_length_m > 0.0
            && self
                .link_collision_radii_m
                .iter()
                .all(|radius| *radius > 0.0 && radius.is_finite())
            && self.joint_limits_rad.iter().all(|limits| {
                limits[0].is_finite() && limits[1].is_finite() && limits[0] <= limits[1]
            })
            && self
                .tendon_joints
                .iter()
                .copied()
                .all(TendonJointConfig::is_valid)
    }

    pub fn maximum_reach_m(self) -> f64 {
        self.upper_arm_length_m + self.forearm_length_m + self.wrist_length_m
    }

    pub fn clamp_positions(self, mut positions: SerialJointPositions) -> SerialJointPositions {
        positions.base_z_m = positions
            .base_z_m
            .clamp(self.base_z_limits_m[0], self.base_z_limits_m[1]);
        // Base theta is periodic and canonicalized to [-pi, pi).
        positions.base_theta_rad = wrap_pi(positions.base_theta_rad);
        let mut angles = positions.tendon_joint_angles();
        for (index, angle) in angles.iter_mut().enumerate() {
            *angle = angle.clamp(
                self.joint_limits_rad[index][0],
                self.joint_limits_rad[index][1],
            );
        }
        positions.with_tendon_joint_angles(angles)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SerialArmError {
    InvalidConfig,
    NonFiniteState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToolPositionIkError {
    NonFiniteTarget,
    Unreachable,
    JointLimits,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToolPositionSolution {
    pub positions: SerialJointPositions,
    pub target_position_world_m: Vec3,
    pub position_error_m: f64,
}

/// Position and directed tool axis; roll is deliberately not constrained.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToolAxisSolution {
    pub positions: SerialJointPositions,
    pub position_error_m: f64,
    pub axis_error_rad: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SerialArmKinematics {
    /// Frame fixed to the mobile base, +Z pointing radially inward.
    pub base_pose: Pose,
    /// Shoulder frame after yaw and pitch.
    pub shoulder_pose: Pose,
    /// Elbow frame after elbow pitch.
    pub elbow_pose: Pose,
    /// Wrist frame after wrist roll.
    pub wrist_pose: Pose,
    pub tool_pose: Pose,
    pub collision_capsules: Vec<(Pose, Shape)>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SerialArm {
    pub config: SerialArmConfig,
    pub positions: SerialJointPositions,
    /// Measured/commanded differential cable offsets for yaw, shoulder pitch,
    /// elbow pitch, and wrist roll.
    pub tendon_offsets_m: [[f64; 2]; TENDON_JOINT_COUNT],
    pub tendon_telemetry: [TendonJointTelemetry; TENDON_JOINT_COUNT],
}

impl SerialArm {
    /// Bounded deterministic local 5-DoF IK. Rail Z is scaled by the arm's
    /// reach so the least-squares system is dimensionless. Every trial is
    /// clamped to physical limits, backtracked, and checked with independent
    /// FK residuals. Failure never returns a best-effort pose. Wrist roll is
    /// preserved; no roll measurement is invented from an axis constraint.
    pub fn solve_tool_axis(
        &self,
        target_position_world_m: Vec3,
        target_axis_world: Vec3,
        seed: SerialJointPositions,
    ) -> Result<ToolAxisSolution, ToolPositionIkError> {
        if !target_position_world_m.is_finite()
            || !target_axis_world.is_finite()
            || !seed.is_finite()
            || (target_axis_world.length() - 1.0).abs() > 1.0e-6
        {
            return Err(ToolPositionIkError::NonFiniteTarget);
        }
        let length_scale = self.config.maximum_reach_m();
        let point_seed = self
            .solve_tool_position(target_position_world_m, seed)
            .map(|solution| solution.positions)
            .unwrap_or(seed);
        let coordinates = |p: SerialJointPositions| {
            [
                p.base_z_m / length_scale,
                p.base_theta_rad,
                p.shoulder_yaw_rad,
                p.shoulder_pitch_rad,
                p.elbow_pitch_rad,
            ]
        };
        let positions = |q: [f64; 5]| {
            self.config.clamp_positions(SerialJointPositions {
                base_z_m: q[0] * length_scale,
                base_theta_rad: q[1],
                shoulder_yaw_rad: q[2],
                shoulder_pitch_rad: q[3],
                elbow_pitch_rad: q[4],
                wrist_roll_rad: seed.wrist_roll_rad,
            })
        };
        let residual = |q: [f64; 5]| {
            let mut candidate = self.clone();
            candidate
                .set_positions(positions(q))
                .expect("finite IK trial");
            let pose = candidate.forward_kinematics().tool_pose;
            let p = (target_position_world_m - pose.translation) / length_scale;
            let a = target_axis_world - pose.transform_vector(Vec3::Z);
            [p.x, p.y, p.z, a.x, a.y, a.z]
        };
        let cost = |r: [f64; 6]| r.iter().map(|x| x * x).sum::<f64>();
        for initial in [seed, point_seed] {
            let mut q = coordinates(initial);
            for _ in 0..100 {
                let r = residual(q);
                let position_error_m =
                    (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt() * length_scale;
                let axis_chord = (r[3] * r[3] + r[4] * r[4] + r[5] * r[5]).sqrt();
                if position_error_m <= 1.0e-9 && axis_chord <= 1.0e-7 {
                    return Ok(ToolAxisSolution {
                        positions: positions(q),
                        position_error_m,
                        axis_error_rad: 2.0 * (0.5 * axis_chord).clamp(0.0, 1.0).asin(),
                    });
                }
                let mut jacobian = [[0.0; 5]; 6];
                for column in 0..5 {
                    let mut plus = q;
                    let mut minus = q;
                    plus[column] += 1.0e-5;
                    minus[column] -= 1.0e-5;
                    let rp = residual(plus);
                    let rm = residual(minus);
                    for row in 0..6 {
                        jacobian[row][column] = (rp[row] - rm[row]) / 2.0e-5;
                    }
                }
                let mut normal = [[0.0; 6]; 5];
                for i in 0..5 {
                    for j in 0..5 {
                        normal[i][j] = (0..6).map(|k| jacobian[k][i] * jacobian[k][j]).sum::<f64>()
                            + if i == j { 1.0e-8 } else { 0.0 };
                    }
                    normal[i][5] = -(0..6).map(|k| jacobian[k][i] * r[k]).sum::<f64>();
                }
                let Some(mut step) = solve_ik_system(normal) else {
                    break;
                };
                let largest = step.iter().copied().map(f64::abs).fold(0.0, f64::max);
                if largest > 0.15 {
                    for value in &mut step {
                        *value *= 0.15 / largest;
                    }
                }
                let mut accepted = false;
                for backtrack in 0..12 {
                    let gain = 0.5_f64.powi(backtrack);
                    let trial =
                        coordinates(positions(core::array::from_fn(|i| q[i] + gain * step[i])));
                    if cost(residual(trial)) < cost(r) {
                        q = trial;
                        accepted = true;
                        break;
                    }
                }
                if !accepted {
                    break;
                }
            }
        }
        Err(ToolPositionIkError::Unreachable)
    }

    pub fn new(config: SerialArmConfig) -> Result<Self, SerialArmError> {
        if !config.is_valid() {
            return Err(SerialArmError::InvalidConfig);
        }
        let positions = config.clamp_positions(SerialJointPositions::default());
        let tendon_offsets_m = core::array::from_fn(|index| {
            config.tendon_joints[index].offsets_for_angle(positions.tendon_joint_angles()[index])
        });
        let tendon_telemetry = core::array::from_fn(|index| {
            config.tendon_joints[index].telemetry(tendon_offsets_m[index], 0.0)
        });
        Ok(Self {
            config,
            positions,
            tendon_offsets_m,
            tendon_telemetry,
        })
    }

    /// Set ideal joint coordinates and derive the matching cable travel.
    pub fn set_positions(&mut self, positions: SerialJointPositions) -> Result<(), SerialArmError> {
        if !positions.is_finite() {
            return Err(SerialArmError::NonFiniteState);
        }
        self.positions = self.config.clamp_positions(positions);
        let angles = self.positions.tendon_joint_angles();
        for (index, angle) in angles.into_iter().enumerate() {
            self.tendon_offsets_m[index] =
                self.config.tendon_joints[index].offsets_for_angle(angle);
            self.tendon_telemetry[index] =
                self.config.tendon_joints[index].telemetry(self.tendon_offsets_m[index], 0.0);
        }
        Ok(())
    }

    /// Resolve the four rotary joint angles from antagonistic tendon travel and
    /// load torque. Base Z/theta are rail coordinates and remain directly set.
    pub fn set_tendon_state(
        &mut self,
        base_z_m: f64,
        base_theta_rad: f64,
        tendon_offsets_m: [[f64; 2]; TENDON_JOINT_COUNT],
        external_joint_torques_nm: [f64; TENDON_JOINT_COUNT],
    ) -> Result<(), SerialArmError> {
        if !base_z_m.is_finite()
            || !base_theta_rad.is_finite()
            || tendon_offsets_m
                .iter()
                .flatten()
                .any(|value| !value.is_finite())
            || external_joint_torques_nm
                .iter()
                .any(|value| !value.is_finite())
        {
            return Err(SerialArmError::NonFiniteState);
        }
        let telemetry = core::array::from_fn(|index| {
            self.config.tendon_joints[index]
                .telemetry(tendon_offsets_m[index], external_joint_torques_nm[index])
        });
        let angles = telemetry.map(|item| item.joint_angle_rad);
        self.positions = self.config.clamp_positions(
            SerialJointPositions {
                base_z_m,
                base_theta_rad,
                ..self.positions
            }
            .with_tendon_joint_angles(angles),
        );
        self.tendon_offsets_m = tendon_offsets_m;
        let actual_angles = self.positions.tendon_joint_angles();
        self.tendon_telemetry = core::array::from_fn(|index| {
            self.config.tendon_joints[index]
                .telemetry_at_angle(tendon_offsets_m[index], actual_angles[index])
        });
        Ok(())
    }

    pub fn forward_kinematics(&self) -> SerialArmKinematics {
        let positions = self.positions;
        let radial_out = Vec3::new(
            positions.base_theta_rad.cos(),
            positions.base_theta_rad.sin(),
            0.0,
        );
        let base_origin = radial_out * self.config.rail_radius_m + Vec3::Z * positions.base_z_m;

        // At theta=0: local +Z points -world X (inward), local +Y points
        // +world Z (tube-longitudinal), and local +X points -world Y.
        let inward_alignment = Quat::from_two_vectors(Vec3::Z, -Vec3::X)
            * Quat::from_axis_angle(Vec3::Z, -core::f64::consts::FRAC_PI_2);
        let base_rotation =
            Quat::from_axis_angle(Vec3::Z, positions.base_theta_rad) * inward_alignment;
        let base_pose = Pose::new(base_origin, base_rotation.normalized());

        let shoulder_rotation = Quat::from_axis_angle(Vec3::Y, positions.shoulder_yaw_rad)
            * Quat::from_axis_angle(Vec3::X, positions.shoulder_pitch_rad);
        let shoulder_pose = base_pose * Pose::new(Vec3::ZERO, shoulder_rotation);
        let elbow_origin_pose =
            shoulder_pose * Pose::from_translation(Vec3::Z * self.config.upper_arm_length_m);
        let elbow_pose = elbow_origin_pose
            * Pose::new(
                Vec3::ZERO,
                Quat::from_axis_angle(Vec3::X, positions.elbow_pitch_rad),
            );
        let wrist_origin_pose =
            elbow_pose * Pose::from_translation(Vec3::Z * self.config.forearm_length_m);
        let wrist_pose = wrist_origin_pose
            * Pose::new(
                Vec3::ZERO,
                Quat::from_axis_angle(Vec3::Z, positions.wrist_roll_rad),
            );
        let tool_pose = wrist_pose * Pose::from_translation(Vec3::Z * self.config.wrist_length_m);

        let points = [
            shoulder_pose.translation,
            elbow_origin_pose.translation,
            wrist_origin_pose.translation,
            tool_pose.translation,
        ];
        let collision_capsules = (0..3)
            .map(|index| {
                capsule_between(
                    points[index],
                    points[index + 1],
                    self.config.link_collision_radii_m[index],
                )
            })
            .collect();
        SerialArmKinematics {
            base_pose,
            shoulder_pose,
            elbow_pose,
            wrist_pose,
            tool_pose,
            collision_capsules,
        }
    }

    /// Solve a world-space tool point with a deterministic carriage-first
    /// policy. The rail first aligns its azimuth with the target and places Z
    /// as close to the target as its limits allow. The remaining radial/Z
    /// displacement is solved by the shoulder/elbow pair; yaw is kept at zero
    /// and wrist roll is preserved because M1b controls position, not tool
    /// orientation.
    pub fn solve_tool_position(
        &self,
        target_position_world_m: Vec3,
        seed: SerialJointPositions,
    ) -> Result<ToolPositionSolution, ToolPositionIkError> {
        if !target_position_world_m.is_finite() || !seed.is_finite() {
            return Err(ToolPositionIkError::NonFiniteTarget);
        }

        let radial_m = target_position_world_m.x.hypot(target_position_world_m.y);
        if radial_m > self.config.rail_radius_m + 1.0e-12 {
            return Err(ToolPositionIkError::Unreachable);
        }
        let base_theta_rad = if radial_m > 1.0e-12 {
            target_position_world_m.y.atan2(target_position_world_m.x)
        } else {
            wrap_pi(seed.base_theta_rad)
        };
        let base_z_m = target_position_world_m.z.clamp(
            self.config.base_z_limits_m[0],
            self.config.base_z_limits_m[1],
        );
        let axial_m = target_position_world_m.z - base_z_m;
        let inward_m = self.config.rail_radius_m - radial_m;
        let upper_m = self.config.upper_arm_length_m;
        // There is no wrist-pitch actuator, so the forearm and wrist are one
        // collinear second link for point-position IK.
        let distal_m = self.config.forearm_length_m + self.config.wrist_length_m;
        let reach_squared_m2 = axial_m * axial_m + inward_m * inward_m;
        let minimum_reach_m = (upper_m - distal_m).abs();
        let maximum_reach_m = upper_m + distal_m;
        let reach_m = reach_squared_m2.sqrt();
        if reach_m < minimum_reach_m - 1.0e-12 || reach_m > maximum_reach_m + 1.0e-12 {
            return Err(ToolPositionIkError::Unreachable);
        }

        let elbow_cos = ((reach_squared_m2 - upper_m * upper_m - distal_m * distal_m)
            / (2.0 * upper_m * distal_m))
            .clamp(-1.0, 1.0);
        let elbow_magnitude = elbow_cos.acos();
        let direction_rad = (-axial_m).atan2(inward_m);
        let mut limit_failure = false;
        for elbow_pitch_rad in [elbow_magnitude, -elbow_magnitude] {
            let shoulder_pitch_rad = direction_rad
                - (distal_m * elbow_pitch_rad.sin())
                    .atan2(upper_m + distal_m * elbow_pitch_rad.cos());
            let joint_angles = [
                0.0,
                shoulder_pitch_rad,
                elbow_pitch_rad,
                seed.wrist_roll_rad,
            ];
            if joint_angles
                .iter()
                .zip(self.config.joint_limits_rad)
                .any(|(angle, limits)| !(limits[0]..=limits[1]).contains(angle))
            {
                limit_failure = true;
                continue;
            }
            let positions = SerialJointPositions {
                base_z_m,
                base_theta_rad,
                shoulder_yaw_rad: joint_angles[0],
                shoulder_pitch_rad: joint_angles[1],
                elbow_pitch_rad: joint_angles[2],
                wrist_roll_rad: joint_angles[3],
            };
            let mut candidate = self.clone();
            candidate
                .set_positions(positions)
                .map_err(|_| ToolPositionIkError::NonFiniteTarget)?;
            let error_m = (candidate.forward_kinematics().tool_pose.translation
                - target_position_world_m)
                .length();
            if error_m <= 1.0e-9 {
                return Ok(ToolPositionSolution {
                    positions,
                    target_position_world_m,
                    position_error_m: error_m,
                });
            }
        }
        if limit_failure {
            Err(ToolPositionIkError::JointLimits)
        } else {
            Err(ToolPositionIkError::Unreachable)
        }
    }
}

fn wrap_pi(angle_rad: f64) -> f64 {
    let two_pi = 2.0 * core::f64::consts::PI;
    (angle_rad + core::f64::consts::PI).rem_euclid(two_pi) - core::f64::consts::PI
}

fn solve_ik_system(mut rows: [[f64; 6]; 5]) -> Option<[f64; 5]> {
    for column in 0..5 {
        let pivot =
            (column..5).max_by(|a, b| rows[*a][column].abs().total_cmp(&rows[*b][column].abs()))?;
        rows.swap(column, pivot);
        let divisor = rows[column][column];
        if !divisor.is_finite() || divisor.abs() < 1.0e-14 {
            return None;
        }
        for value in &mut rows[column][column..] {
            *value /= divisor;
        }
        let pivot_row = rows[column];
        for (i, row) in rows.iter_mut().enumerate() {
            if i == column {
                continue;
            }
            let factor = row[column];
            for (value, pivot_value) in row[column..].iter_mut().zip(&pivot_row[column..]) {
                *value -= factor * pivot_value;
            }
        }
    }
    let result = core::array::from_fn(|i| rows[i][5]);
    result.iter().all(|x| x.is_finite()).then_some(result)
}

fn capsule_between(a: Vec3, b: Vec3, radius_m: f64) -> (Pose, Shape) {
    let delta = b - a;
    let length = delta.length();
    (
        Pose::new(
            (a + b) * 0.5,
            Quat::from_two_vectors(Vec3::Z, delta.normalized_or(Vec3::Z)),
        ),
        Shape::Capsule {
            radius_m,
            half_segment_m: length * 0.5,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3) {
        assert!((a - b).length() < 1.0e-10, "{a:?} != {b:?}");
    }

    #[test]
    fn axis_ik_reaches_generated_poses_and_preserves_unconstrained_roll() {
        let mut arm = SerialArm::new(SerialArmConfig::default()).unwrap();
        let seed = arm
            .solve_tool_position(Vec3::new(0.020, 0.0, 0.0), arm.positions)
            .unwrap()
            .positions;
        arm.set_positions(seed).unwrap();
        for sign in [-1.0, 1.0] {
            let target_joints = SerialJointPositions {
                base_z_m: seed.base_z_m + sign * 0.0001,
                base_theta_rad: sign * 0.012,
                shoulder_yaw_rad: sign * 0.025,
                shoulder_pitch_rad: seed.shoulder_pitch_rad + sign * 0.03,
                elbow_pitch_rad: seed.elbow_pitch_rad - sign * 0.02,
                wrist_roll_rad: 0.7,
            };
            let mut reference = arm.clone();
            reference.set_positions(target_joints).unwrap();
            let target = reference.forward_kinematics().tool_pose;
            let solution = arm
                .solve_tool_axis(target.translation, target.transform_vector(Vec3::Z), seed)
                .unwrap();
            let replay = arm
                .solve_tool_axis(target.translation, target.transform_vector(Vec3::Z), seed)
                .unwrap();
            assert_eq!(solution, replay);
            assert_eq!(solution.positions.wrist_roll_rad, seed.wrist_roll_rad);
            let mut executed = arm.clone();
            executed.set_positions(solution.positions).unwrap();
            let pose = executed.forward_kinematics().tool_pose;
            assert!((pose.translation - target.translation).length() < 1.0e-9);
            assert!(
                (pose.transform_vector(Vec3::Z) - target.transform_vector(Vec3::Z)).length()
                    < 1.0e-7
            );
        }
    }

    #[test]
    fn axis_ik_refuses_invalid_and_unreachable_constraints() {
        let arm = SerialArm::new(SerialArmConfig::default()).unwrap();
        for axis in [Vec3::ZERO, Vec3::Z * 2.0, Vec3::new(f64::NAN, 0.0, 1.0)] {
            assert!(arm
                .solve_tool_axis(Vec3::new(0.02, 0.0, 0.0), axis, arm.positions)
                .is_err());
        }
        assert!(arm
            .solve_tool_axis(Vec3::new(0.5, 0.0, 0.0), Vec3::Z, arm.positions)
            .is_err());
        let mut limited = arm.clone();
        limited.config.joint_limits_rad = [[0.0, 0.0]; 4];
        assert!(limited
            .solve_tool_axis(Vec3::new(0.02, 0.0, 0.0), Vec3::Z, limited.positions)
            .is_err());
    }

    #[test]
    fn baseline_link_stack_has_77_mm_reach() {
        let config = SerialArmConfig::default();
        assert!((config.maximum_reach_m() - 77.0e-3).abs() < 1.0e-15);
    }

    #[test]
    fn carriage_first_ik_reaches_a_central_calibration_point() {
        let arm = SerialArm::new(SerialArmConfig::default()).unwrap();
        let target = Vec3::new(20.0e-3, 0.0, 12.0e-3);
        let solution = arm
            .solve_tool_position(target, arm.positions)
            .expect("central point must be reachable");
        assert_eq!(solution.positions.base_z_m, target.z);
        assert_eq!(solution.positions.base_theta_rad, 0.0);
        assert!(solution.position_error_m < 1.0e-10);
        assert!(solution.positions.elbow_pitch_rad > 0.0);
    }

    #[test]
    fn carriage_first_ik_rejects_non_finite_and_outward_targets() {
        let arm = SerialArm::new(SerialArmConfig::default()).unwrap();
        assert_eq!(
            arm.solve_tool_position(Vec3::new(f64::NAN, 0.0, 0.0), arm.positions),
            Err(ToolPositionIkError::NonFiniteTarget)
        );
        assert_eq!(
            arm.solve_tool_position(Vec3::new(90.0e-3, 0.0, 0.0), arm.positions),
            Err(ToolPositionIkError::Unreachable)
        );
    }

    #[test]
    fn baseline_tendon_hardware_constants_are_locked() {
        let config = SerialArmConfig::default();
        assert_eq!(config.rail_radius_m, 72.0e-3);
        assert_eq!(config.tendon_joints.len(), TENDON_JOINT_COUNT);
        for joint in config.tendon_joints {
            assert_eq!(joint.routing_radius_m, 1.65e-3);
            assert_eq!(joint.pretension_n, 1.2);
            assert_eq!(joint.tendon_stiffness_n_m, 7_500.0);
            assert_eq!(joint.differential_backlash_m, 18.0e-6);
            assert_eq!(joint.max_tension_n, 4.0);
        }
    }

    #[test]
    fn zero_pose_points_radially_inward() {
        let arm = SerialArm::new(SerialArmConfig::default()).unwrap();
        let kinematics = arm.forward_kinematics();
        close(
            kinematics.tool_pose.translation,
            Vec3::X * (arm.config.rail_radius_m - arm.config.maximum_reach_m()),
        );
        assert!(kinematics.tool_pose.transform_vector(Vec3::Z).dot(-Vec3::X) > 0.999999);
    }

    #[test]
    fn base_theta_rotates_whole_arm_around_tube() {
        let mut arm = SerialArm::new(SerialArmConfig::default()).unwrap();
        arm.set_positions(SerialJointPositions {
            base_theta_rad: core::f64::consts::FRAC_PI_2,
            ..SerialJointPositions::default()
        })
        .unwrap();
        close(
            arm.forward_kinematics().tool_pose.translation,
            Vec3::Y * (arm.config.rail_radius_m - arm.config.maximum_reach_m()),
        );
    }

    #[test]
    fn link_capsules_match_three_physical_links() {
        let arm = SerialArm::new(SerialArmConfig::default()).unwrap();
        let kinematics = arm.forward_kinematics();
        assert_eq!(kinematics.collision_capsules.len(), 3);
        for (_, shape) in kinematics.collision_capsules {
            assert!(shape.is_valid());
        }
    }

    #[test]
    fn ideal_tendon_mapping_round_trips_outside_backlash() {
        let mut config = SerialArmConfig::default();
        for joint in &mut config.tendon_joints {
            joint.differential_backlash_m = 0.0;
        }
        let transmission = config.tendon_joints[0];
        let expected = 0.35;
        assert!(
            (transmission.solve_angle(transmission.offsets_for_angle(expected), 0.0) - expected)
                .abs()
                < 1e-14
        );
    }

    #[test]
    fn differential_backlash_creates_joint_deadband() {
        let transmission = SerialArmConfig::default().tendon_joints[0];
        let sub_backlash = transmission.differential_backlash_m * 0.4;
        assert_eq!(
            transmission.solve_angle([sub_backlash, -sub_backlash], 0.0),
            0.0
        );
    }

    #[test]
    fn external_torque_deflection_obeys_torsional_stiffness() {
        let transmission = SerialArmConfig::default().tendon_joints[0];
        let torque = 1.0e-3;
        let angle = transmission.solve_angle([0.0; 2], torque);
        assert!((angle - torque / transmission.torsional_stiffness_nm_rad()).abs() < 1e-14);
    }

    #[test]
    fn positive_external_torque_loads_positive_path_tendon() {
        let transmission = SerialArmConfig::default().tendon_joints[0];
        let offsets = transmission.offsets_for_angle(0.2);
        let telemetry = transmission.telemetry(offsets, 1.0e-3);
        assert!(telemetry.tendon_tensions_n[0] > transmission.pretension_n);
        assert!(telemetry.tendon_tensions_n[1] < transmission.pretension_n);
    }

    #[test]
    fn hard_limit_keeps_fk_and_tendon_telemetry_consistent() {
        let mut arm = SerialArm::new(SerialArmConfig::default()).unwrap();
        let excessive = arm
            .config
            .tendon_joints
            .map(|joint| joint.offsets_for_angle(10.0));
        arm.set_tendon_state(0.0, 0.0, excessive, [0.0; TENDON_JOINT_COUNT])
            .unwrap();
        for (angle, telemetry) in arm
            .positions
            .tendon_joint_angles()
            .iter()
            .zip(arm.tendon_telemetry)
        {
            assert_eq!(*angle, telemetry.joint_angle_rad);
        }
    }

    #[test]
    fn joint_and_rail_limits_are_enforced() {
        let mut arm = SerialArm::new(SerialArmConfig::default()).unwrap();
        arm.set_positions(SerialJointPositions {
            base_z_m: 10.0,
            base_theta_rad: 5.0 * core::f64::consts::PI,
            shoulder_yaw_rad: 10.0,
            shoulder_pitch_rad: -10.0,
            elbow_pitch_rad: 10.0,
            wrist_roll_rad: 10.0,
        })
        .unwrap();
        assert_eq!(arm.positions.base_z_m, arm.config.base_z_limits_m[1]);
        assert!(arm.positions.base_theta_rad >= -core::f64::consts::PI);
        assert!(arm.positions.base_theta_rad < core::f64::consts::PI);
        for (angle, limits) in arm
            .positions
            .tendon_joint_angles()
            .iter()
            .zip(arm.config.joint_limits_rad)
        {
            assert!(*angle >= limits[0] && *angle <= limits[1]);
        }
    }
}
