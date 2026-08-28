//! Piecewise constant-curvature tendon-driven continuum arm.
//!
//! Three tendons run at configurable angular positions around every segment.
//! Differential tendon length determines planar curvature; common-mode length
//! determines axial strain.  The model is intentionally low order, but it
//! preserves the important coupling, saturation, compliance, and backlash for
//! controller and assembly-sequence development.

use crate::actuator::{ActuatorConfig, ActuatorState};
use crate::geometry::Shape;
use crate::math::{Pose, Quat, Vec3, EPSILON};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmSegmentConfig {
    pub nominal_length_m: f64,
    pub tendon_radius_m: f64,
    pub backbone_radius_m: f64,
    pub tendon_angles_rad: [f64; 3],
    pub max_curvature_m_inv: f64,
    pub max_axial_strain: f64,
    /// Number of centerline intervals emitted for collision and visualization.
    pub sample_intervals: u16,
}

impl Default for ArmSegmentConfig {
    fn default() -> Self {
        Self {
            nominal_length_m: 8.0e-3,
            tendon_radius_m: 0.65e-3,
            backbone_radius_m: 0.45e-3,
            tendon_angles_rad: [
                0.0,
                2.0 * core::f64::consts::PI / 3.0,
                4.0 * core::f64::consts::PI / 3.0,
            ],
            max_curvature_m_inv: 160.0,
            max_axial_strain: 0.04,
            sample_intervals: 8,
        }
    }
}

impl ArmSegmentConfig {
    pub fn is_valid(self) -> bool {
        if self.nominal_length_m <= 0.0
            || self.tendon_radius_m <= 0.0
            || self.backbone_radius_m <= 0.0
            || self.tendon_radius_m < self.backbone_radius_m
            || self.max_curvature_m_inv <= 0.0
            || self.max_axial_strain < 0.0
            || self.sample_intervals == 0
            || !self.tendon_angles_rad.iter().all(|value| value.is_finite())
        {
            return false;
        }
        // Reject layouts whose least-squares curvature matrix is singular.
        let (mut cc, mut cs, mut ss) = (0.0, 0.0, 0.0);
        for angle in self.tendon_angles_rad {
            let (s, c) = angle.sin_cos();
            cc += c * c;
            cs += c * s;
            ss += s * s;
        }
        cc * ss - cs * cs > 1.0e-9
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ArmConfig {
    pub segments: Vec<ArmSegmentConfig>,
    pub tendon_actuator: ActuatorConfig,
}

impl ArmConfig {
    pub fn is_valid(&self) -> bool {
        !self.segments.is_empty()
            && self
                .segments
                .iter()
                .copied()
                .all(ArmSegmentConfig::is_valid)
            && self.tendon_actuator.is_valid()
    }
}

impl Default for ArmConfig {
    fn default() -> Self {
        Self {
            segments: vec![ArmSegmentConfig::default(), ArmSegmentConfig::default()],
            tendon_actuator: ActuatorConfig::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmSegmentState {
    /// Actual tendon length offsets from nominal after actuator mechanics.
    pub tendon_offsets_m: [f64; 3],
    pub arc_length_m: f64,
    pub curvature_x_m_inv: f64,
    pub curvature_y_m_inv: f64,
    pub curvature_saturated: bool,
    pub axial_strain_saturated: bool,
}

impl ArmSegmentState {
    fn straight(config: ArmSegmentConfig) -> Self {
        Self {
            tendon_offsets_m: [0.0; 3],
            arc_length_m: config.nominal_length_m,
            curvature_x_m_inv: 0.0,
            curvature_y_m_inv: 0.0,
            curvature_saturated: false,
            axial_strain_saturated: false,
        }
    }

    pub fn curvature_m_inv(self) -> f64 {
        self.curvature_x_m_inv.hypot(self.curvature_y_m_inv)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SegmentFrame {
    pub segment_index: u16,
    pub sample_index: u16,
    pub arc_fraction: f64,
    pub centerline_pose: Pose,
    pub curvature_x_m_inv: f64,
    pub curvature_y_m_inv: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ArmKinematics {
    pub frames: Vec<SegmentFrame>,
    pub tip_pose: Pose,
    pub total_arc_length_m: f64,
}

impl ArmKinematics {
    /// Produce conservative straight capsules between adjacent centerline
    /// samples. Each capsule is expressed in world coordinates.
    pub fn collision_capsules(&self, radius_m: f64) -> Vec<(Pose, Shape)> {
        let mut result = Vec::new();
        for pair in self.frames.windows(2) {
            if pair[0].segment_index != pair[1].segment_index {
                continue;
            }
            let a = pair[0].centerline_pose.translation;
            let b = pair[1].centerline_pose.translation;
            let delta = b - a;
            let length = delta.length();
            if length <= EPSILON {
                continue;
            }
            result.push((
                Pose::new(
                    (a + b) * 0.5,
                    Quat::from_two_vectors(Vec3::Z, delta / length),
                ),
                Shape::Capsule {
                    radius_m,
                    half_segment_m: length * 0.5,
                },
            ));
        }
        result
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmError {
    InvalidConfig,
    SegmentOutOfRange,
    LoadCountMismatch,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContinuumArm {
    pub config: ArmConfig,
    pub base_pose: Pose,
    pub segments: Vec<ArmSegmentState>,
    /// Three actuator states for every segment, in segment-major order.
    pub actuators: Vec<[ActuatorState; 3]>,
}

impl ContinuumArm {
    pub fn new(config: ArmConfig, base_pose: Pose) -> Result<Self, ArmError> {
        if !config.is_valid() {
            return Err(ArmError::InvalidConfig);
        }
        let segments = config
            .segments
            .iter()
            .copied()
            .map(ArmSegmentState::straight)
            .collect::<Vec<_>>();
        let actuators = config
            .segments
            .iter()
            .map(|_| core::array::from_fn(|_| ActuatorState::new(0.0, config.tendon_actuator)))
            .collect();
        Ok(Self {
            config,
            base_pose,
            segments,
            actuators,
        })
    }

    pub fn set_tendon_commands(
        &mut self,
        segment_index: usize,
        offsets_m: [f64; 3],
    ) -> Result<(), ArmError> {
        let actuators = self
            .actuators
            .get_mut(segment_index)
            .ok_or(ArmError::SegmentOutOfRange)?;
        for (actuator, command) in actuators.iter_mut().zip(offsets_m) {
            actuator.set_command(command, self.config.tendon_actuator);
        }
        Ok(())
    }

    /// Bypass actuator dynamics for calibration and inverse-kinematics tests.
    pub fn set_ideal_tendon_offsets(
        &mut self,
        segment_index: usize,
        offsets_m: [f64; 3],
    ) -> Result<(), ArmError> {
        if segment_index >= self.segments.len() {
            return Err(ArmError::SegmentOutOfRange);
        }
        for (actuator, offset) in self.actuators[segment_index].iter_mut().zip(offsets_m) {
            let value = offset.clamp(
                self.config.tendon_actuator.min_position_m,
                self.config.tendon_actuator.max_position_m,
            );
            actuator.command_position_m = value;
            actuator.motor_position_m = value;
            actuator.transmission_position_m = value;
            actuator.output_position_m = value;
            actuator.output_velocity_m_s = 0.0;
        }
        self.refresh_segment_state(segment_index);
        Ok(())
    }

    pub fn step_actuators(
        &mut self,
        dt_s: f64,
        external_tendon_loads_n: &[[f64; 3]],
    ) -> Result<(), ArmError> {
        if external_tendon_loads_n.len() != self.actuators.len() {
            return Err(ArmError::LoadCountMismatch);
        }
        for (segment_index, (actuators, loads)) in self
            .actuators
            .iter_mut()
            .zip(external_tendon_loads_n.iter())
            .enumerate()
        {
            for tendon_index in 0..3 {
                actuators[tendon_index].step(
                    dt_s,
                    loads[tendon_index],
                    self.config.tendon_actuator,
                );
            }
            // Inline refresh to avoid borrowing self twice.
            self.segments[segment_index] = solve_segment_state(
                self.config.segments[segment_index],
                core::array::from_fn(|index| actuators[index].output_position_m),
            );
        }
        Ok(())
    }

    fn refresh_segment_state(&mut self, segment_index: usize) {
        let offsets =
            core::array::from_fn(|index| self.actuators[segment_index][index].output_position_m);
        self.segments[segment_index] =
            solve_segment_state(self.config.segments[segment_index], offsets);
    }

    pub fn forward_kinematics(&self) -> ArmKinematics {
        let mut parent = self.base_pose;
        let mut frames = Vec::new();
        let mut total_arc_length_m = 0.0;

        for (segment_index, (config, state)) in self
            .config
            .segments
            .iter()
            .zip(self.segments.iter())
            .enumerate()
        {
            for sample_index in 0..=config.sample_intervals {
                let fraction = sample_index as f64 / config.sample_intervals as f64;
                let local = constant_curvature_pose(*state, state.arc_length_m * fraction);
                frames.push(SegmentFrame {
                    segment_index: segment_index as u16,
                    sample_index,
                    arc_fraction: fraction,
                    centerline_pose: parent * local,
                    curvature_x_m_inv: state.curvature_x_m_inv,
                    curvature_y_m_inv: state.curvature_y_m_inv,
                });
            }
            parent = parent * constant_curvature_pose(*state, state.arc_length_m);
            total_arc_length_m += state.arc_length_m;
        }

        ArmKinematics {
            frames,
            tip_pose: parent,
            total_arc_length_m,
        }
    }
}

fn solve_segment_state(config: ArmSegmentConfig, tendon_offsets_m: [f64; 3]) -> ArmSegmentState {
    let mean_offset = tendon_offsets_m.iter().sum::<f64>() / 3.0;
    let unconstrained_strain = mean_offset / config.nominal_length_m;
    let axial_strain =
        unconstrained_strain.clamp(-config.max_axial_strain, config.max_axial_strain);
    let arc_length_m = config.nominal_length_m * (1.0 + axial_strain);
    let deviations = tendon_offsets_m.map(|offset| offset - mean_offset);

    let (mut cc, mut cs, mut ss, mut cd, mut sd) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for ((angle, deviation), _) in config
        .tendon_angles_rad
        .iter()
        .zip(deviations.iter())
        .zip(0..3)
    {
        let (s, c) = angle.sin_cos();
        cc += c * c;
        cs += c * s;
        ss += s * s;
        cd += c * deviation;
        sd += s * deviation;
    }
    let determinant = cc * ss - cs * cs;
    let scale = -1.0 / (config.tendon_radius_m * arc_length_m * determinant);
    let mut curvature_x_m_inv = scale * (ss * cd - cs * sd);
    let mut curvature_y_m_inv = scale * (-cs * cd + cc * sd);
    let curvature = curvature_x_m_inv.hypot(curvature_y_m_inv);
    let curvature_saturated = curvature > config.max_curvature_m_inv;
    if curvature_saturated {
        let ratio = config.max_curvature_m_inv / curvature;
        curvature_x_m_inv *= ratio;
        curvature_y_m_inv *= ratio;
    }

    ArmSegmentState {
        tendon_offsets_m,
        arc_length_m,
        curvature_x_m_inv,
        curvature_y_m_inv,
        curvature_saturated,
        axial_strain_saturated: unconstrained_strain.abs() > config.max_axial_strain,
    }
}

fn constant_curvature_pose(state: ArmSegmentState, arc_distance_m: f64) -> Pose {
    let curvature = state.curvature_m_inv();
    if curvature <= 1.0e-9 {
        return Pose::from_translation(Vec3::Z * arc_distance_m);
    }
    let bend_direction = Vec3::new(
        state.curvature_x_m_inv / curvature,
        state.curvature_y_m_inv / curvature,
        0.0,
    );
    let bend_axis = Vec3::Z.cross(bend_direction).normalized_or(Vec3::X);
    let angle = curvature * arc_distance_m;
    let position =
        bend_direction * ((1.0 - angle.cos()) / curvature) + Vec3::Z * (angle.sin() / curvature);
    Pose::new(position, Quat::from_axis_angle(bend_axis, angle))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one_segment() -> ContinuumArm {
        let config = ArmConfig {
            segments: vec![ArmSegmentConfig {
                sample_intervals: 4,
                ..ArmSegmentConfig::default()
            }],
            tendon_actuator: ActuatorConfig::default(),
        };
        ContinuumArm::new(config, Pose::IDENTITY).unwrap()
    }

    #[test]
    fn equal_tendons_produce_straight_axial_extension() {
        let mut arm = one_segment();
        arm.set_ideal_tendon_offsets(0, [100.0e-6; 3]).unwrap();
        let kinematics = arm.forward_kinematics();
        assert!(kinematics.tip_pose.translation.x.abs() < 1e-12);
        assert!(kinematics.tip_pose.translation.y.abs() < 1e-12);
        assert!((kinematics.tip_pose.translation.z - 8.1e-3).abs() < 1e-12);
    }

    #[test]
    fn shortening_positive_x_tendon_bends_toward_positive_x() {
        let mut arm = one_segment();
        arm.set_ideal_tendon_offsets(0, [-100.0e-6, 50.0e-6, 50.0e-6])
            .unwrap();
        let kinematics = arm.forward_kinematics();
        assert!(arm.segments[0].curvature_x_m_inv > 0.0);
        assert!(kinematics.tip_pose.translation.x > 0.0);
        assert!(kinematics.tip_pose.translation.z > 0.0);
    }

    #[test]
    fn curvature_saturates_without_changing_direction() {
        let mut arm = one_segment();
        arm.set_ideal_tendon_offsets(0, [-1.0e-3, 0.5e-3, 0.5e-3])
            .unwrap();
        assert!(arm.segments[0].curvature_saturated);
        assert!(
            (arm.segments[0].curvature_m_inv() - arm.config.segments[0].max_curvature_m_inv).abs()
                < 1e-10
        );
    }

    #[test]
    fn two_segments_compose_in_parent_tip_frame() {
        let mut arm = ContinuumArm::new(ArmConfig::default(), Pose::IDENTITY).unwrap();
        arm.set_ideal_tendon_offsets(0, [-80.0e-6, 40.0e-6, 40.0e-6])
            .unwrap();
        arm.set_ideal_tendon_offsets(1, [0.0; 3]).unwrap();
        let tip = arm.forward_kinematics().tip_pose.translation;
        assert!(
            tip.x
                > arm.forward_kinematics().frames[8]
                    .centerline_pose
                    .translation
                    .x
        );
    }

    #[test]
    fn centerline_capsules_cover_each_sample_interval() {
        let arm = one_segment();
        let kinematics = arm.forward_kinematics();
        let capsules = kinematics.collision_capsules(0.2e-3);
        assert_eq!(capsules.len(), 4);
        for (_, shape) in capsules {
            assert!(shape.is_valid());
        }
    }

    #[test]
    fn singular_tendon_layout_is_rejected() {
        let mut config = ArmConfig::default();
        config.segments[0].tendon_angles_rad = [0.0, 0.0, core::f64::consts::PI];
        assert_eq!(
            ContinuumArm::new(config, Pose::IDENTITY),
            Err(ArmError::InvalidConfig)
        );
    }
}
