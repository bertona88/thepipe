//! Cell-level machine configuration and deterministic manipulator motion.
//!
//! This module is deliberately independent of the task executive and the
//! browser.  It defines the plant-facing command vocabulary shared by simulated
//! and future hardware backends.  The existing [`crate::SerialArm`] remains the
//! local tendon/FK model; [`ManipulatorMotionState`] owns the carriage and
//! actuator targets that drive it.

use crate::gripper::GripperConfig;
use crate::serial_arm::{SerialArmConfig, SerialJointPositions, TENDON_JOINT_COUNT};

pub const MACHINE_CONFIG_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RailTopology {
    /// One axial rail per arm, carried around common end tracks by paired
    /// belt-driven bogies.  Azimuth motion rotates the complete axial rail.
    PairedBeltEndBogies,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TubeGeometry {
    pub inner_radius_m: f64,
    pub working_length_m: f64,
    pub central_work_radius_m: f64,
}

impl TubeGeometry {
    pub fn is_valid(self) -> bool {
        self.inner_radius_m > 0.0
            && self.inner_radius_m.is_finite()
            && self.working_length_m > 0.0
            && self.working_length_m.is_finite()
            && self.central_work_radius_m > 0.0
            && self.central_work_radius_m <= self.inner_radius_m
            && self.central_work_radius_m.is_finite()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarriageConfig {
    pub topology: RailTopology,
    /// Shoulder datum radius, not the carbon-rail body radius.
    pub rail_radius_m: f64,
    pub z_limits_m: [f64; 2],
    pub max_z_speed_m_s: f64,
    pub max_theta_speed_rad_s: f64,
    pub max_z_accel_m_s2: f64,
    pub max_theta_accel_rad_s2: f64,
}

impl Default for CarriageConfig {
    fn default() -> Self {
        Self {
            topology: RailTopology::PairedBeltEndBogies,
            rail_radius_m: 72.0e-3,
            z_limits_m: [-150.0e-3, 150.0e-3],
            max_z_speed_m_s: 40.0e-3,
            max_theta_speed_rad_s: 0.60,
            max_z_accel_m_s2: 0.20,
            max_theta_accel_rad_s2: 2.0,
        }
    }
}

impl CarriageConfig {
    pub fn from_serial_arm(config: SerialArmConfig) -> Self {
        Self {
            rail_radius_m: config.rail_radius_m,
            z_limits_m: config.base_z_limits_m,
            ..Self::default()
        }
    }

    pub fn is_valid(self) -> bool {
        self.rail_radius_m > 0.0
            && self.rail_radius_m.is_finite()
            && self.z_limits_m[0].is_finite()
            && self.z_limits_m[1].is_finite()
            && self.z_limits_m[0] <= self.z_limits_m[1]
            && self.max_z_speed_m_s > 0.0
            && self.max_z_speed_m_s.is_finite()
            && self.max_theta_speed_rad_s > 0.0
            && self.max_theta_speed_rad_s.is_finite()
            && self.max_z_accel_m_s2 > 0.0
            && self.max_z_accel_m_s2.is_finite()
            && self.max_theta_accel_rad_s2 > 0.0
            && self.max_theta_accel_rad_s2.is_finite()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CarriageState {
    pub z_m: f64,
    pub theta_rad: f64,
    pub z_velocity_m_s: f64,
    pub theta_velocity_rad_s: f64,
}

impl CarriageState {
    pub fn is_finite(self) -> bool {
        self.z_m.is_finite()
            && self.theta_rad.is_finite()
            && self.z_velocity_m_s.is_finite()
            && self.theta_velocity_rad_s.is_finite()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CarriageTarget {
    pub z_m: f64,
    pub theta_rad: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManipulatorMotionConfig {
    pub max_joint_speed_rad_s: [f64; TENDON_JOINT_COUNT],
    pub max_joint_accel_rad_s2: [f64; TENDON_JOINT_COUNT],
}

impl Default for ManipulatorMotionConfig {
    fn default() -> Self {
        Self {
            max_joint_speed_rad_s: [1.50, 1.25, 1.25, 2.00],
            max_joint_accel_rad_s2: [6.0, 5.0, 5.0, 8.0],
        }
    }
}

impl ManipulatorMotionConfig {
    pub fn is_valid(self) -> bool {
        self.max_joint_speed_rad_s
            .iter()
            .chain(self.max_joint_accel_rad_s2.iter())
            .all(|value| value.is_finite() && *value > 0.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SafetyConfig {
    pub minimum_unplanned_clearance_m: f64,
    pub watchdog_timeout_s: f64,
    pub commissioning_speed_scale: f64,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            minimum_unplanned_clearance_m: 0.20e-3,
            watchdog_timeout_s: 0.100,
            commissioning_speed_scale: 0.20,
        }
    }
}

impl SafetyConfig {
    pub fn is_valid(self) -> bool {
        self.minimum_unplanned_clearance_m >= 0.0
            && self.minimum_unplanned_clearance_m.is_finite()
            && self.watchdog_timeout_s > 0.0
            && self.watchdog_timeout_s.is_finite()
            && self.commissioning_speed_scale > 0.0
            && self.commissioning_speed_scale <= 1.0
            && self.commissioning_speed_scale.is_finite()
    }
}

/// Hardware qualification targets, not claims about the current simulator or
/// an unmeasured prototype.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QualificationTargets {
    pub minimum_payload_kg: f64,
    pub minimum_tool_force_n: f64,
    pub force_hold_duration_s: f64,
    pub maximum_observed_position_sigma_m: f64,
    pub maximum_closed_loop_tcp_error_m: f64,
    pub smallest_declared_feature_m: f64,
}

impl Default for QualificationTargets {
    fn default() -> Self {
        Self {
            minimum_payload_kg: 5.0e-3,
            minimum_tool_force_n: 50.0e-3,
            force_hold_duration_s: 2.0,
            maximum_observed_position_sigma_m: 10.0e-6,
            maximum_closed_loop_tcp_error_m: 12.0e-6,
            smallest_declared_feature_m: 8.0e-6,
        }
    }
}

impl QualificationTargets {
    pub fn is_valid(self) -> bool {
        self.minimum_payload_kg > 0.0
            && self.minimum_payload_kg.is_finite()
            && self.minimum_tool_force_n > 0.0
            && self.minimum_tool_force_n.is_finite()
            && self.force_hold_duration_s > 0.0
            && self.force_hold_duration_s.is_finite()
            && self.maximum_observed_position_sigma_m > 0.0
            && self.maximum_observed_position_sigma_m.is_finite()
            && self.maximum_closed_loop_tcp_error_m > 0.0
            && self.maximum_closed_loop_tcp_error_m.is_finite()
            && self.smallest_declared_feature_m > 0.0
            && self.smallest_declared_feature_m.is_finite()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PipeCellConfig {
    pub schema_version: u32,
    pub manipulator_count: u8,
    pub tube: TubeGeometry,
    pub carriage: CarriageConfig,
    pub arm: SerialArmConfig,
    pub motion: ManipulatorMotionConfig,
    pub gripper: GripperConfig,
    pub safety: SafetyConfig,
    pub qualification: QualificationTargets,
}

impl Default for PipeCellConfig {
    fn default() -> Self {
        let arm = SerialArmConfig::default();
        Self {
            schema_version: MACHINE_CONFIG_SCHEMA_VERSION,
            manipulator_count: 4,
            tube: TubeGeometry {
                inner_radius_m: 80.0e-3,
                working_length_m: 320.0e-3,
                central_work_radius_m: 40.0e-3,
            },
            carriage: CarriageConfig::from_serial_arm(arm),
            arm,
            motion: ManipulatorMotionConfig::default(),
            gripper: GripperConfig::default(),
            safety: SafetyConfig::default(),
            qualification: QualificationTargets::default(),
        }
    }
}

impl PipeCellConfig {
    pub fn is_valid(self) -> bool {
        self.schema_version == MACHINE_CONFIG_SCHEMA_VERSION
            && self.manipulator_count > 0
            && self.tube.is_valid()
            && self.carriage.is_valid()
            && self.arm.is_valid()
            && self.motion.is_valid()
            && self.gripper.is_valid()
            && self.safety.is_valid()
            && self.qualification.is_valid()
            && (self.carriage.rail_radius_m - self.arm.rail_radius_m).abs() <= f64::EPSILON
            && self.carriage.z_limits_m == self.arm.base_z_limits_m
            && self.carriage.rail_radius_m < self.tube.inner_radius_m
            && self.arm.maximum_reach_m() >= self.carriage.rail_radius_m
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ManipulatorId(pub u32);

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MachineCommand {
    MoveCarriageZ {
        manipulator: ManipulatorId,
        target_z_m: f64,
    },
    MoveCarriageTheta {
        manipulator: ManipulatorId,
        target_theta_rad: f64,
    },
    SetJointTargets {
        manipulator: ManipulatorId,
        target_rad: [f64; TENDON_JOINT_COUNT],
    },
    SetGripperOpening {
        manipulator: ManipulatorId,
        target_opening_m: f64,
    },
    /// Controlled hold. `None` addresses every manipulator.
    Stop { manipulator: Option<ManipulatorId> },
}

impl MachineCommand {
    pub fn manipulator(self) -> Option<ManipulatorId> {
        match self {
            Self::MoveCarriageZ { manipulator, .. }
            | Self::MoveCarriageTheta { manipulator, .. }
            | Self::SetJointTargets { manipulator, .. }
            | Self::SetGripperOpening { manipulator, .. } => Some(manipulator),
            Self::Stop { manipulator } => manipulator,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MachineCommandError {
    NonFiniteTarget,
    TargetOutOfRange,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MachineCommandEvent {
    pub sequence: u64,
    pub issued_at_tick: u64,
    pub command: MachineCommand,
}

/// Interface implemented by the deterministic simulator and, later, by the
/// hardware I/O adapter. Controllers depend on this boundary rather than on a
/// particular physics engine.
pub trait MachineBackend {
    type Error;

    fn submit_command(&mut self, command: MachineCommand) -> Result<u64, Self::Error>;
    fn advance_fixed_step(&mut self) -> Result<(), Self::Error>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManipulatorMotionState {
    pub carriage: CarriageState,
    pub carriage_target: CarriageTarget,
    pub joint_positions_rad: [f64; TENDON_JOINT_COUNT],
    pub joint_velocities_rad_s: [f64; TENDON_JOINT_COUNT],
    pub joint_targets_rad: [f64; TENDON_JOINT_COUNT],
    pub stopped: bool,
}

impl ManipulatorMotionState {
    pub fn from_positions(positions: SerialJointPositions) -> Self {
        let joints = positions.tendon_joint_angles();
        Self {
            carriage: CarriageState {
                z_m: positions.base_z_m,
                theta_rad: wrap_angle_pi(positions.base_theta_rad),
                z_velocity_m_s: 0.0,
                theta_velocity_rad_s: 0.0,
            },
            carriage_target: CarriageTarget {
                z_m: positions.base_z_m,
                theta_rad: wrap_angle_pi(positions.base_theta_rad),
            },
            joint_positions_rad: joints,
            joint_velocities_rad_s: [0.0; TENDON_JOINT_COUNT],
            joint_targets_rad: joints,
            stopped: false,
        }
    }

    pub fn positions(self) -> SerialJointPositions {
        SerialJointPositions {
            base_z_m: self.carriage.z_m,
            base_theta_rad: self.carriage.theta_rad,
            shoulder_yaw_rad: self.joint_positions_rad[0],
            shoulder_pitch_rad: self.joint_positions_rad[1],
            elbow_pitch_rad: self.joint_positions_rad[2],
            wrist_roll_rad: self.joint_positions_rad[3],
        }
    }

    pub fn validate_command(
        command: MachineCommand,
        carriage: CarriageConfig,
        arm: SerialArmConfig,
        gripper: GripperConfig,
    ) -> Result<(), MachineCommandError> {
        match command {
            MachineCommand::MoveCarriageZ { target_z_m, .. } => {
                if !target_z_m.is_finite() {
                    Err(MachineCommandError::NonFiniteTarget)
                } else if !(carriage.z_limits_m[0]..=carriage.z_limits_m[1]).contains(&target_z_m) {
                    Err(MachineCommandError::TargetOutOfRange)
                } else {
                    Ok(())
                }
            }
            MachineCommand::MoveCarriageTheta {
                target_theta_rad, ..
            } => {
                if target_theta_rad.is_finite() {
                    Ok(())
                } else {
                    Err(MachineCommandError::NonFiniteTarget)
                }
            }
            MachineCommand::SetJointTargets { target_rad, .. } => {
                if target_rad.iter().any(|value| !value.is_finite()) {
                    return Err(MachineCommandError::NonFiniteTarget);
                }
                if target_rad
                    .iter()
                    .zip(arm.joint_limits_rad)
                    .any(|(value, limits)| !(limits[0]..=limits[1]).contains(value))
                {
                    Err(MachineCommandError::TargetOutOfRange)
                } else {
                    Ok(())
                }
            }
            MachineCommand::SetGripperOpening {
                target_opening_m, ..
            } => {
                if !target_opening_m.is_finite() {
                    Err(MachineCommandError::NonFiniteTarget)
                } else if !(gripper.min_opening_m..=gripper.max_opening_m)
                    .contains(&target_opening_m)
                {
                    Err(MachineCommandError::TargetOutOfRange)
                } else {
                    Ok(())
                }
            }
            MachineCommand::Stop { .. } => Ok(()),
        }
    }

    pub fn apply_command(&mut self, command: MachineCommand) {
        match command {
            MachineCommand::MoveCarriageZ { target_z_m, .. } => {
                self.carriage_target.z_m = target_z_m;
                self.stopped = false;
            }
            MachineCommand::MoveCarriageTheta {
                target_theta_rad, ..
            } => {
                self.carriage_target.theta_rad = wrap_angle_pi(target_theta_rad);
                self.stopped = false;
            }
            MachineCommand::SetJointTargets { target_rad, .. } => {
                self.joint_targets_rad = target_rad;
                self.stopped = false;
            }
            MachineCommand::SetGripperOpening { .. } => {}
            MachineCommand::Stop { .. } => self.stop(),
        }
    }

    pub fn stop(&mut self) {
        self.carriage_target = CarriageTarget {
            z_m: self.carriage.z_m,
            theta_rad: self.carriage.theta_rad,
        };
        self.joint_targets_rad = self.joint_positions_rad;
        self.carriage.z_velocity_m_s = 0.0;
        self.carriage.theta_velocity_rad_s = 0.0;
        self.joint_velocities_rad_s = [0.0; TENDON_JOINT_COUNT];
        self.stopped = true;
    }

    pub fn step(
        &mut self,
        dt_s: f64,
        carriage_config: CarriageConfig,
        arm_config: SerialArmConfig,
        motion_config: ManipulatorMotionConfig,
    ) {
        if self.stopped
            || dt_s <= 0.0
            || !dt_s.is_finite()
            || !carriage_config.is_valid()
            || !arm_config.is_valid()
            || !motion_config.is_valid()
        {
            return;
        }

        step_linear_axis(
            &mut self.carriage.z_m,
            &mut self.carriage.z_velocity_m_s,
            self.carriage_target.z_m,
            carriage_config.max_z_speed_m_s,
            carriage_config.max_z_accel_m_s2,
            dt_s,
        );
        self.carriage.z_m = self
            .carriage
            .z_m
            .clamp(carriage_config.z_limits_m[0], carriage_config.z_limits_m[1]);

        let theta_error = wrap_angle_pi(self.carriage_target.theta_rad - self.carriage.theta_rad);
        let theta_unwrapped_target = self.carriage.theta_rad + theta_error;
        step_linear_axis(
            &mut self.carriage.theta_rad,
            &mut self.carriage.theta_velocity_rad_s,
            theta_unwrapped_target,
            carriage_config.max_theta_speed_rad_s,
            carriage_config.max_theta_accel_rad_s2,
            dt_s,
        );
        self.carriage.theta_rad = wrap_angle_pi(self.carriage.theta_rad);

        for index in 0..TENDON_JOINT_COUNT {
            step_linear_axis(
                &mut self.joint_positions_rad[index],
                &mut self.joint_velocities_rad_s[index],
                self.joint_targets_rad[index],
                motion_config.max_joint_speed_rad_s[index],
                motion_config.max_joint_accel_rad_s2[index],
                dt_s,
            );
            self.joint_positions_rad[index] = self.joint_positions_rad[index].clamp(
                arm_config.joint_limits_rad[index][0],
                arm_config.joint_limits_rad[index][1],
            );
        }
    }
}

fn step_linear_axis(
    position: &mut f64,
    velocity: &mut f64,
    target: f64,
    max_speed: f64,
    max_acceleration: f64,
    dt_s: f64,
) {
    let error = target - *position;
    let stop_speed = (2.0 * max_acceleration * error.abs()).sqrt();
    let desired_velocity = error.signum() * max_speed.min(stop_speed);
    let velocity_delta =
        (desired_velocity - *velocity).clamp(-max_acceleration * dt_s, max_acceleration * dt_s);
    *velocity = (*velocity + velocity_delta).clamp(-max_speed, max_speed);

    let displacement = *velocity * dt_s;
    if displacement.signum() == error.signum() && displacement.abs() >= error.abs() {
        *position = target;
        *velocity = 0.0;
    } else {
        *position += displacement;
    }
}

pub fn wrap_angle_pi(angle_rad: f64) -> f64 {
    let two_pi = 2.0 * core::f64::consts::PI;
    (angle_rad + core::f64::consts::PI).rem_euclid(two_pi) - core::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baseline_configuration_is_internally_consistent() {
        let config = PipeCellConfig::default();
        assert!(config.is_valid());
        assert_eq!(config.carriage.topology, RailTopology::PairedBeltEndBogies);
        assert_eq!(config.tube.inner_radius_m, 80.0e-3);
        assert_eq!(config.tube.working_length_m, 320.0e-3);
    }

    #[test]
    fn carriage_motion_obeys_velocity_and_acceleration_limits() {
        let carriage_config = CarriageConfig::default();
        let arm_config = SerialArmConfig::default();
        let motion_config = ManipulatorMotionConfig::default();
        let mut state = ManipulatorMotionState::from_positions(SerialJointPositions::default());
        state.carriage_target.z_m = 50.0e-3;
        let dt_s = 0.001;
        let mut previous_velocity = 0.0;
        for _ in 0..2_000 {
            state.step(dt_s, carriage_config, arm_config, motion_config);
            assert!(state.carriage.z_velocity_m_s.abs() <= carriage_config.max_z_speed_m_s);
            assert!(
                (state.carriage.z_velocity_m_s - previous_velocity).abs()
                    <= carriage_config.max_z_accel_m_s2 * dt_s + 1.0e-15
            );
            previous_velocity = state.carriage.z_velocity_m_s;
        }
        assert!((state.carriage.z_m - 50.0e-3).abs() < 1.0e-12);
        assert_eq!(state.carriage.z_velocity_m_s, 0.0);
    }

    #[test]
    fn theta_uses_the_short_periodic_path() {
        let carriage_config = CarriageConfig::default();
        let arm_config = SerialArmConfig::default();
        let motion_config = ManipulatorMotionConfig::default();
        let mut positions = SerialJointPositions::default();
        positions.base_theta_rad = 179.0_f64.to_radians();
        let mut state = ManipulatorMotionState::from_positions(positions);
        state.carriage_target.theta_rad = (-179.0_f64).to_radians();
        for _ in 0..1_000 {
            state.step(0.001, carriage_config, arm_config, motion_config);
        }
        assert!(
            wrap_angle_pi(state.carriage.theta_rad - state.carriage_target.theta_rad).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn identical_commands_produce_identical_traces() {
        let carriage_config = CarriageConfig::default();
        let arm_config = SerialArmConfig::default();
        let motion_config = ManipulatorMotionConfig::default();
        let mut a = ManipulatorMotionState::from_positions(SerialJointPositions::default());
        let mut b = a;
        let command = MachineCommand::SetJointTargets {
            manipulator: ManipulatorId(1),
            target_rad: [0.25, -0.30, 0.45, 0.10],
        };
        a.apply_command(command);
        b.apply_command(command);
        for _ in 0..750 {
            a.step(0.001, carriage_config, arm_config, motion_config);
            b.step(0.001, carriage_config, arm_config, motion_config);
            assert_eq!(a, b);
        }
    }

    #[test]
    fn out_of_range_targets_are_rejected_instead_of_silently_clamped() {
        let result = ManipulatorMotionState::validate_command(
            MachineCommand::MoveCarriageZ {
                manipulator: ManipulatorId(1),
                target_z_m: 1.0,
            },
            CarriageConfig::default(),
            SerialArmConfig::default(),
            GripperConfig::default(),
        );
        assert_eq!(result, Err(MachineCommandError::TargetOutOfRange));
    }
}
