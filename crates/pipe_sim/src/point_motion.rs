//! Standalone M1b runtime for deterministic one-arm Cartesian point motion.

use pipe_optics::StructuredLightRig;
use pipe_sim_core::{
    ArmId, MachineCommand, ManipulatorId, PipeCellConfig, SerialArm, SerialArmInstance,
    SerialJointPositions, Simulation, SimulationConfig, ToolMotionStatus, Vec3,
};
use serde::Serialize;

use crate::{
    build_optics, machine_config, scene, serialize_json, SceneDescription, SceneFrame, SimError,
};

pub const POINT_MOTION_REPORT_SCHEMA_VERSION: u32 = 1;
pub const CALIBRATION_TARGET_WORLD_M: [f64; 3] = [20.0e-3, 0.0, 0.0];
pub const CALIBRATION_APPROACH_WORLD_M: [f64; 3] = [20.0e-3, 0.0, 5.0e-3];

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct PointMotionTraceRecord {
    pub tick: u64,
    pub time_s: f64,
    pub manipulator_id: u32,
    pub target_position_world_m: [f64; 3],
    pub actual_position_world_m: [f64; 3],
    pub position_error_m: f64,
    pub progress: f64,
    pub carriage_z_m: f64,
    pub carriage_theta_rad: f64,
    pub joint_positions_rad: [f64; 4],
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct PointMotionReport {
    pub schema_version: u32,
    pub machine_config_id: String,
    pub machine_config_sha256: String,
    pub command_sequence: u64,
    pub manipulator_id: u32,
    pub status: &'static str,
    pub target_position_world_m: Option<[f64; 3]>,
    pub planned_duration_s: Option<f64>,
    pub final_position_error_m: Option<f64>,
    pub trace: Vec<PointMotionTraceRecord>,
}

impl PointMotionReport {
    pub fn to_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(self, pretty)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct CalibrationCycleReport {
    pub schema_version: u32,
    pub sequence: [&'static str; 3],
    pub legs: Vec<PointMotionReport>,
    pub maximum_final_position_error_m: f64,
}

impl CalibrationCycleReport {
    pub fn to_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(self, pretty)
    }
}

/// Isolated machine runtime used for M1b acceptance and browser integration.
/// It has no gearbox executive: submitted Cartesian commands remain in control
/// until completion or stop.
pub struct PointMotionRuntime {
    machine_config_id: String,
    machine_config_sha256: String,
    cell_config: PipeCellConfig,
    mechanics: Simulation,
    optics: StructuredLightRig,
    active_manipulator: ManipulatorId,
}

impl PointMotionRuntime {
    pub fn new() -> Result<Self, SimError> {
        let loaded = machine_config::load_baseline_machine_config()?;
        let mut mechanics = Simulation::new(SimulationConfig {
            fixed_dt_s: 0.001,
            gravity_m_s2: Vec3::ZERO,
            ..SimulationConfig::default()
        })
        .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
        for arm_index in 0..loaded.cell.manipulator_count {
            let mut arm = SerialArm::new(loaded.cell.arm)
                .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
            let theta_rad = f64::from(arm_index) * core::f64::consts::TAU
                / f64::from(loaded.cell.manipulator_count);
            let (base_z_m, shoulder_pitch_rad) = if arm_index == 0 {
                (0.0, 0.0)
            } else {
                let parked_z = [-120.0e-3, 120.0e-3, -80.0e-3][usize::from(arm_index - 1) % 3];
                (parked_z, core::f64::consts::FRAC_PI_2)
            };
            arm.set_positions(SerialJointPositions {
                base_z_m,
                base_theta_rad: theta_rad,
                shoulder_yaw_rad: 0.0,
                shoulder_pitch_rad,
                elbow_pitch_rad: 0.0,
                wrist_roll_rad: 0.0,
            })
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
            let mut instance =
                SerialArmInstance::new(ArmId(u32::from(arm_index) + 1), arm, loaded.cell.gripper)
                    .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
            instance.carriage_config = loaded.cell.carriage;
            instance.motion_config = loaded.cell.motion;
            instance.tool_motion_speed_scale = loaded.cell.safety.commissioning_speed_scale;
            mechanics
                .add_serial_arm(instance)
                .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
        }
        Ok(Self {
            machine_config_id: loaded.id,
            machine_config_sha256: loaded.source_sha256,
            cell_config: loaded.cell,
            mechanics,
            optics: build_optics(0),
            active_manipulator: ManipulatorId(1),
        })
    }

    pub fn submit_tool_target(
        &mut self,
        manipulator: ManipulatorId,
        target_position_world_m: Vec3,
    ) -> Result<u64, SimError> {
        let sequence = self
            .mechanics
            .submit_machine_command(MachineCommand::SetToolPoseTarget {
                manipulator,
                target_position_world_m,
            })
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
        self.mechanics.tool_motion_trace.clear();
        self.active_manipulator = manipulator;
        Ok(sequence)
    }

    pub fn submit_calibration_target(&mut self) -> Result<u64, SimError> {
        self.submit_tool_target(
            ManipulatorId(1),
            Vec3::new(
                CALIBRATION_TARGET_WORLD_M[0],
                CALIBRATION_TARGET_WORLD_M[1],
                CALIBRATION_TARGET_WORLD_M[2],
            ),
        )
    }

    pub fn run_calibration_cycle(
        &mut self,
        max_steps_per_leg: u32,
    ) -> Result<CalibrationCycleReport, SimError> {
        let points = [
            CALIBRATION_APPROACH_WORLD_M,
            CALIBRATION_TARGET_WORLD_M,
            CALIBRATION_APPROACH_WORLD_M,
        ];
        let mut legs = Vec::with_capacity(points.len());
        for point in points {
            self.submit_tool_target(ManipulatorId(1), Vec3::new(point[0], point[1], point[2]))?;
            legs.push(self.run_until_settled(max_steps_per_leg)?);
        }
        let maximum_final_position_error_m = legs
            .iter()
            .filter_map(|leg| leg.final_position_error_m)
            .fold(0.0, f64::max);
        Ok(CalibrationCycleReport {
            schema_version: POINT_MOTION_REPORT_SCHEMA_VERSION,
            sequence: ["approach", "touch", "retreat"],
            legs,
            maximum_final_position_error_m,
        })
    }

    pub fn step(&mut self) -> Result<SceneFrame, SimError> {
        self.mechanics
            .step()
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
        Ok(self.scene_frame())
    }

    pub fn run_until_settled(&mut self, max_steps: u32) -> Result<PointMotionReport, SimError> {
        for _ in 0..max_steps {
            if !self.is_active() {
                return Ok(self.report());
            }
            self.step()?;
        }
        if self.is_active() {
            return Err(SimError::CycleLimit(max_steps));
        }
        Ok(self.report())
    }

    pub fn is_active(&self) -> bool {
        self.mechanics
            .serial_arm(ArmId(self.active_manipulator.0))
            .and_then(|arm| arm.motion.tool_motion)
            .is_some_and(|plan| plan.status == ToolMotionStatus::Active)
    }

    pub fn scene_description(&self) -> SceneDescription {
        scene::build_scene_description(
            &self.machine_config_id,
            &self.machine_config_sha256,
            self.cell_config,
            &self.mechanics,
            &self.optics,
            &[],
        )
    }

    pub fn scene_frame(&self) -> SceneFrame {
        let collisions = self
            .mechanics
            .query_collisions_with_arms(self.mechanics.config.collision);
        scene::build_scene_frame(&self.mechanics, &collisions.contacts)
    }

    pub fn scene_description_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(&self.scene_description(), pretty)
    }

    pub fn scene_frame_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(&self.scene_frame(), pretty)
    }

    pub fn report(&self) -> PointMotionReport {
        let arm = self.mechanics.serial_arm(ArmId(self.active_manipulator.0));
        let plan = arm.and_then(|arm| arm.motion.tool_motion);
        let final_position_error_m = arm.zip(plan).map(|(arm, plan)| {
            (arm.tool_pose().translation - plan.target_position_world_m).length()
        });
        PointMotionReport {
            schema_version: POINT_MOTION_REPORT_SCHEMA_VERSION,
            machine_config_id: self.machine_config_id.clone(),
            machine_config_sha256: self.machine_config_sha256.clone(),
            command_sequence: self.mechanics.machine_command_sequence,
            manipulator_id: self.active_manipulator.0,
            status: plan.map_or("idle", |plan| tool_motion_status_name(plan.status)),
            target_position_world_m: plan.map(|plan| vec3(plan.target_position_world_m)),
            planned_duration_s: plan.map(|plan| plan.duration_s),
            final_position_error_m,
            trace: self
                .mechanics
                .tool_motion_trace
                .iter()
                .map(|sample| PointMotionTraceRecord {
                    tick: sample.tick,
                    time_s: sample.time_s,
                    manipulator_id: sample.manipulator.0,
                    target_position_world_m: vec3(sample.target_position_world_m),
                    actual_position_world_m: vec3(sample.actual_position_world_m),
                    position_error_m: sample.position_error_m,
                    progress: sample.progress,
                    carriage_z_m: sample.positions.base_z_m,
                    carriage_theta_rad: sample.positions.base_theta_rad,
                    joint_positions_rad: sample.positions.tendon_joint_angles(),
                })
                .collect(),
        }
    }
}

impl Default for PointMotionRuntime {
    fn default() -> Self {
        Self::new().expect("embedded baseline machine config is valid")
    }
}

fn tool_motion_status_name(status: ToolMotionStatus) -> &'static str {
    match status {
        ToolMotionStatus::Active => "active",
        ToolMotionStatus::Complete => "complete",
        ToolMotionStatus::Stopped => "stopped",
    }
}

fn vec3(value: Vec3) -> [f64; 3] {
    [value.x, value.y, value.z]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_target_completes_with_a_replayable_trace() {
        let mut runtime = PointMotionRuntime::new().unwrap();
        runtime.submit_calibration_target().unwrap();
        let report = runtime.run_until_settled(20_000).unwrap();
        assert_eq!(report.status, "complete");
        assert!(report.final_position_error_m.unwrap() < 1.0e-9);
        assert!(!report.trace.is_empty());
        assert_eq!(report.trace.last().unwrap().progress, 1.0);
        let json = report.to_json(false).unwrap();
        assert!(json.contains("\"target_position_world_m\""));
        assert!(json.contains("\"position_error_m\""));

        let mut replay = PointMotionRuntime::new().unwrap();
        replay.submit_calibration_target().unwrap();
        let replay_report = replay.run_until_settled(20_000).unwrap();
        assert_eq!(report.trace, replay_report.trace);
    }

    #[test]
    fn scene_exposes_authoritative_tool_motion_status() {
        let mut runtime = PointMotionRuntime::new().unwrap();
        runtime.submit_calibration_target().unwrap();
        runtime.step().unwrap();
        let frame = runtime.scene_frame();
        let commanded = &frame.commanded.manipulators[0];
        let tool_motion = commanded.tool_motion.as_ref().unwrap();
        assert_eq!(tool_motion.status, "active");
        assert!(tool_motion.progress > 0.0);
        assert_eq!(
            tool_motion.target_position_world_m,
            CALIBRATION_TARGET_WORLD_M
        );
    }

    #[test]
    fn calibration_cycle_approaches_touches_and_retreats() {
        let mut runtime = PointMotionRuntime::new().unwrap();
        let report = runtime.run_calibration_cycle(20_000).unwrap();
        assert_eq!(report.sequence, ["approach", "touch", "retreat"]);
        assert_eq!(report.legs.len(), 3);
        assert!(report.legs.iter().all(|leg| leg.status == "complete"));
        assert!(report.maximum_final_position_error_m < 1.0e-9);
        assert_eq!(
            report.legs[1].target_position_world_m,
            Some(CALIBRATION_TARGET_WORLD_M)
        );
        assert_eq!(
            report.legs[2].target_position_world_m,
            Some(CALIBRATION_APPROACH_WORLD_M)
        );
    }

    #[test]
    fn rejected_target_preserves_the_previous_replay_trace() {
        let mut runtime = PointMotionRuntime::new().unwrap();
        runtime.submit_calibration_target().unwrap();
        runtime.run_until_settled(20_000).unwrap();
        let trace = runtime.mechanics.tool_motion_trace.clone();

        let result = runtime.submit_tool_target(ManipulatorId(1), Vec3::new(0.090, 0.0, 0.0));

        assert!(result.is_err());
        assert_eq!(runtime.mechanics.tool_motion_trace, trace);
    }
}
