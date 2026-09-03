use pipe_sim_core::{
    ArmId, CarriageConfig, GripperConfig, ManipulatorMotionConfig, PipeCellConfig,
    QualificationTargets, RailTopology, SafetyConfig, SerialArm, SerialArmConfig,
    SerialArmInstance, SerialJointPositions, Simulation, SimulationConfig, TendonJointConfig,
    TubeGeometry, Vec3, MACHINE_CONFIG_SCHEMA_VERSION, TENDON_JOINT_COUNT,
};
use serde::Deserialize;

use crate::{sha256_hex, SimError};

const BASELINE_MACHINE_CONFIG_JSON: &str =
    include_str!("../../../scenarios/machine_baseline_v1.json");

#[derive(Clone, Debug)]
pub(crate) struct LoadedMachineConfig {
    pub id: String,
    pub source_sha256: String,
    pub cell: PipeCellConfig,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MachineConfigDocument {
    schema_version: u32,
    id: String,
    status: String,
    manipulator_count: u8,
    tube: TubeDocument,
    carriage: CarriageDocument,
    arm: ArmDocument,
    gripper: GripperDocument,
    safety: SafetyDocument,
    qualification_targets: QualificationDocument,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct TubeDocument {
    inner_radius_m: f64,
    working_length_m: f64,
    central_work_radius_m: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CarriageDocument {
    topology: String,
    shoulder_datum_radius_m: f64,
    z_limits_m: [f64; 2],
    max_z_speed_m_s: f64,
    max_theta_speed_rad_s: f64,
    max_z_accel_m_s2: f64,
    max_theta_accel_rad_s2: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ArmDocument {
    link_lengths_m: [f64; 3],
    link_collision_radii_m: [f64; 3],
    joint_limits_rad: [[f64; 2]; TENDON_JOINT_COUNT],
    max_joint_speed_rad_s: [f64; TENDON_JOINT_COUNT],
    max_joint_accel_rad_s2: [f64; TENDON_JOINT_COUNT],
    tendon: TendonDocument,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct TendonDocument {
    routing_radius_m: f64,
    stiffness_n_m: f64,
    pretension_n: f64,
    differential_backlash_m: f64,
    max_tension_n: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct GripperDocument {
    min_opening_m: f64,
    max_opening_m: f64,
    max_speed_m_s: f64,
    jaw_half_extents_m: [f64; 3],
    pad_compliance_m: f64,
    max_grip_force_n: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SafetyDocument {
    minimum_unplanned_clearance_m: f64,
    watchdog_timeout_s: f64,
    commissioning_speed_scale: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct QualificationDocument {
    minimum_payload_kg: f64,
    minimum_tool_force_n: f64,
    force_hold_duration_s: f64,
    maximum_observed_position_sigma_m: f64,
    maximum_closed_loop_tcp_error_m: f64,
    smallest_declared_feature_m: f64,
}

pub(crate) fn load_baseline_machine_config() -> Result<LoadedMachineConfig, SimError> {
    let document: MachineConfigDocument = serde_json::from_str(BASELINE_MACHINE_CONFIG_JSON)
        .map_err(|error| SimError::InvalidScenario(format!("machine config: {error}")))?;
    if document.schema_version != MACHINE_CONFIG_SCHEMA_VERSION {
        return Err(SimError::InvalidScenario(format!(
            "machine config schema {} does not match runtime schema {}",
            document.schema_version, MACHINE_CONFIG_SCHEMA_VERSION
        )));
    }
    if document.status != "simulation_baseline_not_hardware_qualified" {
        return Err(SimError::InvalidScenario(
            "machine config must preserve its unqualified status".to_owned(),
        ));
    }
    let topology = match document.carriage.topology.as_str() {
        "paired_belt_end_bogies" => RailTopology::PairedBeltEndBogies,
        value => {
            return Err(SimError::InvalidScenario(format!(
                "unsupported rail topology '{value}'"
            )))
        }
    };
    let tendon = TendonJointConfig {
        routing_radius_m: document.arm.tendon.routing_radius_m,
        tendon_stiffness_n_m: document.arm.tendon.stiffness_n_m,
        pretension_n: document.arm.tendon.pretension_n,
        differential_backlash_m: document.arm.tendon.differential_backlash_m,
        max_tension_n: document.arm.tendon.max_tension_n,
    };
    let arm = SerialArmConfig {
        rail_radius_m: document.carriage.shoulder_datum_radius_m,
        base_z_limits_m: document.carriage.z_limits_m,
        upper_arm_length_m: document.arm.link_lengths_m[0],
        forearm_length_m: document.arm.link_lengths_m[1],
        wrist_length_m: document.arm.link_lengths_m[2],
        link_collision_radii_m: document.arm.link_collision_radii_m,
        joint_limits_rad: document.arm.joint_limits_rad,
        tendon_joints: [tendon; TENDON_JOINT_COUNT],
    };
    let cell = PipeCellConfig {
        schema_version: document.schema_version,
        manipulator_count: document.manipulator_count,
        tube: TubeGeometry {
            inner_radius_m: document.tube.inner_radius_m,
            working_length_m: document.tube.working_length_m,
            central_work_radius_m: document.tube.central_work_radius_m,
        },
        carriage: CarriageConfig {
            topology,
            rail_radius_m: document.carriage.shoulder_datum_radius_m,
            z_limits_m: document.carriage.z_limits_m,
            max_z_speed_m_s: document.carriage.max_z_speed_m_s,
            max_theta_speed_rad_s: document.carriage.max_theta_speed_rad_s,
            max_z_accel_m_s2: document.carriage.max_z_accel_m_s2,
            max_theta_accel_rad_s2: document.carriage.max_theta_accel_rad_s2,
        },
        arm,
        motion: ManipulatorMotionConfig {
            max_joint_speed_rad_s: document.arm.max_joint_speed_rad_s,
            max_joint_accel_rad_s2: document.arm.max_joint_accel_rad_s2,
        },
        gripper: GripperConfig {
            min_opening_m: document.gripper.min_opening_m,
            max_opening_m: document.gripper.max_opening_m,
            max_speed_m_s: document.gripper.max_speed_m_s,
            jaw_half_extents_m: Vec3::new(
                document.gripper.jaw_half_extents_m[0],
                document.gripper.jaw_half_extents_m[1],
                document.gripper.jaw_half_extents_m[2],
            ),
            pad_compliance_m: document.gripper.pad_compliance_m,
            max_grip_force_n: document.gripper.max_grip_force_n,
        },
        safety: SafetyConfig {
            minimum_unplanned_clearance_m: document.safety.minimum_unplanned_clearance_m,
            watchdog_timeout_s: document.safety.watchdog_timeout_s,
            commissioning_speed_scale: document.safety.commissioning_speed_scale,
        },
        qualification: QualificationTargets {
            minimum_payload_kg: document.qualification_targets.minimum_payload_kg,
            minimum_tool_force_n: document.qualification_targets.minimum_tool_force_n,
            force_hold_duration_s: document.qualification_targets.force_hold_duration_s,
            maximum_observed_position_sigma_m: document
                .qualification_targets
                .maximum_observed_position_sigma_m,
            maximum_closed_loop_tcp_error_m: document
                .qualification_targets
                .maximum_closed_loop_tcp_error_m,
            smallest_declared_feature_m: document.qualification_targets.smallest_declared_feature_m,
        },
    };
    if !cell.is_valid() {
        return Err(SimError::InvalidScenario(
            "machine config failed physical consistency checks".to_owned(),
        ));
    }
    Ok(LoadedMachineConfig {
        id: document.id,
        source_sha256: sha256_hex(BASELINE_MACHINE_CONFIG_JSON.as_bytes()),
        cell,
    })
}

/// Construct the shared deterministic M1 machine plant. Manipulator 1 starts
/// at the commissioning datum; the remaining manipulators are parked at
/// separated axial datums so standalone single-arm milestones cannot obtain
/// clearance by silently removing them from the machine.
pub(crate) fn build_baseline_machine(
    loaded: &LoadedMachineConfig,
) -> Result<Simulation, SimError> {
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
    Ok(mechanics)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_machine_contract_is_valid_and_hashed() {
        let loaded = load_baseline_machine_config().unwrap();
        assert_eq!(loaded.id, "pipe_machine_baseline_v1");
        assert_eq!(loaded.source_sha256.len(), 64);
        assert!(loaded.cell.is_valid());
        assert_eq!(loaded.cell.arm.upper_arm_length_m, 32.0e-3);
        assert_eq!(loaded.cell.carriage.rail_radius_m, 72.0e-3);
    }
}
