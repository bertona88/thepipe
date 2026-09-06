//! Stationary two-arm handoff coupon. Metrology is injected at the measured
//! feature-packet boundary; this does not claim fixed-head optical feasibility.
pub mod controller;
mod plant;

use crate::scene::SceneFrame;
use controller::{Action, Controller, HandoffInput, Phase, Policy};
use serde::{Deserialize, Serialize};

pub const SCENARIO_JSON: &str =
    include_str!("../../../../scenarios/stationary_handoff_m1g_v1.json");

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    pub schema_version: u32,
    pub id: String,
    pub machine_config_id: String,
    pub seed: u64,
    pub peg_radius_m: f64,
    pub peg_half_segment_m: f64,
    pub tool_to_peg_center_m: f64,
    pub minimum_axial_overlap_m: f64,
    pub closed_opening_m: f64,
    pub measurement_sigma_m: f64,
    pub measurement_latency_ticks: u64,
    pub maximum_observation_age_ticks: u64,
    pub maximum_capture_error_m: f64,
    pub maximum_position_sigma_m: f64,
    pub maximum_axis_sigma_rad: f64,
    pub maximum_axis_error_rad: f64,
    pub minimum_force_n: f64,
    pub maximum_force_n: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Fault {
    #[default]
    None,
    ObservationLostBeforeClose,
    ObservationLostBeforeTransfer,
    ObservationLostAfterTransfer,
    StaleObservation,
    ReceiverContactMissing,
    TransferRejected,
    InconsistentPose,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Decision {
    pub phase: Phase,
    pub input: Option<HandoffInput>,
    pub plant_failure: Option<String>,
    pub action: Action,
    pub acknowledged: bool,
    pub owner_after_ack: Option<u32>,
    pub ack_tick: u64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Report {
    pub schema_version: u32,
    pub scenario_id: String,
    pub scenario_sha256: String,
    pub machine_config_sha256: String,
    pub fault: Fault,
    pub status: Phase,
    pub terminal_reason: Option<String>,
    pub owner_from_acknowledgements: Option<u32>,
    pub decisions: Vec<Decision>,
    pub controller_trace_sha256: String,
    pub fidelity: &'static str,
    pub initialization: &'static str,
    pub metrology_boundary: &'static str,
    pub physics_boundary: &'static str,
    pub evaluation_only_frames: Vec<SceneFrame>,
    pub evaluation_only_final_owner: Option<u32>,
    pub evaluation_only_maximum_interarm_penetration_m: f64,
    pub evaluation_only_transfer_pose_jump_m: f64,
}

pub fn run_stationary_coupon(fault: Fault) -> Result<Report, String> {
    let scenario: Scenario = serde_json::from_str(SCENARIO_JSON).map_err(|e| e.to_string())?;
    let policy = Policy {
        maximum_age_ticks: scenario.maximum_observation_age_ticks,
        maximum_position_sigma_m: scenario.maximum_position_sigma_m,
        maximum_axis_sigma_rad: scenario.maximum_axis_sigma_rad,
        maximum_capture_error_m: scenario.maximum_capture_error_m,
        maximum_axis_error_rad: scenario.maximum_axis_error_rad,
        tool_to_peg_center_m: scenario.tool_to_peg_center_m,
        minimum_force_n: scenario.minimum_force_n,
        maximum_force_n: scenario.maximum_force_n,
    };
    let mut controller = Controller::new(policy)?;
    let mut plant = plant::Plant::new(&scenario)?;
    let mut decisions = Vec::new();
    for sequence in 0..8 {
        let phase = controller.phase;
        let mut input = match plant.observe(sequence) {
            Ok(input) => input,
            Err(error) => {
                controller.stop("plant_observation_failed");
                plant.stop()?;
                decisions.push(Decision {
                    phase,
                    input: None,
                    plant_failure: Some(error),
                    action: Action::StopBoth,
                    acknowledged: false,
                    owner_after_ack: controller.owner,
                    ack_tick: plant.tick(),
                });
                plant.record();
                break;
            }
        };
        match fault {
            Fault::ObservationLostBeforeClose if phase == Phase::CloseReceiver => {
                input.estimates.clear()
            }
            Fault::ObservationLostBeforeTransfer if phase == Phase::Transfer => {
                input.estimates.clear()
            }
            Fault::ObservationLostAfterTransfer if phase == Phase::OpenDonor => {
                input.estimates.clear()
            }
            Fault::ReceiverContactMissing if phase == Phase::Transfer => {
                input.receiver_contact.right_contact = false
            }
            Fault::StaleObservation if phase == Phase::Transfer => {
                for estimate in &mut input.estimates {
                    estimate.oldest_capture_tick = Some(0);
                }
            }
            Fault::InconsistentPose if phase == Phase::Transfer => {
                if let Some(pose) = input.estimates[1].pose.as_mut() {
                    pose.center_world_m[1] += 0.001;
                }
            }
            _ => {}
        }
        let action = controller.authorize(&input);
        let plant_failure = if action == Action::StopBoth {
            None
        } else {
            plant.apply(action, fault == Fault::TransferRejected).err()
        };
        let accepted = action != Action::StopBoth && plant_failure.is_none();
        if action != Action::StopBoth {
            controller.acknowledge(action, plant.tick(), accepted);
        }
        if controller.phase == Phase::Stopped {
            plant.stop()?;
        }
        decisions.push(Decision {
            phase,
            input: Some(input),
            plant_failure,
            action,
            acknowledged: accepted,
            owner_after_ack: controller.owner,
            ack_tick: plant.tick(),
        });
        plant.record();
        if matches!(controller.phase, Phase::Complete | Phase::Stopped) {
            break;
        }
    }
    if !matches!(controller.phase, Phase::Complete | Phase::Stopped) {
        controller.stop("protocol_budget_exhausted");
        plant.stop()?;
    }
    let trace = serde_json::to_vec(&(
        &decisions,
        controller.phase,
        &controller.terminal_reason,
        controller.owner,
    ))
    .map_err(|e| e.to_string())?;
    Ok(Report {
        schema_version: 1, scenario_id: scenario.id,
        scenario_sha256: crate::sha256_hex(SCENARIO_JSON.as_bytes()),
        machine_config_sha256: plant.machine_hash.clone(), fault,
        status: controller.phase, terminal_reason: controller.terminal_reason,
        owner_from_acknowledgements: controller.owner, decisions,
        controller_trace_sha256: crate::sha256_hex(&trace),
        fidelity: "F0_stationary_handoff_protocol_coupon_not_M1f_optical_or_hardware_qualification",
        initialization: "two_arms_prepositioned_by_IK; donor_jaws_preclosed; donor_acquisition_requires_observed_authorization; no_approach_or_retreat_trajectory",
        metrology_boundary: "deterministic_noisy_labelled_feature_packet_injection_plus_5DoF_WLS; no_camera_projection_visibility_or_image_detection; not_an_optical_feasibility_result",
        physics_boundary: "authoritative_tendon_scheduler_and_jaw_motion; stationary_atomic_kinematic_ownership_transfer; no_dual_grasp_load_sharing_gravity_friction_slip_or_breakaway",
        evaluation_only_final_owner: plant.owner(),
        evaluation_only_maximum_interarm_penetration_m: plant.maximum_penetration_m,
        evaluation_only_transfer_pose_jump_m: plant.transfer_jump_m,
        evaluation_only_frames: plant.frames,
    })
}
