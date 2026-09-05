use super::controller::{
    bounded_axis_correction, guard_target_rail_sweep, AxialEnvelopeSweep, TargetRailDatum,
};

#[test]
fn quantized_diagonal_correction_reduces_error_without_subfloor_commands() {
    use super::controller::{decide_quantized_correction, CorrectionDecision, CorrectionPolicy};
    let policy = CorrectionPolicy {
        gain: 0.85,
        convergence_m: 9.0e-6,
        maximum_magnitude_m: 400.0e-6,
        minimum_reproducible_m: 5.0e-6,
    };
    let error = [5.4e-6; 3];
    let CorrectionDecision::Command {
        correction_world_m, ..
    } = decide_quantized_correction([0.0; 3], error, policy)
    else {
        panic!("a legal minimum step must be available");
    };
    assert_eq!(correction_world_m, [5.0e-6, 0.0, 0.0]);
    let before: f64 = error.iter().map(|x| x * x).sum();
    let after: f64 = error
        .into_iter()
        .zip(correction_world_m)
        .map(|(a, b)| (a - b) * (a - b))
        .sum();
    assert!(after < before);
}
use super::{
    M1eFault, ObservedManipulationRuntime, ObservedManipulationScenario, BASELINE_M1F_SCENARIO_JSON,
};

#[test]
fn fixed_head_cycle_corrects_tilt_and_replays_without_truth_scoring() {
    let run = || {
        ObservedManipulationRuntime::from_scenario_json(BASELINE_M1F_SCENARIO_JSON, M1eFault::None)
            .unwrap()
            .run_cycle()
            .unwrap()
    };
    let first = run();
    assert_eq!(first.status, "complete", "{:?}", first.terminal_reason);
    assert!(first
        .acceptance_gates
        .iter()
        .all(|g| g.applicable && g.passed));
    let scoring = first.evaluation_only_truth.as_ref().unwrap();
    assert!(scoring.within_declared_seat_tolerances && scoring.physical_release_verified);
    assert_eq!(scoring.maximum_unplanned_penetration_m, 0.0);
    assert!(first.decisions.iter().any(|d| d.action == "axis_correction"
        && d.phase == super::controller::ControlPhase::SocketCorrection));
    assert!(first
        .decisions
        .iter()
        .any(|d| d.target_axis_world.is_some() && d.command_sequence.is_some()));
    assert_eq!(first.schema_version, 2);
    assert!(!first.roll_observable);
    let replay = run();
    assert_eq!(first, replay);
}

#[test]
fn fixed_head_configuration_is_strict_and_versioned() {
    let mut value: serde_json::Value = serde_json::from_str(BASELINE_M1F_SCENARIO_JSON).unwrap();
    value["schema_version"] = 1.into();
    assert!(ObservedManipulationScenario::from_json(&value.to_string()).is_err());
    value["schema_version"] = 2.into();
    value["fixed_head"]["view_direction_world"] = serde_json::json!([0.0, 0.0, 0.0]);
    assert!(ObservedManipulationScenario::from_json(&value.to_string()).is_err());
    value = serde_json::from_str(BASELINE_M1F_SCENARIO_JSON).unwrap();
    value["fixed_head"]["axis_convergence_rad"] = 0.2.into();
    assert!(ObservedManipulationScenario::from_json(&value.to_string()).is_err());
}

#[test]
fn observed_axis_controller_uses_bounded_minimum_rotation_and_refuses_capture_loss() {
    let moving = [0.0, 0.0, 1.0];
    let target = [0.06_f64.sin(), 0.0, 0.06_f64.cos()];
    let result = bounded_axis_correction(moving, target, moving, 0.009, 0.020, 0.10)
        .unwrap()
        .unwrap();
    assert!((result[0] - 0.020_f64.sin()).abs() < 1.0e-12);
    assert_eq!(result[1], 0.0);
    assert!(bounded_axis_correction(moving, [0.0, 1.0, 0.0], moving, 0.009, 0.02, 0.10).is_err());
    assert_eq!(
        bounded_axis_correction(moving, moving, moving, 0.009, 0.02, 0.10).unwrap(),
        None
    );
    assert!(bounded_axis_correction([0.0; 3], target, moving, 0.009, 0.02, 0.10).is_err());
}

#[test]
fn target_rail_sweep_covers_roll_uncertainty_and_between_sample_motion() {
    let rail = TargetRailDatum {
        center_world_m: [0.0; 3],
        axis_world: [0.0, 0.0, 1.0],
        position_bound_m: 3.0e-6,
        axis_bound_rad: 0.01,
        lateral_offset_m: 0.001,
        radius_m: 0.0001,
        half_length_m: 0.00035,
    };
    let mut sweep = AxialEnvelopeSweep {
        center_world_m: [0.0, 0.0, -0.0008],
        axis_world: [0.0, 0.0, 1.0],
        translation_world_m: [0.0, 0.0, 0.0002],
        center_axial_offset_m: 0.0,
        half_length_m: 0.0007,
        radius_m: 0.0002,
        position_bound_m: 0.00003,
        axis_bound_rad: 0.03,
        path_deviation_bound_m: 0.00001,
    };
    assert!(guard_target_rail_sweep(sweep, rail, 0.0001).is_ok());
    // Unknown roll cannot turn a diagonal rail collision into clearance.
    sweep.center_world_m = [0.0007, 0.0007, 0.0];
    assert!(guard_target_rail_sweep(sweep, rail, 0.0001).is_err());
    sweep.center_world_m = [-0.002, 0.0, 0.0];
    sweep.translation_world_m = [0.004, 0.0, 0.0];
    assert!(guard_target_rail_sweep(sweep, rail, 0.0001).is_err());
    sweep.center_world_m = [0.0, 0.0, -0.0008];
    sweep.translation_world_m = [0.0; 3];
    sweep.path_deviation_bound_m = 0.001;
    assert!(guard_target_rail_sweep(sweep, rail, 0.0001).is_err());
}
