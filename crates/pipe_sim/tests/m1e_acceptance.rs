use pipe_sim::observed_manipulation::{
    M1eFault, ObservedManipulationReport, ObservedManipulationRuntime, ObservedManipulationScenario,
};

fn run_report(fault: M1eFault) -> ObservedManipulationReport {
    let mut runtime = ObservedManipulationRuntime::new(fault).unwrap();
    runtime.run_cycle().unwrap()
}

fn serialized_phase<T: serde::Serialize>(phase: &T) -> String {
    serde_json::to_value(phase)
        .unwrap()
        .as_str()
        .expect("control phases serialize as strings")
        .to_owned()
}

#[test]
fn nominal_observed_state_cycle_passes_every_controller_gate() {
    let (scenario, _) = ObservedManipulationScenario::baseline().unwrap();
    let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
    let report = runtime.run_cycle().unwrap();

    assert_eq!(report.status, "complete", "{report:#?}");
    assert!(report.expected_outcome_observed);
    assert!(report.acceptance_gates.iter().all(|gate| gate.passed));
    assert_eq!(report.truth_firewall.controller_truth_access_count, 0);
    assert_eq!(
        report.fidelity,
        "F1_reduced_M1e_observed_feature_geometry_not_hardware_qualified"
    );
    assert_eq!(report.timing.stale_near_contact_command_count, 0);
    assert!(report.metrics.guarded_grasp_confirmed);
    assert!(report.metrics.transfer_preflight_passed);
    assert!(report.metrics.seated_from_observation_and_contact);
    assert!(report.metrics.release_confirmed);
    assert!(report.metrics.retreat_confirmed);
    let evaluation = report
        .evaluation_only_truth
        .as_ref()
        .expect("terminal report contains separated post-run scoring");
    assert!(evaluation.within_declared_seat_tolerances);
    assert_eq!(evaluation.maximum_unplanned_penetration_m, 0.0);
    assert!(!evaluation.physical_grasp_attachment_present);
    assert!(evaluation.physical_release_verified);
    assert_eq!(
        evaluation.peak_grip_force_proxy_n,
        report.metrics.maximum_grip_force_proxy_n
    );
    assert_eq!(
        evaluation.peak_insertion_force_proxy_n,
        report.metrics.maximum_insertion_force_proxy_n
    );
    assert!(evaluation.peak_grip_force_proxy_n <= scenario.grasp.maximum_grip_force_n);
    assert!(evaluation.peak_insertion_force_proxy_n <= scenario.contact.maximum_force_proxy_n);
    assert!(evaluation
        .final_tool_center_to_socket_center_distance_m
        .is_finite());
    assert_eq!(report.machine_config_sha256.len(), 64);
}

#[test]
fn nominal_report_is_bit_for_bit_replayable() {
    let mut first = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
    let mut second = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
    let first = first.run_cycle().unwrap();
    let second = second.run_cycle().unwrap();

    assert_eq!(first, second);
    assert_eq!(
        first.to_json(false).unwrap(),
        second.to_json(false).unwrap()
    );
    assert_eq!(
        first.controller_report_sha256,
        second.controller_report_sha256
    );
}

#[test]
fn every_versioned_fault_reaches_its_exact_fail_closed_reason() {
    let (scenario, _) = ObservedManipulationScenario::baseline().unwrap();
    for fault_name in &M1eFault::available()[1..] {
        let fault = fault_name.parse::<M1eFault>().unwrap();
        let mut runtime = ObservedManipulationRuntime::new(fault).unwrap();
        let report = runtime.run_cycle().unwrap();

        assert_eq!(report.status, "failed_safe", "{fault_name}: {report:#?}");
        assert_eq!(
            report.terminal_reason.as_deref(),
            scenario.expected_failure_reason(fault),
            "{fault_name}: {report:#?}"
        );
        assert!(
            report.expected_outcome_observed,
            "{fault_name}: {report:#?}"
        );
    }
}

#[test]
fn stale_measurement_never_drives_a_near_contact_command() {
    let mut runtime = ObservedManipulationRuntime::new(M1eFault::StaleObservation).unwrap();
    let report = runtime.run_cycle().unwrap();

    assert_eq!(report.terminal_reason.as_deref(), Some("stale_measurement"));
    assert_eq!(report.timing.stale_near_contact_command_count, 0);
    assert!(report.decisions.iter().all(|decision| {
        !(decision.near_contact
            && decision.command_sequence.is_some()
            && decision.action == "set_tool_position")
    }));
}

#[test]
fn carried_collision_is_rejected_before_transfer_command_submission() {
    let mut runtime = ObservedManipulationRuntime::new(M1eFault::CarriedPartCollision).unwrap();
    let report = runtime.run_cycle().unwrap();

    assert_eq!(
        report.terminal_reason.as_deref(),
        Some("carried_part_collision_risk")
    );
    let rejection = report
        .decisions
        .iter()
        .find(|decision| {
            matches!(
                decision.action,
                "reject_transfer" | "reject_estimated_sweep"
            )
        })
        .expect("estimated-scene transfer rejection must be reported");
    assert!(rejection.command_sequence.is_none());
}

#[test]
fn controller_visible_report_sections_contain_no_raw_truth_fields() {
    let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
    let report = runtime.run_cycle().unwrap();
    let value = serde_json::to_value(report).unwrap();

    for section in ["observation_bursts", "correction_iterations", "decisions"] {
        let serialized = serde_json::to_string(&value[section]).unwrap();
        assert!(
            !serialized.contains("true_"),
            "truth field leaked into {section}"
        );
        assert!(!serialized.contains("range_error"));
    }
}

#[test]
fn truth_scoring_is_absent_before_terminal_control() {
    let runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
    let report = runtime.report();

    assert!(!runtime.is_terminal());
    assert!(report.evaluation_only_truth.is_none());
}

#[test]
fn every_named_fault_report_has_finite_decision_estimate_numbers() {
    for fault_name in &M1eFault::available()[1..] {
        let fault = fault_name.parse::<M1eFault>().unwrap();
        let report = run_report(fault);
        let json = report
            .to_json(false)
            .unwrap_or_else(|error| panic!("{fault_name}: report JSON failed: {error}"));
        let value: serde_json::Value = serde_json::from_str(&json)
            .unwrap_or_else(|error| panic!("{fault_name}: invalid report JSON: {error}"));
        let json_decisions = value["decisions"]
            .as_array()
            .expect("decisions serialize as an array");

        assert_eq!(json_decisions.len(), report.decisions.len(), "{fault_name}");
        for (decision_index, (decision, json_decision)) in
            report.decisions.iter().zip(json_decisions).enumerate()
        {
            let json_estimates = json_decision["relevant_estimates"]
                .as_array()
                .expect("relevant estimates serialize as an array");
            assert_eq!(
                json_estimates.len(),
                decision.relevant_estimates.len(),
                "{fault_name}: decision {decision_index}"
            );

            for (estimate_index, (estimate, json_estimate)) in decision
                .relevant_estimates
                .iter()
                .zip(json_estimates)
                .enumerate()
            {
                for (field, number) in [
                    ("position_sigma_m", estimate.position_sigma_m),
                    ("axis_sigma_rad", estimate.axis_sigma_rad),
                    ("residual_rms_m", estimate.residual_rms_m),
                    (
                        "provenance.unobservable_roll_position_bound_m",
                        estimate.provenance.unobservable_roll_position_bound_m,
                    ),
                    (
                        "provenance.unobservable_roll_axis_bound_rad",
                        estimate.provenance.unobservable_roll_axis_bound_rad,
                    ),
                ] {
                    assert!(
                        number.is_finite(),
                        "{fault_name}: decision {decision_index} estimate {estimate_index} has non-finite {field}={number:?}"
                    );
                }
                for (field, vector) in [
                    ("position_world_m", estimate.position_world_m),
                    ("axis_world", estimate.axis_world),
                ] {
                    assert!(
                        vector.iter().all(|number| number.is_finite()),
                        "{fault_name}: decision {decision_index} estimate {estimate_index} has non-finite {field}={vector:?}"
                    );
                }

                for field in [
                    "object_id",
                    "position_sigma_m",
                    "axis_sigma_rad",
                    "capture_tick",
                    "available_tick",
                    "distinct_feature_count",
                    "triangulation_head_count",
                    "minimum_calibrated_rays_per_point",
                    "residual_rms_m",
                ] {
                    assert!(
                        json_estimate[field].is_number(),
                        "{fault_name}: decision {decision_index} estimate {estimate_index} field {field} became {:?}",
                        json_estimate[field]
                    );
                }
                for field in ["position_world_m", "axis_world"] {
                    assert!(
                        json_estimate[field]
                            .as_array()
                            .is_some_and(|values| values.iter().all(serde_json::Value::is_number)),
                        "{fault_name}: decision {decision_index} estimate {estimate_index} field {field} contains a non-number"
                    );
                }
                for field in [
                    "supporting_tool_distinct_feature_count",
                    "supporting_tool_triangulation_head_count",
                    "supporting_tool_minimum_calibrated_rays_per_point",
                    "unobservable_roll_position_bound_m",
                    "unobservable_roll_axis_bound_rad",
                ] {
                    assert!(
                        json_estimate["provenance"][field].is_number(),
                        "{fault_name}: decision {decision_index} estimate {estimate_index} provenance field {field} became {:?}",
                        json_estimate["provenance"][field]
                    );
                }
            }
        }
    }
}

#[test]
fn injected_failures_do_not_claim_unreached_lifecycle_transitions() {
    let pre_grasp_faults = [
        M1eFault::OpticalDropout,
        M1eFault::ExcessiveCalibrationBias,
        M1eFault::StaleObservation,
        M1eFault::CorrectionFloorTooLarge,
        M1eFault::GraspOutsideCapture,
        M1eFault::InconsistentObservation,
        M1eFault::NonConvergence,
    ];

    for fault_name in &M1eFault::available()[1..] {
        let fault = fault_name.parse::<M1eFault>().unwrap();
        let report = run_report(fault);
        let evaluation = report
            .evaluation_only_truth
            .as_ref()
            .expect("failed-safe terminal reports include evaluation-only scoring");

        assert!(!report.metrics.release_confirmed, "{fault_name}");
        assert!(!report.metrics.retreat_confirmed, "{fault_name}");
        assert!(!evaluation.physical_release_verified, "{fault_name}");
        assert!(
            !report.decisions.iter().any(|decision| matches!(
                decision.action,
                "release_committed" | "release_confirmed" | "retreat_verified"
            )),
            "{fault_name}: a failed run reported a release/retreat transition"
        );

        if pre_grasp_faults.contains(&fault) {
            assert!(!report.metrics.guarded_grasp_confirmed, "{fault_name}");
            assert!(
                !evaluation.physical_grasp_attachment_present,
                "{fault_name}"
            );
            assert!(
                !report
                    .decisions
                    .iter()
                    .any(|decision| decision.action == "grasp_committed"),
                "{fault_name}: an early failure reported a committed grasp"
            );
        }
    }
}

#[test]
fn nominal_phase_uncertainty_convergence_and_insertion_gates_have_evidence() {
    let (scenario, _) = ObservedManipulationScenario::baseline().unwrap();
    let report = run_report(M1eFault::None);

    let uncertainty_gate = report
        .acceptance_gates
        .iter()
        .find(|gate| gate.id == "estimator_phase_uncertainty")
        .expect("phase-uncertainty gate");
    assert!(uncertainty_gate.applicable && uncertainty_gate.passed);
    assert!(uncertainty_gate
        .evidence
        .contains("all_per_phase_guards=true"));
    assert!(uncertainty_gate.evidence.contains("guard_count="));

    for required_phase in [
        "pick_correction",
        "guarded_grasp",
        "transfer",
        "socket_correction",
        "guarded_insertion",
        "seat_verification",
        "release",
        "retreat",
    ] {
        let phase_guards = report
            .uncertainty_guards
            .iter()
            .filter(|guard| serialized_phase(&guard.phase) == required_phase)
            .collect::<Vec<_>>();
        assert!(
            !phase_guards.is_empty(),
            "missing uncertainty evidence for {required_phase}"
        );
        for guard in phase_guards {
            assert!(guard.passed, "{required_phase}: {guard:#?}");
            let position_sigma_m = guard
                .position_sigma_m
                .expect("a passing uncertainty guard has position uncertainty");
            let axis_sigma_rad = guard
                .axis_sigma_rad
                .expect("a passing uncertainty guard has axis uncertainty");
            assert!(position_sigma_m.is_finite() && axis_sigma_rad.is_finite());
            assert!(
                position_sigma_m <= guard.position_limit_m,
                "{required_phase}: {guard:#?}"
            );
            assert!(
                axis_sigma_rad <= guard.axis_limit_rad,
                "{required_phase}: {guard:#?}"
            );
        }
    }

    let convergence_gate = report
        .acceptance_gates
        .iter()
        .find(|gate| gate.id == "correction_convergence")
        .expect("correction-convergence gate");
    assert!(convergence_gate.applicable && convergence_gate.passed);
    assert!(convergence_gate.evidence.contains("pick=true"));
    assert!(convergence_gate.evidence.contains("socket=true"));
    for required_phase in ["pick_correction", "socket_correction"] {
        let converged = report
            .correction_iterations
            .iter()
            .filter(|record| {
                serialized_phase(&record.phase) == required_phase && record.outcome == "converged"
            })
            .collect::<Vec<_>>();
        assert_eq!(
            converged.len(),
            1,
            "expected one convergence record for {required_phase}"
        );
        assert!(converged[0].iteration <= scenario.motion.maximum_correction_iterations);
    }

    let insertion_gate = report
        .acceptance_gates
        .iter()
        .find(|gate| gate.id == "guarded_insertion")
        .expect("guarded-insertion gate");
    assert!(insertion_gate.applicable && insertion_gate.passed);
    assert!(insertion_gate.evidence.contains("guarded_transition=true"));
    assert!(insertion_gate.evidence.contains("interlock_trips=0"));
    assert!(report.metrics.insertion_increment_count > 0);
    assert!(report.metrics.guarded_insertion_confirmed);
    assert_eq!(report.metrics.force_interlock_trip_count, 0);
    let seat_candidate = report
        .decisions
        .iter()
        .find(|decision| decision.action == "insertion_seat_candidate")
        .expect("guarded insertion has an explicit seated transition");
    assert!(seat_candidate.near_contact);
    let contact = seat_candidate
        .contact_evidence
        .as_ref()
        .expect("seated transition includes contact evidence");
    assert_eq!(serde_json::to_value(contact).unwrap()["state"], "seated");
}
