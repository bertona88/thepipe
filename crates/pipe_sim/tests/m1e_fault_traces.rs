use pipe_sim::observed_manipulation::{
    M1eFault, ObservedManipulationReport, ObservedManipulationRuntime, ObservedManipulationScenario,
};

const BASELINE_SCENARIO_JSON: &str =
    include_str!("../../../scenarios/observed_manipulation_m1e_v1.json");

fn scenario_with_force_limit(section: &str, field: &str, limit_n: f64) -> String {
    let mut scenario: serde_json::Value =
        serde_json::from_str(BASELINE_SCENARIO_JSON).expect("embedded M1e scenario JSON");
    scenario[section][field] = serde_json::json!(limit_n);
    serde_json::to_string(&scenario).expect("modified M1e scenario JSON")
}

fn run_custom_fault_twice(json: &str, fault: M1eFault) -> ObservedManipulationReport {
    let mut first =
        ObservedManipulationRuntime::from_scenario_json(json, fault).expect("first custom runtime");
    let first = first.run_cycle().expect("first custom report");
    let mut replay = ObservedManipulationRuntime::from_scenario_json(json, fault)
        .expect("replay custom runtime");
    let replay = replay.run_cycle().expect("replay custom report");

    assert_eq!(first, replay, "custom force-interlock replay changed");
    assert_eq!(
        first.to_json(false).unwrap(),
        replay.to_json(false).unwrap(),
        "custom force-interlock JSON changed"
    );
    assert_eq!(
        first.controller_report_sha256, replay.controller_report_sha256,
        "custom force-interlock controller hash changed"
    );
    first
}

fn run_custom_scenario_twice(json: &str) -> ObservedManipulationReport {
    run_custom_fault_twice(json, M1eFault::None)
}

fn run_expected_failure(
    fault: M1eFault,
    expected_reason: &'static str,
) -> ObservedManipulationReport {
    let mut runtime = ObservedManipulationRuntime::new(fault).unwrap();
    let report = runtime.run_cycle().unwrap();

    assert_eq!(report.injected_fault, fault.id(), "{fault:?}: {report:#?}");
    assert_eq!(report.status, "failed_safe", "{fault:?}: {report:#?}");
    assert_eq!(
        report.terminal_reason.as_deref(),
        Some(expected_reason),
        "{fault:?}: {report:#?}"
    );
    assert_eq!(
        report.expected_terminal_reason.as_deref(),
        Some(expected_reason),
        "{fault:?}: scenario/report reason mismatch"
    );
    assert!(
        report.expected_outcome_observed,
        "{fault:?}: declared fault did not reach its intended reason"
    );
    assert_eq!(report.truth_firewall.controller_truth_access_count, 0);
    assert_eq!(report.timing.stale_near_contact_command_count, 0);

    let first = report.decisions.first().expect("initial decision");
    assert_eq!(first.action, "initialize");
    for (expected_sequence, decision) in report.decisions.iter().enumerate() {
        assert_eq!(decision.sequence as usize, expected_sequence);
    }
    assert!(report
        .decisions
        .windows(2)
        .all(|pair| pair[0].tick <= pair[1].tick));

    let terminal_hold = report.decisions.last().expect("terminal hold decision");
    assert_eq!(terminal_hold.action, "fail_closed_hold");
    assert!(terminal_hold.command_sequence.is_some());
    assert!(terminal_hold.target_world_m.is_none());
    assert!(terminal_hold.reason.contains(expected_reason));
    assert!(terminal_hold.reason.contains("authoritative stop accepted"));

    for forbidden in [
        "seat_verified",
        "release_authorized",
        "release_committed",
        "release_confirmed",
        "retreat_verified",
        "complete",
        "fail_closed_retreat",
    ] {
        assert_no_action(&report, forbidden);
    }

    for gate_id in [
        "no_controller_truth_access",
        "no_stale_near_contact_command",
    ] {
        let gate = report
            .acceptance_gates
            .iter()
            .find(|gate| gate.id == gate_id)
            .unwrap_or_else(|| panic!("{fault:?}: missing safety gate {gate_id}"));
        assert!(
            gate.applicable,
            "{fault:?}: {gate_id} must remain applicable"
        );
        assert!(gate.passed, "{fault:?}: {}", gate.evidence);
    }

    assert!(
        report.evaluation_only_truth.is_some(),
        "terminal report must carry separately labelled scoring"
    );
    report
}

fn has_action(report: &ObservedManipulationReport, action: &str) -> bool {
    report
        .decisions
        .iter()
        .any(|decision| decision.action == action)
}

fn assert_no_action(report: &ObservedManipulationReport, action: &str) {
    assert!(
        !has_action(report, action),
        "{} unexpectedly reached action {action}",
        report.injected_fault
    );
}

fn assert_no_near_contact_tool_motion(report: &ObservedManipulationReport) {
    let forbidden = report
        .decisions
        .iter()
        .filter(|decision| {
            decision.action == "set_tool_position"
                && decision.near_contact
                && decision.command_sequence.is_some()
        })
        .map(|decision| (decision.sequence, decision.reason.as_str()))
        .collect::<Vec<_>>();
    assert!(
        forbidden.is_empty(),
        "{} issued near-contact tool motion: {forbidden:?}",
        report.injected_fault
    );
}

fn expected_attempt_count() -> usize {
    let (scenario, _) = ObservedManipulationScenario::baseline().unwrap();
    scenario.safety.maximum_phase_retries as usize + 1
}

#[test]
fn optical_dropout_retries_observation_without_driving_contact_motion() {
    let report = run_expected_failure(M1eFault::OpticalDropout, "optical_dropout");

    assert_eq!(report.observation_bursts.len(), expected_attempt_count());
    assert_eq!(
        report.metrics.recovery_count as usize,
        expected_attempt_count() - 1
    );
    assert!(report.observation_bursts.iter().all(|burst| {
        burst.observed_feature_count == 0
            && burst.missing_feature_count > 0
            && burst.accepted_object_ids.is_empty()
            && burst
                .rejection_reasons
                .iter()
                .any(|reason| reason.contains("optical_dropout"))
    }));
    assert_eq!(
        report
            .decisions
            .iter()
            .filter(|decision| decision.action == "hold_for_reacquisition")
            .count(),
        expected_attempt_count()
    );
    assert_no_near_contact_tool_motion(&report);
    assert_no_action(&report, "close_gripper");
}

#[test]
fn excessive_calibration_bias_rejects_populated_bursts_before_pose_use() {
    let report = run_expected_failure(
        M1eFault::ExcessiveCalibrationBias,
        "excessive_calibration_bias",
    );

    assert_eq!(report.observation_bursts.len(), expected_attempt_count());
    assert!(report.observation_bursts.iter().all(|burst| {
        burst.observed_feature_count > 0
            && burst.accepted_object_ids.is_empty()
            && burst
                .rejection_reasons
                .iter()
                .any(|reason| reason.contains("excessive_calibration_bias"))
    }));
    assert!(report.correction_iterations.is_empty());
    assert_no_near_contact_tool_motion(&report);
    assert_no_action(&report, "close_gripper");
}

#[test]
fn stale_observations_are_retried_but_never_drive_near_contact_motion() {
    let report = run_expected_failure(M1eFault::StaleObservation, "stale_measurement");
    let maximum_age_ticks =
        (report.timing.maximum_measurement_age_s / report.timing.fixed_step_s).floor() as u64;

    assert_eq!(report.observation_bursts.len(), expected_attempt_count());
    assert!(report.observation_bursts.iter().all(|burst| {
        burst
            .available_tick
            .saturating_sub(burst.capture_start_tick)
            > maximum_age_ticks
            && burst.accepted_object_ids.is_empty()
            && burst
                .rejection_reasons
                .iter()
                .any(|reason| reason.contains("stale_measurement"))
    }));
    assert!(report.decisions.iter().any(|decision| {
        decision.action == "hold_for_reacquisition"
            && decision.reason.contains("stale_measurement")
            && decision.command_sequence.is_none()
    }));
    assert_no_near_contact_tool_motion(&report);
    assert_no_action(&report, "close_gripper");
}

#[test]
fn excessive_correction_floor_is_rejected_before_any_correction_command() {
    let report = run_expected_failure(
        M1eFault::CorrectionFloorTooLarge,
        "correction_floor_too_large",
    );

    assert!(report.metrics.accepted_pose_estimates > 0);
    assert!(report
        .correction_iterations
        .iter()
        .any(|iteration| iteration.outcome == "correction_floor_too_large"));
    assert_eq!(report.metrics.correction_iterations, 0);
    assert_no_near_contact_tool_motion(&report);
    assert_no_action(&report, "close_gripper");
}

#[test]
fn grasp_capture_fault_closes_jaws_but_never_confirms_or_transfers_the_peg() {
    let report = run_expected_failure(
        M1eFault::GraspOutsideCapture,
        "grasp_outside_capture_region",
    );

    assert!(has_action(&report, "correction_converged"));
    let close = report
        .decisions
        .iter()
        .find(|decision| decision.action == "close_gripper")
        .expect("grasp fault must occur after a guarded closure command");
    assert!(close.near_contact);
    assert!(close.command_sequence.is_some());
    assert!(!report.metrics.guarded_grasp_confirmed);
    assert_no_action(&report, "post_grasp_pose_confirmed");
    assert_no_action(&report, "transfer_preflight_accepted");
    assert!(
        !report
            .evaluation_only_truth
            .as_ref()
            .unwrap()
            .physical_grasp_attachment_present
    );
}

#[test]
fn insertion_jam_is_a_bounded_classified_jam_not_a_force_trip() {
    let report = run_expected_failure(M1eFault::InsertionJam, "insertion_jam");
    let (scenario, _) = ObservedManipulationScenario::baseline().unwrap();

    assert!(report.metrics.guarded_grasp_confirmed);
    assert!(report.metrics.transfer_preflight_passed);
    for action in [
        "post_grasp_pose_confirmed",
        "transfer_preflight_accepted",
        "transfer_complete",
        "begin_socket_reacquisition",
    ] {
        assert!(has_action(&report, action), "missing action {action}");
    }
    assert!(report.metrics.insertion_increment_count > 0);
    assert!(report.decisions.iter().any(|decision| {
        decision.action == "set_tool_position"
            && decision.near_contact
            && decision.command_sequence.is_some()
            && decision.reason.contains("Insertion")
    }));
    assert!(report
        .observation_bursts
        .iter()
        .any(|burst| burst.roi == "insertion_contact"));
    assert!(
        report.metrics.maximum_insertion_force_proxy_n <= scenario.contact.maximum_force_proxy_n
    );
    assert_eq!(report.metrics.force_interlock_trip_count, 0);
    assert!(report.force_interlocks.is_empty());
    assert_no_action(&report, "force_interlock_trip");
    let jam_evidence = report
        .decisions
        .iter()
        .filter_map(|decision| decision.contact_evidence.as_ref())
        .find(|evidence| {
            serde_json::to_value(evidence).expect("serializable contact evidence")["state"]
                == "jammed"
        })
        .expect("insertion fault must produce classified Jammed evidence");
    assert!(
        jam_evidence.insertion_force_proxy_n <= scenario.contact.maximum_force_proxy_n,
        "classified jam must remain below the separate force-interlock limit"
    );
    let force_gate = report
        .acceptance_gates
        .iter()
        .find(|gate| gate.id == "force_proxy_limit")
        .expect("force gate");
    assert!(force_gate.applicable);
    assert!(force_gate.passed, "{}", force_gate.evidence);
    assert!(!report.metrics.guarded_insertion_confirmed);
    assert!(!report.metrics.seated_from_observation_and_contact);
    assert!(!report.metrics.release_confirmed);
    assert_no_action(&report, "insertion_seat_candidate");
    assert_no_action(&report, "seat_verified");
    assert_no_action(&report, "release_authorized");
    assert_no_action(&report, "release_committed");
    assert_no_action(&report, "release_confirmed");
}

#[test]
fn insertion_jam_preserves_force_headroom_for_seed_29() {
    let mut scenario: serde_json::Value =
        serde_json::from_str(BASELINE_SCENARIO_JSON).expect("embedded M1e scenario JSON");
    scenario["seed"] = serde_json::json!(29);
    let json = serde_json::to_string(&scenario).expect("seeded M1e scenario JSON");
    let report = run_custom_fault_twice(&json, M1eFault::InsertionJam);
    let (baseline, _) = ObservedManipulationScenario::baseline().unwrap();

    assert_eq!(report.terminal_reason.as_deref(), Some("insertion_jam"));
    assert!(report.expected_outcome_observed);
    assert_eq!(report.metrics.force_interlock_trip_count, 0);
    assert!(report.force_interlocks.is_empty());
    assert!(
        report.metrics.maximum_insertion_force_proxy_n < baseline.contact.maximum_force_proxy_n
    );
    assert!(report.decisions.iter().any(|decision| {
        decision.contact_evidence.as_ref().is_some_and(|evidence| {
            serde_json::to_value(evidence).expect("serializable contact evidence")["state"]
                == "jammed"
        })
    }));
}

#[test]
fn insertion_force_limit_trips_during_active_motion_and_replays_exactly() {
    let configured_limit_n = 0.002;
    let json = scenario_with_force_limit("contact", "maximum_force_proxy_n", configured_limit_n);
    let report = run_custom_scenario_twice(&json);

    assert_eq!(report.status, "failed_safe");
    assert_eq!(
        report.terminal_reason.as_deref(),
        Some("contact_force_limit")
    );
    assert!(report.metrics.guarded_grasp_confirmed);
    assert!(report.metrics.transfer_completed);
    assert!(!report.metrics.guarded_insertion_confirmed);
    assert!(!report.metrics.seated_from_observation_and_contact);
    assert!(!report.metrics.release_confirmed);
    assert_eq!(report.metrics.force_interlock_trip_count, 1);
    assert_eq!(report.force_interlocks.len(), 1);

    let interlock = &report.force_interlocks[0];
    assert_eq!(interlock.channel, "insertion");
    assert_eq!(interlock.limit_force_proxy_n, configured_limit_n);
    assert!(interlock.measured_force_proxy_n > configured_limit_n);
    assert!(interlock.motion_was_active);
    assert_eq!(interlock.packet.captured_at_tick, interlock.tick);
    assert_eq!(
        interlock.packet.insertion_force_proxy_n,
        interlock.measured_force_proxy_n
    );
    let protective_stop_sequence = interlock
        .stop_command_sequence
        .expect("interlock must submit an authoritative Stop");

    let trip_index = report
        .decisions
        .iter()
        .position(|decision| decision.action == "force_interlock_trip")
        .expect("force-interlock decision");
    let accepted_motion = report.decisions[..trip_index]
        .iter()
        .rfind(|decision| {
            decision.action == "set_tool_position"
                && decision.reason.contains("Insertion")
                && decision.command_sequence.is_some()
        })
        .expect("accepted insertion command must remain in the report");
    assert_eq!(trip_index, accepted_motion.sequence as usize + 1);
    assert!(accepted_motion.tick < interlock.tick);
    assert!(accepted_motion.command_sequence.unwrap() < protective_stop_sequence);

    let trip = &report.decisions[trip_index];
    assert_eq!(trip.tick, interlock.tick);
    assert_eq!(trip.command_sequence, Some(protective_stop_sequence));
    assert!(trip.reason.contains("same fixed tick"));
    assert!(report
        .observation_bursts
        .iter()
        .all(|burst| burst.available_tick <= accepted_motion.tick));
    assert_no_action(&report, "insertion_seat_candidate");
    assert_no_action(&report, "seat_verified");
    assert_no_action(&report, "release_authorized");
    assert_no_action(&report, "release_committed");
    assert_no_action(&report, "release_confirmed");

    let force_gate = report
        .acceptance_gates
        .iter()
        .find(|gate| gate.id == "force_proxy_limit")
        .expect("force gate");
    assert!(force_gate.applicable);
    assert!(!force_gate.passed);
}

#[test]
fn grip_force_limit_stops_closure_before_attachment_and_replays_exactly() {
    let configured_limit_n = 0.050;
    let json = scenario_with_force_limit("grasp", "maximum_grip_force_n", configured_limit_n);
    let report = run_custom_scenario_twice(&json);

    assert_eq!(report.status, "failed_safe");
    assert_eq!(
        report.terminal_reason.as_deref(),
        Some("contact_force_limit")
    );
    assert!(!report.metrics.guarded_grasp_confirmed);
    assert!(!report.metrics.transfer_completed);
    assert!(!report.metrics.guarded_insertion_confirmed);
    assert!(!report.metrics.seated_from_observation_and_contact);
    assert!(!report.metrics.release_confirmed);
    assert_eq!(report.metrics.force_interlock_trip_count, 1);
    assert_eq!(report.force_interlocks.len(), 1);

    let interlock = &report.force_interlocks[0];
    assert_eq!(interlock.channel, "gripper");
    assert_eq!(interlock.limit_force_proxy_n, configured_limit_n);
    assert!(interlock.measured_force_proxy_n > configured_limit_n);
    assert!(interlock.motion_was_active);
    assert_eq!(interlock.packet.captured_at_tick, interlock.tick);
    assert_eq!(
        interlock.packet.grip_force_proxy_n,
        interlock.measured_force_proxy_n
    );
    let protective_stop_sequence = interlock
        .stop_command_sequence
        .expect("interlock must submit an authoritative Stop");

    let trip_index = report
        .decisions
        .iter()
        .position(|decision| decision.action == "force_interlock_trip")
        .expect("force-interlock decision");
    let accepted_close = report.decisions[..trip_index]
        .iter()
        .rfind(|decision| decision.action == "close_gripper" && decision.command_sequence.is_some())
        .expect("accepted gripper command must remain in the report");
    assert_eq!(trip_index, accepted_close.sequence as usize + 1);
    assert!(accepted_close.tick < interlock.tick);
    assert!(accepted_close.command_sequence.unwrap() < protective_stop_sequence);

    let trip = &report.decisions[trip_index];
    assert_eq!(trip.tick, interlock.tick);
    assert_eq!(trip.command_sequence, Some(protective_stop_sequence));
    assert!(trip.reason.contains("same fixed tick"));
    assert!(report
        .observation_bursts
        .iter()
        .all(|burst| burst.available_tick <= accepted_close.tick));
    assert_no_action(&report, "gripper_close_motion_complete");
    assert_no_action(&report, "post_grasp_pose_confirmed");
    assert_no_action(&report, "transfer_preflight_accepted");
    assert_no_action(&report, "insertion_seat_candidate");
    assert_no_action(&report, "release_authorized");
    assert_no_action(&report, "release_committed");
    assert!(
        !report
            .evaluation_only_truth
            .as_ref()
            .expect("terminal truth-only evaluation")
            .physical_grasp_attachment_present
    );

    let force_gate = report
        .acceptance_gates
        .iter()
        .find(|gate| gate.id == "force_proxy_limit")
        .expect("force gate");
    assert!(force_gate.applicable);
    assert!(!force_gate.passed);
}

#[test]
fn occluded_mating_feature_stops_after_transfer_and_before_insertion() {
    let report = run_expected_failure(
        M1eFault::OccludedMatingFeature,
        "required_mating_feature_occluded",
    );

    assert!(report.metrics.guarded_grasp_confirmed);
    assert!(report.metrics.transfer_preflight_passed);
    for action in ["transfer_complete", "begin_socket_reacquisition"] {
        assert!(has_action(&report, action), "missing action {action}");
    }
    assert!(report.observation_bursts.iter().any(|burst| {
        burst.roi == "socket"
            && burst.missing_feature_count > 0
            && burst
                .rejection_reasons
                .iter()
                .any(|reason| reason.contains("required_mating_feature_occluded"))
    }));
    assert_eq!(report.metrics.insertion_increment_count, 0);
    assert_no_action(&report, "guarded_insertion_increment");
    assert_no_action(&report, "insertion_seat_candidate");
}

#[test]
fn carried_part_collision_is_rejected_by_estimated_preflight_without_transfer_motion() {
    let report = run_expected_failure(
        M1eFault::CarriedPartCollision,
        "carried_part_collision_risk",
    );

    assert!(report.metrics.guarded_grasp_confirmed);
    assert!(!report.metrics.transfer_preflight_passed);
    let rejection = report
        .decisions
        .iter()
        .find(|decision| {
            matches!(
                decision.action,
                "reject_estimated_sweep" | "reject_transfer"
            )
        })
        .expect("carried obstacle must produce an explicit transfer rejection");
    assert!(rejection.command_sequence.is_none());
    assert!(rejection.target_world_m.is_some());
    assert!(rejection.reason.contains("carried_part_collision_risk"));
    assert_no_action(&report, "transfer_preflight_accepted");
    assert_no_action(&report, "transfer_complete");
    assert_no_action(&report, "begin_socket_reacquisition");
    assert_eq!(report.metrics.insertion_increment_count, 0);
}

#[test]
fn inconsistent_observation_preserves_outlier_reason_and_commands_only_holds() {
    let report = run_expected_failure(M1eFault::InconsistentObservation, "observation_outlier");

    assert!(report.observation_bursts.iter().any(|burst| {
        burst
            .rejection_reasons
            .iter()
            .any(|reason| reason.contains("observation_outlier"))
    }));
    assert!(report.decisions.iter().any(|decision| {
        decision.action == "hold_for_reacquisition"
            && decision.reason.contains("observation_outlier")
            && decision.command_sequence.is_none()
    }));
    assert_no_near_contact_tool_motion(&report);
    assert_no_action(&report, "close_gripper");
}

#[test]
fn non_convergence_records_attempted_correction_and_never_proceeds_to_grasp() {
    let report = run_expected_failure(M1eFault::NonConvergence, "correction_non_convergence");

    assert!(report.metrics.correction_iterations > 0);
    assert!(report
        .correction_iterations
        .iter()
        .any(|iteration| iteration.outcome == "commanded"));
    assert!(!report
        .correction_iterations
        .iter()
        .any(|iteration| iteration.outcome == "converged"));
    assert!(report.decisions.iter().any(|decision| {
        decision.action == "set_tool_position"
            && decision.near_contact
            && decision.command_sequence.is_some()
    }));
    assert!(report.correction_iterations.iter().any(|iteration| {
        matches!(
            iteration.outcome,
            "iteration_budget_exhausted" | "nonprogress_rejected"
        )
    }));
    assert_no_action(&report, "correction_converged");
    assert_no_action(&report, "close_gripper");
}
