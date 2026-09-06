use pipe_sim::handoff::{
    controller::{Action, Phase},
    run_stationary_coupon, Fault,
};

#[test]
fn stationary_exchange_has_one_owner_and_no_pose_jump() {
    let report = run_stationary_coupon(Fault::None).unwrap();
    assert_eq!(
        report.status,
        Phase::Complete,
        "{:?}",
        report.terminal_reason
    );
    assert_eq!(report.owner_from_acknowledgements, Some(2));
    assert_eq!(report.evaluation_only_final_owner, Some(2));
    assert_eq!(report.evaluation_only_maximum_interarm_penetration_m, 0.0);
    assert_eq!(report.evaluation_only_transfer_pose_jump_m, 0.0);
    let actions: Vec<_> = report.decisions.iter().map(|d| d.action).collect();
    assert_eq!(
        actions,
        vec![
            Action::AcquireDonor,
            Action::CloseReceiver,
            Action::TransferOwnership,
            Action::OpenDonor,
            Action::VerifyReceiver
        ]
    );
    for frame in &report.evaluation_only_frames {
        let truth = frame.truth.as_ref().unwrap();
        assert!(
            truth
                .manipulators
                .iter()
                .filter(|arm| arm.gripper.held_body_id.is_some())
                .count()
                <= 1
        );
    }
    assert_eq!(report, run_stationary_coupon(Fault::None).unwrap());
}

#[test]
fn observation_loss_never_releases_the_current_owner() {
    for (fault, owner, reason) in [
        (
            Fault::ObservationLostBeforeClose,
            1,
            "observation_unavailable",
        ),
        (
            Fault::ObservationLostBeforeTransfer,
            1,
            "observation_unavailable",
        ),
        (
            Fault::ObservationLostAfterTransfer,
            2,
            "observation_unavailable",
        ),
        (Fault::StaleObservation, 1, "stale_observation"),
        (Fault::ReceiverContactMissing, 1, "required_contact_missing"),
        (Fault::TransferRejected, 1, "plant_transaction_rejected"),
        (Fault::InconsistentPose, 1, "relative_pose_outside_capture"),
    ] {
        let report = run_stationary_coupon(fault).unwrap();
        assert_eq!(report.status, Phase::Stopped, "{fault:?}");
        assert_eq!(report.terminal_reason.as_deref(), Some(reason), "{fault:?}");
        assert_eq!(report.owner_from_acknowledgements, Some(owner), "{fault:?}");
        assert_eq!(report.evaluation_only_final_owner, Some(owner), "{fault:?}");
        assert_eq!(report.evaluation_only_maximum_interarm_penetration_m, 0.0);
        assert!(report
            .evaluation_only_frames
            .last()
            .unwrap()
            .commanded
            .manipulators
            .iter()
            .all(|arm| arm.stopped));
        assert!(!report
            .decisions
            .iter()
            .any(|d| d.action == Action::OpenDonor));
    }
}

#[test]
fn handoff_controller_cannot_import_physical_state() {
    let source = include_str!("../src/handoff/controller.rs");
    for forbidden in ["pipe_sim_core", "SceneFrame", "RigidBody", "super::plant"] {
        assert!(!source.contains(forbidden), "truth boundary: {forbidden}");
    }
}
