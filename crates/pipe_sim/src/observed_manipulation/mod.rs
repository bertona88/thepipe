//! M1e observed-state single-arm calibration-coupon vertical slice.

mod controller;
mod estimator;
mod plant;
mod report;
mod runtime;
mod scenario;

pub use estimator::{
    AxisymmetricPose5d, EstimateValidity, EstimatorConfig, EstimatorConfigError,
    FeatureMeasurement, KnownAxialFeature, MeasurementKey, MeasurementRejectionReason,
    ObservedPoseEstimator, PoseEstimate, PoseInnovation, ReducedPoseUncertainty,
    RejectedMeasurement, AXISYMMETRIC_POSE_DOF,
};
pub use report::{ObservedManipulationReport, OBSERVED_MANIPULATION_REPORT_SCHEMA_VERSION};
pub use runtime::ObservedManipulationRuntime;
pub use scenario::{
    M1eFault, ObservedManipulationScenario, ScenarioError, M1E_SCENARIO_SCHEMA_VERSION,
};

#[cfg(test)]
mod truth_firewall_tests {
    use super::controller::{
        classify_contact_packet, decide_correction, guard_estimate, ClassifiedContactEvidence,
        ContactClassificationPolicy, ContactPacket, ContactState, CorrectionDecision,
        CorrectionPolicy, EstimateGate, EstimateProvenance, EstimateView, RelativeMatingPose,
    };

    fn sanitized_estimate(object_id: u32, position_world_m: [f64; 3]) -> EstimateView {
        EstimateView {
            object_id,
            valid: true,
            invalid_reason: None,
            position_world_m,
            axis_world: [0.0, 0.0, 1.0],
            position_sigma_m: 4.0e-6,
            axis_sigma_rad: 0.005,
            capture_tick: 100,
            available_tick: 105,
            distinct_feature_count: 4,
            triangulation_head_count: 1,
            minimum_calibrated_rays_per_point: 2,
            residual_rms_m: 2.0e-6,
            provenance: EstimateProvenance::direct_feature_fit(),
        }
    }

    fn decisions_from_sanitized_boundary(
        tool: &EstimateView,
        peg: &EstimateView,
        packet: ContactPacket,
        relative: RelativeMatingPose,
    ) -> (CorrectionDecision, ClassifiedContactEvidence) {
        let estimate_gate = EstimateGate {
            maximum_age_ticks: 20,
            maximum_position_sigma_m: 10.0e-6,
            maximum_axis_sigma_rad: 0.020,
            minimum_distinct_features: 4,
            minimum_triangulation_heads: 1,
            minimum_calibrated_rays_per_point: 2,
            maximum_residual_m: 20.0e-6,
        };
        guard_estimate(110, tool, estimate_gate).expect("sanitized tool estimate must pass");
        guard_estimate(110, peg, estimate_gate).expect("sanitized peg estimate must pass");

        let tool_to_peg_axial_offset_m = 0.750e-3;
        let desired_tool_world_m = [
            peg.position_world_m[0] - peg.axis_world[0] * tool_to_peg_axial_offset_m,
            peg.position_world_m[1] - peg.axis_world[1] * tool_to_peg_axial_offset_m,
            peg.position_world_m[2] - peg.axis_world[2] * tool_to_peg_axial_offset_m,
        ];
        let correction = decide_correction(
            tool.position_world_m,
            desired_tool_world_m,
            CorrectionPolicy {
                gain: 1.0,
                convergence_m: 9.0e-6,
                maximum_magnitude_m: 250.0e-6,
                minimum_reproducible_m: 5.0e-6,
            },
        );
        let contact = classify_contact_packet(
            110,
            packet,
            Some(relative),
            ContactClassificationPolicy {
                maximum_packet_age_ticks: 20,
                maximum_pose_age_ticks: 20,
                lead_in_start_m: 300.0e-6,
                recoverable_lateral_error_m: 95.0e-6,
                maximum_lateral_error_m: 140.0e-6,
                seat_axial_tolerance_m: 18.0e-6,
                seat_lateral_tolerance_m: 30.0e-6,
                seat_axis_tolerance_rad: 0.010,
                axis_lever_arm_m: 300.0e-6,
                maximum_position_sigma_m: 10.0e-6,
                maximum_axis_sigma_rad: 0.020,
                maximum_force_proxy_n: 0.080,
            },
        )
        .expect("sanitized contact packet must classify");
        (correction, contact)
    }

    #[test]
    fn controller_and_estimator_sources_cannot_import_plant_truth_types() {
        for (name, source) in [
            ("controller", include_str!("controller.rs")),
            ("estimator", include_str!("estimator.rs")),
        ] {
            for forbidden in [
                "pipe_sim_core",
                "Simulation",
                "RigidBody",
                "DepthSample",
                "SceneFrame",
            ] {
                assert!(
                    !source.contains(forbidden),
                    "{name} source crossed the truth firewall with {forbidden}"
                );
            }
        }
    }

    #[test]
    fn plant_cannot_publish_a_contact_verdict_to_the_controller() {
        let plant = include_str!("plant.rs");
        for forbidden in ["ClassifiedContactEvidence", "ContactState"] {
            assert!(
                !plant.contains(forbidden),
                "plant crossed the contact truth firewall with {forbidden}"
            );
        }

        let runtime = include_str!("runtime.rs");
        assert!(runtime.contains("classify_contact_packet"));
        assert!(runtime.contains("RelativeMatingPose"));
        assert!(!runtime.contains(".private_insertion_contact"));
        assert!(!runtime.contains(".private_jaw_contact_channels"));
    }

    #[test]
    fn pure_decisions_replay_and_change_only_with_sanitized_inputs() {
        let tool = sanitized_estimate(1, [0.0, 0.0, 0.0]);
        let peg = sanitized_estimate(2, [40.0e-6, -20.0e-6, 0.750e-3]);
        let packet = ContactPacket {
            captured_at_tick: 108,
            contact_detected: true,
            left_pad_contact: true,
            right_pad_contact: true,
            left_pad_deflection_m: 5.0e-6,
            right_pad_deflection_m: 5.0e-6,
            grip_force_proxy_n: 0.020,
            insertion_force_proxy_n: 0.010,
        };
        let relative = RelativeMatingPose {
            captured_at_tick: 100,
            available_at_tick: 105,
            axial_error_m: 80.0e-6,
            lateral_error_m: 20.0e-6,
            axis_error_rad: 0.005,
            position_sigma_m: 5.0e-6,
            axis_sigma_rad: 0.005,
        };

        let first = decisions_from_sanitized_boundary(&tool, &peg, packet, relative);
        let replay = decisions_from_sanitized_boundary(&tool, &peg, packet, relative);
        assert_eq!(first, replay, "pure sanitized decision replay changed");
        assert_eq!(first.1.state, ContactState::LeadInContact);
        match first.0 {
            CorrectionDecision::Command {
                correction_world_m, ..
            } => assert_eq!(correction_world_m, [40.0e-6, -20.0e-6, 0.0]),
            other => panic!("expected bounded correction, got {other:?}"),
        }

        let mut changed_peg_observation = peg.clone();
        changed_peg_observation.position_world_m[0] = 80.0e-6;
        let changed_pose =
            decisions_from_sanitized_boundary(&tool, &changed_peg_observation, packet, relative);
        match changed_pose.0 {
            CorrectionDecision::Command {
                correction_world_m, ..
            } => assert_eq!(correction_world_m, [80.0e-6, -20.0e-6, 0.0]),
            other => panic!("expected observation-driven correction, got {other:?}"),
        }
        assert_eq!(changed_pose.1, first.1, "unchanged contact input drifted");

        let mut changed_contact_observation = relative;
        changed_contact_observation.lateral_error_m = 160.0e-6;
        let changed_contact =
            decisions_from_sanitized_boundary(&tool, &peg, packet, changed_contact_observation);
        assert_eq!(changed_contact.0, first.0, "unchanged pose input drifted");
        assert_eq!(changed_contact.1.state, ContactState::Jammed);
    }
}
