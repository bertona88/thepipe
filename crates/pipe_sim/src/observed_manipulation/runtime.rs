use std::collections::BTreeMap;

use serde::Serialize;

use super::controller::{
    add, bounded_axis_correction, classify_contact_packet, decide_correction,
    derive_held_peg_from_tool, dot, estimate_held_transform, guard_axial_grasp_overlap,
    guard_estimate, guard_grasp_evidence, guard_jaw_socket_axial_clearance,
    guard_palm_peg_axial_clearance, guard_relative_estimates, guard_target_rail_sweep, norm,
    preflight_swept_envelope, scale, sub, AxialEnvelopeSweep, AxialGraspOverlapPolicy,
    ClassifiedContactEvidence, ContactClassificationFailure, ContactClassificationPolicy,
    ContactState, ControlPhase, CorrectionDecision, CorrectionPolicy, EstimateGate,
    EstimateGuardFailure, EstimateProvenance, EstimateView, GraspEvidenceGate,
    HeldTransformEstimate, JawSocketAxialClearanceEvidence, JawSocketAxialClearancePolicy,
    JawSocketMotionPreview, PalmPegAxialClearancePolicy, RelativeEstimateUncertainty,
    RelativeMatingPose, SweptEnvelope, SweptPreflightFailure, TargetRailDatum,
};
use super::estimator::{EstimateValidity, EstimatorConfig, ObservedPoseEstimator, PoseEstimate};
use super::plant::{
    MotionClass, ObservationBurst, ObservedPlant, PlantFailure, PEG_OBJECT_ID, SOCKET_OBJECT_ID,
    TOOL_OBJECT_ID,
};
use super::report::{
    AcceptanceGate, ControllerMetrics, CorrectionIterationRecord, DecisionRecord,
    EstimatorUpdateRecord, EvaluationOnlyTruthMetrics, ForceInterlockRecord,
    ObservationBurstRecord, ObservedManipulationReport, TimingReport, TruthFirewallReport,
    UncertaintyGuardRecord, OBSERVED_MANIPULATION_REPORT_SCHEMA_VERSION,
};
use super::scenario::{M1eFault, ObservedManipulationScenario};
use crate::{sha256_hex, SimError};

const HASH_SCOPE: &str =
    "scenario+machine_config+fault+terminal+timing+controller_metrics+observations+estimator_updates+uncertainty_guards+force_interlocks+corrections+decisions+gates; excludes evaluation_only_truth and hash field";

#[derive(Clone, Copy, Debug, PartialEq)]
struct AnticipatedMotionUncertainty {
    position_sigma_m: f64,
    axis_sigma_rad: f64,
    commanded_tool_axis_change_bound_rad: f64,
    tool_path_deviation_bound_m: f64,
    interval_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CaptureEntryGeometry {
    start_world_m: [f64; 3],
    target_world_m: [f64; 3],
    blind_sweep_length_m: f64,
    maximum_required_correction_m: f64,
    commanded_tool_axis_change_bound_rad: f64,
    tool_path_deviation_bound_m: f64,
    minimum_open_jaw_clearance_m: f64,
    minimum_side_target_to_peg_clearance_m: f64,
    minimum_palm_to_peg_clearance_m: f64,
    minimum_jaw_to_peg_feature_clearance_m: f64,
}

pub struct ObservedManipulationRuntime {
    scenario: ObservedManipulationScenario,
    scenario_sha256: String,
    fault: M1eFault,
    plant: ObservedPlant,
    estimators: BTreeMap<u32, ObservedPoseEstimator>,
    commanded_tool_position_world_m: [f64; 3],
    phase: ControlPhase,
    terminal_reason: Option<String>,
    observations: Vec<ObservationBurstRecord>,
    estimator_updates: Vec<EstimatorUpdateRecord>,
    uncertainty_guards: Vec<UncertaintyGuardRecord>,
    force_interlocks: Vec<ForceInterlockRecord>,
    corrections: Vec<CorrectionIterationRecord>,
    decisions: Vec<DecisionRecord>,
    metrics: ControllerMetrics,
    insertion_recovery_count: u32,
    stale_near_contact_command_count: u32,
    moving_pose_reacquisition_required: bool,
    grasp_confirmed: bool,
    held_transform: Option<HeldTransformEstimate>,
    last_socket_axis_world: Option<[f64; 3]>,
}

impl ObservedManipulationRuntime {
    pub fn new(fault: M1eFault) -> Result<Self, SimError> {
        let (scenario, hash) = ObservedManipulationScenario::baseline()
            .map_err(|error| SimError::InvalidScenario(error.to_string()))?;
        Self::from_validated_scenario(scenario, hash, fault)
    }

    pub fn from_scenario_json(json: &str, fault: M1eFault) -> Result<Self, SimError> {
        let (scenario, hash) = ObservedManipulationScenario::from_json(json)
            .map_err(|error| SimError::InvalidScenario(error.to_string()))?;
        Self::from_validated_scenario(scenario, hash, fault)
    }

    fn from_validated_scenario(
        scenario: ObservedManipulationScenario,
        scenario_sha256: String,
        fault: M1eFault,
    ) -> Result<Self, SimError> {
        if fault != M1eFault::None && scenario.expected_failure_reason(fault).is_none() {
            return Err(SimError::InvalidScenario(format!(
                "fault '{}' has no expected terminal reason",
                fault.id()
            )));
        }
        let plant = ObservedPlant::new(&scenario, fault)?;
        let fixed_dt_s = plant.fixed_dt_s();
        let estimator_config = estimator_config(&scenario, fixed_dt_s);
        let mut estimators = BTreeMap::new();
        for object_id in [PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID] {
            let estimator = ObservedPoseEstimator::new(
                estimator_config.clone(),
                object_id,
                plant.feature_model(object_id),
            )
            .map_err(|error| SimError::InvalidScenario(error.to_string()))?;
            estimators.insert(object_id, estimator);
        }
        let commanded_tool_position_world_m = plant.commanded_tool_position_world_m();
        let mut runtime = Self {
            scenario,
            scenario_sha256,
            fault,
            plant,
            estimators,
            commanded_tool_position_world_m,
            phase: ControlPhase::Initialize,
            terminal_reason: None,
            observations: Vec::new(),
            estimator_updates: Vec::new(),
            uncertainty_guards: Vec::new(),
            force_interlocks: Vec::new(),
            corrections: Vec::new(),
            decisions: Vec::new(),
            metrics: ControllerMetrics::default(),
            insertion_recovery_count: 0,
            stale_near_contact_command_count: 0,
            moving_pose_reacquisition_required: false,
            grasp_confirmed: false,
            held_transform: None,
            last_socket_axis_world: None,
        };
        runtime.record_decision(
            "initialize",
            "loaded versioned M1e scenario and constructed truth-isolated plant",
            false,
            None,
            None,
            Vec::new(),
            None,
        );
        Ok(runtime)
    }

    pub fn run_cycle(&mut self) -> Result<ObservedManipulationReport, SimError> {
        if self.is_terminal() {
            return Ok(self.report());
        }
        if let Err(reason) = self.execute() {
            self.fail_closed(reason);
        }
        Ok(self.report())
    }

    fn execute(&mut self) -> Result<(), String> {
        self.enter_capture_volume()?;

        self.phase = ControlPhase::PickCorrection;
        let pick_estimates = self.stop_and_look_correction(
            "pick",
            self.scenario.coupon.pick_peg_center_nominal_world_m,
            TOOL_OBJECT_ID,
            PEG_OBJECT_ID,
        )?;

        self.phase = ControlPhase::GuardedGrasp;
        let post_grasp_estimates = self.perform_guarded_grasp(&pick_estimates)?;

        self.phase = ControlPhase::Transfer;
        self.transfer_to_socket(post_grasp_estimates)?;

        self.phase = ControlPhase::SocketCorrection;
        let socket_estimates = self.stop_and_look_socket_alignment()?;

        self.phase = ControlPhase::GuardedInsertion;
        self.perform_guarded_insertion(socket_estimates)?;

        self.phase = ControlPhase::SeatVerification;
        let seated_estimates = self.verify_seated()?;

        self.phase = ControlPhase::Release;
        self.release(&seated_estimates)?;

        self.phase = ControlPhase::Retreat;
        self.retreat_and_reobserve()?;

        self.phase = ControlPhase::Complete;
        self.record_decision(
            "complete",
            "all observed-state nominal acceptance transitions completed",
            false,
            None,
            None,
            Vec::new(),
            None,
        );
        Ok(())
    }

    fn enter_capture_volume(&mut self) -> Result<(), String> {
        self.phase = ControlPhase::EnterCapture;
        let capture = self.capture_entry_geometry()?;
        let maximum_opening_m = self.plant.maximum_gripper_opening_m();
        let open = self
            .plant
            .command_gripper(maximum_opening_m)
            .map_err(plant_reason)?;
        self.record_decision(
            "open_gripper_for_capture",
            "authoritative runtime accepted the calibrated maximum jaw-opening command before blind capture entry",
            false,
            Some(open.command_sequence),
            None,
            Vec::new(),
            None,
        );
        self.advance_until_idle_with_interlock_report()?;
        // Enter from the peg tail using only the calibrated coupon datum and
        // axis. Configured physical errors remain plant-private; the declared
        // capture bounds below are calibration/fixture limits, not a truth
        // correction vector.
        let target = capture.target_world_m;
        self.record_decision(
            "capture_entry_guard_accepted",
            format!(
                "calibrated {:.9e} m tail-axis blind sweep preserves lower-bound clearances: open jaws {:.9e} m, side target {:.9e} m, recessed palm {:.9e} m, jaw-to-peg features {:.9e} m; certified TCP path departure {:.9e} m and tool-axis excursion {:.9e} rad; subsequent observed correction bound {:.9e} m",
                capture.blind_sweep_length_m,
                capture.minimum_open_jaw_clearance_m,
                capture.minimum_side_target_to_peg_clearance_m,
                capture.minimum_palm_to_peg_clearance_m,
                capture.minimum_jaw_to_peg_feature_clearance_m,
                capture.tool_path_deviation_bound_m,
                capture.commanded_tool_axis_change_bound_rad,
                capture.maximum_required_correction_m,
            ),
            false,
            None,
            Some(target),
            Vec::new(),
            None,
        );
        self.command_motion(target, MotionClass::Transit, false, Vec::new())?;
        self.record_decision(
            "enter_capture_complete",
            "coarse calibrated tail-side standoff entered the local macro field after swept-clearance preflight",
            false,
            None,
            Some(target),
            Vec::new(),
            None,
        );
        Ok(())
    }

    fn capture_entry_geometry(&self) -> Result<CaptureEntryGeometry, String> {
        let axis_world = self.plant.calibrated_socket_axis_world();
        if axis_world.iter().any(|value| !value.is_finite())
            || (norm(axis_world) - 1.0).abs() > 1.0e-6
        {
            return Err("invalid_calibrated_capture_axis".to_owned());
        }
        let motion = &self.scenario.motion;
        let grasp = &self.scenario.grasp;
        let start_world_m = sub(
            self.scenario.coupon.pick_peg_center_nominal_world_m,
            scale(axis_world, motion.pick_capture_start_axial_standoff_m),
        );
        if norm(sub(self.commanded_tool_position_world_m, start_world_m)) > 1.0e-12 {
            return Err("capture_start_not_on_calibrated_tail_axis".to_owned());
        }
        let target_world_m = sub(
            self.scenario.coupon.pick_peg_center_nominal_world_m,
            scale(axis_world, motion.pick_capture_axial_standoff_m),
        );
        let commanded_tool_axis_change_bound_rad = self
            .plant
            .preview_tool_axis_change_bound_rad(target_world_m, MotionClass::Transit)
            .map_err(plant_reason)?;
        let tool_path_deviation_bound_m = self
            .plant
            .preview_tool_path_deviation_bound_m(target_world_m, MotionClass::Transit)
            .map_err(plant_reason)?;
        let complete_relative_axis_bound_rad =
            motion.pick_capture_relative_axis_bound_rad + commanded_tool_axis_change_bound_rad;

        // This triangle/chord bound covers translational capture uncertainty,
        // the difference between capture and grasp stations, and rotation of
        // the desired grasp offset away from the calibrated coupon axis.
        let maximum_required_correction_m = motion.pick_capture_relative_position_bound_m
            + (motion.pick_capture_axial_standoff_m - grasp.tool_to_peg_axial_offset_m).abs()
            + 2.0
                * grasp.tool_to_peg_axial_offset_m
                * (0.5 * motion.pick_capture_relative_axis_bound_rad).sin();
        if maximum_required_correction_m > motion.maximum_correction_m + 1.0e-15 {
            return Err("capture_pose_outside_correction_authority".to_owned());
        }

        // The open-jaw half-gap must contain the entire possible transverse
        // peg envelope during blind entry. The position bound is applied in
        // full (rather than only laterally), and the tilt term bounds shaft
        // centerline excursion; this is deliberately conservative.
        let peg_tip_offset_m = self.peg_tip_offset_m();
        let open_jaw_half_gap_m = 0.5 * self.plant.maximum_gripper_opening_m();
        let peg_transverse_envelope_m = 0.5 * self.scenario.coupon.peg_diameter_m
            + motion.pick_capture_relative_position_bound_m
            + peg_tip_offset_m * complete_relative_axis_bound_rad.sin()
            + tool_path_deviation_bound_m;
        let minimum_open_jaw_clearance_m = open_jaw_half_gap_m - peg_transverse_envelope_m;
        if !minimum_open_jaw_clearance_m.is_finite()
            || minimum_open_jaw_clearance_m + 1.0e-15
                < self.scenario.safety.minimum_obstacle_clearance_m
        {
            return Err("capture_open_jaw_clearance_insufficient".to_owned());
        }

        let tool_geometry = self.plant.jaw_socket_clearance_geometry();
        let minimum_side_target_to_peg_clearance_m = tool_geometry
            .side_target_center_transverse_distance_m
            - tool_geometry.side_target_cross_section_radius_m
            - peg_transverse_envelope_m
            - 2.0
                * tool_geometry.side_target_center_transverse_distance_m
                * (0.5 * commanded_tool_axis_change_bound_rad.min(core::f64::consts::PI)).sin()
            - tool_geometry.side_target_axial_half_extent_m
                * commanded_tool_axis_change_bound_rad.sin().abs();
        if !minimum_side_target_to_peg_clearance_m.is_finite()
            || minimum_side_target_to_peg_clearance_m + 1.0e-15
                < self.scenario.safety.minimum_obstacle_clearance_m
        {
            return Err("capture_side_target_clearance_insufficient".to_owned());
        }
        let minimum_palm_to_peg_clearance_m = motion.pick_capture_axial_standoff_m
            - peg_tip_offset_m
            - tool_geometry.central_palm_forward_plane_tool_z_m
            - motion.pick_capture_relative_position_bound_m;
        let minimum_palm_to_peg_clearance_m = minimum_palm_to_peg_clearance_m
            - tool_path_deviation_bound_m
            - tool_geometry.central_palm_radial_extent_m
                * commanded_tool_axis_change_bound_rad.sin().abs();
        if !minimum_palm_to_peg_clearance_m.is_finite()
            || minimum_palm_to_peg_clearance_m + 1.0e-15
                < self.scenario.safety.minimum_obstacle_clearance_m
        {
            return Err("capture_central_palm_clearance_insufficient".to_owned());
        }

        let jaw_axial_half_length_m = tool_geometry.jaw_axial_half_length_m;
        let tailmost_peg_feature_m = self
            .plant
            .feature_model(PEG_OBJECT_ID)
            .into_iter()
            .map(|feature| feature.axial_coordinate_m)
            .min_by(f64::total_cmp)
            .ok_or_else(|| "capture_peg_feature_model_empty".to_owned())?;
        let minimum_jaw_to_peg_feature_clearance_m =
            motion.pick_capture_axial_standoff_m - jaw_axial_half_length_m + tailmost_peg_feature_m
                - motion.pick_capture_relative_position_bound_m
                - self.scenario.coupon.peg_half_segment_m * complete_relative_axis_bound_rad.sin()
                - tool_path_deviation_bound_m
                - tool_geometry.open_jaw_transverse_radius_m
                    * commanded_tool_axis_change_bound_rad.sin().abs();
        if !minimum_jaw_to_peg_feature_clearance_m.is_finite()
            || minimum_jaw_to_peg_feature_clearance_m + 1.0e-15
                < self.scenario.safety.minimum_obstacle_clearance_m
        {
            return Err("capture_peg_feature_clearance_insufficient".to_owned());
        }

        Ok(CaptureEntryGeometry {
            start_world_m,
            target_world_m,
            blind_sweep_length_m: motion.pick_capture_start_axial_standoff_m
                - motion.pick_capture_axial_standoff_m,
            maximum_required_correction_m,
            commanded_tool_axis_change_bound_rad,
            tool_path_deviation_bound_m,
            minimum_open_jaw_clearance_m,
            minimum_side_target_to_peg_clearance_m,
            minimum_palm_to_peg_clearance_m,
            minimum_jaw_to_peg_feature_clearance_m,
        })
    }

    fn stop_and_look_correction(
        &mut self,
        roi: &'static str,
        roi_world_m: [f64; 3],
        moving_object_id: u32,
        target_object_id: u32,
    ) -> Result<BTreeMap<u32, EstimateView>, String> {
        for iteration in 0..=self.scenario.motion.maximum_correction_iterations {
            let estimates = self.observe_required(
                self.phase,
                roi,
                roi_world_m,
                &[moving_object_id, target_object_id],
                true,
            )?;
            let moving = estimates
                .get(&moving_object_id)
                .ok_or_else(|| "missing_moving_object_estimate".to_owned())?;
            let target = estimates
                .get(&target_object_id)
                .ok_or_else(|| "missing_target_object_estimate".to_owned())?;
            self.guard_relative_view(
                moving,
                target,
                self.scenario.estimator.grasp_position_sigma_limit_m,
                self.scenario.estimator.axis_sigma_limit_rad,
            )?;
            if self.correct_observed_axis(moving, target, iteration, &estimates)? {
                continue;
            }
            let desired = sub(
                target.position_world_m,
                scale(
                    target.axis_world,
                    self.scenario.grasp.tool_to_peg_axial_offset_m,
                ),
            );
            let policy = self.correction_policy();
            let decision = if self.scenario.fixed_head.is_some() {
                super::controller::decide_quantized_correction(
                    moving.position_world_m,
                    desired,
                    policy,
                )
            } else {
                decide_correction(moving.position_world_m, desired, policy)
            };
            match decision {
                CorrectionDecision::Converged { residual_m } => {
                    self.corrections.push(CorrectionIterationRecord {
                        phase: self.phase,
                        iteration,
                        decision_tick: self.plant.now_tick(),
                        measurement_age_s: measurement_age_s(
                            self.plant.now_tick(),
                            moving,
                            self.plant.fixed_dt_s(),
                        ),
                        residual_before_m: residual_m,
                        requested_correction_world_m: None,
                        requested_correction_m: 0.0,
                        estimator_position_sigma_m: moving.position_sigma_m,
                        estimator_axis_sigma_rad: moving.axis_sigma_rad,
                        outcome: "converged",
                    });
                    self.metrics.successful_corrections += 1;
                    self.record_decision(
                        "correction_converged",
                        format!("{roi} residual {residual_m:.9e} m is within threshold"),
                        true,
                        None,
                        None,
                        estimates.values().cloned().collect(),
                        None,
                    );
                    return Ok(estimates);
                }
                CorrectionDecision::Command {
                    residual_before_m,
                    correction_world_m,
                    ..
                } => {
                    if iteration == self.scenario.motion.maximum_correction_iterations {
                        self.corrections.push(CorrectionIterationRecord {
                            phase: self.phase,
                            iteration,
                            decision_tick: self.plant.now_tick(),
                            measurement_age_s: measurement_age_s(
                                self.plant.now_tick(),
                                moving,
                                self.plant.fixed_dt_s(),
                            ),
                            residual_before_m,
                            requested_correction_world_m: Some(correction_world_m),
                            requested_correction_m: norm(correction_world_m),
                            estimator_position_sigma_m: moving.position_sigma_m,
                            estimator_axis_sigma_rad: moving.axis_sigma_rad,
                            outcome: "iteration_budget_exhausted",
                        });
                        return Err("correction_non_convergence".to_owned());
                    }
                    let target_world_m =
                        add(self.commanded_tool_position_world_m, correction_world_m);
                    self.metrics.correction_iterations += 1;
                    self.corrections.push(CorrectionIterationRecord {
                        phase: self.phase,
                        iteration,
                        decision_tick: self.plant.now_tick(),
                        measurement_age_s: measurement_age_s(
                            self.plant.now_tick(),
                            moving,
                            self.plant.fixed_dt_s(),
                        ),
                        residual_before_m,
                        requested_correction_world_m: Some(correction_world_m),
                        requested_correction_m: norm(correction_world_m),
                        estimator_position_sigma_m: moving.position_sigma_m,
                        estimator_axis_sigma_rad: moving.axis_sigma_rad,
                        outcome: "commanded",
                    });
                    self.command_motion(
                        target_world_m,
                        MotionClass::Correction,
                        true,
                        estimates.values().cloned().collect(),
                    )?;
                    self.predict_after_motion(correction_world_m, false)?;
                    self.invalidate_moving_pose_priors_before_reacquisition(
                        false,
                        true,
                        "pick correction",
                    )?;
                }
                CorrectionDecision::Rejected {
                    reason,
                    requested_magnitude_m,
                } => {
                    self.corrections.push(CorrectionIterationRecord {
                        phase: self.phase,
                        iteration,
                        decision_tick: self.plant.now_tick(),
                        measurement_age_s: measurement_age_s(
                            self.plant.now_tick(),
                            moving,
                            self.plant.fixed_dt_s(),
                        ),
                        residual_before_m: norm(sub(desired, moving.position_world_m)),
                        requested_correction_world_m: None,
                        requested_correction_m: requested_magnitude_m,
                        estimator_position_sigma_m: moving.position_sigma_m,
                        estimator_axis_sigma_rad: moving.axis_sigma_rad,
                        outcome: reason,
                    });
                    return Err(reason.to_owned());
                }
            }
        }
        Err("correction_non_convergence".to_owned())
    }

    fn correct_observed_axis(
        &mut self,
        moving: &EstimateView,
        target: &EstimateView,
        iteration: u32,
        estimates: &BTreeMap<u32, EstimateView>,
    ) -> Result<bool, String> {
        let Some(head) = &self.scenario.fixed_head else {
            return Ok(false);
        };
        let commanded = self
            .plant
            .commanded_tool_axis_world()
            .ok_or_else(|| "axis_command_state_required".to_owned())?;
        let correction = bounded_axis_correction(
            moving.axis_world,
            target.axis_world,
            commanded,
            head.axis_convergence_rad,
            head.maximum_axis_correction_rad,
            head.maximum_axis_capture_error_rad,
        )
        .map_err(str::to_owned)?;
        let Some(axis) = correction else {
            return Ok(false);
        };
        if iteration == self.scenario.motion.maximum_correction_iterations {
            return Err("axis_correction_non_convergence".to_owned());
        }
        let step = axis_angle_rad(commanded, axis);
        if step < head.minimum_axis_correction_rad {
            return Err("axis_correction_floor_too_large".to_owned());
        }
        self.plant
            .set_axis_planning_target(axis)
            .map_err(plant_reason)?;
        self.record_decision("axis_correction", format!(
            "observed axis error {:.9e} rad; bounded commanded rotation {:.9e} rad; roll unconstrained",
            axis_angle_rad(moving.axis_world, target.axis_world), step), true,
            None, Some(self.commanded_tool_position_world_m), estimates.values().cloned().collect(), None);
        self.command_motion(
            self.commanded_tool_position_world_m,
            MotionClass::Correction,
            true,
            estimates.values().cloned().collect(),
        )?;
        self.invalidate_moving_pose_priors_before_reacquisition(
            self.grasp_confirmed,
            true,
            "position-and-axis correction",
        )?;
        Ok(true)
    }

    fn perform_guarded_grasp(
        &mut self,
        estimates: &BTreeMap<u32, EstimateView>,
    ) -> Result<BTreeMap<u32, EstimateView>, String> {
        let tool = required_view(estimates, TOOL_OBJECT_ID)?;
        let peg = required_view(estimates, PEG_OBJECT_ID)?;
        self.guard_view(tool, self.scenario.estimator.grasp_position_sigma_limit_m)?;
        self.guard_view(peg, self.scenario.estimator.grasp_position_sigma_limit_m)?;
        let expected_peg_center_world_m = add(
            tool.position_world_m,
            scale(
                tool.axis_world,
                self.scenario.grasp.tool_to_peg_axial_offset_m,
            ),
        );
        let observed_center_offset_m = norm(sub(peg.position_world_m, expected_peg_center_world_m));
        self.guard_relative_view(
            tool,
            peg,
            self.scenario.estimator.grasp_position_sigma_limit_m,
            self.scenario.estimator.axis_sigma_limit_rad,
        )?;
        if observed_center_offset_m > self.scenario.grasp.maximum_center_offset_m {
            return Err("grasp_outside_capture_region".to_owned());
        }
        let observed_axis_error_rad = axis_angle_rad(peg.axis_world, tool.axis_world);
        if observed_axis_error_rad > self.scenario.grasp.maximum_axis_error_rad {
            return Err("impossible_grasp_axis_geometry".to_owned());
        }
        let jaw_geometry = self.plant.jaw_socket_clearance_geometry();
        let axial_overlap = guard_axial_grasp_overlap(
            tool,
            peg,
            AxialGraspOverlapPolicy {
                jaw_axial_half_length_m: jaw_geometry.jaw_axial_half_length_m,
                peg_cylindrical_half_length_m: self.scenario.coupon.peg_half_segment_m,
                minimum_overlap_m: self.scenario.grasp.minimum_axial_grasp_overlap_m,
            },
        )
        .map_err(str::to_owned)?;
        let palm_clearance = guard_palm_peg_axial_clearance(
            tool,
            peg,
            PalmPegAxialClearancePolicy {
                palm_forward_plane_tool_z_m: jaw_geometry.central_palm_forward_plane_tool_z_m,
                peg_cylindrical_half_length_m: self.scenario.coupon.peg_half_segment_m,
                peg_radius_m: 0.5 * self.scenario.coupon.peg_diameter_m,
                minimum_clearance_m: self.scenario.safety.minimum_obstacle_clearance_m,
            },
        )
        .map_err(str::to_owned)?;

        let target_opening_m =
            self.scenario.coupon.peg_diameter_m - self.scenario.grasp.commanded_pad_compression_m;
        let receipt = self
            .plant
            .command_gripper(target_opening_m)
            .map_err(plant_reason)?;
        self.record_decision(
            "close_gripper",
            format!(
                "observed pose passed grasp covariance/capture gates with {:.9e} m conservative axial shaft/pad overlap and {:.9e} m central-palm/peg clearance; commanded compliant closure",
                axial_overlap.conservative_overlap_m,
                palm_clearance.minimum_clearance_m,
            ),
            true,
            Some(receipt.command_sequence),
            None,
            estimates.values().cloned().collect(),
            None,
        );
        self.advance_until_idle_with_interlock_report()?;
        self.record_decision(
            "gripper_close_motion_complete",
            "bounded jaw closure completed without a protective force-interlock trip",
            true,
            None,
            None,
            estimates.values().cloned().collect(),
            None,
        );

        let evidence = self.classify_contact(None)?;
        guard_grasp_evidence(
            evidence,
            GraspEvidenceGate {
                minimum_pad_deflection_m: self.scenario.grasp.minimum_bilateral_pad_deflection_m,
                maximum_pad_deflection_m: self.scenario.grasp.maximum_bilateral_pad_deflection_m,
                minimum_force_n: self.scenario.grasp.minimum_grip_force_n,
                maximum_force_n: self.scenario.grasp.maximum_grip_force_n,
            },
        )
        .map_err(|reason| {
            if reason == "bilateral_contact_missing" {
                "grasp_outside_capture_region".to_owned()
            } else {
                reason.to_owned()
            }
        })?;
        self.plant.commit_grasp().map_err(plant_reason)?;
        self.grasp_confirmed = true;
        self.metrics.guarded_grasp_confirmed = true;
        self.metrics.maximum_grip_force_proxy_n = self
            .metrics
            .maximum_grip_force_proxy_n
            .max(evidence.grip_force_proxy_n);
        self.record_decision(
            "grasp_committed",
            "guarded bilateral-contact evidence accepted; plant committed the uncertain kinematic attachment",
            true,
            None,
            None,
            estimates.values().cloned().collect(),
            Some(evidence),
        );

        // Jaw closure takes longer than the optical freshness budget.  Do not
        // carry the pre-closure pose across the grasp transition: stop, settle,
        // and observe both the tool and the now-held peg before any retraction.
        let post_grasp_roi = self.held_assembly_roi_world_m(peg.axis_world, false)?;
        let post_grasp_estimates = self.observe_required(
            self.phase,
            "post_grasp",
            post_grasp_roi,
            &[TOOL_OBJECT_ID, PEG_OBJECT_ID],
            true,
        )?;
        let post_grasp_tool = required_view(&post_grasp_estimates, TOOL_OBJECT_ID)?;
        let post_grasp_peg = required_view(&post_grasp_estimates, PEG_OBJECT_ID)?;
        let held_transform = estimate_held_transform(
            post_grasp_peg,
            post_grasp_tool,
            self.scenario.estimator.held_transform_sigma_m,
        )
        .map_err(|failure| format!("held_transform_{failure:?}").to_ascii_lowercase())?;
        self.held_transform = Some(held_transform);
        let evidence = self.classify_contact(Some(&post_grasp_estimates))?;
        self.record_decision(
            "post_grasp_pose_confirmed",
            format!(
                "bilateral contact passed; fresh direct TOOL+PEG estimates anchored a reduced held transform with sigma {:.9e} m/{:.9e} rad and unobservable-roll bounds {:.9e} m/{:.9e} rad",
                held_transform.position_sigma_m,
                held_transform.axis_sigma_rad,
                held_transform.lateral_offset_bound_m,
                held_transform.axis_mismatch_bound_rad,
            ),
            true,
            None,
            None,
            post_grasp_estimates.values().cloned().collect(),
            Some(evidence),
        );
        Ok(post_grasp_estimates)
    }

    fn transfer_to_socket(
        &mut self,
        post_grasp_estimates: BTreeMap<u32, EstimateView>,
    ) -> Result<(), String> {
        let observed_peg = required_view(&post_grasp_estimates, PEG_OBJECT_ID)?;
        let retract_distance_m = self.scenario.fixed_head.as_ref().map_or(
            self.scenario.motion.pick_capture_start_axial_standoff_m,
            |head| head.transfer_standoff_m,
        ) - self.scenario.grasp.tool_to_peg_axial_offset_m;
        let retract_target = sub(
            self.commanded_tool_position_world_m,
            scale(observed_peg.axis_world, retract_distance_m),
        );
        let retract_delta = sub(retract_target, self.commanded_tool_position_world_m);
        if self.scenario.fixed_head.is_some() {
            self.fixed_head_transfer_segments(retract_target, post_grasp_estimates)?;
        } else {
            self.command_motion(
                retract_target,
                MotionClass::Transit,
                false,
                post_grasp_estimates.values().cloned().collect(),
            )?;
            self.predict_after_motion(retract_delta, true)?;
        }
        self.invalidate_moving_pose_priors_before_reacquisition(
            true,
            false,
            "post-grasp retraction",
        )?;
        let post_retract_roi =
            self.held_assembly_roi_world_m(self.plant.calibrated_socket_axis_world(), false)?;
        let direct_transfer_estimates = self.observe_required(
            self.phase,
            "post_retract",
            post_retract_roi,
            &[TOOL_OBJECT_ID, PEG_OBJECT_ID],
            false,
        )?;
        let direct_tool = required_view(&direct_transfer_estimates, TOOL_OBJECT_ID)?;
        let direct_peg = required_view(&direct_transfer_estimates, PEG_OBJECT_ID)?;
        let derived_peg = self.derive_held_peg(direct_tool)?;
        self.guard_view_with_limits(
            &derived_peg,
            self.scenario.estimator.transfer_position_sigma_limit_m,
            self.scenario.estimator.transfer_axis_sigma_limit_rad,
        )
        .map_err(|reason| format!("held_transform_uncertainty_{reason}"))?;
        let transform_consistency = self.guard_relative_view(
            direct_peg,
            &derived_peg,
            self.scenario.estimator.transfer_position_sigma_limit_m,
            self.scenario.estimator.transfer_axis_sigma_limit_rad,
        )?;
        let allowed_position_disagreement_m =
            derived_peg.provenance.unobservable_roll_position_bound_m
                + 3.0 * transform_consistency.position_sigma_m;
        let allowed_axis_disagreement_rad = derived_peg.provenance.unobservable_roll_axis_bound_rad
            + 3.0 * transform_consistency.axis_sigma_rad;
        if norm(sub(
            direct_peg.position_world_m,
            derived_peg.position_world_m,
        )) > allowed_position_disagreement_m
            || axis_angle_rad(direct_peg.axis_world, derived_peg.axis_world)
                > allowed_axis_disagreement_rad
        {
            return Err("held_transform_inconsistent_with_direct_reacquisition".to_owned());
        }
        let mut transfer_estimates = direct_transfer_estimates.clone();
        transfer_estimates.insert(PEG_OBJECT_ID, derived_peg.clone());
        let contact_evidence = self.classify_contact(Some(&transfer_estimates))?;
        let mut reported_estimates = direct_transfer_estimates
            .values()
            .cloned()
            .collect::<Vec<_>>();
        reported_estimates.push(derived_peg);
        self.record_decision(
            "post_retract_pose_reacquired",
            "fresh direct TOOL+PEG estimates validated the reduced held transform; the carried-pose safety estimate is derived from the fresh TOOL with transform/process/roll uncertainty",
            false,
            None,
            None,
            reported_estimates,
            Some(contact_evidence),
        );

        let socket_axis = self.plant.calibrated_socket_axis_world();
        // Stop one bounded correction outside the final insertion approach.
        // The distant point-IK transit can bow away from its endpoint chord;
        // the added axial margin keeps that unobserved motion out of the
        // socket contact region. Fresh mating-feature observations authorize
        // the subsequent local stop-and-look correction.
        let transfer_standoff_m = self.scenario.fixed_head.as_ref().map_or(
            self.scenario.motion.insert_approach_distance_m
                + self.scenario.motion.maximum_correction_m,
            |head| head.transfer_standoff_m,
        );
        let approach_target = add(
            self.scenario.coupon.socket_center_nominal_world_m,
            scale(
                socket_axis,
                0.5 * self.scenario.coupon.socket_depth_m
                    - self.peg_tip_offset_m()
                    - transfer_standoff_m
                    - self.scenario.grasp.tool_to_peg_axial_offset_m,
            ),
        );
        let transfer_delta = sub(approach_target, self.commanded_tool_position_world_m);

        let obstacles = self.plant.calibrated_planning_obstacles();
        let capabilities = self.plant.motion_capabilities();
        if capabilities.maximum_correction_velocity_m_s
            > self.scenario.motion.maximum_correction_velocity_m_s + f64::EPSILON
            || capabilities.maximum_correction_acceleration_m_s2
                > self.scenario.motion.maximum_correction_acceleration_m_s2 + f64::EPSILON
        {
            return Err("motion_capability_exceeds_configured_bound".to_owned());
        }
        let tool = required_view(&transfer_estimates, TOOL_OBJECT_ID)?;
        let peg = required_view(&transfer_estimates, PEG_OBJECT_ID)?;
        self.guard_view_with_limits(
            tool,
            self.scenario.estimator.transfer_position_sigma_limit_m,
            self.scenario.estimator.transfer_axis_sigma_limit_rad,
        )?;
        self.guard_view_with_limits(
            peg,
            self.scenario.estimator.transfer_position_sigma_limit_m,
            self.scenario.estimator.transfer_axis_sigma_limit_rad,
        )?;
        let tool_anticipated =
            self.anticipated_motion_uncertainty(tool, approach_target, MotionClass::Transit)?;
        let peg_anticipated =
            self.anticipated_motion_uncertainty(peg, approach_target, MotionClass::Transit)?;
        let held_offset_radius_m = self.held_transform.map_or(
            self.scenario.grasp.tool_to_peg_axial_offset_m,
            |transform| transform.axial_offset_m.abs() + transform.lateral_offset_bound_m,
        );
        let carried_rotation_bound_m = 2.0
            * held_offset_radius_m
            * (0.5
                * peg_anticipated
                    .commanded_tool_axis_change_bound_rad
                    .min(core::f64::consts::PI))
            .sin();
        let envelopes = [
            (
                "tool",
                SweptEnvelope {
                    center_start_world_m: tool.position_world_m,
                    center_end_world_m: add(tool.position_world_m, transfer_delta),
                    radius_m: self.scenario.safety.tool_envelope_radius_m,
                    position_sigma_m: tool_anticipated.position_sigma_m,
                    hard_position_bound_m: 0.0,
                    path_deviation_bound_m: tool_anticipated.tool_path_deviation_bound_m,
                },
            ),
            (
                "carried_peg",
                SweptEnvelope {
                    center_start_world_m: peg.position_world_m,
                    center_end_world_m: add(peg.position_world_m, transfer_delta),
                    radius_m: self.scenario.safety.carried_peg_envelope_radius_m,
                    position_sigma_m: peg_anticipated.position_sigma_m,
                    hard_position_bound_m: 0.0,
                    path_deviation_bound_m: peg_anticipated.tool_path_deviation_bound_m
                        + carried_rotation_bound_m,
                },
            ),
        ];
        let mut checks = Vec::new();
        let mut rejections = Vec::new();
        for (component, envelope) in envelopes {
            match preflight_swept_envelope(
                envelope,
                &obstacles,
                self.scenario.safety.minimum_obstacle_clearance_m,
            ) {
                Ok(component_checks) => checks.extend(component_checks),
                Err(failure) => rejections.push((component, failure)),
            }
        }
        if !rejections.is_empty() {
            // Evaluate both independently centred envelopes before choosing a
            // terminal classification. A large conservative tool bound may
            // overlap the same obstacle, but an independently proven carried
            // PEG conflict must remain explicit in the safety report.
            let invalid_geometry = rejections
                .iter()
                .any(|(_, failure)| *failure == SweptPreflightFailure::InvalidGeometry);
            let carried_conflict = rejections.iter().find_map(|(component, failure)| {
                (*component == "carried_peg").then_some(*failure).and_then(
                    |failure| match failure {
                        SweptPreflightFailure::Clearance(check) => Some(check),
                        SweptPreflightFailure::InvalidGeometry => None,
                    },
                )
            });
            let (detail, reason) = if invalid_geometry {
                (
                    "one or more estimated transfer envelopes have invalid geometry".to_owned(),
                    "impossible_geometry",
                )
            } else if let Some(check) = carried_conflict {
                (
                    format!(
                        "carried_part_collision_risk: independent carried PEG sweep conflicts with obstacle {} at {:.9e} m clearance; all component envelopes were evaluated",
                        check.obstacle_id, check.minimum_clearance_m
                    ),
                    "carried_part_collision_risk",
                )
            } else {
                let check = rejections
                    .iter()
                    .find_map(|(_, failure)| match failure {
                        SweptPreflightFailure::Clearance(check) => Some(*check),
                        SweptPreflightFailure::InvalidGeometry => None,
                    })
                    .expect("nonempty valid rejection set contains a clearance result");
                (
                    format!(
                        "estimated swept tool envelope conflicts with obstacle {} at {:.9e} m clearance",
                        check.obstacle_id, check.minimum_clearance_m
                    ),
                    "predicted_swept_geometry_collision_risk",
                )
            };
            self.record_decision(
                "reject_transfer",
                detail,
                false,
                None,
                Some(approach_target),
                transfer_estimates.values().cloned().collect(),
                None,
            );
            return Err(reason.to_owned());
        }
        let minimum_clearance_m = checks
            .iter()
            .map(|check| check.minimum_clearance_m)
            .min_by(f64::total_cmp)
            .ok_or_else(|| "calibrated_planning_obstacles_required".to_owned())?;
        self.metrics.preflight_obstacle_checks = checks.len() as u32;
        self.metrics.minimum_predicted_clearance_m = Some(minimum_clearance_m);
        self.record_decision(
            "transfer_preflight_accepted",
            format!(
                "independent tool and carried-peg sweeps used anticipated sigma up to {:.9e} m and axis sigma up to {:.9e} rad over the {:.6} s planned motion-plus-settle interval, TCP path-deviation bound {:.9e} m, and carried-offset rotation bound {:.9e} m; {} calibrated obstacle checks, minimum clearance {:.9e} m",
                tool_anticipated.position_sigma_m.max(peg_anticipated.position_sigma_m),
                tool_anticipated.axis_sigma_rad.max(peg_anticipated.axis_sigma_rad),
                tool_anticipated.interval_s.max(peg_anticipated.interval_s),
                tool_anticipated.tool_path_deviation_bound_m,
                carried_rotation_bound_m,
                checks.len(),
                minimum_clearance_m,
            ),
            false,
            None,
            Some(approach_target),
            transfer_estimates.values().cloned().collect(),
            None,
        );
        self.metrics.transfer_preflight_passed = true;
        if self.scenario.fixed_head.is_some() {
            self.fixed_head_transfer_segments(approach_target, transfer_estimates)?;
        } else {
            self.command_motion(
                approach_target,
                MotionClass::Transit,
                false,
                transfer_estimates.values().cloned().collect(),
            )?;
            self.predict_after_motion(transfer_delta, true)?;
        }
        let propagated_peg = self
            .estimator(PEG_OBJECT_ID)?
            .last_report()
            .map(estimate_view)
            .into_iter()
            .collect();
        self.metrics.transfer_completed = true;
        self.record_decision(
            "transfer_complete",
            "covariance-inflated gripper and carried-peg swept envelopes passed estimated-scene preflight; held-pose translation uncertainty was propagated",
            false,
            None,
            Some(approach_target),
            propagated_peg,
            None,
        );
        // Cartesian actuation controls translation only; the distal axis
        // changes with tendon-arm IK and this reduced estimator has no honest
        // commanded-orientation transition model.  Preserve the propagated
        // estimate in the report above, then explicitly cold-reacquire instead
        // of applying a mathematically false orientation innovation gate.
        self.invalidate_moving_pose_priors_before_reacquisition(true, false, "socket transfer")?;
        self.record_decision(
            "begin_socket_reacquisition",
            if self.scenario.fixed_head.is_some() {
                "fresh mating features are required after transfer; command endpoints do not replace measured axes"
            } else {
                "translation-only plant has no calibrated axis-transition model; reset prior before fitting fresh mating features"
            },
            false,
            None,
            None,
            Vec::new(),
            None,
        );
        Ok(())
    }

    fn fixed_head_transfer_segments(
        &mut self,
        target: [f64; 3],
        mut estimates: BTreeMap<u32, EstimateView>,
    ) -> Result<(), String> {
        let start = self.commanded_tool_position_world_m;
        let full_delta = sub(target, start);
        let segments = (norm(full_delta) / self.scenario.motion.maximum_correction_m)
            .ceil()
            .max(1.0) as u32;
        if segments > 128 {
            return Err("transfer_segment_budget_exhausted".to_owned());
        }
        for segment in 1..=segments {
            let next = add(
                start,
                scale(full_delta, f64::from(segment) / f64::from(segments)),
            );
            let delta = sub(next, self.commanded_tool_position_world_m);
            self.command_motion(
                next,
                MotionClass::Transit,
                false,
                estimates.values().cloned().collect(),
            )?;
            self.predict_after_motion(delta, true)?;
            self.invalidate_moving_pose_priors_before_reacquisition(
                true,
                false,
                "fixed-head transfer segment",
            )?;
            estimates = self.observe_required(
                self.phase,
                "transfer_segment",
                next,
                &[TOOL_OBJECT_ID, PEG_OBJECT_ID],
                false,
            )?;
        }
        Ok(())
    }

    fn stop_and_look_socket_alignment(&mut self) -> Result<BTreeMap<u32, EstimateView>, String> {
        for iteration in 0..=self.scenario.motion.maximum_correction_iterations {
            // Center the finite macro field between the commanded held-part
            // position and calibrated socket datum.  This is commanded state,
            // not latent peg/socket truth, and keeps both mating feature sets
            // observable at the pre-insertion standoff.
            let roi_axis = self
                .last_socket_axis_world
                .unwrap_or_else(|| self.plant.calibrated_socket_axis_world());
            let roi = self.held_assembly_roi_world_m(roi_axis, true)?;
            let estimates = self.observe_required(
                self.phase,
                "socket",
                roi,
                &[PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID],
                true,
            )?;
            let peg = required_view(&estimates, PEG_OBJECT_ID)?;
            let socket = required_view(&estimates, SOCKET_OBJECT_ID)?;
            self.guard_relative_view(
                peg,
                socket,
                self.scenario.estimator.insertion_position_sigma_limit_m,
                self.scenario.estimator.axis_sigma_limit_rad,
            )?;
            if self.correct_observed_axis(peg, socket, iteration, &estimates)? {
                continue;
            }
            let axis_angle = axis_angle_rad(peg.axis_world, socket.axis_world);
            if axis_angle > self.scenario.contact.seat_axis_tolerance_rad {
                return Err("impossible_mating_axis_geometry".to_owned());
            }
            self.last_socket_axis_world = Some(socket.axis_world);
            let desired_peg = sub(
                add(
                    socket.position_world_m,
                    scale(
                        socket.axis_world,
                        0.5 * self.scenario.coupon.socket_depth_m
                            - self.scenario.motion.insert_approach_distance_m,
                    ),
                ),
                scale(peg.axis_world, self.peg_tip_offset_m()),
            );
            let bounded_desired = if self.scenario.fixed_head.is_some() {
                let error = sub(desired_peg, peg.position_world_m);
                let fraction = (0.95 * self.scenario.motion.maximum_correction_m
                    / norm(error).max(f64::EPSILON))
                .min(1.0);
                add(peg.position_world_m, scale(error, fraction))
            } else {
                desired_peg
            };
            let decision = if self.scenario.fixed_head.is_some() {
                super::controller::decide_quantized_correction(
                    peg.position_world_m,
                    bounded_desired,
                    self.correction_policy(),
                )
            } else {
                decide_correction(
                    peg.position_world_m,
                    bounded_desired,
                    self.correction_policy(),
                )
            };
            match decision {
                CorrectionDecision::Converged { residual_m } => {
                    self.corrections.push(CorrectionIterationRecord {
                        phase: self.phase,
                        iteration,
                        decision_tick: self.plant.now_tick(),
                        measurement_age_s: measurement_age_s(
                            self.plant.now_tick(),
                            peg,
                            self.plant.fixed_dt_s(),
                        ),
                        residual_before_m: residual_m,
                        requested_correction_world_m: None,
                        requested_correction_m: 0.0,
                        estimator_position_sigma_m: peg.position_sigma_m,
                        estimator_axis_sigma_rad: peg.axis_sigma_rad,
                        outcome: "converged",
                    });
                    self.metrics.successful_corrections += 1;
                    self.record_decision(
                        "correction_converged",
                        format!(
                            "socket correction residual {residual_m:.9e} m is within threshold"
                        ),
                        true,
                        None,
                        None,
                        estimates.values().cloned().collect(),
                        None,
                    );
                    return Ok(estimates);
                }
                CorrectionDecision::Command {
                    residual_before_m,
                    correction_world_m,
                    ..
                } => {
                    if iteration == self.scenario.motion.maximum_correction_iterations {
                        self.corrections.push(CorrectionIterationRecord {
                            phase: self.phase,
                            iteration,
                            decision_tick: self.plant.now_tick(),
                            measurement_age_s: measurement_age_s(
                                self.plant.now_tick(),
                                peg,
                                self.plant.fixed_dt_s(),
                            ),
                            residual_before_m,
                            requested_correction_world_m: Some(correction_world_m),
                            requested_correction_m: norm(correction_world_m),
                            estimator_position_sigma_m: peg.position_sigma_m,
                            estimator_axis_sigma_rad: peg.axis_sigma_rad,
                            outcome: "iteration_budget_exhausted",
                        });
                        return Err("correction_non_convergence".to_owned());
                    }
                    self.metrics.correction_iterations += 1;
                    self.corrections.push(CorrectionIterationRecord {
                        phase: self.phase,
                        iteration,
                        decision_tick: self.plant.now_tick(),
                        measurement_age_s: measurement_age_s(
                            self.plant.now_tick(),
                            peg,
                            self.plant.fixed_dt_s(),
                        ),
                        residual_before_m,
                        requested_correction_world_m: Some(correction_world_m),
                        requested_correction_m: norm(correction_world_m),
                        estimator_position_sigma_m: peg.position_sigma_m,
                        estimator_axis_sigma_rad: peg.axis_sigma_rad,
                        outcome: "commanded",
                    });
                    let target = add(self.commanded_tool_position_world_m, correction_world_m);
                    self.command_motion(
                        target,
                        MotionClass::Correction,
                        true,
                        estimates.values().cloned().collect(),
                    )?;
                    self.predict_after_motion(correction_world_m, true)?;
                    self.invalidate_moving_pose_priors_before_reacquisition(
                        true,
                        true,
                        "socket correction",
                    )?;
                }
                CorrectionDecision::Rejected {
                    reason,
                    requested_magnitude_m,
                } => {
                    self.corrections.push(CorrectionIterationRecord {
                        phase: self.phase,
                        iteration,
                        decision_tick: self.plant.now_tick(),
                        measurement_age_s: measurement_age_s(
                            self.plant.now_tick(),
                            peg,
                            self.plant.fixed_dt_s(),
                        ),
                        residual_before_m: norm(sub(desired_peg, peg.position_world_m)),
                        requested_correction_world_m: None,
                        requested_correction_m: requested_magnitude_m,
                        estimator_position_sigma_m: peg.position_sigma_m,
                        estimator_axis_sigma_rad: peg.axis_sigma_rad,
                        outcome: reason,
                    });
                    return Err(reason.to_owned());
                }
            }
        }
        Err("correction_non_convergence".to_owned())
    }

    fn perform_guarded_insertion(
        &mut self,
        mut estimates: BTreeMap<u32, EstimateView>,
    ) -> Result<(), String> {
        for increment in 0..self.scenario.motion.maximum_insertion_increments {
            if increment > 0 {
                let insertion_roi = self.held_assembly_roi_world_m(
                    self.last_socket_axis_world
                        .unwrap_or_else(|| self.plant.calibrated_socket_axis_world()),
                    true,
                )?;
                estimates = self.observe_required(
                    self.phase,
                    "socket",
                    insertion_roi,
                    &[PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID],
                    true,
                )?;
            }
            let peg = required_view(&estimates, PEG_OBJECT_ID)?;
            let socket = required_view(&estimates, SOCKET_OBJECT_ID)?;
            self.guard_view(
                peg,
                self.scenario.estimator.insertion_position_sigma_limit_m,
            )?;
            self.guard_view(
                socket,
                self.scenario.estimator.insertion_position_sigma_limit_m,
            )?;
            self.guard_relative_view(
                peg,
                socket,
                self.scenario.estimator.insertion_position_sigma_limit_m,
                self.scenario.estimator.axis_sigma_limit_rad,
            )?;
            if axis_angle_rad(peg.axis_world, socket.axis_world)
                > self.scenario.contact.seat_axis_tolerance_rad
            {
                return Err("impossible_mating_axis_geometry".to_owned());
            }
            let axis = socket.axis_world;
            self.last_socket_axis_world = Some(axis);
            let peg_tip_world_m = add(
                peg.position_world_m,
                scale(peg.axis_world, self.peg_tip_offset_m()),
            );
            let socket_seat_world_m = add(
                socket.position_world_m,
                scale(socket.axis_world, 0.5 * self.scenario.coupon.socket_depth_m),
            );
            let to_seat = sub(socket_seat_world_m, peg_tip_world_m);
            let axial_remaining_m = dot(to_seat, axis);
            let lateral = sub(to_seat, scale(axis, axial_remaining_m));
            if axial_remaining_m < -self.scenario.contact.seat_axial_tolerance_m {
                return Err("impossible_insertion_geometry".to_owned());
            }
            let lateral_correction = scale(lateral, self.scenario.motion.correction_gain);
            let lateral_step_m = norm(lateral_correction);
            if lateral_step_m > self.scenario.motion.insertion_increment_m {
                return Err("correction_magnitude_limit".to_owned());
            }
            let maximum_axial_step_m = (self.scenario.motion.insertion_increment_m.powi(2)
                - lateral_step_m.powi(2))
            .max(0.0)
            .sqrt();
            let axial_step_m = axial_remaining_m.max(0.0).min(maximum_axial_step_m);
            let correction = add(scale(axis, axial_step_m), lateral_correction);
            if norm(correction) > self.scenario.motion.maximum_correction_m {
                return Err("correction_magnitude_limit".to_owned());
            }
            if axial_step_m <= self.scenario.contact.seat_axial_tolerance_m
                && norm(lateral) <= self.scenario.contact.seat_lateral_tolerance_m
            {
                let evidence = self.classify_contact(Some(&estimates))?;
                if evidence.state == ContactState::Seated {
                    self.metrics.guarded_insertion_confirmed = true;
                    self.record_decision(
                        "insertion_seat_candidate",
                        "fresh observed relative pose and contact packet indicate seating",
                        true,
                        None,
                        None,
                        estimates.values().cloned().collect(),
                        Some(evidence),
                    );
                    return Ok(());
                }
            }
            let target = add(self.commanded_tool_position_world_m, correction);
            self.command_motion(
                target,
                MotionClass::Insertion,
                true,
                estimates.values().cloned().collect(),
            )?;
            self.predict_after_motion(correction, true)?;
            self.invalidate_moving_pose_priors_before_reacquisition(
                true,
                true,
                "guarded insertion increment",
            )?;
            self.metrics.insertion_increment_count += 1;
            let contact_roi = self.held_assembly_roi_world_m(axis, true)?;
            let contact_estimates = self.observe_required(
                self.phase,
                "insertion_contact",
                contact_roi,
                &[PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID],
                true,
            )?;
            let evidence = self.classify_contact(Some(&contact_estimates))?;
            self.record_decision(
                "guarded_insertion_increment",
                format!(
                    "increment {}: observed axial remaining {:.9e} m, lateral {:.9e} m",
                    increment,
                    axial_remaining_m,
                    norm(lateral)
                ),
                true,
                None,
                Some(target),
                contact_estimates.values().cloned().collect(),
                Some(evidence),
            );
            match evidence.state {
                ContactState::Jammed => return Err("insertion_jam".to_owned()),
                ContactState::ExcessiveInterference => {
                    return Err("excessive_insertion_interference".to_owned())
                }
                _ => {}
            }
            if evidence.state == ContactState::RecoverableLateralContact {
                self.insertion_recovery_count += 1;
                self.metrics.recovery_count += 1;
                self.recover_insertion(axis)?;
            }
            if evidence.state == ContactState::Seated {
                self.metrics.guarded_insertion_confirmed = true;
                self.record_decision(
                    "insertion_seat_candidate",
                    "bounded insertion increment followed by fresh optical and contact evidence indicating seating",
                    true,
                    None,
                    Some(target),
                    contact_estimates.values().cloned().collect(),
                    Some(evidence),
                );
                return Ok(());
            }
        }
        Err("insertion_increment_budget_exhausted".to_owned())
    }

    fn recover_insertion(&mut self, socket_axis: [f64; 3]) -> Result<(), String> {
        if self.insertion_recovery_count > self.scenario.safety.maximum_phase_retries {
            return Err("insertion_retry_exhausted".to_owned());
        }
        let roi = self.held_assembly_roi_world_m(socket_axis, true)?;
        let estimates = self.observe_required(
            self.phase,
            "insertion_recovery",
            roi,
            &[PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID],
            true,
        )?;
        let retreat = scale(socket_axis, -self.scenario.safety.retreat_distance_m);
        let target = add(self.commanded_tool_position_world_m, retreat);
        self.command_motion(
            target,
            MotionClass::Retreat,
            true,
            estimates.values().cloned().collect(),
        )?;
        self.predict_after_motion(retreat, true)?;
        self.invalidate_moving_pose_priors_before_reacquisition(
            true,
            true,
            "recoverable-contact retreat",
        )?;
        let post_retreat_estimates = self.observe_required(
            self.phase,
            "insertion_recovery_contact",
            roi,
            &[PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID],
            true,
        )?;
        let evidence = self.classify_contact(Some(&post_retreat_estimates))?;
        self.record_decision(
            "recoverable_contact_retreat",
            "contact proxy requested force unload and optical reacquisition",
            true,
            None,
            Some(target),
            post_retreat_estimates.values().cloned().collect(),
            Some(evidence),
        );
        Ok(())
    }

    fn verify_seated(&mut self) -> Result<BTreeMap<u32, EstimateView>, String> {
        let roi_axis = self
            .last_socket_axis_world
            .unwrap_or_else(|| self.plant.calibrated_socket_axis_world());
        let seat_roi = self.held_assembly_roi_world_m(roi_axis, true)?;
        let estimates = self.observe_required(
            self.phase,
            "socket",
            seat_roi,
            &[PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID],
            true,
        )?;
        let peg = required_view(&estimates, PEG_OBJECT_ID)?;
        let socket = required_view(&estimates, SOCKET_OBJECT_ID)?;
        self.guard_relative_view(
            peg,
            socket,
            self.scenario.estimator.insertion_position_sigma_limit_m,
            self.scenario.estimator.axis_sigma_limit_rad,
        )?;
        let delta = sub(
            add(
                peg.position_world_m,
                scale(peg.axis_world, self.peg_tip_offset_m()),
            ),
            add(
                socket.position_world_m,
                scale(socket.axis_world, 0.5 * self.scenario.coupon.socket_depth_m),
            ),
        );
        let axial_error_m = dot(delta, socket.axis_world).abs();
        let lateral_error_m = norm(sub(
            delta,
            scale(socket.axis_world, dot(delta, socket.axis_world)),
        ));
        let axis_error_rad = axis_angle_rad(peg.axis_world, socket.axis_world);
        let evidence = self.classify_contact(Some(&estimates))?;
        if evidence.state != ContactState::Seated
            || axial_error_m > self.scenario.contact.seat_axial_tolerance_m
            || lateral_error_m > self.scenario.contact.seat_lateral_tolerance_m
            || axis_error_rad > self.scenario.contact.seat_axis_tolerance_rad
        {
            return Err("seating_not_verified".to_owned());
        }
        self.metrics.seated_from_observation_and_contact = true;
        self.record_decision(
            "seat_verified",
            format!(
                "observed axial {:.9e} m, lateral {:.9e} m, and axis {:.9e} rad plus seated contact packet passed",
                axial_error_m, lateral_error_m, axis_error_rad
            ),
            true,
            None,
            None,
            estimates.values().cloned().collect(),
            Some(evidence),
        );
        Ok(estimates)
    }

    fn release(&mut self, seated_estimates: &BTreeMap<u32, EstimateView>) -> Result<(), String> {
        if !self.metrics.seated_from_observation_and_contact {
            return Err("release_without_verified_seat".to_owned());
        }
        let seated_evidence = self.classify_contact(Some(seated_estimates))?;
        if seated_evidence.state != ContactState::Seated {
            return Err("release_without_verified_seat".to_owned());
        }
        self.plant.commit_release().map_err(plant_reason)?;
        // The reduced transform is valid only while the grasp transaction is
        // active. Detachment invalidates it immediately; every post-release
        // PEG claim below must come from direct optical features.
        self.grasp_confirmed = false;
        self.held_transform = None;
        self.record_decision(
            "release_committed",
            "tip-to-seat observation plus contact authorized physical detachment; the held transform was invalidated before jaw opening",
            true,
            None,
            None,
            seated_estimates.values().cloned().collect(),
            Some(seated_evidence),
        );
        let receipt = self
            .plant
            .command_gripper(self.plant.maximum_gripper_opening_m())
            .map_err(plant_reason)?;
        self.record_decision(
            "open_gripper_after_release",
            "authoritative runtime accepted the bounded jaw-opening command after detachment",
            true,
            Some(receipt.command_sequence),
            None,
            seated_estimates.values().cloned().collect(),
            Some(seated_evidence),
        );
        self.advance_until_idle_with_interlock_report()?;
        let release_axis = required_view(seated_estimates, SOCKET_OBJECT_ID)?.axis_world;
        let release_roi = self.seated_assembly_roi_world_m(release_axis)?;
        let release_estimates = self.observe_required(
            self.phase,
            "post_release_contact",
            release_roi,
            &[PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID],
            true,
        )?;
        let evidence = self.classify_contact(Some(&release_estimates))?;
        if evidence.bilateral_jaw_contact || evidence.grip_force_proxy_n > 0.0 {
            return Err("release_contact_not_cleared".to_owned());
        }
        self.metrics.release_confirmed = true;
        self.record_decision(
            "release_confirmed",
            "jaw contact and grip-force proxy cleared after observed seating",
            true,
            None,
            None,
            release_estimates.values().cloned().collect(),
            Some(evidence),
        );
        Ok(())
    }

    fn retreat_and_reobserve(&mut self) -> Result<(), String> {
        // Jaw opening is longer than the observation freshness budget. Acquire
        // and guard tool plus seated mating features again before authorizing
        // the near-contact retreat; a cached socket axis is not sufficient.
        self.estimator_mut(PEG_OBJECT_ID)?.reset();
        self.estimator_mut(SOCKET_OBJECT_ID)?.reset();
        self.estimator_mut(TOOL_OBJECT_ID)?.reset();
        let pre_retreat_axis = self
            .last_socket_axis_world
            .unwrap_or_else(|| self.plant.calibrated_socket_axis_world());
        let pre_retreat_roi = self.seated_assembly_roi_world_m(pre_retreat_axis)?;
        let pre_retreat_estimates = self.observe_required(
            self.phase,
            "pre_retreat",
            pre_retreat_roi,
            &[PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID],
            true,
        )?;
        let peg = required_view(&pre_retreat_estimates, PEG_OBJECT_ID)?;
        let socket = required_view(&pre_retreat_estimates, SOCKET_OBJECT_ID)?;
        let relative = self.guard_relative_view(
            peg,
            socket,
            self.scenario.estimator.insertion_position_sigma_limit_m,
            self.scenario.estimator.axis_sigma_limit_rad,
        )?;
        let relative_position = sub(
            add(
                peg.position_world_m,
                scale(peg.axis_world, self.peg_tip_offset_m()),
            ),
            add(
                socket.position_world_m,
                scale(socket.axis_world, 0.5 * self.scenario.coupon.socket_depth_m),
            ),
        );
        let axial_error_m = dot(relative_position, socket.axis_world).abs();
        let lateral_error_m = norm(sub(
            relative_position,
            scale(socket.axis_world, dot(relative_position, socket.axis_world)),
        ));
        let axis_error_rad = axis_angle_rad(peg.axis_world, socket.axis_world);
        if axial_error_m > self.scenario.contact.seat_axial_tolerance_m
            || lateral_error_m > self.scenario.contact.seat_lateral_tolerance_m
            || axis_error_rad > self.scenario.contact.seat_axis_tolerance_rad
        {
            return Err("post_release_seat_observation_invalid".to_owned());
        }
        let axis = socket.axis_world;
        let contact_evidence = self.classify_contact(Some(&pre_retreat_estimates))?;
        self.record_decision(
            "pre_retreat_pose_reacquired",
            format!(
                "fresh TOOL+PEG+SOCKET burst passed; relative sigma {:.9e} m/{:.9e} rad before retreat",
                relative.position_sigma_m, relative.axis_sigma_rad,
            ),
            true,
            None,
            None,
            pre_retreat_estimates.values().cloned().collect(),
            Some(contact_evidence),
        );
        let retreat = scale(axis, -self.scenario.motion.insert_approach_distance_m);
        let target = add(self.commanded_tool_position_world_m, retreat);
        self.command_motion(
            target,
            MotionClass::Retreat,
            true,
            pre_retreat_estimates.values().cloned().collect(),
        )?;
        self.predict_after_motion(retreat, false)?;
        self.invalidate_moving_pose_priors_before_reacquisition(
            false,
            true,
            "post-release retreat",
        )?;
        let retreat_roi = self.seated_assembly_roi_world_m(axis)?;
        let estimates = self.observe_required(
            self.phase,
            "socket",
            retreat_roi,
            &[PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID],
            false,
        )?;
        let peg = required_view(&estimates, PEG_OBJECT_ID)?;
        let socket = required_view(&estimates, SOCKET_OBJECT_ID)?;
        let tool = required_view(&estimates, TOOL_OBJECT_ID)?;
        self.guard_relative_view(
            peg,
            socket,
            self.scenario.estimator.insertion_position_sigma_limit_m,
            self.scenario.estimator.axis_sigma_limit_rad,
        )?;
        let peg_socket_delta = sub(
            add(
                peg.position_world_m,
                scale(peg.axis_world, self.peg_tip_offset_m()),
            ),
            add(
                socket.position_world_m,
                scale(socket.axis_world, 0.5 * self.scenario.coupon.socket_depth_m),
            ),
        );
        let axial_seat_error_m = dot(peg_socket_delta, socket.axis_world).abs();
        let lateral_seat_error_m = norm(sub(
            peg_socket_delta,
            scale(socket.axis_world, dot(peg_socket_delta, socket.axis_world)),
        ));
        let seat_axis_error_rad = axis_angle_rad(peg.axis_world, socket.axis_world);
        let tool_clearance = norm(sub(tool.position_world_m, peg.position_world_m));
        if axial_seat_error_m > self.scenario.contact.seat_axial_tolerance_m
            || lateral_seat_error_m > self.scenario.contact.seat_lateral_tolerance_m
            || seat_axis_error_rad > self.scenario.contact.seat_axis_tolerance_rad
            || tool_clearance < self.scenario.safety.retreat_distance_m
        {
            return Err("post_release_retreat_verification_failed".to_owned());
        }
        self.metrics.retreat_confirmed = true;
        let evidence = self.classify_contact(Some(&estimates))?;
        self.record_decision(
            "retreat_verified",
            format!(
                "post-release observations retain the peg at the socket (axial {:.9e} m, lateral {:.9e} m, axis {:.9e} rad) and separate the tool by {:.9e} m",
                axial_seat_error_m,
                lateral_seat_error_m,
                seat_axis_error_rad,
                tool_clearance,
            ),
            false,
            None,
            Some(target),
            estimates.values().cloned().collect(),
            Some(evidence),
        );
        Ok(())
    }

    fn observe_required(
        &mut self,
        phase: ControlPhase,
        roi_name: &'static str,
        roi_world_m: [f64; 3],
        object_ids: &[u32],
        near_contact: bool,
    ) -> Result<BTreeMap<u32, EstimateView>, String> {
        let max_attempts = self.scenario.safety.maximum_phase_retries + 1;
        let mut last_reason = "loss_of_observability".to_owned();
        for attempt in 0..max_attempts {
            let stop = self.plant.command_stop().map_err(plant_reason)?;
            self.plant
                .advance_ticks(self.plant.motion_capabilities().settling_ticks)
                .map_err(plant_reason)?;
            self.record_decision(
                "hold_and_settle",
                format!(
                    "stopped motion and satisfied settling interval before burst attempt {attempt}"
                ),
                near_contact,
                Some(stop.command_sequence),
                None,
                Vec::new(),
                None,
            );
            let burst = match self
                .plant
                .acquire_observation_burst(object_ids, roi_world_m)
            {
                Ok(burst) => burst,
                Err(error) => {
                    prefer_failure_reason(&mut last_reason, plant_reason(error));
                    if attempt + 1 < max_attempts {
                        self.metrics.recovery_count += 1;
                        continue;
                    }
                    return Err(last_reason);
                }
            };
            self.metrics.observation_bursts += 1;
            if !burst.calibration_reference_valid {
                let candidate_reason = if burst.calibration_reference_sample_count
                    < self.scenario.optics.minimum_calibration_reference_samples
                {
                    "calibration_reference_unobservable"
                } else {
                    "excessive_calibration_bias"
                };
                prefer_failure_reason(&mut last_reason, candidate_reason.to_owned());
                self.record_burst(
                    phase,
                    roi_name,
                    &burst,
                    Vec::new(),
                    vec![format!(
                        "{}:residual_m={:?}:samples={}/{}:limit_m={:.9e}",
                        candidate_reason,
                        burst.calibration_reference_residual_m,
                        burst.calibration_reference_sample_count,
                        self.scenario.optics.minimum_calibration_reference_samples,
                        self.scenario
                            .optics
                            .maximum_calibration_reference_residual_m,
                    )],
                );
                if attempt + 1 < max_attempts {
                    self.metrics.recovery_count += 1;
                    continue;
                }
                return Err(last_reason);
            }

            let mut views = BTreeMap::new();
            let mut accepted_ids = Vec::new();
            let mut rejection_reasons = burst
                .missing
                .iter()
                .map(|missing| {
                    format!(
                        "object_{}_feature_{}:{}",
                        missing.object_id, missing.feature_id, missing.reason
                    )
                })
                .collect::<Vec<_>>();
            for &object_id in object_ids {
                let now_tick = self.plant.now_tick();
                let mut report = self
                    .estimator_mut(object_id)?
                    .update(now_tick, &burst.measurements);
                if report.is_valid() {
                    if let Some(reference_residual_m) = burst.calibration_reference_residual_m {
                        // The one-point reference constrains translation only.
                        // Treat its residual as a conservative three-sigma
                        // common-bias envelope and apply it once per burst.
                        let additional_sigma_m = reference_residual_m / 3.0;
                        report = self
                            .estimator_mut(object_id)?
                            .inflate_translation_uncertainty(
                                now_tick,
                                [additional_sigma_m; 3],
                                "accepted macro calibration-reference residual",
                            );
                    }
                }
                let view = estimate_view(&report);
                let position_limit = match phase {
                    ControlPhase::GuardedGrasp => {
                        self.scenario.estimator.grasp_position_sigma_limit_m
                    }
                    ControlPhase::Transfer => {
                        self.scenario.estimator.transfer_position_sigma_limit_m
                    }
                    ControlPhase::GuardedInsertion | ControlPhase::SeatVerification => {
                        self.scenario.estimator.insertion_position_sigma_limit_m
                    }
                    _ => self.scenario.estimator.macro_position_sigma_limit_m,
                };
                let axis_limit = if phase == ControlPhase::Transfer {
                    self.scenario.estimator.transfer_axis_sigma_limit_rad
                } else {
                    self.scenario.estimator.axis_sigma_limit_rad
                };
                let accepted_by_controller =
                    match self.guard_view_with_limits(&view, position_limit, axis_limit) {
                        Ok(()) => {
                            accepted_ids.push(object_id);
                            self.metrics.accepted_pose_estimates += 1;
                            self.metrics.maximum_accepted_position_sigma_m = self
                                .metrics
                                .maximum_accepted_position_sigma_m
                                .max(view.position_sigma_m);
                            self.metrics.maximum_accepted_axis_sigma_rad = self
                                .metrics
                                .maximum_accepted_axis_sigma_rad
                                .max(view.axis_sigma_rad);
                            true
                        }
                        Err(reason) => {
                            let correction_commanded_this_phase =
                                self.corrections.iter().any(|record| {
                                    record.phase == phase && record.outcome == "commanded"
                                });
                            let candidate_reason = map_estimate_failure(
                                phase,
                                correction_commanded_this_phase,
                                object_id,
                                &report,
                                &burst,
                                reason,
                            );
                            rejection_reasons
                                .push(format!("object_{object_id}:{candidate_reason}"));
                            prefer_failure_reason(&mut last_reason, candidate_reason);
                            false
                        }
                    };
                self.uncertainty_guards.push(UncertaintyGuardRecord {
                    sequence: self.uncertainty_guards.len() as u32,
                    tick: now_tick,
                    phase,
                    kind: "absolute_pose",
                    object_ids: vec![object_id],
                    position_sigma_m: view.valid.then_some(view.position_sigma_m),
                    axis_sigma_rad: view.valid.then_some(view.axis_sigma_rad),
                    position_limit_m: position_limit,
                    axis_limit_rad: axis_limit,
                    passed: accepted_by_controller,
                });
                self.estimator_updates.push(EstimatorUpdateRecord {
                    sequence: self.estimator_updates.len() as u32,
                    phase,
                    update_kind: "optical_burst",
                    burst_sequence: Some(burst.sequence),
                    applied_position_sigma_limit_m: position_limit,
                    applied_axis_sigma_limit_rad: axis_limit,
                    accepted_by_controller,
                    estimate: report,
                });
                views.insert(object_id, view);
            }
            self.record_burst(
                phase,
                roi_name,
                &burst,
                accepted_ids.clone(),
                rejection_reasons,
            );
            if accepted_ids.len() == object_ids.len() {
                if object_ids.contains(&TOOL_OBJECT_ID)
                    && (!self.grasp_confirmed || object_ids.contains(&PEG_OBJECT_ID))
                {
                    self.moving_pose_reacquisition_required = false;
                }
                return Ok(views);
            }
            self.record_decision(
                "hold_for_reacquisition",
                format!("observation attempt {attempt} rejected: {last_reason}"),
                near_contact,
                None,
                None,
                views.values().cloned().collect(),
                None,
            );
            if attempt + 1 < max_attempts {
                self.metrics.recovery_count += 1;
            }
        }
        Err(last_reason)
    }

    fn record_burst(
        &mut self,
        phase: ControlPhase,
        roi: &'static str,
        burst: &ObservationBurst,
        accepted_object_ids: Vec<u32>,
        rejection_reasons: Vec<String>,
    ) {
        self.observations.push(ObservationBurstRecord {
            sequence: burst.sequence,
            phase,
            roi,
            capture_start_tick: burst.capture_start_tick,
            capture_end_tick: burst.capture_end_tick,
            available_tick: burst.available_tick,
            requested_object_ids: burst.requested_object_ids.clone(),
            observed_feature_count: burst.measurements.len() as u32,
            missing_feature_count: burst.missing.len() as u32,
            accepted_object_ids,
            rejection_reasons,
            triangulation_head_count: burst.triangulation_head_count,
            calibrated_rays_per_observed_point: burst.calibrated_rays_per_observed_point,
            calibration_reference_residual_m: burst.calibration_reference_residual_m,
            calibration_reference_sample_count: burst.calibration_reference_sample_count,
            required_calibration_reference_sample_count: self
                .scenario
                .optics
                .minimum_calibration_reference_samples,
            maximum_calibration_reference_residual_m: self
                .scenario
                .optics
                .maximum_calibration_reference_residual_m,
            calibration_reference_valid: burst.calibration_reference_valid,
        });
    }

    fn guard_view(&self, view: &EstimateView, position_limit_m: f64) -> Result<(), String> {
        self.guard_view_with_limits(
            view,
            position_limit_m,
            self.scenario.estimator.axis_sigma_limit_rad,
        )
    }

    fn guard_view_with_limits(
        &self,
        view: &EstimateView,
        position_limit_m: f64,
        axis_limit_rad: f64,
    ) -> Result<(), String> {
        guard_estimate(
            self.plant.now_tick(),
            view,
            EstimateGate {
                maximum_age_ticks: seconds_to_ticks_floor(
                    self.scenario.optics.maximum_measurement_age_s,
                    self.plant.fixed_dt_s(),
                ),
                maximum_position_sigma_m: position_limit_m,
                maximum_axis_sigma_rad: axis_limit_rad,
                minimum_distinct_features: self.scenario.optics.minimum_distinct_features,
                minimum_triangulation_heads: self.scenario.optics.minimum_triangulation_heads,
                minimum_calibrated_rays_per_point: self
                    .scenario
                    .optics
                    .minimum_calibrated_rays_per_point,
                maximum_residual_m: self.scenario.estimator.maximum_feature_residual_m,
            },
        )
        .map_err(|failure| match failure {
            EstimateGuardFailure::StaleMeasurement => "stale_measurement".to_owned(),
            EstimateGuardFailure::MissingRequiredFeature => "missing_required_feature".to_owned(),
            other => format!("estimate_guard_{other:?}").to_ascii_lowercase(),
        })
    }

    fn guard_relative_view(
        &mut self,
        first: &EstimateView,
        second: &EstimateView,
        position_limit_m: f64,
        axis_limit_rad: f64,
    ) -> Result<RelativeEstimateUncertainty, String> {
        let result = guard_relative_estimates(first, second, position_limit_m, axis_limit_rad);
        let (position_sigma_m, axis_sigma_rad) = if first.valid && second.valid {
            (
                Some(first.position_sigma_m + second.position_sigma_m),
                Some(first.axis_sigma_rad + second.axis_sigma_rad),
            )
        } else {
            (None, None)
        };
        self.uncertainty_guards.push(UncertaintyGuardRecord {
            sequence: self.uncertainty_guards.len() as u32,
            tick: self.plant.now_tick(),
            phase: self.phase,
            kind: "relative_pose",
            object_ids: vec![first.object_id, second.object_id],
            position_sigma_m,
            axis_sigma_rad,
            position_limit_m,
            axis_limit_rad,
            passed: result.is_ok(),
        });
        result
            .map_err(|failure| format!("relative_estimate_guard_{failure:?}").to_ascii_lowercase())
    }

    fn derive_held_peg(&self, fresh_tool: &EstimateView) -> Result<EstimateView, String> {
        if !self.grasp_confirmed {
            return Err("held_transform_without_confirmed_grasp".to_owned());
        }
        let transform = self
            .held_transform
            .ok_or_else(|| "held_transform_required".to_owned())?;
        derive_held_peg_from_tool(
            PEG_OBJECT_ID,
            transform,
            fresh_tool,
            self.plant.fixed_dt_s(),
            self.scenario
                .estimator
                .loaded_hold_process_sigma_m_per_sqrt_s,
            self.scenario.coupon.minimum_feature_axial_span_m,
        )
        .map_err(|failure| format!("held_transform_{failure:?}").to_ascii_lowercase())
    }

    fn peg_tip_offset_m(&self) -> f64 {
        self.scenario.coupon.peg_half_segment_m + 0.5 * self.scenario.coupon.peg_diameter_m
    }

    /// Aim the finite macro field from commanded state and calibrated feature
    /// stations. `peg_center_world_m` is either the held kinematic hypothesis
    /// `T + axis * tool_to_peg_offset` or the fixed B-center seat datum; neither
    /// source contains plant truth.
    fn assembly_roi_world_m(
        &self,
        axis_world: [f64; 3],
        peg_center_world_m: [f64; 3],
        include_socket: bool,
    ) -> Result<[f64; 3], String> {
        if axis_world.iter().any(|value| !value.is_finite())
            || (norm(axis_world) - 1.0).abs() > 1.0e-6
        {
            return Err("invalid_calibrated_roi_axis".to_owned());
        }
        let mut candidates = Vec::new();
        for feature in self.plant.feature_model(TOOL_OBJECT_ID) {
            candidates.push(add(
                self.commanded_tool_position_world_m,
                scale(axis_world, feature.axial_coordinate_m),
            ));
        }
        for feature in self.plant.feature_model(PEG_OBJECT_ID) {
            candidates.push(add(
                peg_center_world_m,
                scale(axis_world, feature.axial_coordinate_m),
            ));
        }
        if include_socket {
            for feature in self.plant.feature_model(SOCKET_OBJECT_ID) {
                candidates.push(add(
                    self.scenario.coupon.socket_center_nominal_world_m,
                    scale(axis_world, feature.axial_coordinate_m),
                ));
            }
        }
        let minimum = candidates
            .iter()
            .min_by(|a, b| dot(**a, axis_world).total_cmp(&dot(**b, axis_world)))
            .copied()
            .ok_or_else(|| "calibrated_roi_has_no_features".to_owned())?;
        let maximum = candidates
            .iter()
            .max_by(|a, b| dot(**a, axis_world).total_cmp(&dot(**b, axis_world)))
            .copied()
            .ok_or_else(|| "calibrated_roi_has_no_features".to_owned())?;
        Ok(scale(add(minimum, maximum), 0.5))
    }

    fn held_assembly_roi_world_m(
        &self,
        axis_world: [f64; 3],
        include_socket: bool,
    ) -> Result<[f64; 3], String> {
        let peg_center_world_m = add(
            self.commanded_tool_position_world_m,
            scale(axis_world, self.scenario.grasp.tool_to_peg_axial_offset_m),
        );
        self.assembly_roi_world_m(axis_world, peg_center_world_m, include_socket)
    }

    fn seated_assembly_roi_world_m(&self, axis_world: [f64; 3]) -> Result<[f64; 3], String> {
        let peg_center_world_m = add(
            self.scenario.coupon.socket_center_nominal_world_m,
            scale(
                axis_world,
                0.5 * self.scenario.coupon.socket_depth_m - self.peg_tip_offset_m(),
            ),
        );
        self.assembly_roi_world_m(axis_world, peg_center_world_m, true)
    }

    fn classify_contact(
        &mut self,
        estimates: Option<&BTreeMap<u32, EstimateView>>,
    ) -> Result<ClassifiedContactEvidence, String> {
        let relative_mating_pose = estimates
            .map(|estimates| {
                relative_mating_pose(
                    estimates,
                    self.peg_tip_offset_m(),
                    0.5 * self.scenario.coupon.socket_depth_m,
                )
            })
            .transpose()?
            .flatten();
        let packet = self.plant.contact_packet();
        self.metrics.maximum_grip_force_proxy_n = self
            .metrics
            .maximum_grip_force_proxy_n
            .max(packet.grip_force_proxy_n);
        self.metrics.maximum_insertion_force_proxy_n = self
            .metrics
            .maximum_insertion_force_proxy_n
            .max(packet.insertion_force_proxy_n);
        classify_contact_packet(
            self.plant.now_tick(),
            packet,
            relative_mating_pose,
            self.contact_classification_policy(),
        )
        .map_err(contact_classification_reason)
    }

    fn contact_classification_policy(&self) -> ContactClassificationPolicy {
        let maximum_age_ticks = seconds_to_ticks_floor(
            self.scenario.optics.maximum_measurement_age_s,
            self.plant.fixed_dt_s(),
        );
        ContactClassificationPolicy {
            maximum_packet_age_ticks: maximum_age_ticks,
            maximum_pose_age_ticks: maximum_age_ticks,
            lead_in_start_m: self.scenario.contact.lead_in_start_m,
            recoverable_lateral_error_m: self.scenario.contact.recoverable_lateral_error_m,
            maximum_lateral_error_m: self.scenario.contact.maximum_lateral_error_m,
            seat_axial_tolerance_m: self.scenario.contact.seat_axial_tolerance_m,
            seat_lateral_tolerance_m: self.scenario.contact.seat_lateral_tolerance_m,
            seat_axis_tolerance_rad: self.scenario.contact.seat_axis_tolerance_rad,
            axis_lever_arm_m: self.peg_tip_offset_m(),
            maximum_position_sigma_m: self.scenario.estimator.insertion_position_sigma_limit_m,
            maximum_axis_sigma_rad: self.scenario.estimator.axis_sigma_limit_rad,
            maximum_force_proxy_n: self.scenario.contact.maximum_force_proxy_n,
        }
    }

    fn command_motion(
        &mut self,
        target_world_m: [f64; 3],
        class: MotionClass,
        near_contact: bool,
        estimates: Vec<EstimateView>,
    ) -> Result<(), String> {
        if (near_contact || self.grasp_confirmed) && self.moving_pose_reacquisition_required {
            return Err("moving_pose_reacquisition_required".to_owned());
        }
        if (near_contact || self.grasp_confirmed)
            && !estimates
                .iter()
                .any(|estimate| estimate.object_id == TOOL_OBJECT_ID && estimate.valid)
        {
            return Err("tool_pose_estimate_required".to_owned());
        }
        if near_contact {
            let now = self.plant.now_tick();
            if estimates.iter().any(|estimate| {
                now.saturating_sub(estimate.capture_tick)
                    > seconds_to_ticks_floor(
                        self.scenario.optics.maximum_measurement_age_s,
                        self.plant.fixed_dt_s(),
                    )
            }) {
                self.stale_near_contact_command_count += 1;
                return Err("stale_measurement".to_owned());
            }
        }
        let jaw_socket_clearance =
            match self.preflight_estimated_motion(target_world_m, class, &estimates) {
                Ok(evidence) => evidence,
                Err(reason) => {
                    self.record_decision(
                        "reject_estimated_sweep",
                        format!(
                            "covariance-inflated tool/carried-part envelope rejected: {reason}"
                        ),
                        near_contact,
                        None,
                        Some(target_world_m),
                        estimates,
                        None,
                    );
                    return Err(reason);
                }
            };
        if let Some(clearance) = jaw_socket_clearance {
            self.record_decision(
                "tool_socket_clearance_accepted",
                format!(
                    "observed swept non-peg tool/socket axial clearance lower bound {:.9e} m passed; projected tool forward extent {:.9e} m (jaw {:.9e} m, side target {:.9e} m), relative sigma {:.9e} m/{:.9e} rad, commanded-axis sweep bound {:.9e} rad, certified TCP path-deviation bound {:.9e} m, maximum axis cone {:.9e} rad, socket-axis projection bound {:.9e} m",
                    clearance.minimum_clearance_m,
                    clearance.projected_tool_forward_extent_m,
                    clearance.projected_jaw_half_extent_m,
                    clearance.projected_side_target_forward_extent_m,
                    clearance.relative_position_sigma_m,
                    clearance.relative_axis_sigma_rad,
                    clearance.commanded_tool_axis_change_bound_rad,
                    clearance.path_deviation_bound_m,
                    clearance.maximum_tool_socket_axis_error_rad,
                    clearance.socket_axis_projection_bound_m,
                ),
                near_contact,
                None,
                Some(target_world_m),
                estimates.clone(),
                None,
            );
        }
        let receipt = self
            .plant
            .command_tool_position(target_world_m, class)
            .map_err(plant_reason)?;
        self.commanded_tool_position_world_m = target_world_m;
        self.record_decision(
            receipt.command_kind,
            format!(
                "bounded {:?} motion accepted by authoritative runtime",
                class
            ),
            near_contact,
            Some(receipt.command_sequence),
            Some(target_world_m),
            estimates,
            None,
        );
        self.advance_until_idle_with_interlock_report()?;
        Ok(())
    }

    fn advance_until_idle_with_interlock_report(&mut self) -> Result<(), String> {
        let result = self
            .plant
            .advance_until_idle(self.scenario.motion.maximum_steps_per_motion);
        let (grip_peak_n, insertion_peak_n) = self.plant.force_channel_peaks();
        self.metrics.maximum_grip_force_proxy_n =
            self.metrics.maximum_grip_force_proxy_n.max(grip_peak_n);
        self.metrics.maximum_insertion_force_proxy_n = self
            .metrics
            .maximum_insertion_force_proxy_n
            .max(insertion_peak_n);
        match result {
            Ok(()) => Ok(()),
            Err(error) => {
                if let Some(event) = self.plant.force_interlock_event() {
                    let already_recorded = self.force_interlocks.iter().any(|record| {
                        record.tick == event.tick && record.channel == event.channel.id()
                    });
                    if !already_recorded {
                        self.metrics.force_interlock_trip_count =
                            self.metrics.force_interlock_trip_count.saturating_add(1);
                        self.metrics.maximum_grip_force_proxy_n = self
                            .metrics
                            .maximum_grip_force_proxy_n
                            .max(event.packet.grip_force_proxy_n);
                        self.metrics.maximum_insertion_force_proxy_n = self
                            .metrics
                            .maximum_insertion_force_proxy_n
                            .max(event.packet.insertion_force_proxy_n);
                        self.force_interlocks.push(ForceInterlockRecord {
                            tick: event.tick,
                            channel: event.channel.id(),
                            measured_force_proxy_n: event.measured_force_proxy_n,
                            limit_force_proxy_n: event.limit_force_proxy_n,
                            motion_was_active: event.motion_was_active,
                            stop_command_sequence: event.stop_command_sequence,
                            packet: event.packet,
                        });
                        self.record_decision(
                            "force_interlock_trip",
                            format!(
                                "{} force proxy {:.9e} N exceeded {:.9e} N; protective Stop was submitted at the same fixed tick",
                                event.channel.id(),
                                event.measured_force_proxy_n,
                                event.limit_force_proxy_n,
                            ),
                            true,
                            event.stop_command_sequence,
                            None,
                            Vec::new(),
                            None,
                        );
                    }
                }
                Err(plant_reason(error))
            }
        }
    }

    fn preflight_estimated_motion(
        &self,
        target_world_m: [f64; 3],
        motion_class: MotionClass,
        estimates: &[EstimateView],
    ) -> Result<Option<JawSocketAxialClearanceEvidence>, String> {
        let obstacles = self.plant.calibrated_planning_obstacles();
        let delta = sub(target_world_m, self.commanded_tool_position_world_m);
        let tool = estimates
            .iter()
            .find(|estimate| estimate.object_id == TOOL_OBJECT_ID);
        if self.grasp_confirmed && tool.is_none() {
            return Err("tool_pose_estimate_required".to_owned());
        }
        let (
            tool_start,
            tool_sigma_m,
            tool_axis_sigma_rad,
            commanded_tool_axis_change_bound_rad,
            tool_path_deviation_bound_m,
        ) = if let Some(tool) = tool {
            self.guard_view_with_limits(
                tool,
                self.motion_position_sigma_limit_m(),
                self.motion_axis_sigma_limit_rad(),
            )?;
            let anticipated =
                self.anticipated_motion_uncertainty(tool, target_world_m, motion_class)?;
            (
                tool.position_world_m,
                anticipated.position_sigma_m,
                anticipated.axis_sigma_rad,
                anticipated.commanded_tool_axis_change_bound_rad,
                anticipated.tool_path_deviation_bound_m,
            )
        } else {
            // Coarse unladen transit is allowed from calibrated commanded state.
            // Once a part is held, the PEG estimate below is mandatory and is
            // never synthesized from a phase threshold.
            (
                self.commanded_tool_position_world_m,
                self.scenario.estimator.macro_position_sigma_limit_m,
                self.scenario.estimator.axis_sigma_limit_rad,
                0.0,
                self.plant
                    .preview_tool_path_deviation_bound_m(target_world_m, motion_class)
                    .map_err(plant_reason)?,
            )
        };
        if let Err(failure) = preflight_swept_envelope(
            SweptEnvelope {
                center_start_world_m: tool_start,
                center_end_world_m: add(tool_start, delta),
                radius_m: self.scenario.safety.tool_envelope_radius_m,
                position_sigma_m: tool_sigma_m,
                hard_position_bound_m: if tool.is_none() && self.phase == ControlPhase::EnterCapture
                {
                    self.scenario.motion.pick_capture_relative_position_bound_m
                } else {
                    0.0
                },
                path_deviation_bound_m: tool_path_deviation_bound_m,
            },
            &obstacles,
            self.scenario.safety.minimum_obstacle_clearance_m,
        ) {
            return Err(match failure {
                SweptPreflightFailure::InvalidGeometry => "impossible_geometry".to_owned(),
                SweptPreflightFailure::Clearance(_) => {
                    "predicted_swept_geometry_collision_risk".to_owned()
                }
            });
        }
        if self.grasp_confirmed {
            let peg = estimates
                .iter()
                .find(|estimate| estimate.object_id == PEG_OBJECT_ID)
                .ok_or_else(|| "carried_pose_estimate_required".to_owned())?;
            self.guard_view_with_limits(
                peg,
                self.motion_position_sigma_limit_m(),
                self.motion_axis_sigma_limit_rad(),
            )?;
            let anticipated =
                self.anticipated_motion_uncertainty(peg, target_world_m, motion_class)?;
            let held_offset_radius_m = self.held_transform.map_or(
                self.scenario.grasp.tool_to_peg_axial_offset_m,
                |transform| transform.axial_offset_m.abs() + transform.lateral_offset_bound_m,
            );
            let carried_rotation_bound_m = 2.0
                * held_offset_radius_m
                * (0.5
                    * anticipated
                        .commanded_tool_axis_change_bound_rad
                        .min(core::f64::consts::PI))
                .sin();
            if let Err(failure) = preflight_swept_envelope(
                SweptEnvelope {
                    center_start_world_m: peg.position_world_m,
                    center_end_world_m: add(peg.position_world_m, delta),
                    radius_m: self.scenario.safety.carried_peg_envelope_radius_m,
                    position_sigma_m: anticipated.position_sigma_m,
                    hard_position_bound_m: 0.0,
                    path_deviation_bound_m: anticipated.tool_path_deviation_bound_m
                        + carried_rotation_bound_m,
                },
                &obstacles,
                self.scenario.safety.minimum_obstacle_clearance_m,
            ) {
                return Err(match failure {
                    SweptPreflightFailure::InvalidGeometry => "impossible_geometry".to_owned(),
                    SweptPreflightFailure::Clearance(_) => "carried_part_collision_risk".to_owned(),
                });
            }
        }
        if let Some(head) = &self.scenario.fixed_head {
            let socket = socket_estimate(estimates);
            let rail = TargetRailDatum {
                center_world_m: socket
                    .map_or(self.scenario.coupon.socket_center_nominal_world_m, |s| {
                        s.position_world_m
                    }),
                axis_world: socket.map_or_else(
                    || self.plant.calibrated_socket_axis_world(),
                    |s| s.axis_world,
                ),
                position_bound_m: socket.map_or(head.surveyed_socket_position_bound_m, |s| {
                    3.0 * s.position_sigma_m
                }),
                axis_bound_rad: socket.map_or(head.surveyed_socket_axis_bound_rad, |s| {
                    3.0 * s.axis_sigma_rad
                }),
                lateral_offset_m: self.scenario.coupon.socket_fiducial_lateral_offset_m,
                radius_m: self.scenario.coupon.socket_fiducial_radius_m,
                half_length_m: self.scenario.coupon.socket_fiducial_axial_half_extent_m,
            };
            self.plant
                .preview_arm_target_rail_clearance(target_world_m, motion_class, rail)
                .map_err(|_| "arm_target_rail_collision_risk".to_owned())?;
            let geometry = self.plant.jaw_socket_clearance_geometry();
            let tool_axis = tool.map_or_else(
                || {
                    self.plant
                        .commanded_tool_axis_world()
                        .expect("fixed-head axis intent")
                },
                |t| t.axis_world,
            );
            let tool_axis_bound = if tool.is_some() {
                3.0 * tool_axis_sigma_rad
            } else {
                self.scenario.motion.pick_capture_relative_axis_bound_rad
            } + self
                .plant
                .preview_tool_axis_change_bound_rad(target_world_m, motion_class)
                .map_err(plant_reason)?;
            let tool_position_bound = 3.0 * tool_sigma_m
                + if tool.is_none() {
                    self.scenario.motion.pick_capture_relative_position_bound_m
                } else {
                    0.0
                };
            let tool_envelopes = [
                (
                    0.0,
                    geometry.jaw_axial_half_length_m,
                    if self.grasp_confirmed {
                        geometry.closed_jaw_transverse_radius_m
                    } else {
                        geometry.open_jaw_transverse_radius_m
                    },
                ),
                (
                    0.0,
                    geometry.side_target_forward_extent_m,
                    geometry.side_target_transverse_radius_m,
                ),
                // Calibrated terminal tube ends 0.35 mm behind the TCP. Its
                // distal collision capsule's artificial round end is excluded;
                // the tube, jaws and side targets each have a separate bound.
                (
                    0.5 * (geometry.central_palm_forward_plane_tool_z_m
                        - geometry.terminal_backing_length_m),
                    0.5 * (geometry.terminal_backing_length_m
                        + geometry.central_palm_forward_plane_tool_z_m),
                    geometry.terminal_backing_radius_m * core::f64::consts::SQRT_2,
                ),
            ];
            for (index, (offset, half_length, radius)) in tool_envelopes.into_iter().enumerate() {
                guard_target_rail_sweep(
                    AxialEnvelopeSweep {
                        center_world_m: tool_start,
                        axis_world: tool_axis,
                        translation_world_m: delta,
                        center_axial_offset_m: offset,
                        half_length_m: half_length,
                        radius_m: radius,
                        position_bound_m: tool_position_bound,
                        axis_bound_rad: tool_axis_bound,
                        path_deviation_bound_m: tool_path_deviation_bound_m,
                    },
                    rail,
                    self.scenario.safety.minimum_obstacle_clearance_m,
                )
                .map_err(|reason| format!("{reason}:tool_component_{index}"))?;
            }
            if self.grasp_confirmed {
                let peg = estimates
                    .iter()
                    .find(|e| e.object_id == PEG_OBJECT_ID)
                    .ok_or_else(|| "carried_pose_estimate_required".to_owned())?;
                let anticipated =
                    self.anticipated_motion_uncertainty(peg, target_world_m, motion_class)?;
                guard_target_rail_sweep(
                    AxialEnvelopeSweep {
                        center_world_m: peg.position_world_m,
                        axis_world: peg.axis_world,
                        translation_world_m: delta,
                        center_axial_offset_m: 0.0,
                        half_length_m: self.scenario.coupon.peg_half_segment_m,
                        radius_m: 0.5 * self.scenario.coupon.peg_diameter_m,
                        position_bound_m: 3.0 * anticipated.position_sigma_m,
                        axis_bound_rad: 3.0 * anticipated.axis_sigma_rad
                            + commanded_tool_axis_change_bound_rad,
                        path_deviation_bound_m: tool_path_deviation_bound_m
                            + 2.0
                                * self.scenario.grasp.tool_to_peg_axial_offset_m
                                * (0.5
                                    * commanded_tool_axis_change_bound_rad
                                        .min(core::f64::consts::PI))
                                .sin(),
                    },
                    rail,
                    self.scenario.safety.minimum_obstacle_clearance_m,
                )
                .map_err(|reason| format!("{reason}:carried_peg"))?;
            }
        }
        if motion_class == MotionClass::Insertion || socket_estimate(estimates).is_some() {
            let tool = tool.ok_or_else(|| "tool_pose_estimate_required".to_owned())?;
            let socket = socket_estimate(estimates)
                .ok_or_else(|| "socket_pose_estimate_required".to_owned())?;
            self.guard_view(
                socket,
                self.scenario.estimator.insertion_position_sigma_limit_m,
            )?;
            let geometry = self.plant.jaw_socket_clearance_geometry();
            let evidence = guard_jaw_socket_axial_clearance(
                tool,
                socket,
                JawSocketMotionPreview {
                    commanded_translation_world_m: delta,
                    anticipated_tool_position_sigma_m: tool_sigma_m,
                    anticipated_tool_axis_sigma_rad: tool_axis_sigma_rad,
                    commanded_tool_axis_change_bound_rad,
                    path_deviation_bound_m: tool_path_deviation_bound_m,
                },
                JawSocketAxialClearancePolicy {
                    jaw_axial_half_length_m: geometry.jaw_axial_half_length_m,
                    jaw_transverse_radius_m: if self.grasp_confirmed {
                        geometry.closed_jaw_transverse_radius_m
                    } else {
                        geometry.open_jaw_transverse_radius_m
                    },
                    side_target_forward_extent_m: geometry.side_target_forward_extent_m,
                    side_target_transverse_radius_m: geometry.side_target_transverse_radius_m,
                    socket_depth_m: geometry.socket_depth_m,
                    minimum_clearance_m: geometry.minimum_clearance_m,
                },
            )
            .map_err(str::to_owned)?;
            return Ok(Some(evidence));
        }
        Ok(None)
    }

    fn motion_position_sigma_limit_m(&self) -> f64 {
        match self.phase {
            ControlPhase::GuardedGrasp => self.scenario.estimator.grasp_position_sigma_limit_m,
            ControlPhase::Transfer => self.scenario.estimator.transfer_position_sigma_limit_m,
            ControlPhase::GuardedInsertion | ControlPhase::SeatVerification => {
                self.scenario.estimator.insertion_position_sigma_limit_m
            }
            _ => self.scenario.estimator.macro_position_sigma_limit_m,
        }
    }

    fn motion_axis_sigma_limit_rad(&self) -> f64 {
        if self.phase == ControlPhase::Transfer {
            self.scenario.estimator.transfer_axis_sigma_limit_rad
        } else {
            self.scenario.estimator.axis_sigma_limit_rad
        }
    }

    /// Conservative preview used before a motion is authorized. The interval
    /// is the authoritative deterministic plan duration plus required optical
    /// settling; the independent watchdog remains a hard upper bound.
    fn anticipated_motion_uncertainty(
        &self,
        start: &EstimateView,
        target_world_m: [f64; 3],
        motion_class: MotionClass,
    ) -> Result<AnticipatedMotionUncertainty, String> {
        let commanded_delta_world_m = sub(target_world_m, self.commanded_tool_position_world_m);
        if !start.valid
            || !start.position_sigma_m.is_finite()
            || start.position_sigma_m < 0.0
            || !start.axis_sigma_rad.is_finite()
            || start.axis_sigma_rad < 0.0
            || commanded_delta_world_m
                .iter()
                .any(|component| !component.is_finite())
        {
            return Err("invalid_estimator_state_for_motion_preview".to_owned());
        }
        let planned_duration_s = self
            .plant
            .preview_tool_motion_duration_s(target_world_m, motion_class)
            .map_err(plant_reason)?;
        let commanded_tool_axis_change_bound_rad = self
            .plant
            .preview_tool_axis_change_bound_rad(target_world_m, motion_class)
            .map_err(plant_reason)?;
        let tool_path_deviation_bound_m = self
            .plant
            .preview_tool_path_deviation_bound_m(target_world_m, motion_class)
            .map_err(plant_reason)?;
        let interval_s = planned_duration_s + self.scenario.optics.settling_interval_s;
        let maximum_interval_s =
            f64::from(self.scenario.motion.maximum_steps_per_motion) * self.plant.fixed_dt_s();
        if !interval_s.is_finite() || interval_s > maximum_interval_s + 1.0e-12 {
            return Err("motion_duration_limit".to_owned());
        }
        let travel_sigma_m =
            self.scenario.estimator.free_prediction_sigma_m_per_m * norm(commanded_delta_world_m);
        let hold_sigma_m = self
            .scenario
            .estimator
            .loaded_hold_process_sigma_m_per_sqrt_s
            * interval_s.max(0.0).sqrt();
        // Do not blindly offset a command for backlash: that can cross an
        // observed target near convergence. The plant applies the configured
        // lost-motion/quantization model, while the preflight envelope carries
        // its hard error bound as an equivalent three-sigma contribution until
        // the mandatory post-motion observation replaces the pose.
        let capabilities = self.plant.motion_capabilities();
        let actuation_hard_bound_m = 0.5 * capabilities.differential_backlash_m
            + 0.5 * capabilities.minimum_reproducible_correction_m
            + if self.grasp_confirmed {
                norm(self.scenario.motion.loaded_hold_error_world_m)
            } else {
                0.0
            };
        let actuation_bound_sigma_m = actuation_hard_bound_m / 3.0;
        let position_sigma_m = (start.position_sigma_m.powi(2)
            + travel_sigma_m.powi(2)
            + hold_sigma_m.powi(2)
            + actuation_bound_sigma_m.powi(2))
        .sqrt();
        let hold_axis_sigma_rad = hold_sigma_m / self.scenario.coupon.minimum_feature_axial_span_m;
        let axis_sigma_rad = (start.axis_sigma_rad.powi(2) + hold_axis_sigma_rad.powi(2)).sqrt();
        if !position_sigma_m.is_finite() || !axis_sigma_rad.is_finite() {
            return Err("invalid_estimator_state_for_motion_preview".to_owned());
        }
        if position_sigma_m > self.motion_position_sigma_limit_m()
            || axis_sigma_rad > self.motion_axis_sigma_limit_rad()
        {
            return Err("predicted_motion_uncertainty_limit".to_owned());
        }
        Ok(AnticipatedMotionUncertainty {
            position_sigma_m,
            axis_sigma_rad,
            commanded_tool_axis_change_bound_rad,
            tool_path_deviation_bound_m,
            interval_s,
        })
    }

    fn invalidate_moving_pose_priors_before_reacquisition(
        &mut self,
        carried: bool,
        near_contact: bool,
        context: &'static str,
    ) -> Result<(), String> {
        let object_ids: &[u32] = if carried {
            &[PEG_OBJECT_ID, TOOL_OBJECT_ID]
        } else {
            &[TOOL_OBJECT_ID]
        };
        for &object_id in object_ids {
            self.estimator_mut(object_id)?.reset();
        }
        self.moving_pose_reacquisition_required = true;
        self.record_decision(
            "invalidate_moving_pose_prior",
            if self.scenario.fixed_head.is_some() {
                format!(
                    "{context}: translation/process uncertainty was propagated for motion safety, then moving TOOL{} pose priors were invalidated before the next closed-loop decision; a fresh stopped burst is mandatory",
                    if carried { "+PEG" } else { "" },
                )
            } else {
                format!(
                    "{context}: translation/process uncertainty was propagated for motion safety, then moving TOOL{} pose priors were invalidated because point IK has no calibrated commanded-axis transition; a fresh stopped burst is mandatory",
                    if carried { "+PEG" } else { "" },
                )
            },
            near_contact,
            None,
            None,
            // A post-motion prediction may be deliberately invalid. The
            // action/reason records invalidation; detailed optional estimator
            // fields remain in EstimatorUpdateRecord rather than non-finite
            // numeric sentinels in this decision DTO.
            Vec::new(),
            None,
        );
        Ok(())
    }

    fn predict_after_motion(
        &mut self,
        commanded_delta_world_m: [f64; 3],
        carried: bool,
    ) -> Result<(), String> {
        let tick = self.plant.now_tick();
        let phase = self.phase;
        let (position_limit, axis_limit) = match phase {
            ControlPhase::GuardedGrasp => (
                self.scenario.estimator.grasp_position_sigma_limit_m,
                self.scenario.estimator.axis_sigma_limit_rad,
            ),
            ControlPhase::Transfer => (
                self.scenario.estimator.transfer_position_sigma_limit_m,
                self.scenario.estimator.transfer_axis_sigma_limit_rad,
            ),
            ControlPhase::GuardedInsertion | ControlPhase::SeatVerification => (
                self.scenario.estimator.insertion_position_sigma_limit_m,
                self.scenario.estimator.axis_sigma_limit_rad,
            ),
            _ => (
                self.scenario.estimator.macro_position_sigma_limit_m,
                self.scenario.estimator.axis_sigma_limit_rad,
            ),
        };
        let tool_has_valid_prior = self
            .estimator(TOOL_OBJECT_ID)?
            .last_report()
            .is_some_and(PoseEstimate::is_valid);
        if tool_has_valid_prior {
            let report = self
                .estimator_mut(TOOL_OBJECT_ID)?
                .predict_commanded_translation(tick, commanded_delta_world_m);
            let validation = validate_post_motion_prediction(&report);
            self.estimator_updates.push(EstimatorUpdateRecord {
                sequence: self.estimator_updates.len() as u32,
                phase,
                update_kind: "command_prediction",
                burst_sequence: None,
                applied_position_sigma_limit_m: position_limit,
                applied_axis_sigma_limit_rad: axis_limit,
                accepted_by_controller: validation.is_ok(),
                estimate: report,
            });
            validation?;
        }
        if carried {
            let report = self
                .estimator_mut(PEG_OBJECT_ID)?
                .predict_commanded_translation(tick, commanded_delta_world_m);
            let validation = validate_post_motion_prediction(&report);
            self.estimator_updates.push(EstimatorUpdateRecord {
                sequence: self.estimator_updates.len() as u32,
                phase,
                update_kind: "command_prediction",
                burst_sequence: None,
                applied_position_sigma_limit_m: position_limit,
                applied_axis_sigma_limit_rad: axis_limit,
                accepted_by_controller: validation.is_ok(),
                estimate: report,
            });
            validation?;
        }
        Ok(())
    }

    fn correction_policy(&self) -> CorrectionPolicy {
        let capabilities = self.plant.motion_capabilities();
        CorrectionPolicy {
            gain: self.scenario.motion.correction_gain,
            convergence_m: self.scenario.motion.correction_convergence_m,
            maximum_magnitude_m: capabilities
                .maximum_correction_m
                .min(self.scenario.motion.maximum_correction_m),
            minimum_reproducible_m: capabilities.minimum_reproducible_correction_m,
        }
    }

    fn estimator(&self, object_id: u32) -> Result<&ObservedPoseEstimator, String> {
        self.estimators
            .get(&object_id)
            .ok_or_else(|| format!("missing_estimator_for_object_{object_id}"))
    }

    fn estimator_mut(&mut self, object_id: u32) -> Result<&mut ObservedPoseEstimator, String> {
        self.estimators
            .get_mut(&object_id)
            .ok_or_else(|| format!("missing_estimator_for_object_{object_id}"))
    }

    fn fail_closed(&mut self, reason: String) {
        let near_contact = self.grasp_confirmed
            || matches!(
                self.phase,
                ControlPhase::PickCorrection
                    | ControlPhase::GuardedGrasp
                    | ControlPhase::SocketCorrection
                    | ControlPhase::GuardedInsertion
                    | ControlPhase::SeatVerification
                    | ControlPhase::Release
                    | ControlPhase::Retreat
            );
        let (command_sequence, stop_outcome) = match self.plant.command_stop() {
            Ok(receipt) => (
                Some(receipt.command_sequence),
                "authoritative stop accepted; controller holds position and does not attempt an unvalidated retreat".to_owned(),
            ),
            Err(error) => (
                None,
                format!(
                    "authoritative stop request was rejected ({}); controller declares failure without issuing further motion",
                    plant_reason(error)
                ),
            ),
        };
        self.record_decision(
            "fail_closed_hold",
            format!("terminal guard '{reason}': {stop_outcome}"),
            near_contact,
            command_sequence,
            None,
            Vec::new(),
            None,
        );
        self.terminal_reason = Some(reason);
        self.phase = ControlPhase::FailedSafe;
    }

    // Keeping the report fields explicit at each call site makes every safety
    // decision auditable; bundling them into partially initialized builders is
    // more error-prone for this deterministic executive.
    #[allow(clippy::too_many_arguments)]
    fn record_decision(
        &mut self,
        action: &'static str,
        reason: impl Into<String>,
        near_contact: bool,
        command_sequence: Option<u64>,
        target_world_m: Option<[f64; 3]>,
        relevant_estimates: Vec<EstimateView>,
        contact_evidence: Option<ClassifiedContactEvidence>,
    ) {
        // Invalid estimates retain their complete optional diagnostics in the
        // corresponding EstimatorUpdateRecord. Exclude their controller-only
        // numeric sentinels from DecisionRecord so serialized zeros cannot be
        // mistaken for perfect pose certainty by a downstream reader.
        let relevant_estimates = relevant_estimates
            .into_iter()
            .filter(|estimate| estimate.valid)
            .collect();
        self.decisions.push(DecisionRecord {
            sequence: self.decisions.len() as u32,
            tick: self.plant.now_tick(),
            time_s: self.plant.now_tick() as f64 * self.plant.fixed_dt_s(),
            phase: self.phase,
            action,
            reason: reason.into(),
            near_contact,
            command_sequence,
            target_world_m,
            target_axis_world: if action == "set_tool_position_and_axis"
                || action == "axis_correction"
            {
                self.plant.commanded_tool_axis_world()
            } else {
                None
            },
            relevant_estimates,
            contact_evidence,
        });
    }

    pub fn is_terminal(&self) -> bool {
        matches!(
            self.phase,
            ControlPhase::Complete | ControlPhase::FailedSafe
        )
    }

    pub fn is_completed(&self) -> bool {
        self.phase == ControlPhase::Complete
    }

    pub fn report(&self) -> ObservedManipulationReport {
        // The fault enum owns the acceptance oracle. Scenario parsing checks
        // that its human-readable profile repeats this exact mapping, so a
        // custom input cannot turn an unrelated early failure into success.
        let expected_reason = self.fault.expected_terminal_reason().map(str::to_owned);
        let expected_outcome_observed = match expected_reason.as_deref() {
            None => self.phase == ControlPhase::Complete && self.terminal_reason.is_none(),
            Some(expected) => {
                self.phase == ControlPhase::FailedSafe
                    && self.terminal_reason.as_deref() == Some(expected)
            }
        };
        let status = match self.phase {
            ControlPhase::Complete => "complete",
            ControlPhase::FailedSafe => "failed_safe",
            ControlPhase::Initialize => "ready",
            _ => "running",
        };
        let gates = self.acceptance_gates();
        let timing = TimingReport {
            fixed_step_s: self.plant.fixed_dt_s(),
            settling_interval_s: self.scenario.optics.settling_interval_s,
            coded_pattern_count: self.scenario.optics.pattern_count,
            pattern_rate_hz: self.scenario.optics.pattern_rate_hz,
            processing_latency_s: self.scenario.optics.processing_latency_s,
            maximum_measurement_age_s: self.scenario.optics.maximum_measurement_age_s,
            stale_near_contact_command_count: self.stale_near_contact_command_count,
        };
        let controller_report_sha256 = controller_hash(
            &self.scenario_sha256,
            self.plant.machine_config_id(),
            self.plant.machine_config_source_sha256(),
            self.fault,
            status,
            &self.terminal_reason,
            &timing,
            &self.metrics,
            &self.observations,
            &self.estimator_updates,
            &self.uncertainty_guards,
            &self.force_interlocks,
            &self.corrections,
            &self.decisions,
            &gates,
        );
        ObservedManipulationReport {
            schema_version: if self.scenario.fixed_head.is_some() { 2 } else { OBSERVED_MANIPULATION_REPORT_SCHEMA_VERSION },
            scenario_id: self.scenario.id.clone(),
            scenario_schema_version: self.scenario.schema_version,
            scenario_sha256: self.scenario_sha256.clone(),
            machine_config_id: self.plant.machine_config_id().to_owned(),
            machine_config_sha256: self.plant.machine_config_source_sha256().to_owned(),
            seed: self.scenario.seed,
            injected_fault: self.fault.id(),
            expected_terminal_reason: expected_reason,
            status,
            phase: self.phase,
            terminal_reason: self.terminal_reason.clone(),
            expected_outcome_observed,
            fixed_head: self.scenario.fixed_head.clone(),
            fidelity: if self.scenario.fixed_head.is_some() { "F1_reduced_M1f_fixed_head_axis_control_not_hardware_qualified" }
                else { "F1_reduced_M1e_observed_feature_geometry_not_hardware_qualified" },
            hardware_qualification_status: "not_started",
            pose_state: self.scenario.pose_state.clone(),
            roll_observable: false,
            optical_fidelity_boundary: if self.scenario.fixed_head.is_some() {
                "fixed surveyed camera/projector and opaque body/mount/target-rail proxies; labelled geometric features with localization/dropout/drift; no images, refraction, MTF, detector ambiguity or hardware qualification"
            } else { "pinhole labelled geometric features with virtual fitted centres, opaque proxy occlusion, deterministic localization/dropout/drift, and ideal ROI head retile; no head actuator/pose error/collision model, socket target-rail mechanics bodies, images, refraction, MTF, photometric or wave optics" },
            contact_fidelity_boundary: "raw pad/contact channels plus controller-classified observed mating geometry and an uncalibrated compliance/force proxy; no identified friction or contact dynamics",
            attachment_fidelity_boundary: "guarded acquisition followed by uncertain kinematic attachment; no breakable grasp dynamics",
            truth_firewall: TruthFirewallReport::default(),
            timing,
            metrics: self.metrics.clone(),
            observation_bursts: self.observations.clone(),
            estimator_updates: self.estimator_updates.clone(),
            uncertainty_guards: self.uncertainty_guards.clone(),
            force_interlocks: self.force_interlocks.clone(),
            correction_iterations: self.corrections.clone(),
            decisions: self.decisions.clone(),
            acceptance_gates: gates,
            controller_report_sha256,
            controller_report_hash_scope: HASH_SCOPE,
            evaluation_only_truth: self.terminal_evaluation(),
        }
    }

    fn terminal_evaluation(&self) -> Option<EvaluationOnlyTruthMetrics> {
        if !self.is_terminal() {
            return None;
        }
        let evaluation = self.plant.evaluation_metrics();
        Some(EvaluationOnlyTruthMetrics {
            label: "evaluation_only_not_controller_visible",
            final_peg_tip_to_socket_seat_error_m: evaluation.final_peg_tip_to_socket_seat_error_m,
            final_peg_lateral_error_m: evaluation.final_peg_lateral_error_m,
            final_peg_axial_error_m: evaluation.final_peg_axial_error_m,
            final_peg_axis_error_rad: evaluation.final_peg_axis_error_rad,
            final_tool_center_to_socket_center_distance_m: evaluation
                .final_tool_center_to_socket_center_distance_m,
            physical_grasp_attachment_present: evaluation.physical_grasp_attachment_present,
            physical_release_verified: evaluation.grasp_committed
                && evaluation.release_committed
                && !evaluation.physical_grasp_attachment_present,
            maximum_unplanned_penetration_m: evaluation.maximum_unplanned_penetration_m,
            peak_grip_force_proxy_n: evaluation.peak_grip_force_proxy_n,
            peak_insertion_force_proxy_n: evaluation.peak_insertion_force_proxy_n,
            within_declared_seat_tolerances: evaluation.final_peg_lateral_error_m
                <= self.scenario.contact.seat_lateral_tolerance_m
                && evaluation.final_peg_axial_error_m
                    <= self.scenario.contact.seat_axial_tolerance_m
                && evaluation.final_peg_axis_error_rad
                    <= self.scenario.contact.seat_axis_tolerance_rad,
            note: "simulation truth is serialized only after terminal control for scoring",
        })
    }

    fn acceptance_gates(&self) -> Vec<AcceptanceGate> {
        let nominal = self.fault == M1eFault::None;
        let firewall = TruthFirewallReport::default();
        let required_uncertainty_phases = [
            ControlPhase::PickCorrection,
            ControlPhase::GuardedGrasp,
            ControlPhase::Transfer,
            ControlPhase::SocketCorrection,
            ControlPhase::GuardedInsertion,
            ControlPhase::SeatVerification,
            ControlPhase::Release,
            ControlPhase::Retreat,
        ];
        let phase_uncertainty_passed = !self.uncertainty_guards.is_empty()
            && self.uncertainty_guards.iter().all(|guard| {
                guard.passed
                    || (self.scenario.fixed_head.is_some()
                        && self
                            .uncertainty_guards
                            .iter()
                            .find(|later| {
                                later.sequence > guard.sequence
                                    && later.phase == guard.phase
                                    && later.kind == guard.kind
                                    && later.object_ids == guard.object_ids
                                    && later.passed
                            })
                            .is_some_and(|recovered| {
                                !self.decisions.iter().any(|decision| {
                                    decision.tick >= guard.tick
                                        && decision.tick < recovered.tick
                                        && decision.command_sequence.is_some()
                                        && !matches!(
                                            decision.action,
                                            "hold_and_settle" | "fail_closed_hold"
                                        )
                                })
                            }))
            })
            && required_uncertainty_phases.iter().all(|phase| {
                self.uncertainty_guards
                    .iter()
                    .any(|guard| guard.phase == *phase && guard.passed)
            });
        let pick_observed = self.observations.iter().any(|burst| {
            burst.phase == ControlPhase::PickCorrection
                && [PEG_OBJECT_ID, TOOL_OBJECT_ID]
                    .iter()
                    .all(|id| burst.accepted_object_ids.contains(id))
        });
        let mating_observed = self.observations.iter().any(|burst| {
            matches!(
                burst.phase,
                ControlPhase::SocketCorrection
                    | ControlPhase::GuardedInsertion
                    | ControlPhase::SeatVerification
            ) && [PEG_OBJECT_ID, SOCKET_OBJECT_ID, TOOL_OBJECT_ID]
                .iter()
                .all(|id| burst.accepted_object_ids.contains(id))
        });
        let correction_budget = self.scenario.motion.maximum_correction_iterations;
        let pick_converged = self.corrections.iter().any(|record| {
            record.phase == ControlPhase::PickCorrection
                && record.outcome == "converged"
                && record.iteration <= correction_budget
        });
        let socket_converged = self.corrections.iter().any(|record| {
            record.phase == ControlPhase::SocketCorrection
                && record.outcome == "converged"
                && record.iteration <= correction_budget
        });
        vec![
            gate(
                "no_controller_truth_access",
                firewall.controller_truth_access_count == 0
                    && !firewall.controller_accepts_raw_depth_samples
                    && !firewall.controller_accepts_scene_truth,
                "controller and estimator modules accept only sanitized DTOs; source firewall test is executable",
            ),
            nominal_gate(
                "observed_acquisition",
                nominal,
                pick_observed && mating_observed,
                format!(
                    "pick_required_set={pick_observed}; mating_required_set={mating_observed}; {} accepted pose estimates",
                    self.metrics.accepted_pose_estimates
                ),
            ),
            nominal_gate(
                "estimator_phase_uncertainty",
                nominal,
                phase_uncertainty_passed,
                if self.scenario.fixed_head.is_some() {
                    format!(
                        "all_phase_decisions_guarded={phase_uncertainty_passed}; rejected_bursts_require_stopped_reacquisition; guard_count={}; accepted_position_sigma_max={:.9e} m; accepted_axis_sigma_max={:.9e} rad",
                        self.uncertainty_guards.len(),
                        self.metrics.maximum_accepted_position_sigma_m,
                        self.metrics.maximum_accepted_axis_sigma_rad,
                    )
                } else {
                    format!(
                        "all_per_phase_guards={phase_uncertainty_passed}; guard_count={}; accepted_position_sigma_max={:.9e} m; accepted_axis_sigma_max={:.9e} rad",
                        self.uncertainty_guards.len(),
                        self.metrics.maximum_accepted_position_sigma_m,
                        self.metrics.maximum_accepted_axis_sigma_rad,
                    )
                },
            ),
            nominal_gate(
                "correction_convergence",
                nominal,
                pick_converged && socket_converged,
                format!(
                    "pick={pick_converged}; socket={socket_converged}; {} stop-and-look stages converged within iteration budget {}",
                    self.metrics.successful_corrections,
                    correction_budget,
                ),
            ),
            nominal_gate(
                "guarded_grasp",
                nominal,
                self.metrics.guarded_grasp_confirmed,
                format!("confirmed={}", self.metrics.guarded_grasp_confirmed),
            ),
            nominal_gate(
                "collision_free_transfer",
                nominal,
                self.metrics.transfer_preflight_passed && self.metrics.transfer_completed,
                format!(
                    "estimated preflight passed={}; transfer completed={}",
                    self.metrics.transfer_preflight_passed, self.metrics.transfer_completed
                ),
            ),
            nominal_gate(
                "guarded_insertion",
                nominal,
                self.metrics.insertion_increment_count > 0
                    && self.metrics.guarded_insertion_confirmed
                    && self.metrics.force_interlock_trip_count == 0,
                format!(
                    "{} bounded increments; guarded_transition={}; interlock_trips={}",
                    self.metrics.insertion_increment_count,
                    self.metrics.guarded_insertion_confirmed,
                    self.metrics.force_interlock_trip_count,
                ),
            ),
            nominal_gate(
                "observed_and_contact_seat",
                nominal,
                self.metrics.seated_from_observation_and_contact,
                format!("verified={}", self.metrics.seated_from_observation_and_contact),
            ),
            gate(
                "no_stale_near_contact_command",
                self.stale_near_contact_command_count == 0,
                format!("count={}", self.stale_near_contact_command_count),
            ),
            gate(
                "force_proxy_limit",
                self.metrics.maximum_grip_force_proxy_n
                    <= self.scenario.grasp.maximum_grip_force_n
                    && self.metrics.maximum_insertion_force_proxy_n
                        <= self.scenario.contact.maximum_force_proxy_n
                    && self.metrics.force_interlock_trip_count == 0,
                format!(
                    "grip_peak={:.9e} N grip_limit={:.9e} N; insertion_peak={:.9e} N insertion_limit={:.9e} N; interlock_trips={}",
                    self.metrics.maximum_grip_force_proxy_n,
                    self.scenario.grasp.maximum_grip_force_n,
                    self.metrics.maximum_insertion_force_proxy_n,
                    self.scenario.contact.maximum_force_proxy_n,
                    self.metrics.force_interlock_trip_count,
                ),
            ),
            nominal_gate(
                "safe_release_and_retreat",
                nominal,
                self.metrics.release_confirmed && self.metrics.retreat_confirmed,
                format!(
                    "release={} retreat={}",
                    self.metrics.release_confirmed, self.metrics.retreat_confirmed
                ),
            ),
            gate(
                "explicit_fidelity_contract",
                self.scenario.status == "modeled_vertical_slice_not_hardware_qualified"
                    && self.scenario.pose_state.starts_with("axisymmetric_5d"),
                "F1 reduced geometric-feature/contact-proxy model; explicitly not hardware-qualified",
            ),
        ]
    }
}

fn estimator_config(
    scenario: &ObservedManipulationScenario,
    tick_period_s: f64,
) -> EstimatorConfig {
    let capture_ticks = seconds_to_ticks_ceil(
        scenario.optics.pattern_count as f64 / scenario.optics.pattern_rate_hz,
        tick_period_s,
    );
    EstimatorConfig {
        tick_period_s,
        maximum_measurement_age_ticks: seconds_to_ticks_floor(
            scenario.optics.maximum_measurement_age_s,
            tick_period_s,
        ),
        maximum_burst_span_ticks: capture_ticks.saturating_add(1),
        minimum_measurement_count: scenario.optics.minimum_distinct_features as usize,
        minimum_feature_count: scenario.optics.minimum_distinct_features as usize,
        minimum_axial_station_count: 2,
        minimum_head_count: scenario.optics.minimum_triangulation_heads as usize,
        minimum_calibrated_ray_count: u64::from(scenario.optics.minimum_distinct_features)
            .saturating_mul(u64::from(scenario.optics.minimum_calibrated_rays_per_point)),
        minimum_confidence: scenario.optics.minimum_confidence,
        minimum_axial_lever_arm_m: scenario.estimator.minimum_accepted_feature_span_m,
        minimum_independent_variance_m2: 0.05e-6_f64.powi(2),
        outlier_sigma_threshold: scenario.estimator.maximum_innovation_sigma,
        outlier_absolute_threshold_m: scenario.estimator.maximum_feature_residual_m,
        maximum_outlier_fraction: 0.34,
        maximum_outlier_iterations: 4,
        maximum_residual_rms_m: scenario.estimator.maximum_feature_residual_m,
        maximum_normalized_residual_rms: scenario.estimator.maximum_innovation_sigma,
        maximum_axis_scale_error: 0.20,
        correlated_position_floor_m: [scenario.optics.correlated_calibration_sigma_m; 3],
        correlated_axis_floor_rad: scenario.optics.correlated_calibration_sigma_m
            / scenario.coupon.minimum_feature_axial_span_m,
        maximum_position_sigma_m: scenario
            .estimator
            .macro_position_sigma_limit_m
            .max(scenario.estimator.grasp_position_sigma_limit_m)
            .max(scenario.estimator.transfer_position_sigma_limit_m)
            .max(scenario.estimator.insertion_position_sigma_limit_m),
        maximum_axis_sigma_rad: scenario.estimator.transfer_axis_sigma_limit_rad,
        maximum_innovation_translation_m: scenario.motion.maximum_correction_m * 1.5,
        maximum_innovation_axis_rad: scenario.estimator.axis_sigma_limit_rad * 4.0,
        maximum_normalized_innovation: scenario.estimator.maximum_innovation_sigma,
        hold_process_sigma_m_per_sqrt_s: [
            scenario.estimator.loaded_hold_process_sigma_m_per_sqrt_s,
            scenario.estimator.loaded_hold_process_sigma_m_per_sqrt_s,
            scenario.estimator.loaded_hold_process_sigma_m_per_sqrt_s,
        ],
        hold_axis_sigma_rad_per_sqrt_s: scenario.estimator.loaded_hold_process_sigma_m_per_sqrt_s
            / scenario.coupon.minimum_feature_axial_span_m,
        commanded_translation_fractional_sigma: scenario.estimator.free_prediction_sigma_m_per_m,
    }
}

fn estimate_view(report: &PoseEstimate) -> EstimateView {
    let pose = report.pose.unwrap_or(super::estimator::AxisymmetricPose5d {
        center_world_m: [0.0; 3],
        axis_world_unit: [0.0, 0.0, 1.0],
        roll_observable: false,
    });
    let (position_sigma_m, axis_sigma_rad) = report.uncertainty.map_or((0.0, 0.0), |uncertainty| {
        (
            uncertainty
                .center_sigma_m
                .iter()
                .copied()
                .fold(0.0, f64::max),
            uncertainty
                .axis_tangent_sigma_rad
                .iter()
                .copied()
                .fold(0.0, f64::max),
        )
    });
    EstimateView {
        object_id: report.object_id,
        valid: report.is_valid(),
        invalid_reason: (!report.is_valid()).then(|| report.validity_detail.clone()),
        position_world_m: pose.center_world_m,
        axis_world: pose.axis_world_unit,
        position_sigma_m,
        axis_sigma_rad,
        capture_tick: report.oldest_capture_tick.unwrap_or(report.controller_tick),
        available_tick: report
            .newest_available_tick
            .unwrap_or(report.controller_tick),
        distinct_feature_count: report.accepted_feature_count as u32,
        triangulation_head_count: report.head_count as u32,
        minimum_calibrated_rays_per_point: report
            .minimum_calibrated_rays_per_measurement
            .unwrap_or(0),
        residual_rms_m: report.residual_rms_m.unwrap_or(0.0),
        provenance: EstimateProvenance::direct_feature_fit(),
    }
}

fn required_view(
    estimates: &BTreeMap<u32, EstimateView>,
    object_id: u32,
) -> Result<&EstimateView, String> {
    estimates
        .get(&object_id)
        .ok_or_else(|| format!("required_object_{object_id}_estimate_missing"))
}

fn socket_estimate(estimates: &[EstimateView]) -> Option<&EstimateView> {
    estimates
        .iter()
        .find(|estimate| estimate.object_id == SOCKET_OBJECT_ID)
}

fn map_estimate_failure(
    phase: ControlPhase,
    correction_commanded_this_phase: bool,
    object_id: u32,
    estimate: &PoseEstimate,
    burst: &ObservationBurst,
    guard_reason: String,
) -> String {
    match estimate.validity {
        EstimateValidity::StaleMeasurements => "stale_measurement".to_owned(),
        EstimateValidity::InnovationTooLarge
            if correction_commanded_this_phase
                && matches!(
                    phase,
                    ControlPhase::PickCorrection | ControlPhase::SocketCorrection
                ) =>
        {
            "correction_non_convergence".to_owned()
        }
        EstimateValidity::OutlierBudgetExceeded
        | EstimateValidity::ResidualTooLarge
        | EstimateValidity::InnovationTooLarge
        | EstimateValidity::AxisScaleInconsistent => "observation_outlier".to_owned(),
        EstimateValidity::InsufficientFeatures
        | EstimateValidity::InsufficientMeasurements
        | EstimateValidity::NoMeasurements
        | EstimateValidity::NoTargetMeasurements => {
            let object_missing = burst
                .missing
                .iter()
                .filter(|missing| missing.object_id == object_id)
                .collect::<Vec<_>>();
            if !object_missing.is_empty()
                && object_missing
                    .iter()
                    .all(|missing| missing.reason == "stochastic_dropout")
            {
                "optical_dropout".to_owned()
            } else if object_id == SOCKET_OBJECT_ID
                && !object_missing.is_empty()
                && object_missing.iter().all(|missing| {
                    matches!(
                        missing.reason.as_str(),
                        "camera_occluded" | "projector_occluded"
                    )
                })
            {
                "required_mating_feature_occluded".to_owned()
            } else {
                "loss_of_observability".to_owned()
            }
        }
        _ => guard_reason,
    }
}

fn validate_post_motion_prediction(report: &PoseEstimate) -> Result<(), String> {
    match report.validity {
        EstimateValidity::Valid | EstimateValidity::StaleMeasurements => Ok(()),
        EstimateValidity::ExcessiveUncertainty => {
            Err("predicted_motion_uncertainty_limit".to_owned())
        }
        _ => Err(format!("estimator_prediction_{:?}", report.validity).to_ascii_lowercase()),
    }
}

fn plant_reason(error: PlantFailure) -> String {
    error.reason().to_owned()
}

fn contact_classification_reason(failure: ContactClassificationFailure) -> String {
    match failure {
        ContactClassificationFailure::ContactForceLimit => "contact_force_limit".to_owned(),
        ContactClassificationFailure::ContactPacketFromFuture => {
            "contact_packet_from_future".to_owned()
        }
        ContactClassificationFailure::StaleContactPacket => "stale_contact_packet".to_owned(),
        ContactClassificationFailure::RelativeGeometryFromFuture => {
            "contact_geometry_from_future".to_owned()
        }
        ContactClassificationFailure::StaleRelativeGeometry => "stale_contact_geometry".to_owned(),
        ContactClassificationFailure::InconsistentContactGeometry => {
            "impossible_contact_geometry".to_owned()
        }
        ContactClassificationFailure::ExcessiveRelativeUncertainty => {
            "excessive_contact_uncertainty".to_owned()
        }
        other => format!("contact_classification_{other:?}").to_ascii_lowercase(),
    }
}

fn prefer_failure_reason(current: &mut String, candidate: String) {
    if failure_reason_priority(&candidate) > failure_reason_priority(current) {
        *current = candidate;
    }
}

fn failure_reason_priority(reason: &str) -> u8 {
    match reason {
        "observation_outlier" => 100,
        "excessive_calibration_bias" => 90,
        "stale_measurement" => 80,
        "required_mating_feature_occluded" => 70,
        "correction_non_convergence" => 60,
        "optical_dropout" => 1,
        "calibration_reference_unobservable" => 0,
        "loss_of_observability" => 0,
        _ => 50,
    }
}

fn relative_mating_pose(
    estimates: &BTreeMap<u32, EstimateView>,
    peg_tip_offset_m: f64,
    socket_seat_offset_m: f64,
) -> Result<Option<RelativeMatingPose>, String> {
    let peg = estimates.get(&PEG_OBJECT_ID);
    let socket = estimates.get(&SOCKET_OBJECT_ID);
    let (Some(peg), Some(socket)) = (peg, socket) else {
        return Ok(None);
    };
    if !peg.valid || !socket.valid {
        return Err("contact_relative_estimate_invalid".to_owned());
    }
    if !peg_tip_offset_m.is_finite()
        || peg_tip_offset_m <= 0.0
        || !socket_seat_offset_m.is_finite()
        || socket_seat_offset_m <= 0.0
    {
        return Err("contact_mating_datum_invalid".to_owned());
    }
    let peg_tip_world_m = add(
        peg.position_world_m,
        scale(peg.axis_world, peg_tip_offset_m),
    );
    let socket_seat_world_m = add(
        socket.position_world_m,
        scale(socket.axis_world, socket_seat_offset_m),
    );
    let delta = sub(peg_tip_world_m, socket_seat_world_m);
    let axial_projection_m = dot(delta, socket.axis_world);
    Ok(Some(RelativeMatingPose {
        captured_at_tick: peg.capture_tick.min(socket.capture_tick),
        available_at_tick: peg.available_tick.max(socket.available_tick),
        axial_error_m: axial_projection_m.abs(),
        lateral_error_m: norm(sub(delta, scale(socket.axis_world, axial_projection_m))),
        axis_error_rad: axis_angle_rad(peg.axis_world, socket.axis_world),
        position_sigma_m: peg.position_sigma_m + socket.position_sigma_m,
        axis_sigma_rad: peg.axis_sigma_rad + socket.axis_sigma_rad,
    }))
}

fn gate(id: &'static str, passed: bool, evidence: impl Into<String>) -> AcceptanceGate {
    AcceptanceGate {
        id,
        applicable: true,
        passed,
        evidence: evidence.into(),
    }
}

fn nominal_gate(
    id: &'static str,
    nominal: bool,
    passed: bool,
    evidence: impl Into<String>,
) -> AcceptanceGate {
    AcceptanceGate {
        id,
        applicable: nominal,
        passed: nominal && passed,
        evidence: evidence.into(),
    }
}

fn measurement_age_s(now_tick: u64, estimate: &EstimateView, fixed_dt_s: f64) -> f64 {
    now_tick.saturating_sub(estimate.capture_tick) as f64 * fixed_dt_s
}

fn seconds_to_ticks_floor(seconds: f64, fixed_dt_s: f64) -> u64 {
    (seconds / fixed_dt_s).floor().max(0.0) as u64
}

fn seconds_to_ticks_ceil(seconds: f64, fixed_dt_s: f64) -> u64 {
    (seconds / fixed_dt_s).ceil().max(0.0) as u64
}

fn axis_angle_rad(a: [f64; 3], b: [f64; 3]) -> f64 {
    let denominator = norm(a) * norm(b);
    if denominator <= f64::EPSILON {
        f64::INFINITY
    } else {
        (dot(a, b) / denominator).clamp(-1.0, 1.0).acos()
    }
}

#[derive(Serialize)]
struct ControllerHashMaterial<'a> {
    schema: &'static str,
    scenario_sha256: &'a str,
    machine_config_id: &'a str,
    machine_config_sha256: &'a str,
    fault: M1eFault,
    status: &'a str,
    terminal_reason: &'a Option<String>,
    timing: &'a TimingReport,
    metrics: &'a ControllerMetrics,
    observations: &'a [ObservationBurstRecord],
    estimator_updates: &'a [EstimatorUpdateRecord],
    uncertainty_guards: &'a [UncertaintyGuardRecord],
    force_interlocks: &'a [ForceInterlockRecord],
    corrections: &'a [CorrectionIterationRecord],
    decisions: &'a [DecisionRecord],
    gates: &'a [AcceptanceGate],
}

#[allow(clippy::too_many_arguments)]
fn controller_hash(
    scenario_sha256: &str,
    machine_config_id: &str,
    machine_config_sha256: &str,
    fault: M1eFault,
    status: &str,
    terminal_reason: &Option<String>,
    timing: &TimingReport,
    metrics: &ControllerMetrics,
    observations: &[ObservationBurstRecord],
    estimator_updates: &[EstimatorUpdateRecord],
    uncertainty_guards: &[UncertaintyGuardRecord],
    force_interlocks: &[ForceInterlockRecord],
    corrections: &[CorrectionIterationRecord],
    decisions: &[DecisionRecord],
    gates: &[AcceptanceGate],
) -> String {
    let material = ControllerHashMaterial {
        schema: "pipe_m1e_controller_report/v1",
        scenario_sha256,
        machine_config_id,
        machine_config_sha256,
        fault,
        status,
        terminal_reason,
        timing,
        metrics,
        observations,
        estimator_updates,
        uncertainty_guards,
        force_interlocks,
        corrections,
        decisions,
        gates,
    };
    let bytes = serde_json::to_vec(&material).expect("M1e hash material is serializable");
    sha256_hex(&bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_m1e_completes_from_observations() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        let report = runtime.run_cycle().unwrap();
        assert_eq!(report.status, "complete", "{report:#?}");
        assert_eq!(report.phase, ControlPhase::Complete);
        assert!(report.expected_outcome_observed);
        assert_eq!(report.truth_firewall.controller_truth_access_count, 0);
        assert_eq!(report.timing.stale_near_contact_command_count, 0);
        assert!(
            report.acceptance_gates.iter().all(|gate| gate.passed),
            "{:#?}",
            report.acceptance_gates
        );
        for action in [
            "post_grasp_pose_confirmed",
            "post_retract_pose_reacquired",
            "transfer_preflight_accepted",
        ] {
            assert!(
                report
                    .decisions
                    .iter()
                    .any(|decision| decision.action == action),
                "missing observed-state transition decision {action}"
            );
        }
    }

    #[test]
    fn calibrated_pick_capture_standoff_is_bounded_clear_and_observable() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        let capture = runtime.capture_entry_geometry().unwrap();
        let calibrated_axis = runtime.plant.calibrated_socket_axis_world();
        let expected_target = sub(
            runtime.scenario.coupon.pick_peg_center_nominal_world_m,
            scale(
                calibrated_axis,
                runtime.scenario.motion.pick_capture_axial_standoff_m,
            ),
        );
        assert_eq!(
            capture.start_world_m,
            runtime.commanded_tool_position_world_m
        );
        assert_eq!(capture.target_world_m, expected_target);
        assert!((capture.blind_sweep_length_m - 1.0e-3).abs() < 1.0e-15);
        assert!(
            capture.maximum_required_correction_m <= runtime.scenario.motion.maximum_correction_m
        );
        assert!(
            capture.minimum_open_jaw_clearance_m
                >= runtime.scenario.safety.minimum_obstacle_clearance_m
        );
        assert!(
            capture.minimum_side_target_to_peg_clearance_m
                >= runtime.scenario.safety.minimum_obstacle_clearance_m
        );
        assert!(
            capture.minimum_palm_to_peg_clearance_m
                >= runtime.scenario.safety.minimum_obstacle_clearance_m
        );
        assert!(
            capture.minimum_jaw_to_peg_feature_clearance_m
                >= runtime.scenario.safety.minimum_obstacle_clearance_m
        );

        runtime.enter_capture_volume().unwrap();
        assert_eq!(runtime.commanded_tool_position_world_m, expected_target);
        assert!(runtime
            .decisions
            .iter()
            .any(|decision| decision.action == "open_gripper_for_capture"));
        assert!(runtime.decisions.iter().any(|decision| {
            decision.action == "capture_entry_guard_accepted"
                && decision.target_world_m == Some(expected_target)
        }));

        runtime.phase = ControlPhase::PickCorrection;
        let estimates = runtime
            .observe_required(
                runtime.phase,
                "pick_capture_test",
                runtime.scenario.coupon.pick_peg_center_nominal_world_m,
                &[TOOL_OBJECT_ID, PEG_OBJECT_ID],
                true,
            )
            .unwrap();
        assert!(required_view(&estimates, TOOL_OBJECT_ID).unwrap().valid);
        assert!(required_view(&estimates, PEG_OBJECT_ID).unwrap().valid);
    }

    #[test]
    fn invalid_capture_clearance_is_rejected_before_any_actuation() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        runtime.scenario.safety.minimum_obstacle_clearance_m = 0.150e-3;
        let decision_count = runtime.decisions.len();

        assert_eq!(
            runtime.enter_capture_volume().unwrap_err(),
            "capture_peg_feature_clearance_insufficient"
        );
        assert_eq!(runtime.decisions.len(), decision_count);
        assert_eq!(runtime.plant.now_tick(), 0);
    }

    #[test]
    fn carried_preflight_rejects_missing_estimate_instead_of_using_phase_limit() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        runtime.phase = ControlPhase::Transfer;
        runtime.grasp_confirmed = true;
        let target = add(runtime.commanded_tool_position_world_m, [1.0e-3, 0.0, 0.0]);
        let tool = valid_test_estimate(TOOL_OBJECT_ID, runtime.plant.now_tick());
        assert_eq!(
            runtime
                .preflight_estimated_motion(target, MotionClass::Transit, &[tool])
                .unwrap_err(),
            "carried_pose_estimate_required"
        );
    }

    #[test]
    fn near_contact_motion_rejects_missing_tool_estimate() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        runtime.phase = ControlPhase::Retreat;
        let target = add(runtime.commanded_tool_position_world_m, [0.0, 0.0, -1.0e-3]);
        assert_eq!(
            runtime
                .command_motion(target, MotionClass::Retreat, true, Vec::new())
                .unwrap_err(),
            "tool_pose_estimate_required"
        );
    }

    #[test]
    fn terminal_failure_records_stop_and_never_uses_an_unvalidated_retreat() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        runtime.grasp_confirmed = true;
        runtime.fail_closed("injected_test_guard".to_owned());

        let hold = runtime.decisions.last().unwrap();
        assert_eq!(hold.action, "fail_closed_hold");
        assert!(hold.command_sequence.is_some());
        assert!(hold.reason.contains("authoritative stop accepted"));
        assert!(!runtime
            .decisions
            .iter()
            .any(|decision| decision.action == "fail_closed_retreat"));
        assert_eq!(runtime.phase, ControlPhase::FailedSafe);
    }

    #[test]
    fn injected_fault_reports_nominal_gates_as_not_applicable() {
        let runtime = ObservedManipulationRuntime::new(M1eFault::OpticalDropout).unwrap();
        let gates = runtime.acceptance_gates();
        for id in [
            "observed_acquisition",
            "estimator_phase_uncertainty",
            "correction_convergence",
            "guarded_grasp",
            "collision_free_transfer",
            "guarded_insertion",
            "observed_and_contact_seat",
            "safe_release_and_retreat",
        ] {
            let gate = gates.iter().find(|gate| gate.id == id).unwrap();
            assert!(!gate.applicable, "{id}");
            assert!(!gate.passed, "{id}");
        }
        for id in [
            "no_controller_truth_access",
            "no_stale_near_contact_command",
            "force_proxy_limit",
        ] {
            assert!(gates.iter().find(|gate| gate.id == id).unwrap().applicable);
        }
    }

    #[test]
    fn moving_pose_prior_invalidation_blocks_old_views_until_fresh_burst() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        runtime.enter_capture_volume().unwrap();
        runtime.phase = ControlPhase::PickCorrection;
        let old_views = runtime
            .observe_required(
                runtime.phase,
                "pick",
                runtime.scenario.coupon.pick_peg_center_nominal_world_m,
                &[TOOL_OBJECT_ID, PEG_OBJECT_ID],
                true,
            )
            .unwrap();
        assert!(runtime
            .estimator(TOOL_OBJECT_ID)
            .unwrap()
            .last_report()
            .is_some());
        assert!(runtime
            .estimator(PEG_OBJECT_ID)
            .unwrap()
            .last_report()
            .is_some());

        runtime
            .invalidate_moving_pose_priors_before_reacquisition(false, true, "test motion")
            .unwrap();
        assert!(runtime
            .estimator(TOOL_OBJECT_ID)
            .unwrap()
            .last_report()
            .is_none());
        assert!(runtime
            .estimator(PEG_OBJECT_ID)
            .unwrap()
            .last_report()
            .is_some());
        assert_eq!(
            runtime
                .command_motion(
                    runtime.commanded_tool_position_world_m,
                    MotionClass::Correction,
                    true,
                    old_views.values().cloned().collect(),
                )
                .unwrap_err(),
            "moving_pose_reacquisition_required"
        );

        runtime
            .observe_required(
                runtime.phase,
                "pick",
                runtime.scenario.coupon.pick_peg_center_nominal_world_m,
                &[TOOL_OBJECT_ID, PEG_OBJECT_ID],
                true,
            )
            .unwrap();
        assert!(!runtime.moving_pose_reacquisition_required);
        assert!(runtime
            .estimator(TOOL_OBJECT_ID)
            .unwrap()
            .last_report()
            .is_some());
    }

    #[test]
    fn motion_preview_adds_travel_and_planned_duration_hold_uncertainty() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        runtime.phase = ControlPhase::Transfer;
        let start = valid_test_estimate(PEG_OBJECT_ID, runtime.plant.now_tick());
        let delta = [50.0e-6, 0.0, 0.0];
        let target = add(runtime.commanded_tool_position_world_m, delta);
        let preview = runtime
            .anticipated_motion_uncertainty(&start, target, MotionClass::Transit)
            .unwrap();
        let expected_interval_s = runtime
            .plant
            .preview_tool_motion_duration_s(target, MotionClass::Transit)
            .unwrap()
            + runtime.scenario.optics.settling_interval_s;
        let travel_sigma_m = runtime.scenario.estimator.free_prediction_sigma_m_per_m * norm(delta);
        let hold_sigma_m = runtime
            .scenario
            .estimator
            .loaded_hold_process_sigma_m_per_sqrt_s
            * expected_interval_s.sqrt();
        let capabilities = runtime.plant.motion_capabilities();
        let actuation_bound_sigma_m = (0.5 * capabilities.differential_backlash_m
            + 0.5 * capabilities.minimum_reproducible_correction_m)
            / 3.0;
        let expected_sigma_m = (start.position_sigma_m.powi(2)
            + travel_sigma_m.powi(2)
            + hold_sigma_m.powi(2)
            + actuation_bound_sigma_m.powi(2))
        .sqrt();
        assert_eq!(preview.interval_s.to_bits(), expected_interval_s.to_bits());
        assert_eq!(
            preview.position_sigma_m.to_bits(),
            expected_sigma_m.to_bits()
        );
        assert!(preview.position_sigma_m > start.position_sigma_m);
        assert!(preview.axis_sigma_rad > start.axis_sigma_rad);
    }

    #[test]
    fn rejected_correction_is_present_in_structured_iteration_report() {
        let mut runtime =
            ObservedManipulationRuntime::new(M1eFault::CorrectionFloorTooLarge).unwrap();
        let report = runtime.run_cycle().unwrap();
        assert!(report
            .correction_iterations
            .iter()
            .any(|record| record.outcome == "correction_floor_too_large"));
    }

    fn valid_test_estimate(object_id: u32, tick: u64) -> EstimateView {
        EstimateView {
            object_id,
            valid: true,
            invalid_reason: None,
            position_world_m: [0.02, 0.0, 0.0],
            axis_world: [0.0, 0.0, 1.0],
            position_sigma_m: 3.0e-6,
            axis_sigma_rad: 3.0e-3,
            capture_tick: tick,
            available_tick: tick,
            distinct_feature_count: 4,
            triangulation_head_count: 1,
            minimum_calibrated_rays_per_point: 2,
            residual_rms_m: 1.0e-6,
            provenance: EstimateProvenance::direct_feature_fit(),
        }
    }

    #[test]
    fn native_report_and_controller_hash_are_exactly_replayable() {
        let mut first = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        let first = first.run_cycle().unwrap();
        let mut second = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        let second = second.run_cycle().unwrap();
        assert_eq!(first, second);
        assert_eq!(
            first.to_json(false).unwrap(),
            second.to_json(false).unwrap()
        );
        assert_eq!(first.controller_report_sha256.len(), 64);
    }

    #[test]
    fn observation_outlier_reason_survives_later_generic_loss() {
        let mut reason = "loss_of_observability".to_owned();
        prefer_failure_reason(&mut reason, "observation_outlier".to_owned());
        prefer_failure_reason(&mut reason, "loss_of_observability".to_owned());
        assert_eq!(reason, "observation_outlier");
    }

    #[test]
    fn every_injected_fault_stops_for_its_declared_reason() {
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
    fn controller_hash_excludes_evaluation_only_truth() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        let mut report = runtime.run_cycle().unwrap();
        let hash = report.controller_report_sha256.clone();
        let recompute = |report: &ObservedManipulationReport| {
            controller_hash(
                &report.scenario_sha256,
                &report.machine_config_id,
                &report.machine_config_sha256,
                M1eFault::None,
                report.status,
                &report.terminal_reason,
                &report.timing,
                &report.metrics,
                &report.observation_bursts,
                &report.estimator_updates,
                &report.uncertainty_guards,
                &report.force_interlocks,
                &report.correction_iterations,
                &report.decisions,
                &report.acceptance_gates,
            )
        };
        assert_eq!(recompute(&report), hash);
        let evaluation = report
            .evaluation_only_truth
            .as_mut()
            .expect("terminal reports carry evaluation-only truth metrics");
        evaluation.final_peg_tip_to_socket_seat_error_m += 1.0;
        evaluation.maximum_unplanned_penetration_m += 1.0;
        evaluation.within_declared_seat_tolerances = false;
        assert_eq!(recompute(&report), hash);
    }

    #[test]
    fn nonterminal_reports_do_not_compute_or_serialize_truth_evaluation() {
        let runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        let report = runtime.report();
        assert_eq!(report.status, "ready");
        assert!(report.evaluation_only_truth.is_none());
        assert!(report
            .to_json(false)
            .unwrap()
            .contains("\"evaluation_only_truth\":null"));
    }

    #[test]
    fn raw_optical_and_plant_types_do_not_enter_runtime_decisions() {
        let source = include_str!("runtime.rs");
        let forbidden = [
            ["Depth", "Sample"].concat(),
            ["Scene", "Frame"].concat(),
            ["true", "_point"].concat(),
            ["true", "_range_m"].concat(),
        ];
        for forbidden in forbidden {
            assert!(
                !source.contains(&forbidden),
                "runtime references {forbidden}"
            );
        }
        assert!(source.contains("evaluation_metrics"));
    }

    #[test]
    fn configured_initial_truth_errors_are_not_referenced_by_the_executive() {
        let source = include_str!("runtime.rs");
        for forbidden in [
            ["initial", "_peg_error_m"].concat(),
            ["initial", "_socket_error_m"].concat(),
            ["initial", "_tool_command_error_m"].concat(),
            ["initial", "_peg_axis_tilt_rad"].concat(),
            ["initial", "_socket_axis_tilt_rad"].concat(),
            ["initial", "_tool_axis_tilt_rad"].concat(),
        ] {
            assert!(
                !source.contains(&forbidden),
                "runtime executive references plant-initialization field {forbidden}"
            );
        }
    }
}
