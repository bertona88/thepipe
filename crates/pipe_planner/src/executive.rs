use crate::{
    AlignmentObservation, AssemblyScenario, Command, ComponentId, ComponentPlan, ConfigError,
    Decision, EventKind, ExecutiveStatus, FailureReason, FailureSeverity, GripperObservation,
    HandoffObservation, InsertionObservation, MeshObservation, Phase, PoseCorrection, PoseError6d,
    SensorFrame, ServoPurpose, TaskEvent, TaskMetrics, VerificationObservation, VisionObservation,
};

/// Deterministic, single-threaded gearbox task executive.
///
/// Call [`tick`](Self::tick) once per synchronized perception/force packet.
/// The executive never reads wall-clock time, uses randomness, or performs I/O;
/// an identical scenario and frame sequence produces an identical command and
/// event sequence.
pub struct GearboxTaskExecutive {
    scenario: AssemblyScenario,
    component_index: usize,
    phase: Phase,
    status: ExecutiveStatus,
    last_control_time_ms: Option<u64>,
    phase_cycles: u16,
    servo_steps: u16,
    retry_counts: [u8; 7],
    next_event_sequence: u64,
    events: Vec<TaskEvent>,
    metrics: TaskMetrics,
    dither_positive: bool,
}

impl GearboxTaskExecutive {
    pub fn new(scenario: AssemblyScenario) -> Result<Self, ConfigError> {
        scenario.validate()?;
        Ok(Self {
            scenario,
            component_index: 0,
            phase: Phase::Locate,
            status: ExecutiveStatus::Running,
            last_control_time_ms: None,
            phase_cycles: 0,
            servo_steps: 0,
            retry_counts: [0; 7],
            next_event_sequence: 0,
            events: Vec::new(),
            metrics: TaskMetrics::default(),
            dither_positive: true,
        })
    }

    pub fn scenario(&self) -> &AssemblyScenario {
        &self.scenario
    }

    pub fn status(&self) -> &ExecutiveStatus {
        &self.status
    }

    pub fn phase(&self) -> Option<Phase> {
        (!matches!(&self.status, ExecutiveStatus::Completed)).then_some(self.phase)
    }

    pub fn active_component(&self) -> Option<&ComponentPlan> {
        self.scenario.recipe.components.get(self.component_index)
    }

    pub fn events(&self) -> &[TaskEvent] {
        &self.events
    }

    pub fn metrics(&self) -> &TaskMetrics {
        &self.metrics
    }

    pub fn retries_for(&self, phase: Phase) -> u8 {
        self.retry_counts[phase.index()]
    }

    pub fn tick(&mut self, frame: &SensorFrame) -> Decision {
        let first_new_event = self.events.len();
        let command = match self.status.clone() {
            ExecutiveStatus::Completed => Command::AssemblyComplete,
            ExecutiveStatus::Aborted(_) => Command::HoldPosition,
            ExecutiveStatus::Running => self.tick_running(frame),
        };

        self.metrics.commands_issued += 1;
        match &command {
            Command::VisualServo { .. } => self.metrics.visual_servo_corrections += 1,
            Command::GuardedInsert { .. } => self.metrics.guarded_insert_commands += 1,
            Command::MeshDither { .. } => self.metrics.mesh_dither_commands += 1,
            Command::CloseRetainerFeature { .. } => self.metrics.retainer_closure_commands += 1,
            _ => {}
        }
        self.emit(
            frame.control_time_ms,
            EventKind::CommandIssued(command.clone()),
        );

        Decision {
            status: self.status.clone(),
            component: self.active_component().map(|part| part.id),
            phase: self.phase(),
            command,
            events: self.events[first_new_event..].to_vec(),
        }
    }

    fn tick_running(&mut self, frame: &SensorFrame) -> Command {
        self.metrics.control_cycles += 1;

        if let Some(previous_ms) = self.last_control_time_ms {
            if frame.control_time_ms <= previous_ms {
                return self.fail(
                    frame.control_time_ms,
                    FailureReason::NonMonotonicTime {
                        previous_ms,
                        received_ms: frame.control_time_ms,
                    },
                    self.phase,
                    Command::HoldPosition,
                );
            }
        }
        self.last_control_time_ms = Some(frame.control_time_ms);

        if frame.capture_time_ms > frame.control_time_ms {
            return self.fail(
                frame.control_time_ms,
                FailureReason::FutureDatedSensorFrame {
                    capture_ms: frame.capture_time_ms,
                    control_ms: frame.control_time_ms,
                },
                self.phase,
                Command::HoldPosition,
            );
        }

        if !finite_nonnegative(frame.min_unplanned_clearance_mm) {
            return self.fail(
                frame.control_time_ms,
                FailureReason::InvalidMeasurement {
                    field: "min_unplanned_clearance_mm",
                },
                self.phase,
                Command::HoldPosition,
            );
        }
        if !finite_nonnegative(frame.unplanned_contact_force_n) {
            return self.fail(
                frame.control_time_ms,
                FailureReason::InvalidMeasurement {
                    field: "unplanned_contact_force_n",
                },
                self.phase,
                Command::HoldPosition,
            );
        }
        self.metrics.min_unplanned_clearance_mm = Some(
            self.metrics
                .min_unplanned_clearance_mm
                .map_or(frame.min_unplanned_clearance_mm, |prior| {
                    prior.min(frame.min_unplanned_clearance_mm)
                }),
        );
        self.metrics.max_unplanned_contact_force_n = self
            .metrics
            .max_unplanned_contact_force_n
            .max(frame.unplanned_contact_force_n);

        let safety = self.scenario.acceptance.safety;
        let age_ms = frame.control_time_ms - frame.capture_time_ms;
        if age_ms > safety.max_sensor_age_ms {
            return self.fail(
                frame.control_time_ms,
                FailureReason::StaleSensorFrame {
                    age_ms,
                    limit_ms: safety.max_sensor_age_ms,
                },
                self.phase,
                Command::HoldPosition,
            );
        }
        if frame.unplanned_contact_force_n > safety.max_unplanned_contact_force_n {
            return self.fail(
                frame.control_time_ms,
                FailureReason::UnplannedContactForceHigh {
                    measured_n: frame.unplanned_contact_force_n,
                    limit_n: safety.max_unplanned_contact_force_n,
                },
                self.phase,
                Command::HoldPosition,
            );
        }
        if frame.min_unplanned_clearance_mm < safety.min_unplanned_clearance_mm {
            return self.fail(
                frame.control_time_ms,
                FailureReason::UnplannedClearanceTooSmall {
                    measured_mm: frame.min_unplanned_clearance_mm,
                    limit_mm: safety.min_unplanned_clearance_mm,
                },
                self.phase,
                Command::HoldPosition,
            );
        }

        self.phase_cycles = self.phase_cycles.saturating_add(1);
        if self.phase_cycles > self.scenario.acceptance.retries.max_cycles_per_attempt {
            let recovery = if self.phase == Phase::Insert {
                Phase::Align
            } else {
                self.phase
            };
            let command = self.recovery_command();
            return self.fail(
                frame.control_time_ms,
                FailureReason::PhaseTimeout {
                    cycles: self.phase_cycles,
                },
                recovery,
                command,
            );
        }

        match self.phase {
            Phase::Locate => self.locate(frame),
            Phase::Pick => self.pick(frame),
            Phase::Handoff => self.handoff(frame),
            Phase::Align => self.align(frame),
            Phase::Insert => self.insert(frame),
            Phase::Mesh => self.mesh(frame),
            Phase::Verify => self.verify(frame),
        }
    }

    fn locate(&mut self, frame: &SensorFrame) -> Command {
        let component = self.component_id();
        let observation = match frame.vision {
            Some(value) => value,
            None => return Command::SearchVolume { component },
        };
        if let Err(reason) = self.check_vision(observation) {
            return self.fail(
                frame.control_time_ms,
                reason,
                Phase::Locate,
                Command::SearchVolume { component },
            );
        }
        self.transition_success(frame.control_time_ms, Phase::Pick);
        Command::MoveToPregrasp {
            component,
            speed_mm_s: self.scenario.acceptance.servo.speed_mm_s,
        }
    }

    fn pick(&mut self, frame: &SensorFrame) -> Command {
        let component = self.component_id();
        let vision = match frame.vision {
            Some(value) => value,
            None => {
                return Command::MoveToPregrasp {
                    component,
                    speed_mm_s: self.scenario.acceptance.servo.speed_mm_s,
                }
            }
        };
        if let Err(reason) = self.check_vision(vision) {
            return self.fail_with_recovery(frame.control_time_ms, reason, Phase::Pick);
        }
        let limits = self.scenario.acceptance.grasp;
        if vision.target_error.translation_norm_mm() > limits.pregrasp_translation_tolerance_mm
            || vision.target_error.rotation_norm_deg() > limits.pregrasp_rotation_tolerance_deg
        {
            return self.servo_or_fail(
                frame.control_time_ms,
                vision.target_error,
                ServoPurpose::Pregrasp,
                Phase::Pick,
            );
        }

        let gripper = match frame.gripper {
            Some(value) => value,
            None => return self.close_gripper_command(component),
        };
        if let Err(reason) = validate_gripper(gripper, component) {
            return self.fail_with_recovery(frame.control_time_ms, reason, Phase::Pick);
        }
        if gripper.force_n > limits.max_force_n {
            return self.fail(
                frame.control_time_ms,
                FailureReason::GripForceHigh {
                    measured_n: gripper.force_n,
                    limit_n: limits.max_force_n,
                },
                Phase::Pick,
                Command::HoldPosition,
            );
        }
        if gripper.slip_mm > limits.max_slip_mm {
            return self.fail_with_recovery(
                frame.control_time_ms,
                FailureReason::GripSlipHigh {
                    measured_mm: gripper.slip_mm,
                    limit_mm: limits.max_slip_mm,
                },
                Phase::Pick,
            );
        }
        if !gripper.part_retained || gripper.force_n < limits.min_force_n {
            return self.close_gripper_command(component);
        }

        if self.component().requires_handoff {
            self.transition_success(frame.control_time_ms, Phase::Handoff);
            Command::PresentForHandoff { component }
        } else {
            self.transition_success(frame.control_time_ms, Phase::Align);
            Command::MoveToBore {
                component,
                speed_mm_s: self.scenario.acceptance.servo.speed_mm_s,
            }
        }
    }

    fn handoff(&mut self, frame: &SensorFrame) -> Command {
        let component = self.component_id();
        let observation = match frame.handoff {
            Some(value) => value,
            None => return Command::PresentForHandoff { component },
        };
        if let Err(reason) = validate_handoff(observation, component) {
            return self.fail_with_recovery(frame.control_time_ms, reason, Phase::Handoff);
        }
        let limits = self.scenario.acceptance.handoff;
        if observation.target_error.translation_norm_mm() > limits.max_translation_error_mm
            || observation.target_error.rotation_norm_deg() > limits.max_rotation_error_deg
        {
            return self.servo_or_fail(
                frame.control_time_ms,
                observation.target_error,
                ServoPurpose::Handoff,
                Phase::Handoff,
            );
        }
        if observation.receiver_force_n > limits.max_receiver_force_n {
            return self.fail(
                frame.control_time_ms,
                FailureReason::HandoffForceHigh {
                    measured_n: observation.receiver_force_n,
                    limit_n: limits.max_receiver_force_n,
                },
                Phase::Handoff,
                Command::HoldPosition,
            );
        }
        if !observation.receiver_has_part
            || observation.receiver_force_n < limits.min_receiver_force_n
        {
            return Command::CloseReceiverGripper {
                component,
                target_force_n: midpoint(limits.min_receiver_force_n, limits.max_receiver_force_n),
            };
        }
        if !observation.donor_released {
            return Command::ReleaseDonorGripper { component };
        }
        self.transition_success(frame.control_time_ms, Phase::Align);
        Command::MoveToBore {
            component,
            speed_mm_s: self.scenario.acceptance.servo.speed_mm_s,
        }
    }

    fn align(&mut self, frame: &SensorFrame) -> Command {
        let component = self.component_id();
        let observation = match frame.alignment {
            Some(value) => value,
            None => {
                return Command::MoveToBore {
                    component,
                    speed_mm_s: self.scenario.acceptance.servo.speed_mm_s,
                }
            }
        };
        if let Err(reason) = validate_alignment(observation, component) {
            return self.fail_with_recovery(frame.control_time_ms, reason, Phase::Align);
        }
        let limits = self.scenario.acceptance.alignment;
        if observation.bore_error.lateral_norm_mm() > limits.max_lateral_error_mm
            || observation.bore_error.axial_abs_mm() > limits.max_axial_error_mm
            || observation.bore_error.rotation_norm_deg() > limits.max_rotation_error_deg
        {
            return self.servo_or_fail(
                frame.control_time_ms,
                observation.bore_error,
                ServoPurpose::BoreAlignment,
                Phase::Align,
            );
        }
        self.transition_success(frame.control_time_ms, Phase::Insert);
        self.guarded_insert_command(component)
    }

    fn insert(&mut self, frame: &SensorFrame) -> Command {
        let component = self.component_id();
        let observation = match frame.insertion {
            Some(value) => value,
            None => return Command::HoldPosition,
        };
        if let Err(reason) = validate_insertion(observation, component) {
            return self.fail_with_recovery(frame.control_time_ms, reason, Phase::Align);
        }
        self.metrics.max_insertion_axial_force_n = self
            .metrics
            .max_insertion_axial_force_n
            .max(observation.axial_force_n);
        self.metrics.max_insertion_lateral_force_n = self
            .metrics
            .max_insertion_lateral_force_n
            .max(observation.lateral_force_n);

        let limits = self.scenario.acceptance.insertion;
        let target = self.component().insertion_travel_mm;
        let depth_tolerance = self.component().insertion_depth_tolerance_mm;
        let axial_force_limit = self.component().max_insertion_axial_force_n;
        let lateral_force_limit = self.component().max_insertion_lateral_force_n;
        let required_closures = self.component().required_closure_features;
        if observation.depth_mm > target + limits.max_overshoot_mm {
            return self.fail(
                frame.control_time_ms,
                FailureReason::InsertionDepthOvershoot {
                    measured_mm: observation.depth_mm,
                    target_mm: target,
                    limit_mm: limits.max_overshoot_mm,
                },
                Phase::Insert,
                Command::HoldPosition,
            );
        }
        if observation.axial_force_n > axial_force_limit {
            return self.fail_with_recovery(
                frame.control_time_ms,
                FailureReason::InsertionAxialForceHigh {
                    measured_n: observation.axial_force_n,
                    limit_n: axial_force_limit,
                },
                Phase::Align,
            );
        }
        if observation.lateral_force_n > lateral_force_limit {
            return self.fail_with_recovery(
                frame.control_time_ms,
                FailureReason::InsertionLateralForceHigh {
                    measured_n: observation.lateral_force_n,
                    limit_n: lateral_force_limit,
                },
                Phase::Align,
            );
        }
        let depth_error = (target - observation.depth_mm).abs();
        if observation.seated && depth_error > depth_tolerance {
            return self.fail_with_recovery(
                frame.control_time_ms,
                FailureReason::PrematureSeat {
                    measured_mm: observation.depth_mm,
                    target_mm: target,
                },
                Phase::Align,
            );
        }
        if observation.seated || depth_error <= depth_tolerance {
            if observation.closure_features_confirmed < required_closures {
                return Command::CloseRetainerFeature {
                    component,
                    feature: observation.closure_features_confirmed + 1,
                    axial_force_limit_n: axial_force_limit,
                };
            }
            if self.component().requires_mesh {
                self.transition_success(frame.control_time_ms, Phase::Mesh);
                self.mesh_dither_command(component)
            } else {
                self.transition_success(frame.control_time_ms, Phase::Verify);
                self.verification_command(component)
            }
        } else {
            self.guarded_insert_command(component)
        }
    }

    fn mesh(&mut self, frame: &SensorFrame) -> Command {
        let component = self.component_id();
        let observation = match frame.mesh {
            Some(value) => value,
            None => return self.mesh_dither_command(component),
        };
        if let Err(reason) = validate_mesh(observation, component) {
            return self.fail_with_mesh_dither(frame.control_time_ms, reason, component);
        }
        self.metrics.max_mesh_torque_mn_mm = self
            .metrics
            .max_mesh_torque_mn_mm
            .max(observation.peak_torque_mn_mm);
        let limits = self.scenario.acceptance.mesh;
        if observation.peak_torque_mn_mm > limits.max_peak_torque_mn_mm {
            return self.fail_with_mesh_dither(
                frame.control_time_ms,
                FailureReason::MeshTorqueHigh {
                    measured_mn_mm: observation.peak_torque_mn_mm,
                    limit_mn_mm: limits.max_peak_torque_mn_mm,
                },
                component,
            );
        }
        if observation.sweep_deg < limits.min_sweep_deg {
            return self.mesh_dither_command(component);
        }
        if !observation.teeth_engaged {
            return self.fail_with_mesh_dither(
                frame.control_time_ms,
                FailureReason::MeshNotEngaged,
                component,
            );
        }
        if !(limits.min_backlash_mm..=limits.max_backlash_mm).contains(&observation.backlash_mm) {
            return self.fail_with_mesh_dither(
                frame.control_time_ms,
                FailureReason::MeshBacklashOutOfRange {
                    measured_mm: observation.backlash_mm,
                    min_mm: limits.min_backlash_mm,
                    max_mm: limits.max_backlash_mm,
                },
                component,
            );
        }
        self.transition_success(frame.control_time_ms, Phase::Verify);
        self.verification_command(component)
    }

    fn verify(&mut self, frame: &SensorFrame) -> Command {
        let component = self.component_id();
        let observation = match frame.verification {
            Some(value) => value,
            None => return self.verification_command(component),
        };
        if let Err(reason) = validate_verification(observation, component) {
            return self.fail_with_verification(frame.control_time_ms, reason, component);
        }
        let limits = self.scenario.acceptance.verification;
        if observation.vision_confidence < limits.min_vision_confidence {
            return self.fail_with_verification(
                frame.control_time_ms,
                FailureReason::VerificationVisionLow {
                    measured: observation.vision_confidence,
                    limit: limits.min_vision_confidence,
                },
                component,
            );
        }
        if observation.target_error.translation_norm_mm() > limits.max_translation_error_mm
            || observation.target_error.rotation_norm_deg() > limits.max_rotation_error_deg
        {
            return self.fail_with_verification(
                frame.control_time_ms,
                FailureReason::VerificationPoseOutOfTolerance,
                component,
            );
        }
        if !observation.required_features_visible {
            return self.fail_with_verification(
                frame.control_time_ms,
                FailureReason::ComponentOccludedAtVerification,
                component,
            );
        }
        if observation.rotation_test_deg < limits.min_rotation_test_deg {
            return self.verification_command(component);
        }
        if observation.peak_torque_mn_mm > limits.max_peak_torque_mn_mm {
            return self.fail_with_verification(
                frame.control_time_ms,
                FailureReason::VerificationTorqueHigh {
                    measured_mn_mm: observation.peak_torque_mn_mm,
                    limit_mn_mm: limits.max_peak_torque_mn_mm,
                },
                component,
            );
        }
        if observation.torque_ripple_fraction > limits.max_torque_ripple_fraction {
            return self.fail_with_verification(
                frame.control_time_ms,
                FailureReason::VerificationTorqueRippleHigh {
                    measured: observation.torque_ripple_fraction,
                    limit: limits.max_torque_ripple_fraction,
                },
                component,
            );
        }
        if !(limits.min_backlash_mm..=limits.max_backlash_mm).contains(&observation.backlash_mm) {
            return self.fail_with_verification(
                frame.control_time_ms,
                FailureReason::VerificationBacklashOutOfRange {
                    measured_mm: observation.backlash_mm,
                    min_mm: limits.min_backlash_mm,
                    max_mm: limits.max_backlash_mm,
                },
                component,
            );
        }

        self.emit(frame.control_time_ms, EventKind::ComponentCompleted);
        self.metrics.components_completed += 1;
        self.component_index += 1;
        self.retry_counts = [0; 7];
        self.phase_cycles = 0;
        self.servo_steps = 0;
        if self.component_index == self.scenario.recipe.components.len() {
            self.status = ExecutiveStatus::Completed;
            self.emit(frame.control_time_ms, EventKind::AssemblyCompleted);
            Command::AssemblyComplete
        } else {
            let from = self.phase;
            self.phase = Phase::Locate;
            self.metrics.transitions += 1;
            self.emit(
                frame.control_time_ms,
                EventKind::Transition {
                    from,
                    to: Phase::Locate,
                },
            );
            Command::SearchVolume {
                component: self.component_id(),
            }
        }
    }

    fn check_vision(&self, observation: VisionObservation) -> Result<(), FailureReason> {
        let component = self.component_id();
        if observation.component != component {
            return Err(FailureReason::WrongComponent {
                expected: component,
                observed: observation.component,
            });
        }
        if !unit_interval(observation.confidence)
            || !finite_nonnegative(observation.position_sigma_mm)
            || !finite_nonnegative(observation.orientation_sigma_deg)
            || !observation.target_error.is_finite()
        {
            return Err(FailureReason::InvalidMeasurement { field: "vision" });
        }
        let limits = self.scenario.acceptance.vision;
        if observation.confidence < limits.min_confidence {
            return Err(FailureReason::VisionConfidenceLow {
                measured: observation.confidence,
                limit: limits.min_confidence,
            });
        }
        if observation.position_sigma_mm > limits.max_position_sigma_mm {
            return Err(FailureReason::PositionUncertaintyHigh {
                measured_mm: observation.position_sigma_mm,
                limit_mm: limits.max_position_sigma_mm,
            });
        }
        if observation.orientation_sigma_deg > limits.max_orientation_sigma_deg {
            return Err(FailureReason::OrientationUncertaintyHigh {
                measured_deg: observation.orientation_sigma_deg,
                limit_deg: limits.max_orientation_sigma_deg,
            });
        }
        Ok(())
    }

    fn servo_or_fail(
        &mut self,
        control_time_ms: u64,
        error: PoseError6d,
        purpose: ServoPurpose,
        retry_phase: Phase,
    ) -> Command {
        let limits = self.scenario.acceptance.servo;
        if self.servo_steps >= limits.max_steps_per_attempt {
            return self.fail_with_recovery(
                control_time_ms,
                FailureReason::VisualServoDidNotConverge {
                    steps: self.servo_steps,
                },
                retry_phase,
            );
        }
        self.servo_steps += 1;
        Command::VisualServo {
            component: self.component_id(),
            purpose,
            correction: PoseCorrection {
                translation_mm: scaled_clamped_negative(
                    error.translation_mm,
                    limits.translation_gain,
                    limits.max_translation_step_mm,
                ),
                rotation_deg: scaled_clamped_negative(
                    error.rotation_deg,
                    limits.rotation_gain,
                    limits.max_rotation_step_deg,
                ),
            },
            speed_mm_s: limits.speed_mm_s,
        }
    }

    fn transition_success(&mut self, control_time_ms: u64, to: Phase) {
        let from = self.phase;
        self.retry_counts[from.index()] = 0;
        self.phase = to;
        self.phase_cycles = 0;
        self.servo_steps = 0;
        self.metrics.transitions += 1;
        self.emit(control_time_ms, EventKind::Transition { from, to });
    }

    fn fail(
        &mut self,
        control_time_ms: u64,
        reason: FailureReason,
        recovery_phase: Phase,
        recovery_command: Command,
    ) -> Command {
        self.emit(
            control_time_ms,
            EventKind::GateRejected {
                reason: reason.clone(),
            },
        );
        if reason.severity() == FailureSeverity::Terminal {
            return self.abort(control_time_ms, reason);
        }

        let failed_phase = self.phase;
        let retry = self.retry_counts[failed_phase.index()];
        let limit = self.scenario.acceptance.retries.for_phase(failed_phase);
        if retry >= limit {
            return self.abort(
                control_time_ms,
                FailureReason::RetryBudgetExhausted {
                    phase: failed_phase,
                    last_failure: Box::new(reason),
                },
            );
        }

        let next_retry = retry + 1;
        self.retry_counts[failed_phase.index()] = next_retry;
        self.metrics.retries += 1;
        self.emit(
            control_time_ms,
            EventKind::RetryScheduled {
                phase: failed_phase,
                retry: next_retry,
                limit,
            },
        );
        if recovery_phase != failed_phase {
            self.phase = recovery_phase;
            self.metrics.transitions += 1;
            self.emit(
                control_time_ms,
                EventKind::Transition {
                    from: failed_phase,
                    to: recovery_phase,
                },
            );
        }
        self.phase_cycles = 0;
        self.servo_steps = 0;
        recovery_command
    }

    fn fail_with_recovery(
        &mut self,
        control_time_ms: u64,
        reason: FailureReason,
        recovery_phase: Phase,
    ) -> Command {
        let command = self.recovery_command();
        self.fail(control_time_ms, reason, recovery_phase, command)
    }

    fn fail_with_mesh_dither(
        &mut self,
        control_time_ms: u64,
        reason: FailureReason,
        component: ComponentId,
    ) -> Command {
        let command = self.mesh_dither_command(component);
        self.fail(control_time_ms, reason, Phase::Mesh, command)
    }

    fn fail_with_verification(
        &mut self,
        control_time_ms: u64,
        reason: FailureReason,
        component: ComponentId,
    ) -> Command {
        let command = self.verification_command(component);
        self.fail(control_time_ms, reason, Phase::Verify, command)
    }

    fn abort(&mut self, control_time_ms: u64, reason: FailureReason) -> Command {
        self.status = ExecutiveStatus::Aborted(reason.clone());
        self.metrics.safety_stops += 1;
        self.emit(control_time_ms, EventKind::Aborted { reason });
        Command::HoldPosition
    }

    fn emit(&mut self, control_time_ms: u64, kind: EventKind) {
        let event = TaskEvent {
            sequence: self.next_event_sequence,
            control_time_ms,
            component: self.active_component().map(|part| part.id),
            phase: self.phase(),
            kind,
        };
        self.next_event_sequence += 1;
        self.events.push(event);
    }

    fn component(&self) -> &ComponentPlan {
        &self.scenario.recipe.components[self.component_index]
    }

    fn component_id(&self) -> ComponentId {
        self.component().id
    }

    fn close_gripper_command(&self, component: ComponentId) -> Command {
        let limits = self.scenario.acceptance.grasp;
        Command::CloseTendonGripper {
            component,
            target_force_n: midpoint(limits.min_force_n, limits.max_force_n),
        }
    }

    fn guarded_insert_command(&mut self, component: ComponentId) -> Command {
        let limits = self.scenario.acceptance.insertion;
        Command::GuardedInsert {
            component,
            target_depth_mm: self.component().insertion_travel_mm,
            speed_mm_s: limits.guarded_speed_mm_s,
            axial_force_limit_n: self.component().max_insertion_axial_force_n,
            lateral_force_limit_n: self.component().max_insertion_lateral_force_n,
        }
    }

    fn mesh_dither_command(&mut self, component: ComponentId) -> Command {
        let limits = self.scenario.acceptance.mesh;
        let sign = if self.dither_positive { 1.0 } else { -1.0 };
        self.dither_positive = !self.dither_positive;
        Command::MeshDither {
            component,
            delta_deg: sign * limits.dither_step_deg,
            torque_limit_mn_mm: limits.max_peak_torque_mn_mm,
        }
    }

    fn verification_command(&self, component: ComponentId) -> Command {
        let limits = self.scenario.acceptance.verification;
        Command::RunVerification {
            component,
            rotation_deg: limits.min_rotation_test_deg,
            torque_limit_mn_mm: limits.max_peak_torque_mn_mm,
        }
    }

    fn recovery_command(&self) -> Command {
        if self.phase == Phase::Locate {
            Command::SearchVolume {
                component: self.component_id(),
            }
        } else {
            Command::RetractAndReacquire {
                component: self.component_id(),
                distance_mm: self.scenario.acceptance.insertion.retract_distance_mm,
            }
        }
    }
}

fn validate_gripper(
    observation: GripperObservation,
    expected: ComponentId,
) -> Result<(), FailureReason> {
    check_component(observation.component, expected)?;
    if !finite_nonnegative(observation.force_n) || !finite_nonnegative(observation.slip_mm) {
        return Err(FailureReason::InvalidMeasurement { field: "gripper" });
    }
    Ok(())
}

fn validate_handoff(
    observation: HandoffObservation,
    expected: ComponentId,
) -> Result<(), FailureReason> {
    check_component(observation.component, expected)?;
    if !observation.target_error.is_finite() || !finite_nonnegative(observation.receiver_force_n) {
        return Err(FailureReason::InvalidMeasurement { field: "handoff" });
    }
    Ok(())
}

fn validate_alignment(
    observation: AlignmentObservation,
    expected: ComponentId,
) -> Result<(), FailureReason> {
    check_component(observation.component, expected)?;
    if !observation.bore_error.is_finite() {
        return Err(FailureReason::InvalidMeasurement { field: "alignment" });
    }
    Ok(())
}

fn validate_insertion(
    observation: InsertionObservation,
    expected: ComponentId,
) -> Result<(), FailureReason> {
    check_component(observation.component, expected)?;
    if !finite_nonnegative(observation.depth_mm)
        || !finite_nonnegative(observation.axial_force_n)
        || !finite_nonnegative(observation.lateral_force_n)
    {
        return Err(FailureReason::InvalidMeasurement { field: "insertion" });
    }
    Ok(())
}

fn validate_mesh(observation: MeshObservation, expected: ComponentId) -> Result<(), FailureReason> {
    check_component(observation.component, expected)?;
    if !finite_nonnegative(observation.sweep_deg)
        || !finite_nonnegative(observation.peak_torque_mn_mm)
        || !finite_nonnegative(observation.backlash_mm)
    {
        return Err(FailureReason::InvalidMeasurement { field: "mesh" });
    }
    Ok(())
}

fn validate_verification(
    observation: VerificationObservation,
    expected: ComponentId,
) -> Result<(), FailureReason> {
    check_component(observation.component, expected)?;
    if !observation.target_error.is_finite()
        || !unit_interval(observation.vision_confidence)
        || !finite_nonnegative(observation.rotation_test_deg)
        || !finite_nonnegative(observation.peak_torque_mn_mm)
        || !unit_interval(observation.torque_ripple_fraction)
        || !finite_nonnegative(observation.backlash_mm)
    {
        return Err(FailureReason::InvalidMeasurement {
            field: "verification",
        });
    }
    Ok(())
}

fn check_component(observed: ComponentId, expected: ComponentId) -> Result<(), FailureReason> {
    if observed == expected {
        Ok(())
    } else {
        Err(FailureReason::WrongComponent { expected, observed })
    }
}

fn finite_nonnegative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

fn unit_interval(value: f64) -> bool {
    value.is_finite() && (0.0..=1.0).contains(&value)
}

fn midpoint(min: f64, max: f64) -> f64 {
    min + (max - min) * 0.5
}

fn scaled_clamped_negative(vector: [f64; 3], gain: f64, max_norm: f64) -> [f64; 3] {
    let mut result = vector.map(|value| -value * gain);
    let norm = result[0].hypot(result[1]).hypot(result[2]);
    if norm > max_norm {
        let scale = max_norm / norm;
        result = result.map(|value| value * scale);
    }
    result
}
