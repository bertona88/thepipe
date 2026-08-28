use pipe_planner::{
    AlignmentObservation, Command, ComponentId, Decision, EventKind, ExecutiveStatus,
    FailureReason, GearboxTaskExecutive, GripperObservation, HandoffObservation,
    InsertionObservation, MeshObservation, MicroGearboxScenario, Phase, PoseError6d, SensorFrame,
    VerificationObservation, VisionObservation,
};

fn one_gear_executive() -> GearboxTaskExecutive {
    let mut scenario = MicroGearboxScenario::gearbox_baseline_v1();
    let mut input_gear = scenario.recipe.components[5].clone();
    // Exercise every executive phase in one compact state-machine test. The
    // canonical recipe only mandates a handoff for the cover.
    input_gear.requires_handoff = true;
    assert!(input_gear.requires_mesh);
    scenario.recipe.components = vec![input_gear];
    scenario.recipe.gear_meshes.clear();
    scenario.recipe.intended_contacts.clear();
    GearboxTaskExecutive::new(scenario).unwrap()
}

fn one_cover_executive() -> GearboxTaskExecutive {
    let mut scenario = MicroGearboxScenario::gearbox_baseline_v1();
    let cover = scenario.recipe.components[6].clone();
    assert_eq!(cover.required_closure_features, 2);
    assert!(cover.requires_handoff);
    scenario.recipe.components = vec![cover];
    scenario.recipe.gear_meshes.clear();
    scenario.recipe.intended_contacts.clear();
    GearboxTaskExecutive::new(scenario).unwrap()
}

fn frame(time_ms: u64, component: ComponentId) -> SensorFrame {
    let mut frame = SensorFrame::empty(time_ms);
    frame.vision = Some(VisionObservation {
        component,
        confidence: 0.95,
        position_sigma_mm: 0.002,
        orientation_sigma_deg: 0.20,
        target_error: PoseError6d::default(),
    });
    frame
}

fn advance_to_insert(executive: &mut GearboxTaskExecutive) -> u64 {
    let component = executive.active_component().unwrap().id;

    let locate = frame(10, component);
    assert!(matches!(
        executive.tick(&locate).command,
        Command::MoveToPregrasp { .. }
    ));

    let mut pick = frame(20, component);
    pick.gripper = Some(GripperObservation {
        component,
        force_n: 0.010,
        slip_mm: 0.001,
        part_retained: true,
    });
    assert!(matches!(
        executive.tick(&pick).command,
        Command::PresentForHandoff { .. }
    ));

    let mut handoff = frame(30, component);
    handoff.handoff = Some(HandoffObservation {
        component,
        target_error: PoseError6d::default(),
        receiver_force_n: 0.010,
        receiver_has_part: true,
        donor_released: true,
    });
    assert!(matches!(
        executive.tick(&handoff).command,
        Command::MoveToBore { .. }
    ));

    let mut align = frame(40, component);
    align.alignment = Some(AlignmentObservation {
        component,
        bore_error: PoseError6d::default(),
    });
    let decision = executive.tick(&align);
    match decision.command {
        Command::GuardedInsert {
            target_depth_mm, ..
        } => assert_eq!(
            target_depth_mm,
            executive.active_component().unwrap().insertion_travel_mm
        ),
        other => panic!("expected guarded insertion, got {other:?}"),
    }
    assert_eq!(executive.phase(), Some(Phase::Insert));
    40
}

fn happy_trace() -> (Vec<Command>, Vec<pipe_planner::TaskEvent>) {
    let mut executive = one_gear_executive();
    let component = executive.active_component().unwrap().id;
    let mut commands = Vec::new();

    commands.push(executive.tick(&frame(10, component)).command);

    let mut pick = frame(20, component);
    pick.gripper = Some(GripperObservation {
        component,
        force_n: 0.010,
        slip_mm: 0.001,
        part_retained: true,
    });
    commands.push(executive.tick(&pick).command);

    let mut handoff = frame(30, component);
    handoff.handoff = Some(HandoffObservation {
        component,
        target_error: PoseError6d::default(),
        receiver_force_n: 0.010,
        receiver_has_part: true,
        donor_released: true,
    });
    commands.push(executive.tick(&handoff).command);

    let mut align = frame(40, component);
    align.alignment = Some(AlignmentObservation {
        component,
        bore_error: PoseError6d::default(),
    });
    commands.push(executive.tick(&align).command);

    let target = executive.active_component().unwrap().insertion_travel_mm;
    let mut insert = frame(50, component);
    insert.insertion = Some(InsertionObservation {
        component,
        depth_mm: target,
        axial_force_n: 0.004,
        lateral_force_n: 0.002,
        seated: true,
        closure_features_confirmed: 0,
    });
    commands.push(executive.tick(&insert).command);

    let mut mesh = frame(60, component);
    mesh.mesh = Some(MeshObservation {
        component,
        sweep_deg: 60.0,
        peak_torque_mn_mm: 0.10,
        backlash_mm: 0.020,
        teeth_engaged: true,
    });
    commands.push(executive.tick(&mesh).command);

    let mut verify = frame(70, component);
    verify.verification = Some(VerificationObservation {
        component,
        target_error: PoseError6d::default(),
        vision_confidence: 0.96,
        rotation_test_deg: 100.0,
        peak_torque_mn_mm: 0.12,
        torque_ripple_fraction: 0.15,
        backlash_mm: 0.020,
        required_features_visible: true,
    });
    commands.push(executive.tick(&verify).command);

    assert_eq!(executive.status(), &ExecutiveStatus::Completed);
    assert_eq!(executive.metrics().components_completed, 1);
    assert_eq!(executive.metrics().retries, 0);
    (commands, executive.events().to_vec())
}

#[test]
fn complete_gear_path_exercises_every_phase() {
    let (commands, events) = happy_trace();
    assert!(matches!(commands.last(), Some(Command::AssemblyComplete)));

    let transitions: Vec<(Phase, Phase)> = events
        .iter()
        .filter_map(|event| match &event.kind {
            EventKind::Transition { from, to } => Some((*from, *to)),
            _ => None,
        })
        .collect();
    assert_eq!(
        transitions,
        vec![
            (Phase::Locate, Phase::Pick),
            (Phase::Pick, Phase::Handoff),
            (Phase::Handoff, Phase::Align),
            (Phase::Align, Phase::Insert),
            (Phase::Insert, Phase::Mesh),
            (Phase::Mesh, Phase::Verify),
        ]
    );
    assert_eq!(
        events
            .iter()
            .filter(|event| matches!(&event.kind, EventKind::ComponentCompleted))
            .count(),
        1
    );
}

#[test]
fn identical_frames_produce_identical_trace() {
    assert_eq!(happy_trace(), happy_trace());
}

#[test]
fn visual_servo_correction_is_gain_scaled_and_norm_clamped() {
    let mut executive = one_gear_executive();
    let component = executive.active_component().unwrap().id;
    executive.tick(&frame(10, component));

    let mut pick = frame(20, component);
    pick.vision.as_mut().unwrap().target_error.translation_mm = [1.0, 0.0, 0.0];
    let decision = executive.tick(&pick);
    match decision.command {
        Command::VisualServo { correction, .. } => {
            assert!((correction.translation_mm[0] + 0.050).abs() < 1.0e-12);
            assert!(correction.translation_mm[1].abs() < 1.0e-12);
            assert!(correction.translation_mm[2].abs() < 1.0e-12);
        }
        other => panic!("expected visual servo correction, got {other:?}"),
    }
    assert_eq!(executive.metrics().visual_servo_corrections, 1);
}

#[test]
fn insertion_force_fault_retracts_and_returns_to_alignment() {
    let mut executive = one_gear_executive();
    let component = executive.active_component().unwrap().id;
    let time = advance_to_insert(&mut executive);

    let mut insert = frame(time + 10, component);
    insert.insertion = Some(InsertionObservation {
        component,
        depth_mm: 0.20,
        axial_force_n: 0.021,
        lateral_force_n: 0.002,
        seated: false,
        closure_features_confirmed: 0,
    });
    let decision = executive.tick(&insert);
    assert!(matches!(
        decision.command,
        Command::RetractAndReacquire { .. }
    ));
    assert_eq!(decision.phase, Some(Phase::Align));
    assert_eq!(executive.retries_for(Phase::Insert), 1);
    assert!(decision.events.iter().any(|event| matches!(
        &event.kind,
        EventKind::GateRejected {
            reason: FailureReason::InsertionAxialForceHigh { .. }
        }
    )));
}

#[test]
fn cover_requires_two_sequential_closure_signatures() {
    let mut executive = one_cover_executive();
    let component = executive.active_component().unwrap().id;
    let mut time = advance_to_insert(&mut executive);
    let target = executive.active_component().unwrap().insertion_travel_mm;

    for confirmed in 0..2 {
        time += 10;
        let mut insert = frame(time, component);
        insert.insertion = Some(InsertionObservation {
            component,
            depth_mm: target,
            axial_force_n: 0.010,
            lateral_force_n: 0.002,
            seated: true,
            closure_features_confirmed: confirmed,
        });
        assert!(matches!(
            executive.tick(&insert).command,
            Command::CloseRetainerFeature { feature, .. } if feature == confirmed + 1
        ));
        assert_eq!(executive.phase(), Some(Phase::Insert));
    }

    time += 10;
    let mut closed = frame(time, component);
    closed.insertion = Some(InsertionObservation {
        component,
        depth_mm: target,
        axial_force_n: 0.010,
        lateral_force_n: 0.002,
        seated: true,
        closure_features_confirmed: 2,
    });
    assert!(matches!(
        executive.tick(&closed).command,
        Command::RunVerification { .. }
    ));
    assert_eq!(executive.phase(), Some(Phase::Verify));
    assert_eq!(executive.metrics().retainer_closure_commands, 2);
}

#[test]
fn excessive_unplanned_contact_is_an_immediate_safety_stop() {
    let mut executive = one_gear_executive();
    let component = executive.active_component().unwrap().id;
    let mut unsafe_frame = frame(10, component);
    unsafe_frame.unplanned_contact_force_n = 0.041;

    let decision = executive.tick(&unsafe_frame);
    assert_eq!(decision.command, Command::HoldPosition);
    assert!(matches!(
        decision.status,
        ExecutiveStatus::Aborted(FailureReason::UnplannedContactForceHigh { .. })
    ));
    assert_eq!(executive.metrics().safety_stops, 1);
}

#[test]
fn stale_frames_exhaust_a_bounded_retry_budget() {
    let mut executive = one_gear_executive();
    let component = executive.active_component().unwrap().id;
    let retry_limit = executive.scenario().acceptance.retries.locate;
    let mut last: Option<Decision> = None;

    for index in 0..=retry_limit {
        let control_time_ms = 100 + u64::from(index) * 100;
        let mut stale = frame(control_time_ms, component);
        stale.capture_time_ms = 0;
        last = Some(executive.tick(&stale));
    }

    assert!(matches!(
        last.unwrap().status,
        ExecutiveStatus::Aborted(FailureReason::RetryBudgetExhausted {
            phase: Phase::Locate,
            ..
        })
    ));
    assert_eq!(executive.metrics().retries, u64::from(retry_limit));
}
