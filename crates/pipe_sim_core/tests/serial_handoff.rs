//! Transaction unit tests. Coincident tools isolate ownership semantics;
//! collision-admitted physical geometry is tested by pipe_sim's M1g coupon.
use pipe_sim_core::{
    ArmId, BodyId, GripperConfig, GripperState, MotionType, RigidBody, SerialArm, SerialArmConfig,
    SerialArmInstance, Shape, Simulation, SimulationConfig, Vec3,
};

fn ownership_fixture() -> Simulation {
    let mut sim = Simulation::new(SimulationConfig {
        gravity_m_s2: Vec3::ZERO,
        ..SimulationConfig::default()
    })
    .unwrap();
    for id in [1, 2] {
        let mut arm = SerialArmInstance::new(
            ArmId(id),
            SerialArm::new(SerialArmConfig::default()).unwrap(),
            GripperConfig::default(),
        )
        .unwrap();
        arm.gripper = GripperState::new(0.00039, arm.gripper_config);
        sim.add_serial_arm(arm).unwrap();
    }
    let pose = sim.serial_arm(ArmId(1)).unwrap().tool_pose();
    sim.add_body(RigidBody::new(
        BodyId(7),
        Shape::Capsule {
            radius_m: 0.0002,
            half_segment_m: 0.0002,
        },
        pose,
        MotionType::Dynamic,
    ))
    .unwrap();
    sim.grasp_body_serial_with_partial_axial_overlap(ArmId(1), BodyId(7), 0.0001)
        .unwrap();
    sim
}

#[test]
fn successful_transaction_preserves_pose_and_transfers_exactly_one_owner() {
    let mut sim = ownership_fixture();
    let body = sim.body(BodyId(7)).unwrap().clone();
    let tick = sim.step_index;
    sim.handoff_body_serial(ArmId(1), ArmId(2), BodyId(7), 0.0001)
        .unwrap();
    assert_eq!(sim.body(BodyId(7)).unwrap(), &body);
    assert_eq!(sim.step_index, tick);
    assert!(sim
        .serial_arm(ArmId(1))
        .unwrap()
        .gripper
        .held_body
        .is_none());
    assert!(sim
        .serial_arm(ArmId(1))
        .unwrap()
        .held_body_local_pose
        .is_none());
    assert_eq!(
        sim.serial_arm(ArmId(2)).unwrap().gripper.held_body,
        Some(BodyId(7))
    );
}

#[test]
fn every_rejected_transaction_leaves_the_complete_state_unchanged() {
    for case in 0..9 {
        let mut sim = ownership_fixture();
        let (mut donor, mut receiver, mut overlap) = (ArmId(1), ArmId(2), 0.0001);
        match case {
            0 => receiver = donor,
            1 => donor = ArmId(9),
            2 => overlap = 1.0,
            3 => {
                sim.serial_arm_mut(receiver).unwrap().gripper =
                    GripperState::new(0.001, GripperConfig::default())
            }
            4 => {
                sim.serial_arm_mut(receiver)
                    .unwrap()
                    .motion
                    .joint_velocities_rad_s[0] = 0.001
            }
            5 => {
                sim.serial_arm_mut(receiver)
                    .unwrap()
                    .motion
                    .joint_targets_rad[0] += 0.001
            }
            6 => sim.serial_arm_mut(donor).unwrap().gripper.opening_m = 0.001,
            7 => sim.serial_arm_mut(donor).unwrap().held_body_local_pose = None,
            8 => {
                let pose = sim.serial_arm(donor).unwrap().held_body_local_pose;
                sim.serial_arm_mut(receiver).unwrap().held_body_local_pose = pose;
            }
            _ => unreachable!(),
        }
        let before = sim.clone();
        assert!(
            sim.handoff_body_serial(donor, receiver, BodyId(7), overlap)
                .is_err(),
            "case {case}"
        );
        assert_eq!(sim, before, "case {case}");
    }
}

#[test]
fn receiver_overlap_does_not_weaken_existing_donor_retention() {
    let mut sim = ownership_fixture();
    sim.release_body_serial(ArmId(1)).unwrap();
    sim.grasp_body_serial_with_partial_axial_overlap(ArmId(1), BodyId(7), 0.0003)
        .unwrap();
    // A displaced peg still overlaps both pads by 0.2 mm, but the donor
    // originally required 0.3 mm. A receiver asking for 0.1 mm cannot waive it.
    let tool = sim.serial_arm(ArmId(1)).unwrap().tool_pose();
    sim.body_mut(BodyId(7)).unwrap().pose.translation = tool.transform_point(Vec3::Z * 0.0006);
    let before = sim.clone();
    assert!(sim
        .handoff_body_serial(ArmId(1), ArmId(2), BodyId(7), 0.0001)
        .is_err());
    assert_eq!(sim, before);
}

#[test]
fn nonfinite_carriage_state_cannot_authorize_stationary_transfer() {
    for id in [ArmId(1), ArmId(2)] {
        for field in 0..4 {
            let mut sim = ownership_fixture();
            let motion = &mut sim.serial_arm_mut(id).unwrap().motion;
            match field {
                0 => motion.carriage.z_m = f64::NAN,
                1 => motion.carriage.theta_rad = f64::NAN,
                2 => motion.carriage_target.z_m = f64::NAN,
                3 => motion.carriage_target.theta_rad = f64::NAN,
                _ => unreachable!(),
            }
            // Debug preserves the NaN fields; ordinary PartialEq cannot compare NaNs.
            let before = format!("{sim:?}");
            assert!(sim
                .handoff_body_serial(ArmId(1), ArmId(2), BodyId(7), 0.0001)
                .is_err());
            assert_eq!(format!("{sim:?}"), before);
        }
    }
}
