//! Private physical fixture and injected metrology adapter.
use super::controller::{
    Action, HandoffInput, PadEvidence, DONOR_OBJECT, PEG_OBJECT, RECEIVER_OBJECT,
};
use super::Scenario;
use crate::observed_manipulation::{
    EstimatorConfig, FeatureMeasurement, KnownAxialFeature, ObservedPoseEstimator,
};
use crate::scene::{build_scene_frame, SceneFrame};
use pipe_sim_core::{
    ArmId, BodyId, CollisionFilter, GripperState, MachineCommand, ManipulatorId,
    ManipulatorMotionState, MotionType, Pose, Quat, RigidBody, Shape, Simulation, Vec3,
};

pub(super) struct Plant {
    simulation: Simulation,
    scenario: Scenario,
    estimators: Vec<ObservedPoseEstimator>,
    pub machine_hash: String,
    pub frames: Vec<SceneFrame>,
    pub maximum_penetration_m: f64,
    pub transfer_jump_m: f64,
    palm_forward_plane_m: f64,
}

impl Plant {
    pub fn new(scenario: &Scenario) -> Result<Self, String> {
        let loaded =
            crate::machine_config::load_m1e_coupon_machine_config().map_err(|e| e.to_string())?;
        if scenario.schema_version != 1 || scenario.machine_config_id != loaded.id {
            return Err("unsupported handoff scenario".into());
        }
        let mut simulation =
            crate::machine_config::build_baseline_machine(&loaded).map_err(|e| e.to_string())?;
        // The inactive third/fourth arms remain parked at their hashed datums.
        for (id, sign) in [(1, 1.0), (2, -1.0)] {
            let arm = simulation.serial_arm_mut(ArmId(id)).ok_or("missing arm")?;
            let target = Vec3::X * (sign * scenario.tool_to_peg_center_m);
            let seed = arm
                .arm
                .solve_tool_position(target, arm.arm.positions)
                .map_err(|e| format!("seed IK: {e:?}"))?
                .positions;
            let solution = arm
                .arm
                .solve_tool_axis(target, Vec3::X * (-sign), seed)
                .map_err(|e| format!("axis IK: {e:?}"))?;
            arm.arm
                .set_positions(solution.positions)
                .map_err(|e| format!("initialization: {e:?}"))?;
            arm.motion = ManipulatorMotionState::from_positions(solution.positions);
            arm.kinematics = arm.arm.forward_kinematics();
            arm.gripper = GripperState::new(
                if id == 1 {
                    scenario.closed_opening_m
                } else {
                    arm.gripper_config.max_opening_m
                },
                arm.gripper_config,
            );
        }
        let mut peg = RigidBody::new(
            BodyId(PEG_OBJECT),
            Shape::Capsule {
                radius_m: scenario.peg_radius_m,
                half_segment_m: scenario.peg_half_segment_m,
            },
            Pose::new(Vec3::ZERO, Quat::from_two_vectors(Vec3::Z, -Vec3::X)),
            MotionType::Dynamic,
        );
        // The intended two-pad process contacts use the gripper model; explicit
        // physical handoff guards below cover inter-arm links, jaws and palms.
        peg.collision_filter = CollisionFilter { group: 2, mask: 0 };
        simulation
            .add_body(peg)
            .map_err(|e| format!("peg: {e:?}"))?;
        let mut estimators = Vec::new();
        for id in [DONOR_OBJECT, RECEIVER_OBJECT, PEG_OBJECT] {
            let config = EstimatorConfig {
                tick_period_s: simulation.config.fixed_dt_s,
                maximum_measurement_age_ticks: scenario.maximum_observation_age_ticks,
                maximum_burst_span_ticks: 4,
                ..EstimatorConfig::default()
            };
            estimators.push(
                ObservedPoseEstimator::new(config, id, feature_model(id))
                    .map_err(|e| e.to_string())?,
            );
        }
        let mut plant = Self {
            simulation,
            scenario: scenario.clone(),
            estimators,
            machine_hash: loaded.source_sha256,
            frames: Vec::new(),
            maximum_penetration_m: 0.0,
            transfer_jump_m: 0.0,
            palm_forward_plane_m: loaded
                .tool_geometry
                .ok_or("missing palm geometry")?
                .palm_forward_plane_tool_z_m,
        };
        plant.guard_geometry()?;
        plant.record();
        Ok(plant)
    }

    pub fn tick(&self) -> u64 {
        self.simulation.step_index
    }
    pub fn owner(&self) -> Option<u32> {
        self.simulation
            .serial_arms
            .iter()
            .find_map(|arm| (arm.gripper.held_body == Some(BodyId(PEG_OBJECT))).then_some(arm.id.0))
    }
    pub fn record(&mut self) {
        self.frames.push(build_scene_frame(&self.simulation, &[]));
    }
    pub fn stop(&mut self) -> Result<(), String> {
        self.simulation
            .submit_machine_command(MachineCommand::Stop { manipulator: None })
            .map_err(|e| format!("Stop: {e:?}"))?;
        Ok(())
    }

    fn step(&mut self) -> Result<(), String> {
        self.simulation.step().map_err(|e| format!("step: {e:?}"))?;
        self.guard_geometry()?;
        for id in [1, 2] {
            if self.contact(id).force_proxy_n > self.scenario.maximum_force_n {
                self.stop()?;
                return Err("force_interlock".into());
            }
        }
        Ok(())
    }

    fn move_jaws(&mut self, id: u32, opening_m: f64) -> Result<(), String> {
        self.simulation
            .submit_machine_command(MachineCommand::SetGripperOpening {
                manipulator: ManipulatorId(id),
                target_opening_m: opening_m,
            })
            .map_err(|e| format!("jaw command: {e:?}"))?;
        for _ in 0..2000 {
            self.step()?;
            let arm = self.simulation.serial_arm(ArmId(id)).ok_or("missing arm")?;
            if (arm.gripper.opening_m - opening_m).abs() < 1e-12
                && arm.gripper.opening_velocity_m_s.abs() < 1e-12
            {
                return Ok(());
            }
        }
        Err("jaw_motion_timeout".into())
    }

    pub fn apply(&mut self, action: Action, reject_transfer: bool) -> Result<(), String> {
        match action {
            Action::AcquireDonor => self
                .simulation
                .grasp_body_serial_with_partial_axial_overlap(
                    ArmId(1),
                    BodyId(PEG_OBJECT),
                    self.scenario.minimum_axial_overlap_m,
                )
                .map_err(|e| format!("acquire: {e:?}")),
            Action::CloseReceiver => self.move_jaws(2, self.scenario.closed_opening_m),
            Action::TransferOwnership => {
                self.guard_geometry()?;
                let before = self
                    .simulation
                    .body(BodyId(PEG_OBJECT))
                    .ok_or("missing peg")?
                    .pose;
                // Fault exercises the physical transaction's reject-before-mutate path.
                let overlap = if reject_transfer {
                    1.0
                } else {
                    self.scenario.minimum_axial_overlap_m
                };
                self.simulation
                    .handoff_body_serial(ArmId(1), ArmId(2), BodyId(PEG_OBJECT), overlap)
                    .map_err(|e| format!("handoff: {e:?}"))?;
                let after = self
                    .simulation
                    .body(BodyId(PEG_OBJECT))
                    .ok_or("missing peg")?
                    .pose;
                self.transfer_jump_m = (after.translation - before.translation).length();
                if before != after {
                    return Err("handoff_changed_pose".into());
                }
                Ok(())
            }
            Action::OpenDonor => {
                let opening = self
                    .simulation
                    .serial_arm(ArmId(1))
                    .ok_or("missing arm")?
                    .gripper_config
                    .max_opening_m;
                self.move_jaws(1, opening)
            }
            Action::VerifyReceiver => Ok(()),
            Action::StopBoth => self.stop(),
        }
    }

    pub fn observe(&mut self, sequence: u64) -> Result<HandoffInput, String> {
        self.step()?;
        let capture = self.tick();
        let poses = [
            self.simulation
                .serial_arm(ArmId(1))
                .ok_or("missing donor")?
                .tool_pose(),
            self.simulation
                .serial_arm(ArmId(2))
                .ok_or("missing receiver")?
                .tool_pose(),
            self.simulation
                .body(BodyId(PEG_OBJECT))
                .ok_or("missing peg")?
                .pose,
        ];
        let available = capture + self.scenario.measurement_latency_ticks;
        let mut packets = Vec::new();
        for (id, pose) in [DONOR_OBJECT, RECEIVER_OBJECT, PEG_OBJECT]
            .into_iter()
            .zip(poses)
        {
            let mut measurements = Vec::new();
            for feature in feature_model(id) {
                let point = pose.transform_point(Vec3::Z * feature.axial_coordinate_m);
                let values = [point.x, point.y, point.z];
                let measured = std::array::from_fn(|axis| {
                    values[axis]
                        + self.scenario.measurement_sigma_m
                            * noise(
                                self.scenario.seed,
                                sequence,
                                id,
                                feature.feature_id,
                                axis as u64,
                            )
                });
                measurements.push(FeatureMeasurement {
                    object_id: id,
                    feature_id: feature.feature_id,
                    head_id: 1,
                    calibrated_ray_count: 2,
                    capture_tick: capture,
                    available_tick: available,
                    measured_point_world_m: measured,
                    covariance_diagonal_m2: [self.scenario.measurement_sigma_m.powi(2); 3],
                    confidence: 1.0,
                });
            }
            packets.push(measurements);
        }
        while self.tick() < available {
            self.step()?;
        }
        let estimates = self
            .estimators
            .iter_mut()
            .zip(packets)
            .map(|(e, p)| e.update(available, &p))
            .collect();
        Ok(HandoffInput {
            sequence,
            tick: available,
            estimates,
            donor_contact: self.contact(1),
            receiver_contact: self.contact(2),
        })
    }

    fn contact(&self, id: u32) -> PadEvidence {
        let arm = self.simulation.serial_arm(ArmId(id)).expect("fixture arm");
        let body = self
            .simulation
            .body(BodyId(PEG_OBJECT))
            .expect("fixture peg");
        let c = arm.gripper.evaluate_partial_axial_overlap_candidate(
            arm.tool_pose(),
            body,
            arm.gripper_config,
            self.scenario.minimum_axial_overlap_m,
        );
        let compression = (c.required_opening_m - arm.gripper.opening_m) * 0.5;
        let left = (compression - c.center_error_m.x).max(0.0);
        let right = (compression + c.center_error_m.x).max(0.0);
        let reachable = c.is_reachable(arm.gripper_config);
        PadEvidence {
            manipulator_id: id,
            capture_tick: self.tick(),
            left_contact: reachable && left > 0.0,
            right_contact: reachable && right > 0.0,
            force_proxy_n: if reachable {
                (left + right) / (2.0 * arm.gripper_config.pad_compliance_m)
                    * arm.gripper_config.max_grip_force_n
            } else {
                0.0
            },
        }
    }

    fn guard_geometry(&mut self) -> Result<(), String> {
        let mut bodies = Vec::new();
        for arm in &self.simulation.serial_arms {
            for (index, (pose, shape)) in arm.kinematics.collision_capsules.iter().enumerate() {
                bodies.push((
                    arm.id.0,
                    RigidBody::new(
                        BodyId(30_000 + arm.id.0 * 10 + index as u32),
                        *shape,
                        *pose,
                        MotionType::Static,
                    ),
                ));
            }
            for (index, (pose, shape)) in arm
                .gripper
                .jaw_poses(arm.tool_pose(), arm.gripper_config)
                .into_iter()
                .zip(GripperState::jaw_shapes(arm.gripper_config))
                .enumerate()
            {
                bodies.push((
                    arm.id.0,
                    RigidBody::new(
                        BodyId(40_000 + arm.id.0 * 10 + index as u32),
                        shape,
                        pose,
                        MotionType::Static,
                    ),
                ));
            }
        }
        for (i, (owner, a)) in bodies.iter().enumerate() {
            for (other, b) in &bodies[i + 1..] {
                if owner == other {
                    continue;
                }
                let proximity =
                    pipe_sim_core::query_pair(a, b).ok_or("unsupported handoff collision pair")?;
                self.maximum_penetration_m = self
                    .maximum_penetration_m
                    .max((-proximity.signed_distance_m).max(0.0));
                if proximity.signed_distance_m < 0.0 {
                    return Err("interarm_collision".into());
                }
            }
        }
        let peg = self
            .simulation
            .body(BodyId(PEG_OBJECT))
            .ok_or("missing peg")?;
        for id in [1, 2] {
            let arm = self.simulation.serial_arm(ArmId(id)).ok_or("missing arm")?;
            let local = peg.shape.aabb(arm.tool_pose().inverse() * peg.pose);
            if local.min.z < self.palm_forward_plane_m {
                return Err("peg_palm_collision".into());
            }
        }
        Ok(())
    }
}

fn feature_model(_object: u32) -> Vec<KnownAxialFeature> {
    [-0.001, -0.000333, 0.000333, 0.001]
        .into_iter()
        .enumerate()
        .map(|(id, z)| KnownAxialFeature {
            feature_id: id as u32,
            axial_coordinate_m: z,
        })
        .collect()
}

// Bounded zero-mean discrete noise with unit variance, from integer-only mixing.
fn noise(seed: u64, sequence: u64, object: u32, feature: u32, axis: u64) -> f64 {
    let mut x = seed.wrapping_add(sequence.wrapping_mul(0x9e3779b97f4a7c15))
        ^ (u64::from(object) << 32)
        ^ (u64::from(feature) << 8)
        ^ axis;
    x ^= x >> 30;
    x = x.wrapping_mul(0xbf58476d1ce4e5b9);
    x ^= x >> 27;
    if x & 1 == 0 {
        -1.0
    } else {
        1.0
    }
}
