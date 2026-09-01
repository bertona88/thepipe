//! Versioned, renderer-neutral projection of the authoritative machine state.

use pipe_optics::StructuredLightRig;
use pipe_sim_core::{
    serial_arm_link_body_id, Contact, ContactKind, MotionType, PipeCellConfig, Pose, RigidBody,
    Shape, Simulation, Vec3, TENDON_JOINT_COUNT,
};
use serde::Serialize;

pub const SCENE_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SceneUnits {
    pub length: &'static str,
    pub angle: &'static str,
    pub time: &'static str,
    pub force: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct WorldFrameDescription {
    pub id: &'static str,
    pub handedness: &'static str,
    pub tube_axis: &'static str,
    pub radial_zero: &'static str,
    pub positive_theta: &'static str,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct PoseSnapshot {
    pub translation_m: [f64; 3],
    /// Persistent scene quaternions use the normative `[x, y, z, w]` order.
    pub rotation_xyzw: [f64; 4],
}

impl From<Pose> for PoseSnapshot {
    fn from(value: Pose) -> Self {
        Self {
            translation_m: vec3(value.translation),
            rotation_xyzw: [
                value.rotation.x,
                value.rotation.y,
                value.rotation.z,
                value.rotation.w,
            ],
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ShapeSnapshot {
    Sphere {
        radius_m: f64,
    },
    Capsule {
        radius_m: f64,
        half_segment_m: f64,
    },
    Box {
        half_extents_m: [f64; 3],
    },
    Gear {
        teeth: u16,
        module_m: f64,
        pressure_angle_rad: f64,
        pitch_radius_m: f64,
        root_radius_m: f64,
        tip_radius_m: f64,
        half_thickness_m: f64,
        bore_radius_m: f64,
        hub_radius_m: f64,
        half_total_height_m: f64,
        tooth_center_offset_m: f64,
    },
}

impl From<Shape> for ShapeSnapshot {
    fn from(value: Shape) -> Self {
        match value {
            Shape::Sphere { radius_m } => Self::Sphere { radius_m },
            Shape::Capsule {
                radius_m,
                half_segment_m,
            } => Self::Capsule {
                radius_m,
                half_segment_m,
            },
            Shape::Box { half_extents_m } => Self::Box {
                half_extents_m: vec3(half_extents_m),
            },
            Shape::Gear(gear) => Self::Gear {
                teeth: gear.teeth,
                module_m: gear.module_m,
                pressure_angle_rad: gear.pressure_angle_rad,
                pitch_radius_m: gear.pitch_radius_m,
                root_radius_m: gear.root_radius_m,
                tip_radius_m: gear.tip_radius_m,
                half_thickness_m: gear.half_thickness_m,
                bore_radius_m: gear.bore_radius_m,
                hub_radius_m: gear.hub_radius_m,
                half_total_height_m: gear.half_total_height_m,
                tooth_center_offset_m: gear.tooth_center_offset_m,
            },
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TubeDescription {
    pub inner_radius_m: f64,
    pub working_length_m: f64,
    pub central_work_radius_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RailSystemDescription {
    pub topology: &'static str,
    pub shoulder_datum_radius_m: f64,
    pub z_limits_m: [f64; 2],
    pub max_z_speed_m_s: f64,
    pub max_theta_speed_rad_s: f64,
    pub max_z_accel_m_s2: f64,
    pub max_theta_accel_rad_s2: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ManipulatorDescription {
    pub id: u32,
    pub joint_names: [&'static str; TENDON_JOINT_COUNT],
    pub joint_limits_rad: [[f64; 2]; TENDON_JOINT_COUNT],
    pub link_lengths_m: [f64; 3],
    pub link_collision_radii_m: [f64; 3],
    pub gripper_jaw_half_extents_m: [f64; 3],
    pub gripper_opening_limits_m: [f64; 2],
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RigidBodyDescription {
    pub id: u32,
    pub name: String,
    pub geometry_id: String,
    pub shape: ShapeSnapshot,
    pub motion: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SensorDescription {
    pub id: u32,
    pub kind: &'static str,
    pub nominal_translation_m: [f64; 3],
    pub nominal_rotation_world_from_sensor: [[f64; 3]; 3],
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SceneDescription {
    pub schema_version: u32,
    pub machine_config_id: String,
    pub machine_config_sha256: String,
    pub units: SceneUnits,
    pub world_frame: WorldFrameDescription,
    pub tube: TubeDescription,
    pub rail_system: RailSystemDescription,
    pub manipulators: Vec<ManipulatorDescription>,
    pub rigid_bodies: Vec<RigidBodyDescription>,
    pub sensors: Vec<SensorDescription>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ColliderSnapshot {
    pub body_id: u32,
    pub geometry_id: String,
    pub pose: PoseSnapshot,
    pub shape: ShapeSnapshot,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct GripperSnapshot {
    pub opening_m: f64,
    pub command_opening_m: f64,
    pub opening_velocity_m_s: f64,
    pub estimated_grip_force_n: f64,
    pub held_body_id: Option<u32>,
    pub jaw_poses: [PoseSnapshot; 2],
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ManipulatorSnapshot {
    pub id: u32,
    pub carriage_z_m: f64,
    pub carriage_theta_rad: f64,
    pub carriage_z_velocity_m_s: f64,
    pub carriage_theta_velocity_rad_s: f64,
    pub carriage_pose: PoseSnapshot,
    pub shoulder_pose: PoseSnapshot,
    pub elbow_pose: PoseSnapshot,
    pub wrist_pose: PoseSnapshot,
    pub tool_pose: PoseSnapshot,
    pub tool_axis_world: [f64; 3],
    pub joint_positions_rad: [f64; TENDON_JOINT_COUNT],
    pub joint_velocities_rad_s: [f64; TENDON_JOINT_COUNT],
    pub tendon_tensions_n: [[f64; 2]; TENDON_JOINT_COUNT],
    pub gripper: GripperSnapshot,
    pub link_colliders: Vec<ColliderSnapshot>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RigidBodySnapshot {
    pub id: u32,
    pub geometry_id: String,
    pub enabled: bool,
    pub pose: PoseSnapshot,
    pub linear_velocity_m_s: [f64; 3],
    pub angular_velocity_rad_s: [f64; 3],
    pub held_by_manipulator_id: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ContactSnapshot {
    pub body_a: u32,
    pub body_b: u32,
    pub point_a_world_m: [f64; 3],
    pub point_b_world_m: [f64; 3],
    pub normal_a_to_b: [f64; 3],
    pub signed_distance_m: f64,
    pub penetration_depth_m: f64,
    pub kind: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct PhysicalSceneState {
    pub manipulators: Vec<ManipulatorSnapshot>,
    pub rigid_bodies: Vec<RigidBodySnapshot>,
    pub contacts: Vec<ContactSnapshot>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct CommandedManipulatorState {
    pub id: u32,
    pub carriage_z_m: f64,
    pub carriage_theta_rad: f64,
    pub joint_positions_rad: [f64; TENDON_JOINT_COUNT],
    pub gripper_opening_m: f64,
    pub stopped: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct CommandedSceneState {
    pub command_sequence: u64,
    pub manipulators: Vec<CommandedManipulatorState>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SceneFrame {
    pub schema_version: u32,
    pub tick: u64,
    pub time_s: f64,
    /// Present only in simulation/evaluation builds.
    pub truth: Option<PhysicalSceneState>,
    /// Future hardware and closed-loop simulation populate this from the
    /// timestamped estimator, never by copying truth implicitly.
    pub estimate: Option<PhysicalSceneState>,
    pub commanded: CommandedSceneState,
}

pub(crate) fn build_scene_description(
    machine_config_id: &str,
    machine_config_sha256: &str,
    cell: PipeCellConfig,
    simulation: &Simulation,
    optics: &StructuredLightRig,
    body_names: &[(u32, String)],
) -> SceneDescription {
    let manipulators = simulation
        .serial_arms
        .iter()
        .map(|instance| ManipulatorDescription {
            id: instance.id.0,
            joint_names: [
                "shoulder_yaw",
                "shoulder_pitch",
                "elbow_pitch",
                "wrist_roll",
            ],
            joint_limits_rad: instance.arm.config.joint_limits_rad,
            link_lengths_m: [
                instance.arm.config.upper_arm_length_m,
                instance.arm.config.forearm_length_m,
                instance.arm.config.wrist_length_m,
            ],
            link_collision_radii_m: instance.arm.config.link_collision_radii_m,
            gripper_jaw_half_extents_m: vec3(instance.gripper_config.jaw_half_extents_m),
            gripper_opening_limits_m: [
                instance.gripper_config.min_opening_m,
                instance.gripper_config.max_opening_m,
            ],
        })
        .collect();
    let rigid_bodies = simulation
        .bodies
        .iter()
        .map(|body| RigidBodyDescription {
            id: body.id.0,
            name: body_names
                .iter()
                .find_map(|(id, name)| (*id == body.id.0).then(|| name.clone()))
                .unwrap_or_else(|| format!("body-{}", body.id.0)),
            geometry_id: body_geometry_id(body),
            shape: body.shape.into(),
            motion: motion_name(body.motion),
        })
        .collect();
    let mut sensors = optics
        .cameras
        .iter()
        .enumerate()
        .map(|(index, camera)| SensorDescription {
            id: camera.id,
            kind: if index < 6 {
                "global_camera"
            } else {
                "macro_camera"
            },
            nominal_translation_m: optical_vec3(camera.nominal.world_from_camera.translation),
            nominal_rotation_world_from_sensor: camera.nominal.world_from_camera.rotation.m,
        })
        .collect::<Vec<_>>();
    sensors.push(SensorDescription {
        id: optics.projector.id,
        kind: "structured_light_projector",
        nominal_translation_m: optical_vec3(optics.projector.nominal.world_from_camera.translation),
        nominal_rotation_world_from_sensor: optics.projector.nominal.world_from_camera.rotation.m,
    });
    sensors.sort_by_key(|sensor| sensor.id);

    SceneDescription {
        schema_version: SCENE_SCHEMA_VERSION,
        machine_config_id: machine_config_id.to_owned(),
        machine_config_sha256: machine_config_sha256.to_owned(),
        units: SceneUnits {
            length: "m",
            angle: "rad",
            time: "s",
            force: "N",
        },
        world_frame: WorldFrameDescription {
            id: "pipe_world",
            handedness: "right_handed",
            tube_axis: "+Z",
            radial_zero: "+X",
            positive_theta: "+X_toward_+Y",
        },
        tube: TubeDescription {
            inner_radius_m: cell.tube.inner_radius_m,
            working_length_m: cell.tube.working_length_m,
            central_work_radius_m: cell.tube.central_work_radius_m,
        },
        rail_system: RailSystemDescription {
            topology: "paired_belt_end_bogies",
            shoulder_datum_radius_m: cell.carriage.rail_radius_m,
            z_limits_m: cell.carriage.z_limits_m,
            max_z_speed_m_s: cell.carriage.max_z_speed_m_s,
            max_theta_speed_rad_s: cell.carriage.max_theta_speed_rad_s,
            max_z_accel_m_s2: cell.carriage.max_z_accel_m_s2,
            max_theta_accel_rad_s2: cell.carriage.max_theta_accel_rad_s2,
        },
        manipulators,
        rigid_bodies,
        sensors,
    }
}

pub(crate) fn build_scene_frame(simulation: &Simulation, contacts: &[Contact]) -> SceneFrame {
    let manipulators = simulation
        .serial_arms
        .iter()
        .map(|instance| {
            let kinematics = instance.arm.forward_kinematics();
            let jaw_poses = instance
                .gripper
                .jaw_poses(kinematics.tool_pose, instance.gripper_config)
                .map(PoseSnapshot::from);
            let link_colliders = kinematics
                .collision_capsules
                .iter()
                .enumerate()
                .map(|(index, (pose, shape))| ColliderSnapshot {
                    body_id: serial_arm_link_body_id(instance.id, index as u8)
                        .expect("serial-arm FK exposes only reserved physical link indices")
                        .0,
                    geometry_id: format!("manipulator/{}/link/{index}", instance.id.0),
                    pose: (*pose).into(),
                    shape: (*shape).into(),
                })
                .collect();
            ManipulatorSnapshot {
                id: instance.id.0,
                carriage_z_m: instance.motion.carriage.z_m,
                carriage_theta_rad: instance.motion.carriage.theta_rad,
                carriage_z_velocity_m_s: instance.motion.carriage.z_velocity_m_s,
                carriage_theta_velocity_rad_s: instance.motion.carriage.theta_velocity_rad_s,
                carriage_pose: kinematics.base_pose.into(),
                shoulder_pose: kinematics.shoulder_pose.into(),
                elbow_pose: kinematics.elbow_pose.into(),
                wrist_pose: kinematics.wrist_pose.into(),
                tool_pose: kinematics.tool_pose.into(),
                tool_axis_world: vec3(kinematics.tool_pose.transform_vector(Vec3::Z)),
                joint_positions_rad: instance.motion.joint_positions_rad,
                joint_velocities_rad_s: instance.motion.joint_velocities_rad_s,
                tendon_tensions_n: instance
                    .arm
                    .tendon_telemetry
                    .map(|telemetry| telemetry.tendon_tensions_n),
                gripper: GripperSnapshot {
                    opening_m: instance.gripper.opening_m,
                    command_opening_m: instance.gripper.command_opening_m,
                    opening_velocity_m_s: instance.gripper.opening_velocity_m_s,
                    estimated_grip_force_n: instance.gripper.estimated_grip_force_n,
                    held_body_id: instance.gripper.held_body.map(|id| id.0),
                    jaw_poses,
                },
                link_colliders,
            }
        })
        .collect::<Vec<_>>();
    let rigid_bodies = simulation
        .bodies
        .iter()
        .map(|body| RigidBodySnapshot {
            id: body.id.0,
            geometry_id: body_geometry_id(body),
            enabled: body.enabled,
            pose: body.pose.into(),
            linear_velocity_m_s: vec3(body.linear_velocity_m_s),
            angular_velocity_rad_s: vec3(body.angular_velocity_rad_s),
            held_by_manipulator_id: simulation
                .serial_arms
                .iter()
                .find_map(|arm| (arm.gripper.held_body == Some(body.id)).then_some(arm.id.0)),
        })
        .collect();
    let contacts = contacts.iter().copied().map(contact_snapshot).collect();
    let commanded = CommandedSceneState {
        command_sequence: simulation.machine_command_sequence,
        manipulators: simulation
            .serial_arms
            .iter()
            .map(|instance| CommandedManipulatorState {
                id: instance.id.0,
                carriage_z_m: instance.motion.carriage_target.z_m,
                carriage_theta_rad: instance.motion.carriage_target.theta_rad,
                joint_positions_rad: instance.motion.joint_targets_rad,
                gripper_opening_m: instance.gripper.command_opening_m,
                stopped: instance.motion.stopped,
            })
            .collect(),
    };
    SceneFrame {
        schema_version: SCENE_SCHEMA_VERSION,
        tick: simulation.step_index,
        time_s: simulation.time_s,
        truth: Some(PhysicalSceneState {
            manipulators,
            rigid_bodies,
            contacts,
        }),
        estimate: None,
        commanded,
    }
}

fn body_geometry_id(body: &RigidBody) -> String {
    format!("body/{}/geometry", body.id.0)
}

fn contact_snapshot(contact: Contact) -> ContactSnapshot {
    ContactSnapshot {
        body_a: contact.body_a.0,
        body_b: contact.body_b.0,
        point_a_world_m: vec3(contact.point_a_world_m),
        point_b_world_m: vec3(contact.point_b_world_m),
        normal_a_to_b: vec3(contact.normal_a_to_b),
        signed_distance_m: contact.signed_distance_m,
        penetration_depth_m: contact.penetration_depth_m,
        kind: contact_kind_name(contact.kind),
    }
}

fn contact_kind_name(kind: ContactKind) -> &'static str {
    match kind {
        ContactKind::ExactAnalytic => "exact_analytic",
        ContactKind::CapsuleBoxApproximation => "capsule_box_approximation",
        ContactKind::OrientedBoxSatApproximation => "oriented_box_sat_approximation",
        ContactKind::GearAnnularEnvelopeApproximation => "gear_annular_envelope_approximation",
        ContactKind::GearMeshApproximation => "gear_mesh_approximation",
        ContactKind::GearBoxEnvelopeApproximation => "gear_box_envelope_approximation",
    }
}

fn motion_name(motion: MotionType) -> &'static str {
    match motion {
        MotionType::Static => "static",
        MotionType::Kinematic => "kinematic",
        MotionType::Dynamic => "dynamic",
    }
}

fn vec3(value: Vec3) -> [f64; 3] {
    [value.x, value.y, value.z]
}

fn optical_vec3(value: pipe_optics::Vec3) -> [f64; 3] {
    [value.x, value.y, value.z]
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipe_sim_core::{
        ArmId, GripperConfig, Quat, SerialArm, SerialArmConfig, SerialArmInstance, SimulationConfig,
    };

    #[test]
    fn persistent_quaternion_order_is_xyzw() {
        let pose = Pose::new(Vec3::new(1.0, 2.0, 3.0), Quat::new(0.5, 0.1, 0.2, 0.3));
        let snapshot = PoseSnapshot::from(pose);
        assert_eq!(snapshot.translation_m, [1.0, 2.0, 3.0]);
        assert_eq!(snapshot.rotation_xyzw, [0.1, 0.2, 0.3, 0.5]);
        let json = serde_json::to_value(snapshot).unwrap();
        assert!(json.get("rotation_xyzw").is_some());
        assert!(json.get("rotation_wxyz").is_none());
    }

    #[test]
    fn frame_keeps_truth_estimate_and_commands_distinct() {
        let mut simulation = Simulation::new(SimulationConfig::default()).unwrap();
        let arm = SerialArm::new(SerialArmConfig::default()).unwrap();
        simulation
            .add_serial_arm(
                SerialArmInstance::new(ArmId(1), arm, GripperConfig::default()).unwrap(),
            )
            .unwrap();
        let frame = build_scene_frame(&simulation, &[]);
        assert!(frame.truth.is_some());
        assert!(frame.estimate.is_none());
        assert_eq!(frame.commanded.manipulators.len(), 1);
        let truth = frame.truth.unwrap();
        assert_eq!(truth.manipulators.len(), 1);
        let collider_ids = truth.manipulators[0]
            .link_colliders
            .iter()
            .map(|collider| collider.body_id)
            .collect::<Vec<_>>();
        assert_eq!(collider_ids.len(), 3);
        assert!(collider_ids.windows(2).all(|ids| ids[0] < ids[1]));
    }
}
