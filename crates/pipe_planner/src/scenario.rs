use crate::{
    AcceptanceParameters, AlignmentLimits, AssemblyContactDefinition, AssemblyContactKind,
    AssemblyScenario, CellBaseline, ComponentGeometry, ComponentId, ComponentKind, ComponentPlan,
    ContactFeature, GearMeshPlan, GearboxRecipe, GraspLimits, HandoffLimits, HousingPlan,
    InsertionLimits, MeshLimits, NominalAssemblyPose, Phase, RetryLimits, SafetyLimits,
    VerificationLimits, VisionLimits, VisualServoLimits,
};

/// Reproducible first geometry target for a micro-printed gearbox.
///
/// The 0.10 mm module teeth are beyond ordinary hobby masked-SLA fidelity; the
/// intended final parts are 2PP or equivalent micro-printing, while the cell
/// structure, cameras, tendons, and electronics remain DIY-oriented. These are
/// acceptance assumptions, not process guarantees. Measured print coupons,
/// camera calibration, and force calibration must replace the defaults before
/// commanding physical hardware.
pub struct MicroGearboxScenario;

impl MicroGearboxScenario {
    pub fn gearbox_baseline_v1() -> AssemblyScenario {
        AssemblyScenario {
            name: "gearbox_baseline_v1".to_owned(),
            deterministic_seed: 0x5049_5045_5F47_4258,
            physics_step_ms: 1,
            cell: CellBaseline {
                mobile_arm_count: 4,
                each_base_has_z_motion: true,
                each_base_has_theta_motion: true,
                usable_tube_length_mm: 320.0,
                global_camera_count: 6,
                simultaneous_macro_views: 2,
            },
            recipe: GearboxRecipe {
                name: "single-stage three-gear 2PP acceptance article".to_owned(),
                registered_housing: HousingPlan {
                    geometry: ComponentGeometry::Housing {
                        outer_size_mm: [6.0, 4.0, 1.60],
                        wall_mm: 0.030,
                    },
                    cad_registered_origin_pose: pose([0.0, 0.0, 0.0], 0.0),
                    target_pose: pose([3.0, 2.0, 0.80], 0.0),
                    shaft_seat_diameter_mm: 0.340,
                    shaft_seat_depth_mm: 0.250,
                },
                components: vec![
                    ComponentPlan {
                        id: ComponentId(1),
                        name: "S1 input shaft".to_owned(),
                        kind: ComponentKind::Shaft,
                        geometry: ComponentGeometry::Shaft {
                            diameter_mm: 0.350,
                            length_mm: 1.55,
                        },
                        cad_seated_origin_pose: pose([0.750, 2.000, 0.0], 0.0),
                        target_pose: pose([0.750, 2.000, 0.775], 0.0),
                        insertion_direction: [0.0, 0.0, -1.0],
                        requires_handoff: false,
                        requires_mesh: false,
                        insertion_travel_mm: 0.250,
                        insertion_depth_tolerance_mm: 0.015,
                        max_insertion_axial_force_n: 0.050,
                        max_insertion_lateral_force_n: 0.010,
                        required_closure_features: 0,
                        nominal_fit_clearance_mm: -0.010,
                    },
                    ComponentPlan {
                        id: ComponentId(2),
                        name: "S2 idler shaft".to_owned(),
                        kind: ComponentKind::Shaft,
                        geometry: ComponentGeometry::Shaft {
                            diameter_mm: 0.350,
                            length_mm: 1.55,
                        },
                        cad_seated_origin_pose: pose([2.250, 2.000, 0.0], 0.0),
                        target_pose: pose([2.250, 2.000, 0.775], 0.0),
                        insertion_direction: [0.0, 0.0, -1.0],
                        requires_handoff: false,
                        requires_mesh: false,
                        insertion_travel_mm: 0.250,
                        insertion_depth_tolerance_mm: 0.015,
                        max_insertion_axial_force_n: 0.050,
                        max_insertion_lateral_force_n: 0.010,
                        required_closure_features: 0,
                        nominal_fit_clearance_mm: -0.010,
                    },
                    ComponentPlan {
                        id: ComponentId(3),
                        name: "S3 output shaft".to_owned(),
                        kind: ComponentKind::Shaft,
                        geometry: ComponentGeometry::Shaft {
                            diameter_mm: 0.350,
                            length_mm: 1.55,
                        },
                        cad_seated_origin_pose: pose([4.350, 2.000, 0.0], 0.0),
                        target_pose: pose([4.350, 2.000, 0.775], 0.0),
                        insertion_direction: [0.0, 0.0, -1.0],
                        requires_handoff: false,
                        requires_mesh: false,
                        insertion_travel_mm: 0.250,
                        insertion_depth_tolerance_mm: 0.015,
                        max_insertion_axial_force_n: 0.050,
                        max_insertion_lateral_force_n: 0.010,
                        required_closure_features: 0,
                        nominal_fit_clearance_mm: -0.010,
                    },
                    ComponentPlan {
                        id: ComponentId(4),
                        name: "G3 24 tooth output gear".to_owned(),
                        kind: ComponentKind::Gear,
                        geometry: ComponentGeometry::SpurGear {
                            module_mm: 0.10,
                            teeth: 24,
                            pressure_angle_deg: 25.0,
                            thickness_mm: 0.35,
                            bore_diameter_mm: 0.420,
                            hub_diameter_mm: 0.550,
                            total_height_mm: 1.30,
                        },
                        cad_seated_origin_pose: pose([4.350, 2.000, 0.250], 0.0),
                        target_pose: pose([4.350, 2.000, 0.900], 0.0),
                        insertion_direction: [0.0, 0.0, -1.0],
                        requires_handoff: false,
                        requires_mesh: false,
                        insertion_travel_mm: 1.20,
                        insertion_depth_tolerance_mm: 0.020,
                        max_insertion_axial_force_n: 0.005,
                        max_insertion_lateral_force_n: 0.003,
                        required_closure_features: 0,
                        nominal_fit_clearance_mm: 0.070,
                    },
                    ComponentPlan {
                        id: ComponentId(5),
                        name: "G2 18 tooth idler gear".to_owned(),
                        kind: ComponentKind::Gear,
                        geometry: ComponentGeometry::SpurGear {
                            module_mm: 0.10,
                            teeth: 18,
                            pressure_angle_deg: 25.0,
                            thickness_mm: 0.35,
                            bore_diameter_mm: 0.420,
                            hub_diameter_mm: 0.550,
                            total_height_mm: 1.30,
                        },
                        cad_seated_origin_pose: pose([2.250, 2.000, 0.250], 170.0),
                        target_pose: pose([2.250, 2.000, 0.900], 170.0),
                        insertion_direction: [0.0, 0.0, -1.0],
                        requires_handoff: false,
                        requires_mesh: true,
                        insertion_travel_mm: 1.20,
                        insertion_depth_tolerance_mm: 0.020,
                        max_insertion_axial_force_n: 0.005,
                        max_insertion_lateral_force_n: 0.003,
                        required_closure_features: 0,
                        nominal_fit_clearance_mm: 0.070,
                    },
                    ComponentPlan {
                        id: ComponentId(6),
                        name: "G1 12 tooth input gear".to_owned(),
                        kind: ComponentKind::Gear,
                        geometry: ComponentGeometry::SpurGear {
                            module_mm: 0.10,
                            teeth: 12,
                            pressure_angle_deg: 25.0,
                            thickness_mm: 0.35,
                            bore_diameter_mm: 0.420,
                            hub_diameter_mm: 0.550,
                            total_height_mm: 1.30,
                        },
                        cad_seated_origin_pose: pose([0.750, 2.000, 0.250], 0.0),
                        target_pose: pose([0.750, 2.000, 0.900], 0.0),
                        insertion_direction: [0.0, 0.0, -1.0],
                        requires_handoff: false,
                        requires_mesh: true,
                        insertion_travel_mm: 1.20,
                        insertion_depth_tolerance_mm: 0.020,
                        max_insertion_axial_force_n: 0.005,
                        max_insertion_lateral_force_n: 0.003,
                        required_closure_features: 0,
                        nominal_fit_clearance_mm: 0.070,
                    },
                    ComponentPlan {
                        id: ComponentId(7),
                        name: "cover / two-latch shaft retainer".to_owned(),
                        kind: ComponentKind::Retainer,
                        geometry: ComponentGeometry::Retainer {
                            outer_size_mm: [6.00, 4.00, 0.20],
                            thickness_mm: 0.20,
                        },
                        cad_seated_origin_pose: pose([0.0, 0.0, 1.60], 0.0),
                        target_pose: pose([3.0, 2.0, 1.70], 0.0),
                        insertion_direction: [0.0, 0.0, -1.0],
                        requires_handoff: true,
                        requires_mesh: false,
                        insertion_travel_mm: 0.20,
                        insertion_depth_tolerance_mm: 0.015,
                        max_insertion_axial_force_n: 0.050,
                        max_insertion_lateral_force_n: 0.010,
                        required_closure_features: 2,
                        nominal_fit_clearance_mm: -0.020,
                    },
                ],
                gear_meshes: vec![
                    GearMeshPlan {
                        driver: ComponentId(5),
                        driven: ComponentId(4),
                        nominal_center_distance_mm: 2.10,
                    },
                    GearMeshPlan {
                        driver: ComponentId(6),
                        driven: ComponentId(5),
                        nominal_center_distance_mm: 1.50,
                    },
                ],
                intended_contacts: intended_contacts(),
            },
            acceptance: AcceptanceParameters {
                safety: SafetyLimits {
                    max_sensor_age_ms: 80,
                    min_unplanned_clearance_mm: 0.010,
                    max_unplanned_contact_force_n: 0.010,
                },
                vision: VisionLimits {
                    min_confidence: 0.82,
                    max_position_sigma_mm: 0.010,
                    max_orientation_sigma_deg: 0.50,
                },
                grasp: GraspLimits {
                    min_force_n: 0.002,
                    max_force_n: 0.100,
                    max_slip_mm: 0.010,
                    pregrasp_translation_tolerance_mm: 0.020,
                    pregrasp_rotation_tolerance_deg: 1.5,
                },
                handoff: HandoffLimits {
                    min_receiver_force_n: 0.002,
                    max_receiver_force_n: 0.100,
                    max_translation_error_mm: 0.010,
                    max_rotation_error_deg: 1.8,
                },
                alignment: AlignmentLimits {
                    max_lateral_error_mm: 0.010,
                    max_axial_error_mm: 0.020,
                    max_rotation_error_deg: 0.8,
                },
                insertion: InsertionLimits {
                    max_axial_force_n: 0.050,
                    max_lateral_force_n: 0.010,
                    depth_tolerance_mm: 0.015,
                    max_overshoot_mm: 0.020,
                    guarded_speed_mm_s: 0.05,
                    retract_distance_mm: 0.08,
                },
                mesh: MeshLimits {
                    min_sweep_deg: 20.0,
                    max_peak_torque_mn_mm: 20.0,
                    min_backlash_mm: 0.010,
                    max_backlash_mm: 0.040,
                    dither_step_deg: 7.5,
                },
                verification: VerificationLimits {
                    min_vision_confidence: 0.85,
                    max_translation_error_mm: 0.010,
                    max_rotation_error_deg: 1.0,
                    min_rotation_test_deg: 20.0,
                    max_peak_torque_mn_mm: 20.0,
                    max_torque_ripple_fraction: 0.30,
                    min_backlash_mm: 0.010,
                    max_backlash_mm: 0.040,
                },
                servo: VisualServoLimits {
                    translation_gain: 0.65,
                    rotation_gain: 0.55,
                    max_translation_step_mm: 0.050,
                    max_rotation_step_deg: 2.0,
                    speed_mm_s: 0.10,
                    max_steps_per_attempt: 10,
                },
                retries: RetryLimits {
                    locate: 2,
                    pick: 2,
                    handoff: 2,
                    align: 2,
                    insert: 2,
                    mesh: 2,
                    verify: 2,
                    // The executive runs at 20 ms. A 1.20 mm guarded gear
                    // descent at 0.05 mm/s needs 1,200 cycles; leave margin
                    // for visual corrections and two latch signatures.
                    max_cycles_per_attempt: 1800,
                },
            },
        }
    }
}

fn intended_contacts() -> Vec<AssemblyContactDefinition> {
    let mut contacts = Vec::new();
    for shaft in [ComponentId(1), ComponentId(2), ComponentId(3)] {
        contacts.push(AssemblyContactDefinition {
            activates_with: shaft,
            activates_at: Phase::Insert,
            feature_a: ContactFeature::HousingShaftSeat { shaft },
            feature_b: ContactFeature::ShaftExterior { shaft },
            kind: AssemblyContactKind::ShaftSeat,
        });
    }

    for (gear, shaft) in [
        (ComponentId(4), ComponentId(3)),
        (ComponentId(5), ComponentId(2)),
        (ComponentId(6), ComponentId(1)),
    ] {
        contacts.push(AssemblyContactDefinition {
            activates_with: gear,
            activates_at: Phase::Insert,
            feature_a: ContactFeature::GearBore { gear },
            feature_b: ContactFeature::ShaftExterior { shaft },
            kind: AssemblyContactKind::BoreOnShaft,
        });
        contacts.push(AssemblyContactDefinition {
            activates_with: gear,
            activates_at: Phase::Insert,
            feature_a: ContactFeature::GearLowerFace { gear },
            feature_b: ContactFeature::HousingFloor,
            kind: AssemblyContactKind::GearOnFloor,
        });
    }

    for (activates_with, gear_a, gear_b) in [
        (ComponentId(5), ComponentId(5), ComponentId(4)),
        (ComponentId(6), ComponentId(6), ComponentId(5)),
    ] {
        contacts.push(AssemblyContactDefinition {
            activates_with,
            activates_at: Phase::Mesh,
            feature_a: ContactFeature::GearTeeth { gear: gear_a },
            feature_b: ContactFeature::GearTeeth { gear: gear_b },
            kind: AssemblyContactKind::GearMesh,
        });
    }

    let cover = ComponentId(7);
    contacts.push(AssemblyContactDefinition {
        activates_with: cover,
        activates_at: Phase::Insert,
        feature_a: ContactFeature::CoverLip { cover },
        feature_b: ContactFeature::HousingCoverRim,
        kind: AssemblyContactKind::CoverRim,
    });
    for feature in 1..=2 {
        contacts.push(AssemblyContactDefinition {
            activates_with: cover,
            activates_at: Phase::Insert,
            feature_a: ContactFeature::CoverLatch { cover, feature },
            feature_b: ContactFeature::HousingLatchCapture { feature },
            kind: AssemblyContactKind::CoverLatch,
        });
    }
    contacts
}

fn pose(position_mm: [f64; 3], phase_deg: f64) -> NominalAssemblyPose {
    NominalAssemblyPose {
        position_mm,
        part_axis: [0.0, 0.0, 1.0],
        phase_deg,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_scenario_is_valid_and_explicit() {
        let scenario = MicroGearboxScenario::gearbox_baseline_v1();
        scenario.validate().unwrap();
        assert_eq!(scenario.recipe.components.len(), 7);
        assert_eq!(scenario.cell.mobile_arm_count, 4);
        assert_eq!(scenario.cell.usable_tube_length_mm, 320.0);
        assert_eq!(scenario.cell.global_camera_count, 6);
        assert_eq!(scenario.cell.simultaneous_macro_views, 2);
        assert_eq!(scenario.recipe.gear_meshes.len(), 2);
        assert!(scenario
            .recipe
            .components
            .iter()
            .any(|part| part.requires_mesh));
        assert!(scenario
            .recipe
            .components
            .iter()
            .any(|part| part.requires_handoff));
    }

    #[test]
    fn reference_geometry_matches_cad_centres_and_insertion_travel() {
        let scenario = MicroGearboxScenario::gearbox_baseline_v1();
        let housing = scenario.recipe.registered_housing;
        assert!(matches!(
            housing.geometry,
            ComponentGeometry::Housing {
                outer_size_mm: [6.0, 4.0, 1.6],
                wall_mm: 0.030
            }
        ));
        assert_eq!(
            housing.cad_registered_origin_pose.position_mm,
            [0.0, 0.0, 0.0]
        );
        assert_eq!(housing.target_pose.position_mm, [3.0, 2.0, 0.8]);

        let parts = &scenario.recipe.components;
        assert_eq!(
            parts.iter().map(|part| part.id).collect::<Vec<_>>(),
            (1..=7).map(ComponentId).collect::<Vec<_>>()
        );
        for shaft in &parts[..3] {
            assert_eq!(shaft.cad_seated_origin_pose.position_mm[2], 0.0);
            assert_eq!(shaft.target_pose.position_mm[2], 0.775);
            assert_eq!(shaft.insertion_travel_mm, 0.250);
            assert_eq!(shaft.insertion_start_collision_pose().position_mm[2], 1.025);
            assert_eq!(
                shaft
                    .collision_pose_at_travel_mm(shaft.insertion_travel_mm)
                    .unwrap(),
                shaft.target_pose
            );
        }

        let expected_teeth = [24, 18, 12];
        for (gear, teeth) in parts[3..6].iter().zip(expected_teeth) {
            assert!(matches!(
                gear.geometry,
                ComponentGeometry::SpurGear {
                    module_mm: 0.10,
                    teeth: actual_teeth,
                    pressure_angle_deg: 25.0,
                    thickness_mm: 0.35,
                    bore_diameter_mm: 0.420,
                    hub_diameter_mm: 0.550,
                    total_height_mm: 1.30,
                } if actual_teeth == teeth
            ));
            assert_eq!(gear.cad_seated_origin_pose.position_mm[2], 0.250);
            assert_eq!(gear.target_pose.position_mm[2], 0.900);
            assert_eq!(gear.insertion_travel_mm, 1.20);
            assert_eq!(gear.insertion_start_collision_pose().position_mm[2], 2.10);
        }
        assert_eq!(parts[4].target_pose.phase_deg, 170.0);

        let cover = &parts[6];
        assert_eq!(cover.cad_seated_origin_pose.position_mm, [0.0, 0.0, 1.60]);
        assert_eq!(cover.target_pose.position_mm, [3.0, 2.0, 1.70]);
        assert_eq!(cover.insertion_travel_mm, 0.20);
        assert_eq!(cover.insertion_start_collision_pose().position_mm[2], 1.90);
    }

    #[test]
    fn intended_contacts_activate_by_feature_and_assembly_phase() {
        let scenario = MicroGearboxScenario::gearbox_baseline_v1();
        let recipe = &scenario.recipe;
        assert_eq!(recipe.intended_contacts.len(), 14);

        // G2 insertion permits its bore/shaft and floor support, but not its
        // teeth until the explicit mesh phase.
        let g2_insert = recipe.intended_contacts_for(ComponentId(5), Phase::Insert);
        assert_eq!(g2_insert.len(), 7);
        assert!(!g2_insert.iter().any(|contact| {
            contact.kind == AssemblyContactKind::GearMesh
                && contact.activates_with == ComponentId(5)
        }));
        let g2_mesh = recipe.intended_contacts_for(ComponentId(5), Phase::Mesh);
        assert_eq!(g2_mesh.len(), 8);

        // Existing seat, bore, floor, and mesh relations remain active while
        // the cover is handled; only feature-level cover contacts are added.
        assert_eq!(
            recipe
                .intended_contacts_for(ComponentId(7), Phase::Locate)
                .len(),
            11
        );
        let cover_insert = recipe.intended_contacts_for(ComponentId(7), Phase::Insert);
        assert_eq!(cover_insert.len(), 14);
        assert_eq!(
            cover_insert
                .iter()
                .filter(|contact| contact.kind == AssemblyContactKind::CoverLatch)
                .count(),
            2
        );
    }
}
