//! Deterministic observed-state estimation for axisymmetric manipulation features.
//!
//! This module intentionally has no dependency on simulation scene or rigid-body
//! truth.  Its boundary is a list of labelled, timestamped 3-D feature
//! measurements and a calibrated axial feature model.  A burst is solved by
//! weighted least squares for
//!
//! `measured_point_world = center_world + axis_world * axial_coordinate`.
//!
//! The resulting state has five observable degrees of freedom: three for the
//! object center and two for its unit axis.  Roll about that axis is explicitly
//! unobservable.  Each burst replaces (rather than repeatedly information-fuses)
//! the previous optical solution.  That deliberate choice avoids treating the
//! configured correlated calibration floor as independent noise that averages
//! away.  The prior is still used for a reported/gated innovation, and commanded
//! translation predicts the state and grows its uncertainty between bursts.
//!
//! The input points are expected to be centerline-constraining features such as
//! fitted ring centers or paired-feature midpoints.  Raw image pixels, unpaired
//! points on a ring, and photometric confidence are outside this estimator's
//! fidelity boundary and must be converted by the optics adapter.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use serde::{Deserialize, Serialize};

pub const AXISYMMETRIC_POSE_DOF: usize = 5;

/// Calibrated geometry for one labelled centerline-constraining feature.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct KnownAxialFeature {
    pub feature_id: u32,
    /// Signed coordinate along the object's local symmetry axis, in metres.
    /// The object center reported by the estimator is at coordinate zero.
    pub axial_coordinate_m: f64,
}

/// Controller-safe observation DTO produced by the optics boundary.
///
/// `covariance_diagonal_m2` is the independent localization contribution for
/// this observation.  Shared calibration error belongs in
/// [`EstimatorConfig::correlated_position_floor_m`] and must not be copied into
/// every observation as though it were independent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FeatureMeasurement {
    pub object_id: u32,
    pub feature_id: u32,
    /// A calibrated optical head identifier.  It is also the observation-source
    /// identifier for this reduced DTO.
    pub head_id: u32,
    pub calibrated_ray_count: u32,
    /// Fixed-step tick at which sensor exposure occurred.
    pub capture_tick: u64,
    /// Fixed-step tick at which the measurement became controller-visible.
    pub available_tick: u64,
    pub measured_point_world_m: [f64; 3],
    pub covariance_diagonal_m2: [f64; 3],
    /// Deterministic quality weight in `(0, 1]`; this is not a probability.
    pub confidence: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EstimatorConfig {
    /// Duration represented by one deterministic scheduler tick.
    pub tick_period_s: f64,
    pub maximum_measurement_age_ticks: u64,
    pub maximum_burst_span_ticks: u64,
    pub minimum_measurement_count: usize,
    pub minimum_feature_count: usize,
    pub minimum_axial_station_count: usize,
    pub minimum_head_count: usize,
    pub minimum_calibrated_ray_count: u64,
    pub minimum_confidence: f64,
    /// Minimum span between the most-negative and most-positive accepted axial
    /// feature coordinates. The WLS covariance separately captures how the
    /// samples are distributed within that span.
    pub minimum_axial_lever_arm_m: f64,
    /// Numerical lower bound for an independent measurement variance.
    pub minimum_independent_variance_m2: f64,
    pub outlier_sigma_threshold: f64,
    pub outlier_absolute_threshold_m: f64,
    pub maximum_outlier_fraction: f64,
    pub maximum_outlier_iterations: usize,
    pub maximum_residual_rms_m: f64,
    pub maximum_normalized_residual_rms: f64,
    /// Allowed deviation of the fitted (pre-normalization) axis magnitude from
    /// unity.  Large deviations indicate inconsistent feature geometry/scale.
    pub maximum_axis_scale_error: f64,
    /// Shared position floor. It is added once, after all independent samples
    /// are fused, so more heads or rays can never average it away. The floor is
    /// treated as a static common calibration mode across successive estimates
    /// from the same non-empty set of `head_id` values; time-varying drift must
    /// be represented separately as process or measurement uncertainty.
    pub correlated_position_floor_m: [f64; 3],
    /// Shared angular calibration floor in each tangent direction.
    pub correlated_axis_floor_rad: f64,
    pub maximum_position_sigma_m: f64,
    pub maximum_axis_sigma_rad: f64,
    pub maximum_innovation_translation_m: f64,
    pub maximum_innovation_axis_rad: f64,
    pub maximum_normalized_innovation: f64,
    /// Loaded hold/process noise, expressed per square-root second.
    pub hold_process_sigma_m_per_sqrt_s: [f64; 3],
    pub hold_axis_sigma_rad_per_sqrt_s: f64,
    /// Additional translation uncertainty as a fraction of commanded travel.
    pub commanded_translation_fractional_sigma: f64,
}

impl Default for EstimatorConfig {
    fn default() -> Self {
        Self {
            tick_period_s: 0.020,
            maximum_measurement_age_ticks: 5,
            maximum_burst_span_ticks: 2,
            minimum_measurement_count: 4,
            minimum_feature_count: 4,
            minimum_axial_station_count: 2,
            minimum_head_count: 1,
            minimum_calibrated_ray_count: 8,
            minimum_confidence: 0.25,
            minimum_axial_lever_arm_m: 50.0e-6,
            minimum_independent_variance_m2: 0.05e-6_f64.powi(2),
            outlier_sigma_threshold: 5.0,
            outlier_absolute_threshold_m: 25.0e-6,
            maximum_outlier_fraction: 0.34,
            maximum_outlier_iterations: 4,
            maximum_residual_rms_m: 12.0e-6,
            maximum_normalized_residual_rms: 3.0,
            maximum_axis_scale_error: 0.20,
            correlated_position_floor_m: [3.0e-6, 3.0e-6, 3.4e-6],
            correlated_axis_floor_rad: 1.0e-3,
            maximum_position_sigma_m: 25.0e-6,
            maximum_axis_sigma_rad: 0.08,
            maximum_innovation_translation_m: 0.75e-3,
            maximum_innovation_axis_rad: 0.35,
            maximum_normalized_innovation: 6.0,
            hold_process_sigma_m_per_sqrt_s: [1.0e-6, 1.0e-6, 1.5e-6],
            hold_axis_sigma_rad_per_sqrt_s: 2.0e-3,
            commanded_translation_fractional_sigma: 0.01,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct AxisymmetricPose5d {
    pub center_world_m: [f64; 3],
    pub axis_world_unit: [f64; 3],
    /// Always false for this estimator.  Kept in reports to prevent a 5-DOF
    /// result from being mistaken for an image-derived 6-DOF pose.
    pub roll_observable: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ReducedPoseUncertainty {
    pub center_variance_m2: [f64; 3],
    pub center_sigma_m: [f64; 3],
    /// Deterministic orthonormal basis of the plane tangent to the estimated
    /// unit axis.  The next two variance entries are expressed in this basis.
    pub axis_tangent_basis_world: [[f64; 3]; 2],
    pub axis_tangent_variance_rad2: [f64; 2],
    pub axis_tangent_sigma_rad: [f64; 2],
    pub correlated_position_floor_m: [f64; 3],
    pub correlated_axis_floor_rad: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct PoseInnovation {
    pub translation_world_m: [f64; 3],
    pub translation_norm_m: f64,
    pub axis_angle_rad: f64,
    /// Approximate reduced-state Mahalanobis norm. The published marginal
    /// variances conservatively diagonalize the center/axis coupling that this
    /// deliberately small report type does not expose.
    pub normalized_norm: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum EstimateValidity {
    Valid,
    NoMeasurements,
    NoTargetMeasurements,
    UnknownFeature,
    NonFiniteMeasurement,
    InvalidMeasurementConfidence,
    InvalidMeasurementCovariance,
    InvalidTimestampOrder,
    ObservationTimestampRegression,
    MeasurementNotAvailable,
    StaleMeasurements,
    BurstSpanExceeded,
    DuplicateMeasurement,
    InsufficientMeasurements,
    InsufficientFeatures,
    InsufficientAxialStations,
    InsufficientHeads,
    InsufficientCalibratedRays,
    InsufficientAxialLeverArm,
    RankDeficient,
    AxisScaleInconsistent,
    OutlierBudgetExceeded,
    ResidualTooLarge,
    InnovationTooLarge,
    ExcessiveUncertainty,
    InvalidUncertaintyInflation,
    NoPriorEstimate,
    PredictionTimestampRegression,
    NonFinitePrediction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct MeasurementKey {
    pub feature_id: u32,
    pub head_id: u32,
    pub capture_tick: u64,
    pub available_tick: u64,
}

impl From<&FeatureMeasurement> for MeasurementKey {
    fn from(value: &FeatureMeasurement) -> Self {
        Self {
            feature_id: value.feature_id,
            head_id: value.head_id,
            capture_tick: value.capture_tick,
            available_tick: value.available_tick,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MeasurementRejectionReason {
    OtherObject,
    UnknownFeature,
    LowConfidence,
    NoCalibratedRays,
    NotYetAvailable,
    Stale,
    StatisticalOutlier,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct RejectedMeasurement {
    pub key: MeasurementKey,
    pub reason: MeasurementRejectionReason,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct PoseEstimate {
    pub object_id: u32,
    pub controller_tick: u64,
    pub state_tick: u64,
    pub oldest_capture_tick: Option<u64>,
    pub newest_capture_tick: Option<u64>,
    pub oldest_available_tick: Option<u64>,
    pub newest_available_tick: Option<u64>,
    /// Conservative age from the oldest accepted exposure in the burst.
    pub measurement_age_ticks: Option<u64>,
    pub measurement_age_s: Option<f64>,
    pub maximum_capture_to_available_latency_ticks: Option<u64>,
    pub observation_source_count: usize,
    pub head_count: usize,
    pub view_count: usize,
    pub calibrated_ray_count: u64,
    pub minimum_calibrated_rays_per_measurement: Option<u32>,
    pub accepted_measurement_count: usize,
    pub accepted_feature_count: usize,
    pub rejected_measurement_count: usize,
    pub rejected_measurements: Vec<RejectedMeasurement>,
    pub outlier_count: usize,
    pub residual_rms_m: Option<f64>,
    pub residual_max_m: Option<f64>,
    pub normalized_residual_rms: Option<f64>,
    pub innovation: Option<PoseInnovation>,
    pub prediction_count: u32,
    pub last_prediction_distance_m: Option<f64>,
    pub last_prediction_interval_s: Option<f64>,
    pub pose: Option<AxisymmetricPose5d>,
    pub uncertainty: Option<ReducedPoseUncertainty>,
    pub validity: EstimateValidity,
    pub validity_detail: String,
}

impl PoseEstimate {
    pub fn is_valid(&self) -> bool {
        self.validity == EstimateValidity::Valid
    }

    /// Returns a pose only after checking validity, which is safer than reading
    /// the diagnostic `pose` field directly in a controller.
    pub fn usable_pose(&self) -> Option<&AxisymmetricPose5d> {
        self.is_valid().then_some(self.pose.as_ref()).flatten()
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EstimatorConfigError {
    pub field: &'static str,
    pub reason: &'static str,
}

impl fmt::Display for EstimatorConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "invalid estimator field '{}': {}",
            self.field, self.reason
        )
    }
}

impl std::error::Error for EstimatorConfigError {}

#[derive(Clone, Debug)]
struct FilterState {
    pose: AxisymmetricPose5d,
    uncertainty: ReducedPoseUncertainty,
    state_tick: u64,
    oldest_capture_tick: u64,
    newest_capture_tick: u64,
    prediction_count: u32,
}

#[derive(Clone, Debug)]
pub struct ObservedPoseEstimator {
    config: EstimatorConfig,
    object_id: u32,
    features: BTreeMap<u32, f64>,
    last_state: Option<FilterState>,
    last_report: Option<PoseEstimate>,
}

impl ObservedPoseEstimator {
    pub fn new(
        config: EstimatorConfig,
        object_id: u32,
        feature_model: Vec<KnownAxialFeature>,
    ) -> Result<Self, EstimatorConfigError> {
        validate_config(&config)?;
        let mut features = BTreeMap::new();
        for feature in feature_model {
            if !feature.axial_coordinate_m.is_finite() {
                return Err(config_error(
                    "feature_model.axial_coordinate_m",
                    "must be finite",
                ));
            }
            if features
                .insert(
                    feature.feature_id,
                    canonical_zero(feature.axial_coordinate_m),
                )
                .is_some()
            {
                return Err(config_error("feature_model.feature_id", "must be unique"));
            }
        }
        if features.len() < config.minimum_feature_count {
            return Err(config_error(
                "feature_model",
                "contains fewer entries than minimum_feature_count",
            ));
        }
        let stations = distinct_station_count(features.values().copied());
        if stations < config.minimum_axial_station_count {
            return Err(config_error(
                "feature_model",
                "contains too few distinct axial stations",
            ));
        }
        let minimum_station_m = features
            .values()
            .copied()
            .min_by(f64::total_cmp)
            .expect("validated feature model is non-empty");
        let maximum_station_m = features
            .values()
            .copied()
            .max_by(f64::total_cmp)
            .expect("validated feature model is non-empty");
        if maximum_station_m - minimum_station_m < config.minimum_axial_lever_arm_m {
            return Err(config_error(
                "feature_model",
                "axial station span is below minimum_axial_lever_arm_m",
            ));
        }

        Ok(Self {
            config,
            object_id,
            features,
            last_state: None,
            last_report: None,
        })
    }

    pub fn config(&self) -> &EstimatorConfig {
        &self.config
    }

    pub fn object_id(&self) -> u32 {
        self.object_id
    }

    pub fn reset(&mut self) {
        self.last_state = None;
        self.last_report = None;
    }

    pub fn last_report(&self) -> Option<&PoseEstimate> {
        self.last_report.as_ref()
    }

    /// Estimate an axisymmetric pose from all currently available observations.
    /// Input order cannot affect the result. Bursts must advance beyond the
    /// previous accepted exposures and must not predate commanded prediction.
    /// Delayed observations are not replayed through a motion history.
    pub fn update(
        &mut self,
        controller_tick: u64,
        measurements: &[FeatureMeasurement],
    ) -> PoseEstimate {
        if self
            .last_state
            .as_ref()
            .is_some_and(|state| controller_tick < state.state_tick)
        {
            return self.store_invalid(invalid_report(
                self.object_id,
                controller_tick,
                EstimateValidity::ObservationTimestampRegression,
                "observation update precedes the current estimator state tick",
                Vec::new(),
            ));
        }
        let mut ordered = measurements.to_vec();
        ordered.sort_by(measurement_order);

        let mut rejected = Vec::new();
        let mut working = Vec::new();
        let mut saw_target = false;
        let mut saw_unknown_feature = false;
        let mut saw_unavailable = false;
        let mut saw_stale = false;

        for measurement in ordered {
            let key = MeasurementKey::from(&measurement);
            if measurement.object_id != self.object_id {
                rejected.push(RejectedMeasurement {
                    key,
                    reason: MeasurementRejectionReason::OtherObject,
                });
                continue;
            }
            saw_target = true;

            let Some(&axial_coordinate_m) = self.features.get(&measurement.feature_id) else {
                saw_unknown_feature = true;
                rejected.push(RejectedMeasurement {
                    key,
                    reason: MeasurementRejectionReason::UnknownFeature,
                });
                continue;
            };

            if !array_is_finite(measurement.measured_point_world_m)
                || !measurement.confidence.is_finite()
            {
                return self.store_invalid(invalid_report(
                    self.object_id,
                    controller_tick,
                    EstimateValidity::NonFiniteMeasurement,
                    "a target measurement contains a non-finite point or confidence",
                    rejected,
                ));
            }
            if !array_is_finite(measurement.covariance_diagonal_m2)
                || measurement
                    .covariance_diagonal_m2
                    .iter()
                    .any(|variance| *variance <= 0.0)
            {
                return self.store_invalid(invalid_report(
                    self.object_id,
                    controller_tick,
                    EstimateValidity::InvalidMeasurementCovariance,
                    "a target measurement covariance is non-finite or non-positive",
                    rejected,
                ));
            }
            if measurement.confidence <= 0.0 || measurement.confidence > 1.0 {
                return self.store_invalid(invalid_report(
                    self.object_id,
                    controller_tick,
                    EstimateValidity::InvalidMeasurementConfidence,
                    "measurement confidence must lie in (0, 1]",
                    rejected,
                ));
            }
            if measurement.capture_tick > measurement.available_tick {
                return self.store_invalid(invalid_report(
                    self.object_id,
                    controller_tick,
                    EstimateValidity::InvalidTimestampOrder,
                    "capture_tick is later than available_tick",
                    rejected,
                ));
            }
            if measurement.available_tick > controller_tick {
                saw_unavailable = true;
                rejected.push(RejectedMeasurement {
                    key,
                    reason: MeasurementRejectionReason::NotYetAvailable,
                });
                continue;
            }
            if controller_tick.saturating_sub(measurement.capture_tick)
                > self.config.maximum_measurement_age_ticks
            {
                saw_stale = true;
                rejected.push(RejectedMeasurement {
                    key,
                    reason: MeasurementRejectionReason::Stale,
                });
                continue;
            }
            if measurement.confidence < self.config.minimum_confidence {
                rejected.push(RejectedMeasurement {
                    key,
                    reason: MeasurementRejectionReason::LowConfidence,
                });
                continue;
            }
            if measurement.calibrated_ray_count == 0 {
                rejected.push(RejectedMeasurement {
                    key,
                    reason: MeasurementRejectionReason::NoCalibratedRays,
                });
                continue;
            }

            working.push(WorkingMeasurement {
                measurement,
                axial_coordinate_m,
            });
        }

        if working.is_empty() {
            let (validity, detail) = if !saw_target {
                (
                    if measurements.is_empty() {
                        EstimateValidity::NoMeasurements
                    } else {
                        EstimateValidity::NoTargetMeasurements
                    },
                    "no measurements for the configured object",
                )
            } else if saw_stale {
                (
                    EstimateValidity::StaleMeasurements,
                    "all usable target measurements exceed the age limit",
                )
            } else if saw_unavailable {
                (
                    EstimateValidity::MeasurementNotAvailable,
                    "target measurements have not reached their availability tick",
                )
            } else if saw_unknown_feature {
                (
                    EstimateValidity::UnknownFeature,
                    "target observations do not match the calibrated feature model",
                )
            } else {
                (
                    EstimateValidity::InsufficientMeasurements,
                    "all target measurements failed quality or ray gates",
                )
            };
            return self.store_invalid(invalid_report(
                self.object_id,
                controller_tick,
                validity,
                detail,
                rejected,
            ));
        }

        for pair in working.windows(2) {
            if same_exposure_identity(&pair[0].measurement, &pair[1].measurement) {
                return self.store_invalid(invalid_report_with_working(
                    self.object_id,
                    controller_tick,
                    EstimateValidity::DuplicateMeasurement,
                    "duplicate object/feature/head/capture exposure",
                    &working,
                    rejected,
                    self.config.tick_period_s,
                ));
            }
        }

        let oldest_capture_tick = working
            .iter()
            .map(|item| item.measurement.capture_tick)
            .min()
            .expect("working is non-empty");
        let newest_capture_tick = working
            .iter()
            .map(|item| item.measurement.capture_tick)
            .max()
            .expect("working is non-empty");
        if self.last_state.as_ref().is_some_and(|state| {
            oldest_capture_tick <= state.newest_capture_tick
                || (state.prediction_count > 0 && oldest_capture_tick < state.state_tick)
        }) {
            return self.store_invalid(invalid_report_with_working(
                self.object_id,
                controller_tick,
                EstimateValidity::ObservationTimestampRegression,
                "burst reuses accepted exposures or predates commanded prediction",
                &working,
                rejected,
                self.config.tick_period_s,
            ));
        }
        if newest_capture_tick.saturating_sub(oldest_capture_tick)
            > self.config.maximum_burst_span_ticks
        {
            return self.store_invalid(invalid_report_with_working(
                self.object_id,
                controller_tick,
                EstimateValidity::BurstSpanExceeded,
                "accepted exposure timestamps span more than the configured burst limit",
                &working,
                rejected,
                self.config.tick_period_s,
            ));
        }

        if let Some((validity, detail)) = sample_gate_failure(&self.config, &working) {
            return self.store_invalid(invalid_report_with_working(
                self.object_id,
                controller_tick,
                validity,
                detail,
                &working,
                rejected,
                self.config.tick_period_s,
            ));
        }

        let original_count = working.len();
        let mut active: Vec<usize> = (0..working.len()).collect();
        let mut outlier_count = 0usize;
        let fit = loop {
            let fit = match weighted_fit(&working, &active, &self.config) {
                Ok(fit) => fit,
                Err(validity) => {
                    return self.store_invalid(invalid_report_with_working(
                        self.object_id,
                        controller_tick,
                        validity,
                        "weighted normal matrix is rank deficient",
                        &working,
                        rejected,
                        self.config.tick_period_s,
                    ));
                }
            };

            let worst = worst_residual(&working, &active, &fit, &self.config);
            let is_outlier = worst.normalized > self.config.outlier_sigma_threshold
                || worst.distance_m > self.config.outlier_absolute_threshold_m;
            if !is_outlier {
                break fit;
            }

            let proposed_outlier_count = outlier_count + 1;
            let fraction = proposed_outlier_count as f64 / original_count as f64;
            if proposed_outlier_count > self.config.maximum_outlier_iterations
                || fraction > self.config.maximum_outlier_fraction
            {
                return self.store_invalid(invalid_report_with_working(
                    self.object_id,
                    controller_tick,
                    EstimateValidity::OutlierBudgetExceeded,
                    "residual rejection would exceed the configured outlier budget",
                    &working,
                    rejected,
                    self.config.tick_period_s,
                ));
            }

            let removed_index = active.remove(worst.active_position);
            let removed = &working[removed_index].measurement;
            rejected.push(RejectedMeasurement {
                key: MeasurementKey::from(removed),
                reason: MeasurementRejectionReason::StatisticalOutlier,
            });
            outlier_count = proposed_outlier_count;

            let remaining: Vec<WorkingMeasurement> =
                active.iter().map(|index| working[*index].clone()).collect();
            if let Some((validity, detail)) = sample_gate_failure(&self.config, &remaining) {
                return self.store_invalid(invalid_report_with_working(
                    self.object_id,
                    controller_tick,
                    validity,
                    detail,
                    &remaining,
                    rejected,
                    self.config.tick_period_s,
                ));
            }
        };

        let residuals = residual_summary(&working, &active, &fit, &self.config);
        if residuals.rms_m > self.config.maximum_residual_rms_m
            || residuals.normalized_rms > self.config.maximum_normalized_residual_rms
        {
            return self.store_invalid(invalid_report_with_working(
                self.object_id,
                controller_tick,
                EstimateValidity::ResidualTooLarge,
                "post-rejection residual exceeds the configured limit",
                &working,
                rejected,
                self.config.tick_period_s,
            ));
        }
        if (fit.raw_axis_norm - 1.0).abs() > self.config.maximum_axis_scale_error {
            return self.store_invalid(invalid_report_with_working(
                self.object_id,
                controller_tick,
                EstimateValidity::AxisScaleInconsistent,
                "fitted feature scale is inconsistent with a unit object axis",
                &working,
                rejected,
                self.config.tick_period_s,
            ));
        }

        let active_working: Vec<WorkingMeasurement> =
            active.iter().map(|index| working[*index].clone()).collect();
        let mut uncertainty = fit_uncertainty(&fit, &residuals, active.len(), &self.config);
        let age_ticks = controller_tick.saturating_sub(oldest_capture_tick);
        add_process_noise(
            &mut uncertainty,
            age_ticks as f64 * self.config.tick_period_s,
            0.0,
            &self.config,
        );

        if uncertainty_exceeds_limits(&uncertainty, &self.config) {
            return self.store_invalid(invalid_report_with_working(
                self.object_id,
                controller_tick,
                EstimateValidity::ExcessiveUncertainty,
                "pose covariance exceeds configured controller limits",
                &active_working,
                rejected,
                self.config.tick_period_s,
            ));
        }

        let pose = AxisymmetricPose5d {
            center_world_m: fit.center_world_m,
            axis_world_unit: fit.axis_world_unit,
            roll_observable: false,
        };
        let innovation = self.last_state.as_ref().map(|prior| {
            pose_innovation(
                prior.pose,
                prior.uncertainty,
                pose,
                uncertainty,
                // A repeated head identifier does not prove that its common
                // calibration mode stayed fixed between bursts. M1e models
                // configurable drift but carries no cross-time calibration
                // covariance, so production innovation must retain both
                // correlated floors instead of assuming cancellation.
                false,
            )
        });
        if innovation.is_some_and(|value| {
            value.translation_norm_m > self.config.maximum_innovation_translation_m
                || value.axis_angle_rad > self.config.maximum_innovation_axis_rad
                || value.normalized_norm > self.config.maximum_normalized_innovation
        }) {
            return self.store_invalid(invalid_report_with_working(
                self.object_id,
                controller_tick,
                EstimateValidity::InnovationTooLarge,
                "observation innovation exceeds configured pose limits",
                &active_working,
                rejected,
                self.config.tick_period_s,
            ));
        }

        rejected.sort_by(rejection_order);
        let counts = observation_counts(&active_working);
        let report = PoseEstimate {
            object_id: self.object_id,
            controller_tick,
            state_tick: controller_tick,
            oldest_capture_tick: Some(oldest_capture_tick),
            newest_capture_tick: Some(newest_capture_tick),
            oldest_available_tick: counts.oldest_available_tick,
            newest_available_tick: counts.newest_available_tick,
            measurement_age_ticks: Some(age_ticks),
            measurement_age_s: Some(age_ticks as f64 * self.config.tick_period_s),
            maximum_capture_to_available_latency_ticks: counts.maximum_latency_ticks,
            observation_source_count: counts.head_count,
            head_count: counts.head_count,
            view_count: counts.view_count,
            calibrated_ray_count: counts.calibrated_ray_count,
            minimum_calibrated_rays_per_measurement: counts.minimum_rays,
            accepted_measurement_count: active_working.len(),
            accepted_feature_count: counts.feature_count,
            rejected_measurement_count: rejected.len(),
            rejected_measurements: rejected,
            outlier_count,
            residual_rms_m: Some(residuals.rms_m),
            residual_max_m: Some(residuals.max_m),
            normalized_residual_rms: Some(residuals.normalized_rms),
            innovation,
            prediction_count: 0,
            last_prediction_distance_m: None,
            last_prediction_interval_s: None,
            pose: Some(pose),
            uncertainty: Some(uncertainty),
            validity: EstimateValidity::Valid,
            validity_detail: "weighted axial feature solution accepted".to_owned(),
        };
        self.last_state = Some(FilterState {
            pose,
            uncertainty,
            state_tick: controller_tick,
            oldest_capture_tick,
            newest_capture_tick,
            prediction_count: 0,
        });
        self.last_report = Some(report.clone());
        report
    }

    /// Predict through a controller-commanded pure translation.  This does not
    /// claim that motor position equals distal pose: configured hold and travel
    /// process noise are accumulated, and the original observation timestamp is
    /// retained for staleness gating.
    pub fn predict_commanded_translation(
        &mut self,
        to_tick: u64,
        commanded_translation_world_m: [f64; 3],
    ) -> PoseEstimate {
        if !array_is_finite(commanded_translation_world_m) {
            return self.store_invalid(invalid_report(
                self.object_id,
                to_tick,
                EstimateValidity::NonFinitePrediction,
                "commanded translation contains a non-finite component",
                Vec::new(),
            ));
        }
        let Some(mut state) = self.last_state.clone() else {
            return self.store_invalid(invalid_report(
                self.object_id,
                to_tick,
                EstimateValidity::NoPriorEstimate,
                "translation prediction requires a previously accepted observation",
                Vec::new(),
            ));
        };
        if !self
            .last_report
            .as_ref()
            .is_some_and(PoseEstimate::is_valid)
        {
            return self.store_invalid(invalid_report(
                self.object_id,
                to_tick,
                EstimateValidity::NoPriorEstimate,
                "the most recent estimator decision is invalid; reacquisition is required",
                Vec::new(),
            ));
        }
        if to_tick < state.state_tick {
            return self.store_invalid(invalid_report(
                self.object_id,
                to_tick,
                EstimateValidity::PredictionTimestampRegression,
                "prediction tick precedes the current estimator state tick",
                Vec::new(),
            ));
        }

        let interval_ticks = to_tick - state.state_tick;
        let interval_s = interval_ticks as f64 * self.config.tick_period_s;
        let distance_m = norm(commanded_translation_world_m);
        state.pose.center_world_m = add(state.pose.center_world_m, commanded_translation_world_m);
        add_process_noise(&mut state.uncertainty, interval_s, distance_m, &self.config);
        state.state_tick = to_tick;
        state.prediction_count = state.prediction_count.saturating_add(1);

        let age_ticks = to_tick.saturating_sub(state.oldest_capture_tick);
        let stale = age_ticks > self.config.maximum_measurement_age_ticks;
        let excessive_uncertainty = uncertainty_exceeds_limits(&state.uncertainty, &self.config);
        let validity = if stale {
            EstimateValidity::StaleMeasurements
        } else if excessive_uncertainty {
            EstimateValidity::ExcessiveUncertainty
        } else {
            EstimateValidity::Valid
        };
        let detail = match validity {
            EstimateValidity::Valid => "commanded translation prediction accepted",
            EstimateValidity::StaleMeasurements => {
                "predicted pose retained for diagnostics, but its observation is stale"
            }
            EstimateValidity::ExcessiveUncertainty => {
                "predicted pose retained for diagnostics, but process uncertainty is excessive"
            }
            _ => unreachable!("prediction validity is constrained above"),
        };

        let pose = (validity == EstimateValidity::Valid).then_some(state.pose);
        let uncertainty = (validity == EstimateValidity::Valid).then_some(state.uncertainty);
        let report = PoseEstimate {
            object_id: self.object_id,
            controller_tick: to_tick,
            state_tick: to_tick,
            oldest_capture_tick: Some(state.oldest_capture_tick),
            newest_capture_tick: Some(state.newest_capture_tick),
            oldest_available_tick: self
                .last_report
                .as_ref()
                .and_then(|report| report.oldest_available_tick),
            newest_available_tick: self
                .last_report
                .as_ref()
                .and_then(|report| report.newest_available_tick),
            measurement_age_ticks: Some(age_ticks),
            measurement_age_s: Some(age_ticks as f64 * self.config.tick_period_s),
            maximum_capture_to_available_latency_ticks: self
                .last_report
                .as_ref()
                .and_then(|report| report.maximum_capture_to_available_latency_ticks),
            observation_source_count: self
                .last_report
                .as_ref()
                .map_or(0, |report| report.observation_source_count),
            head_count: self
                .last_report
                .as_ref()
                .map_or(0, |report| report.head_count),
            view_count: self
                .last_report
                .as_ref()
                .map_or(0, |report| report.view_count),
            calibrated_ray_count: self
                .last_report
                .as_ref()
                .map_or(0, |report| report.calibrated_ray_count),
            minimum_calibrated_rays_per_measurement: self
                .last_report
                .as_ref()
                .and_then(|report| report.minimum_calibrated_rays_per_measurement),
            accepted_measurement_count: self
                .last_report
                .as_ref()
                .map_or(0, |report| report.accepted_measurement_count),
            accepted_feature_count: self
                .last_report
                .as_ref()
                .map_or(0, |report| report.accepted_feature_count),
            rejected_measurement_count: 0,
            rejected_measurements: Vec::new(),
            outlier_count: self
                .last_report
                .as_ref()
                .map_or(0, |report| report.outlier_count),
            residual_rms_m: self
                .last_report
                .as_ref()
                .and_then(|report| report.residual_rms_m),
            residual_max_m: self
                .last_report
                .as_ref()
                .and_then(|report| report.residual_max_m),
            normalized_residual_rms: self
                .last_report
                .as_ref()
                .and_then(|report| report.normalized_residual_rms),
            innovation: None,
            prediction_count: state.prediction_count,
            last_prediction_distance_m: Some(distance_m),
            last_prediction_interval_s: Some(interval_s),
            pose,
            uncertainty,
            validity,
            validity_detail: detail.to_owned(),
        };
        self.last_state = Some(state);
        self.last_report = Some(report.clone());
        report
    }

    /// Add an independently characterized held-part transform uncertainty.
    ///
    /// This is intended for the grasp transition: the controller can carry a
    /// measured jaw-to-part attachment covariance without querying the latent
    /// attachment transform.  The supplied standard deviations are added in
    /// quadrature exactly once.  Loaded hold/process uncertainty remains a
    /// separate time-dependent contribution applied here and by subsequent
    /// predictions.
    pub fn inflate_translation_uncertainty(
        &mut self,
        at_tick: u64,
        additional_sigma_m: [f64; 3],
        detail: &str,
    ) -> PoseEstimate {
        if !array_is_finite(additional_sigma_m)
            || additional_sigma_m.iter().any(|sigma| *sigma < 0.0)
        {
            return self.store_invalid(invalid_report(
                self.object_id,
                at_tick,
                EstimateValidity::InvalidUncertaintyInflation,
                "held-transform standard deviation is non-finite or negative",
                Vec::new(),
            ));
        }
        let Some(mut state) = self.last_state.clone() else {
            return self.store_invalid(invalid_report(
                self.object_id,
                at_tick,
                EstimateValidity::NoPriorEstimate,
                "uncertainty inflation requires a previously accepted observation",
                Vec::new(),
            ));
        };
        let Some(previous_report) = self.last_report.clone().filter(PoseEstimate::is_valid) else {
            return self.store_invalid(invalid_report(
                self.object_id,
                at_tick,
                EstimateValidity::NoPriorEstimate,
                "the most recent estimator decision is invalid; reacquisition is required",
                Vec::new(),
            ));
        };
        if at_tick < state.state_tick {
            return self.store_invalid(invalid_report(
                self.object_id,
                at_tick,
                EstimateValidity::PredictionTimestampRegression,
                "uncertainty inflation tick precedes the estimator state tick",
                Vec::new(),
            ));
        }

        let interval_s = (at_tick - state.state_tick) as f64 * self.config.tick_period_s;
        add_process_noise(&mut state.uncertainty, interval_s, 0.0, &self.config);
        for (dimension, sigma) in additional_sigma_m.into_iter().enumerate() {
            state.uncertainty.center_variance_m2[dimension] += sigma * sigma;
            state.uncertainty.center_variance_m2[dimension] = state.uncertainty.center_variance_m2
                [dimension]
                .max(self.config.correlated_position_floor_m[dimension].powi(2));
            state.uncertainty.center_sigma_m[dimension] =
                state.uncertainty.center_variance_m2[dimension].sqrt();
        }
        state.state_tick = at_tick;

        let age_ticks = at_tick.saturating_sub(state.oldest_capture_tick);
        let validity = if age_ticks > self.config.maximum_measurement_age_ticks {
            EstimateValidity::StaleMeasurements
        } else if uncertainty_exceeds_limits(&state.uncertainty, &self.config) {
            EstimateValidity::ExcessiveUncertainty
        } else {
            EstimateValidity::Valid
        };
        let validity_detail = match validity {
            EstimateValidity::Valid => format!("held-transform uncertainty applied: {detail}"),
            EstimateValidity::StaleMeasurements => {
                "held-transform uncertainty applied, but the underlying observation is stale"
                    .to_owned()
            }
            EstimateValidity::ExcessiveUncertainty => {
                "held-transform uncertainty exceeds configured controller limits".to_owned()
            }
            _ => unreachable!("inflation validity is constrained above"),
        };
        let report = PoseEstimate {
            object_id: self.object_id,
            controller_tick: at_tick,
            state_tick: at_tick,
            oldest_capture_tick: Some(state.oldest_capture_tick),
            newest_capture_tick: Some(state.newest_capture_tick),
            oldest_available_tick: previous_report.oldest_available_tick,
            newest_available_tick: previous_report.newest_available_tick,
            measurement_age_ticks: Some(age_ticks),
            measurement_age_s: Some(age_ticks as f64 * self.config.tick_period_s),
            maximum_capture_to_available_latency_ticks: previous_report
                .maximum_capture_to_available_latency_ticks,
            observation_source_count: previous_report.observation_source_count,
            head_count: previous_report.head_count,
            view_count: previous_report.view_count,
            calibrated_ray_count: previous_report.calibrated_ray_count,
            minimum_calibrated_rays_per_measurement: previous_report
                .minimum_calibrated_rays_per_measurement,
            accepted_measurement_count: previous_report.accepted_measurement_count,
            accepted_feature_count: previous_report.accepted_feature_count,
            rejected_measurement_count: 0,
            rejected_measurements: Vec::new(),
            outlier_count: previous_report.outlier_count,
            residual_rms_m: previous_report.residual_rms_m,
            residual_max_m: previous_report.residual_max_m,
            normalized_residual_rms: previous_report.normalized_residual_rms,
            innovation: None,
            prediction_count: state.prediction_count,
            last_prediction_distance_m: None,
            last_prediction_interval_s: Some(interval_s),
            pose: (validity == EstimateValidity::Valid).then_some(state.pose),
            uncertainty: (validity == EstimateValidity::Valid).then_some(state.uncertainty),
            validity,
            validity_detail,
        };
        self.last_state = Some(state);
        self.last_report = Some(report.clone());
        report
    }

    fn store_invalid(&mut self, report: PoseEstimate) -> PoseEstimate {
        debug_assert!(!report.is_valid());
        self.last_report = Some(report.clone());
        report
    }
}

#[derive(Clone, Debug)]
struct WorkingMeasurement {
    measurement: FeatureMeasurement,
    axial_coordinate_m: f64,
}

#[derive(Clone, Copy, Debug)]
struct WeightedFit {
    center_world_m: [f64; 3],
    axis_world_unit: [f64; 3],
    raw_axis_norm: f64,
    weighted_mean_variance_m2: [f64; 3],
    weighted_axial_mean_m: [f64; 3],
    slope_variance: [f64; 3],
}

#[derive(Clone, Copy, Debug)]
struct ResidualSummary {
    rms_m: f64,
    max_m: f64,
    normalized_rms: f64,
    normalized_sum_squares: f64,
}

#[derive(Clone, Copy, Debug)]
struct WorstResidual {
    active_position: usize,
    distance_m: f64,
    normalized: f64,
}

#[derive(Clone, Copy, Debug, Default)]
struct ObservationCounts {
    head_count: usize,
    view_count: usize,
    feature_count: usize,
    calibrated_ray_count: u64,
    minimum_rays: Option<u32>,
    maximum_latency_ticks: Option<u64>,
    oldest_available_tick: Option<u64>,
    newest_available_tick: Option<u64>,
}

fn validate_config(config: &EstimatorConfig) -> Result<(), EstimatorConfigError> {
    positive_finite("tick_period_s", config.tick_period_s)?;
    nonzero(
        "minimum_measurement_count",
        config.minimum_measurement_count,
    )?;
    nonzero("minimum_feature_count", config.minimum_feature_count)?;
    if config.minimum_axial_station_count < 2 {
        return Err(config_error(
            "minimum_axial_station_count",
            "must be at least two for axis observability",
        ));
    }
    nonzero("minimum_head_count", config.minimum_head_count)?;
    if config.minimum_calibrated_ray_count == 0 {
        return Err(config_error(
            "minimum_calibrated_ray_count",
            "must be non-zero",
        ));
    }
    if !config.minimum_confidence.is_finite()
        || config.minimum_confidence <= 0.0
        || config.minimum_confidence > 1.0
    {
        return Err(config_error(
            "minimum_confidence",
            "must be finite and in (0, 1]",
        ));
    }
    positive_finite(
        "minimum_axial_lever_arm_m",
        config.minimum_axial_lever_arm_m,
    )?;
    positive_finite(
        "minimum_independent_variance_m2",
        config.minimum_independent_variance_m2,
    )?;
    positive_finite("outlier_sigma_threshold", config.outlier_sigma_threshold)?;
    positive_finite(
        "outlier_absolute_threshold_m",
        config.outlier_absolute_threshold_m,
    )?;
    if !config.maximum_outlier_fraction.is_finite()
        || !(0.0..1.0).contains(&config.maximum_outlier_fraction)
    {
        return Err(config_error(
            "maximum_outlier_fraction",
            "must be finite and in [0, 1)",
        ));
    }
    positive_finite("maximum_residual_rms_m", config.maximum_residual_rms_m)?;
    positive_finite(
        "maximum_normalized_residual_rms",
        config.maximum_normalized_residual_rms,
    )?;
    if !config.maximum_axis_scale_error.is_finite()
        || !(0.0..1.0).contains(&config.maximum_axis_scale_error)
    {
        return Err(config_error(
            "maximum_axis_scale_error",
            "must be finite and in [0, 1)",
        ));
    }
    positive_array(
        "correlated_position_floor_m",
        config.correlated_position_floor_m,
        true,
    )?;
    nonnegative_finite(
        "correlated_axis_floor_rad",
        config.correlated_axis_floor_rad,
    )?;
    positive_finite("maximum_position_sigma_m", config.maximum_position_sigma_m)?;
    positive_finite("maximum_axis_sigma_rad", config.maximum_axis_sigma_rad)?;
    positive_finite(
        "maximum_innovation_translation_m",
        config.maximum_innovation_translation_m,
    )?;
    positive_finite(
        "maximum_innovation_axis_rad",
        config.maximum_innovation_axis_rad,
    )?;
    positive_finite(
        "maximum_normalized_innovation",
        config.maximum_normalized_innovation,
    )?;
    positive_array(
        "hold_process_sigma_m_per_sqrt_s",
        config.hold_process_sigma_m_per_sqrt_s,
        true,
    )?;
    nonnegative_finite(
        "hold_axis_sigma_rad_per_sqrt_s",
        config.hold_axis_sigma_rad_per_sqrt_s,
    )?;
    nonnegative_finite(
        "commanded_translation_fractional_sigma",
        config.commanded_translation_fractional_sigma,
    )?;
    Ok(())
}

fn config_error(field: &'static str, reason: &'static str) -> EstimatorConfigError {
    EstimatorConfigError { field, reason }
}

fn positive_finite(field: &'static str, value: f64) -> Result<(), EstimatorConfigError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(config_error(field, "must be finite and positive"))
    }
}

fn nonnegative_finite(field: &'static str, value: f64) -> Result<(), EstimatorConfigError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(config_error(field, "must be finite and non-negative"))
    }
}

fn positive_array(
    field: &'static str,
    value: [f64; 3],
    allow_zero: bool,
) -> Result<(), EstimatorConfigError> {
    let valid = value.iter().all(|component| {
        component.is_finite() && (*component > 0.0 || allow_zero && *component == 0.0)
    });
    if valid {
        Ok(())
    } else {
        Err(config_error(
            field,
            if allow_zero {
                "components must be finite and non-negative"
            } else {
                "components must be finite and positive"
            },
        ))
    }
}

fn nonzero(field: &'static str, value: usize) -> Result<(), EstimatorConfigError> {
    if value > 0 {
        Ok(())
    } else {
        Err(config_error(field, "must be non-zero"))
    }
}

fn sample_gate_failure(
    config: &EstimatorConfig,
    working: &[WorkingMeasurement],
) -> Option<(EstimateValidity, &'static str)> {
    if working.len() < config.minimum_measurement_count {
        return Some((
            EstimateValidity::InsufficientMeasurements,
            "too few accepted feature measurements",
        ));
    }
    let features: BTreeSet<u32> = working
        .iter()
        .map(|item| item.measurement.feature_id)
        .collect();
    if features.len() < config.minimum_feature_count {
        return Some((
            EstimateValidity::InsufficientFeatures,
            "too few distinct calibrated features",
        ));
    }
    let stations = distinct_station_count(working.iter().map(|item| item.axial_coordinate_m));
    if stations < config.minimum_axial_station_count {
        return Some((
            EstimateValidity::InsufficientAxialStations,
            "too few distinct axial feature stations",
        ));
    }
    let heads: BTreeSet<u32> = working
        .iter()
        .map(|item| item.measurement.head_id)
        .collect();
    if heads.len() < config.minimum_head_count {
        return Some((
            EstimateValidity::InsufficientHeads,
            "too few independent calibrated optical heads",
        ));
    }
    let rays = working.iter().fold(0u64, |total, item| {
        total.saturating_add(u64::from(item.measurement.calibrated_ray_count))
    });
    if rays < config.minimum_calibrated_ray_count {
        return Some((
            EstimateValidity::InsufficientCalibratedRays,
            "too few calibrated rays in the accepted burst",
        ));
    }
    let minimum_s = working
        .iter()
        .map(|item| item.axial_coordinate_m)
        .min_by(f64::total_cmp)
        .expect("working is non-empty");
    let maximum_s = working
        .iter()
        .map(|item| item.axial_coordinate_m)
        .max_by(f64::total_cmp)
        .expect("working is non-empty");
    let lever_arm_m = maximum_s - minimum_s;
    if !lever_arm_m.is_finite() || lever_arm_m < config.minimum_axial_lever_arm_m {
        return Some((
            EstimateValidity::InsufficientAxialLeverArm,
            "accepted feature stations do not provide the required axial lever arm",
        ));
    }
    None
}

fn weighted_fit(
    working: &[WorkingMeasurement],
    active: &[usize],
    config: &EstimatorConfig,
) -> Result<WeightedFit, EstimateValidity> {
    let mut center = [0.0; 3];
    let mut raw_axis = [0.0; 3];
    let mut weighted_mean_variance = [0.0; 3];
    let mut weighted_axial_mean = [0.0; 3];
    let mut slope_variance = [0.0; 3];

    for dimension in 0..3 {
        let mut sum_w = 0.0;
        let mut sum_ws = 0.0;
        let mut sum_wss = 0.0;
        let mut sum_wy = 0.0;
        let mut sum_wsy = 0.0;
        for index in active {
            let item = &working[*index];
            let variance = item.measurement.covariance_diagonal_m2[dimension]
                .max(config.minimum_independent_variance_m2);
            let weight = item.measurement.confidence / variance;
            let s = item.axial_coordinate_m;
            let y = item.measurement.measured_point_world_m[dimension];
            sum_w += weight;
            sum_ws += weight * s;
            sum_wss += weight * s * s;
            sum_wy += weight * y;
            sum_wsy += weight * s * y;
        }
        let determinant = sum_w * sum_wss - sum_ws * sum_ws;
        let scale = (sum_w * sum_wss).abs().max(1.0);
        if !determinant.is_finite() || determinant <= f64::EPSILON * scale {
            return Err(EstimateValidity::RankDeficient);
        }
        center[dimension] = (sum_wss * sum_wy - sum_ws * sum_wsy) / determinant;
        raw_axis[dimension] = (sum_w * sum_wsy - sum_ws * sum_wy) / determinant;
        // The refitted center is formed from the weighted point mean minus
        // the normalized axis times this weighted axial mean. In a linear WLS
        // fit the weighted point mean is uncorrelated with the raw slope, so
        // these are the sufficient covariance terms for the constrained refit.
        weighted_mean_variance[dimension] = 1.0 / sum_w;
        weighted_axial_mean[dimension] = sum_ws / sum_w;
        slope_variance[dimension] = sum_w / determinant;
    }

    let raw_axis_norm = norm(raw_axis);
    if !raw_axis_norm.is_finite() || raw_axis_norm <= 1.0e-12 {
        return Err(EstimateValidity::RankDeficient);
    }
    let axis = scale(raw_axis, 1.0 / raw_axis_norm);

    // Once the direction is constrained to unit length, refit the intercept at
    // local axial coordinate zero.  This is the explicit correction for a
    // feature model whose weighted axial mean is not zero.
    for dimension in 0..3 {
        let mut sum_w = 0.0;
        let mut sum_adjusted = 0.0;
        for index in active {
            let item = &working[*index];
            let variance = item.measurement.covariance_diagonal_m2[dimension]
                .max(config.minimum_independent_variance_m2);
            let weight = item.measurement.confidence / variance;
            sum_w += weight;
            sum_adjusted += weight
                * (item.measurement.measured_point_world_m[dimension]
                    - axis[dimension] * item.axial_coordinate_m);
        }
        center[dimension] = sum_adjusted / sum_w;
    }

    Ok(WeightedFit {
        center_world_m: center,
        axis_world_unit: axis,
        raw_axis_norm,
        weighted_mean_variance_m2: weighted_mean_variance,
        weighted_axial_mean_m: weighted_axial_mean,
        slope_variance,
    })
}

fn residual_summary(
    working: &[WorkingMeasurement],
    active: &[usize],
    fit: &WeightedFit,
    config: &EstimatorConfig,
) -> ResidualSummary {
    let mut distance_squared_sum = 0.0;
    let mut maximum_distance: f64 = 0.0;
    let mut normalized_sum_squares = 0.0;
    for index in active {
        let item = &working[*index];
        let expected = add(
            fit.center_world_m,
            scale(fit.axis_world_unit, item.axial_coordinate_m),
        );
        let residual = subtract(item.measurement.measured_point_world_m, expected);
        let distance_squared = dot(residual, residual);
        distance_squared_sum += distance_squared;
        maximum_distance = maximum_distance.max(distance_squared.sqrt());
        for (dimension, residual_component) in residual.iter().enumerate() {
            let variance = item.measurement.covariance_diagonal_m2[dimension]
                .max(config.minimum_independent_variance_m2)
                / item.measurement.confidence;
            normalized_sum_squares += residual_component * residual_component / variance;
        }
    }
    ResidualSummary {
        rms_m: (distance_squared_sum / active.len() as f64).sqrt(),
        max_m: maximum_distance,
        normalized_rms: (normalized_sum_squares / (active.len() * 3) as f64).sqrt(),
        normalized_sum_squares,
    }
}

fn worst_residual(
    working: &[WorkingMeasurement],
    active: &[usize],
    fit: &WeightedFit,
    config: &EstimatorConfig,
) -> WorstResidual {
    let mut worst = WorstResidual {
        active_position: 0,
        distance_m: -1.0,
        normalized: -1.0,
    };
    for (active_position, index) in active.iter().enumerate() {
        let item = &working[*index];
        let expected = add(
            fit.center_world_m,
            scale(fit.axis_world_unit, item.axial_coordinate_m),
        );
        let residual = subtract(item.measurement.measured_point_world_m, expected);
        let distance_m = norm(residual);
        let normalized = (residual
            .iter()
            .enumerate()
            .map(|(dimension, value)| {
                let variance = item.measurement.covariance_diagonal_m2[dimension]
                    .max(config.minimum_independent_variance_m2)
                    / item.measurement.confidence;
                value * value / variance
            })
            .sum::<f64>()
            / 3.0)
            .sqrt();
        // Input order is canonical.  Strict comparison means equal residuals
        // retain the earlier canonical key.
        if normalized > worst.normalized
            || (normalized == worst.normalized && distance_m > worst.distance_m)
        {
            worst = WorstResidual {
                active_position,
                distance_m,
                normalized,
            };
        }
    }
    worst
}

fn fit_uncertainty(
    fit: &WeightedFit,
    residuals: &ResidualSummary,
    measurement_count: usize,
    config: &EstimatorConfig,
) -> ReducedPoseUncertainty {
    let degrees_of_freedom = reduced_pose_residual_degrees_of_freedom(measurement_count) as f64;
    let residual_scale = (residuals.normalized_sum_squares / degrees_of_freedom).max(1.0);
    let tangent_basis = tangent_basis(fit.axis_world_unit);

    // Propagate the raw three-vector slope through u = a / |a|.  The Jacobian
    // is (I - uu') / |a|, which introduces cross-axis covariance even though
    // the input point covariance is diagonal.
    let mut axis_covariance_world_rad2 = [[0.0; 3]; 3];
    for (row, covariance_row) in axis_covariance_world_rad2.iter_mut().enumerate() {
        for (column, covariance_entry) in covariance_row.iter_mut().enumerate() {
            for (source, slope_variance) in fit.slope_variance.iter().copied().enumerate() {
                let row_projection = if row == source { 1.0 } else { 0.0 }
                    - fit.axis_world_unit[row] * fit.axis_world_unit[source];
                let column_projection = if column == source { 1.0 } else { 0.0 }
                    - fit.axis_world_unit[column] * fit.axis_world_unit[source];
                *covariance_entry +=
                    row_projection * column_projection * slope_variance * residual_scale
                        / fit.raw_axis_norm.powi(2);
            }
        }
    }

    // State ordering is center xyz followed by the two tangent-axis angles.
    // The refitted center c = mean(y) - mean(s) u is correlated with u whenever
    // the visible axial feature coordinates have a non-zero weighted mean.
    let mut covariance = [[0.0; AXISYMMETRIC_POSE_DOF]; AXISYMMETRIC_POSE_DOF];
    for row in 0..3 {
        for column in 0..3 {
            covariance[row][column] = if row == column {
                fit.weighted_mean_variance_m2[row] * residual_scale
            } else {
                0.0
            } + fit.weighted_axial_mean_m[row]
                * fit.weighted_axial_mean_m[column]
                * axis_covariance_world_rad2[row][column];
        }
    }
    for center_dimension in 0..3 {
        for tangent in 0..2 {
            let center_axis_covariance = -fit.weighted_axial_mean_m[center_dimension]
                * dot(
                    axis_covariance_world_rad2[center_dimension],
                    tangent_basis[tangent],
                );
            covariance[center_dimension][3 + tangent] = center_axis_covariance;
            covariance[3 + tangent][center_dimension] = center_axis_covariance;
        }
    }
    for tangent_row in 0..2 {
        for tangent_column in 0..2 {
            let projected = (0..3)
                .map(|row| {
                    (0..3)
                        .map(|column| {
                            tangent_basis[tangent_row][row]
                                * axis_covariance_world_rad2[row][column]
                                * tangent_basis[tangent_column][column]
                        })
                        .sum::<f64>()
                })
                .sum::<f64>();
            covariance[3 + tangent_row][3 + tangent_column] = projected;
        }
    }

    // The public reduced type intentionally has no cross-covariance fields.
    // Diagonalize conservatively in standardized coordinates: each marginal is
    // multiplied by its absolute correlation-row sum. The resulting diagonal
    // matrix PSD-dominates the complete independent 5x5 covariance, so omitted
    // center/axis and tangent cross terms cannot make the report overconfident.
    let conservative_diagonal = conservative_covariance_diagonal(covariance);
    let center_variance_m2 = std::array::from_fn(|dimension| {
        conservative_diagonal[dimension] + config.correlated_position_floor_m[dimension].powi(2)
    });
    let center_sigma_m = center_variance_m2.map(f64::sqrt);
    let axis_tangent_variance_rad2 = std::array::from_fn(|tangent| {
        conservative_diagonal[3 + tangent] + config.correlated_axis_floor_rad.powi(2)
    });
    ReducedPoseUncertainty {
        center_variance_m2,
        center_sigma_m,
        axis_tangent_basis_world: tangent_basis,
        axis_tangent_variance_rad2,
        axis_tangent_sigma_rad: axis_tangent_variance_rad2.map(f64::sqrt),
        correlated_position_floor_m: config.correlated_position_floor_m,
        correlated_axis_floor_rad: config.correlated_axis_floor_rad,
    }
}

fn reduced_pose_residual_degrees_of_freedom(measurement_count: usize) -> usize {
    measurement_count
        .saturating_mul(3)
        .saturating_sub(AXISYMMETRIC_POSE_DOF)
        .max(1)
}

fn conservative_covariance_diagonal<const N: usize>(covariance: [[f64; N]; N]) -> [f64; N] {
    std::array::from_fn(|row| {
        let row_variance = covariance[row][row].max(0.0);
        if row_variance == 0.0 {
            return if covariance[row]
                .iter()
                .any(|value| value.abs() > f64::EPSILON)
            {
                f64::INFINITY
            } else {
                0.0
            };
        }
        let mut absolute_correlation_row_sum = 1.0;
        for (column, column_covariance) in covariance.iter().enumerate() {
            if column == row {
                continue;
            }
            let column_variance = column_covariance[column].max(0.0);
            let cross = 0.5 * (covariance[row][column] + column_covariance[row]);
            if column_variance == 0.0 {
                if cross.abs() > f64::EPSILON {
                    return f64::INFINITY;
                }
            } else {
                absolute_correlation_row_sum +=
                    cross.abs() / (row_variance * column_variance).sqrt();
            }
        }
        row_variance * absolute_correlation_row_sum
    })
}

fn add_process_noise(
    uncertainty: &mut ReducedPoseUncertainty,
    interval_s: f64,
    commanded_distance_m: f64,
    config: &EstimatorConfig,
) {
    let travel_sigma = config.commanded_translation_fractional_sigma * commanded_distance_m;
    for dimension in 0..3 {
        let hold_variance =
            config.hold_process_sigma_m_per_sqrt_s[dimension].powi(2) * interval_s.max(0.0);
        uncertainty.center_variance_m2[dimension] += hold_variance + travel_sigma.powi(2);
        uncertainty.center_variance_m2[dimension] = uncertainty.center_variance_m2[dimension]
            .max(config.correlated_position_floor_m[dimension].powi(2));
        uncertainty.center_sigma_m[dimension] = uncertainty.center_variance_m2[dimension].sqrt();
    }
    let axis_variance = config.hold_axis_sigma_rad_per_sqrt_s.powi(2) * interval_s.max(0.0);
    for tangent in 0..2 {
        uncertainty.axis_tangent_variance_rad2[tangent] += axis_variance;
        uncertainty.axis_tangent_variance_rad2[tangent] = uncertainty.axis_tangent_variance_rad2
            [tangent]
            .max(config.correlated_axis_floor_rad.powi(2));
        uncertainty.axis_tangent_sigma_rad[tangent] =
            uncertainty.axis_tangent_variance_rad2[tangent].sqrt();
    }
}

fn uncertainty_exceeds_limits(
    uncertainty: &ReducedPoseUncertainty,
    config: &EstimatorConfig,
) -> bool {
    uncertainty
        .center_sigma_m
        .iter()
        .any(|sigma| !sigma.is_finite() || *sigma > config.maximum_position_sigma_m)
        || uncertainty
            .axis_tangent_sigma_rad
            .iter()
            .any(|sigma| !sigma.is_finite() || *sigma > config.maximum_axis_sigma_rad)
}

fn pose_innovation(
    prior_pose: AxisymmetricPose5d,
    prior_uncertainty: ReducedPoseUncertainty,
    observed_pose: AxisymmetricPose5d,
    observed_uncertainty: ReducedPoseUncertainty,
    shared_calibration_mode: bool,
) -> PoseInnovation {
    let translation = subtract(observed_pose.center_world_m, prior_pose.center_world_m);
    let axis_dot = dot(prior_pose.axis_world_unit, observed_pose.axis_world_unit).clamp(-1.0, 1.0);
    let axis_cross_norm = norm(cross(
        prior_pose.axis_world_unit,
        observed_pose.axis_world_unit,
    ));
    let axis_angle_rad = axis_cross_norm.atan2(axis_dot);
    let mut normalized_squared = 0.0;
    for (dimension, translation_component) in translation.iter().enumerate() {
        let variance = if shared_calibration_mode {
            shared_floor_difference_variance(
                prior_uncertainty.center_variance_m2[dimension],
                observed_uncertainty.center_variance_m2[dimension],
                prior_uncertainty.correlated_position_floor_m[dimension],
                observed_uncertainty.correlated_position_floor_m[dimension],
            )
        } else {
            prior_uncertainty.center_variance_m2[dimension]
                + observed_uncertainty.center_variance_m2[dimension]
        };
        normalized_squared += translation_component.powi(2) / variance.max(f64::MIN_POSITIVE);
    }
    let mut angular_variance = prior_uncertainty
        .axis_tangent_variance_rad2
        .iter()
        .copied()
        .fold(0.0, f64::max)
        + observed_uncertainty
            .axis_tangent_variance_rad2
            .iter()
            .copied()
            .fold(0.0, f64::max);
    if shared_calibration_mode {
        angular_variance -= 2.0
            * prior_uncertainty
                .correlated_axis_floor_rad
                .min(observed_uncertainty.correlated_axis_floor_rad)
                .powi(2);
    }
    normalized_squared += axis_angle_rad.powi(2) / angular_variance.max(f64::MIN_POSITIVE);
    PoseInnovation {
        translation_world_m: translation,
        translation_norm_m: norm(translation),
        axis_angle_rad,
        normalized_norm: normalized_squared.sqrt(),
    }
}

fn shared_floor_difference_variance(
    prior_variance: f64,
    observed_variance: f64,
    prior_shared_floor_sigma: f64,
    observed_shared_floor_sigma: f64,
) -> f64 {
    let shared_variance = prior_shared_floor_sigma
        .min(observed_shared_floor_sigma)
        .powi(2);
    (prior_variance + observed_variance - 2.0 * shared_variance).max(0.0)
}

fn observation_counts(working: &[WorkingMeasurement]) -> ObservationCounts {
    let mut heads = BTreeSet::new();
    let mut views = BTreeSet::new();
    let mut features = BTreeSet::new();
    let mut rays = 0u64;
    let mut minimum_rays = None::<u32>;
    let mut maximum_latency = None::<u64>;
    let mut oldest_available = None::<u64>;
    let mut newest_available = None::<u64>;
    for item in working {
        let measurement = &item.measurement;
        heads.insert(measurement.head_id);
        views.insert((measurement.head_id, measurement.capture_tick));
        features.insert(measurement.feature_id);
        rays = rays.saturating_add(u64::from(measurement.calibrated_ray_count));
        minimum_rays = Some(
            minimum_rays.map_or(measurement.calibrated_ray_count, |current| {
                current.min(measurement.calibrated_ray_count)
            }),
        );
        let latency = measurement.available_tick - measurement.capture_tick;
        maximum_latency = Some(maximum_latency.map_or(latency, |current| current.max(latency)));
        oldest_available = Some(
            oldest_available.map_or(measurement.available_tick, |current| {
                current.min(measurement.available_tick)
            }),
        );
        newest_available = Some(
            newest_available.map_or(measurement.available_tick, |current| {
                current.max(measurement.available_tick)
            }),
        );
    }
    ObservationCounts {
        head_count: heads.len(),
        view_count: views.len(),
        feature_count: features.len(),
        calibrated_ray_count: rays,
        minimum_rays,
        maximum_latency_ticks: maximum_latency,
        oldest_available_tick: oldest_available,
        newest_available_tick: newest_available,
    }
}

fn invalid_report(
    object_id: u32,
    controller_tick: u64,
    validity: EstimateValidity,
    detail: &str,
    mut rejected: Vec<RejectedMeasurement>,
) -> PoseEstimate {
    rejected.sort_by(rejection_order);
    PoseEstimate {
        object_id,
        controller_tick,
        state_tick: controller_tick,
        oldest_capture_tick: None,
        newest_capture_tick: None,
        oldest_available_tick: None,
        newest_available_tick: None,
        measurement_age_ticks: None,
        measurement_age_s: None,
        maximum_capture_to_available_latency_ticks: None,
        observation_source_count: 0,
        head_count: 0,
        view_count: 0,
        calibrated_ray_count: 0,
        minimum_calibrated_rays_per_measurement: None,
        accepted_measurement_count: 0,
        accepted_feature_count: 0,
        rejected_measurement_count: rejected.len(),
        rejected_measurements: rejected,
        outlier_count: 0,
        residual_rms_m: None,
        residual_max_m: None,
        normalized_residual_rms: None,
        innovation: None,
        prediction_count: 0,
        last_prediction_distance_m: None,
        last_prediction_interval_s: None,
        pose: None,
        uncertainty: None,
        validity,
        validity_detail: detail.to_owned(),
    }
}

fn invalid_report_with_working(
    object_id: u32,
    controller_tick: u64,
    validity: EstimateValidity,
    detail: &str,
    working: &[WorkingMeasurement],
    mut rejected: Vec<RejectedMeasurement>,
    tick_period_s: f64,
) -> PoseEstimate {
    rejected.sort_by(rejection_order);
    let counts = observation_counts(working);
    let oldest = working
        .iter()
        .map(|item| item.measurement.capture_tick)
        .min();
    let newest = working
        .iter()
        .map(|item| item.measurement.capture_tick)
        .max();
    let age = oldest.map(|tick| controller_tick.saturating_sub(tick));
    PoseEstimate {
        object_id,
        controller_tick,
        state_tick: controller_tick,
        oldest_capture_tick: oldest,
        newest_capture_tick: newest,
        oldest_available_tick: counts.oldest_available_tick,
        newest_available_tick: counts.newest_available_tick,
        measurement_age_ticks: age,
        measurement_age_s: age.map(|ticks| ticks as f64 * tick_period_s),
        maximum_capture_to_available_latency_ticks: counts.maximum_latency_ticks,
        observation_source_count: counts.head_count,
        head_count: counts.head_count,
        view_count: counts.view_count,
        calibrated_ray_count: counts.calibrated_ray_count,
        minimum_calibrated_rays_per_measurement: counts.minimum_rays,
        accepted_measurement_count: working.len(),
        accepted_feature_count: counts.feature_count,
        rejected_measurement_count: rejected.len(),
        rejected_measurements: rejected,
        outlier_count: 0,
        residual_rms_m: None,
        residual_max_m: None,
        normalized_residual_rms: None,
        innovation: None,
        prediction_count: 0,
        last_prediction_distance_m: None,
        last_prediction_interval_s: None,
        pose: None,
        uncertainty: None,
        validity,
        validity_detail: detail.to_owned(),
    }
}

fn measurement_order(left: &FeatureMeasurement, right: &FeatureMeasurement) -> std::cmp::Ordering {
    (
        left.object_id,
        left.feature_id,
        left.head_id,
        left.capture_tick,
        left.available_tick,
    )
        .cmp(&(
            right.object_id,
            right.feature_id,
            right.head_id,
            right.capture_tick,
            right.available_tick,
        ))
        .then_with(|| float_array_order(left.measured_point_world_m, right.measured_point_world_m))
        .then_with(|| float_array_order(left.covariance_diagonal_m2, right.covariance_diagonal_m2))
        .then_with(|| left.confidence.total_cmp(&right.confidence))
        .then_with(|| left.calibrated_ray_count.cmp(&right.calibrated_ray_count))
}

/// Delivery time is metadata, not sensor-exposure identity. Accepting the
/// same exposure again under a changed availability tick would double-count
/// one optical sample and make covariance depend on transport retries.
fn same_exposure_identity(left: &FeatureMeasurement, right: &FeatureMeasurement) -> bool {
    left.object_id == right.object_id
        && left.feature_id == right.feature_id
        && left.head_id == right.head_id
        && left.capture_tick == right.capture_tick
}

fn rejection_order(left: &RejectedMeasurement, right: &RejectedMeasurement) -> std::cmp::Ordering {
    left.key
        .cmp(&right.key)
        .then_with(|| (left.reason as u8).cmp(&(right.reason as u8)))
}

fn float_array_order(left: [f64; 3], right: [f64; 3]) -> std::cmp::Ordering {
    left[0]
        .total_cmp(&right[0])
        .then_with(|| left[1].total_cmp(&right[1]))
        .then_with(|| left[2].total_cmp(&right[2]))
}

fn distinct_station_count(values: impl IntoIterator<Item = f64>) -> usize {
    let mut stations: Vec<f64> = values.into_iter().map(canonical_zero).collect();
    stations.sort_by(f64::total_cmp);
    stations.dedup_by(|left, right| *left == *right);
    stations.len()
}

fn tangent_basis(axis: [f64; 3]) -> [[f64; 3]; 2] {
    let absolute = axis.map(f64::abs);
    let helper = if absolute[0] <= absolute[1] && absolute[0] <= absolute[2] {
        [1.0, 0.0, 0.0]
    } else if absolute[1] <= absolute[2] {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    let first = normalize(cross(axis, helper));
    let second = normalize(cross(axis, first));
    [first, second]
}

fn array_is_finite(value: [f64; 3]) -> bool {
    value.iter().all(|component| component.is_finite())
}

fn canonical_zero(value: f64) -> f64 {
    if value == 0.0 {
        0.0
    } else {
        value
    }
}

fn add(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|dimension| left[dimension] + right[dimension])
}

fn subtract(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|dimension| left[dimension] - right[dimension])
}

fn scale(value: [f64; 3], factor: f64) -> [f64; 3] {
    value.map(|component| component * factor)
}

fn dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    (0..3)
        .map(|dimension| left[dimension] * right[dimension])
        .sum()
}

fn cross(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn norm(value: [f64; 3]) -> f64 {
    dot(value, value).sqrt()
}

fn normalize(value: [f64; 3]) -> [f64; 3] {
    scale(value, 1.0 / norm(value))
}

#[cfg(test)]
mod tests {
    use super::*;

    const OBJECT_ID: u32 = 17;
    const CENTER: [f64; 3] = [0.012, -0.003, 0.021];
    const AXIS: [f64; 3] = [0.0, 0.0, 1.0];

    fn four_feature_model() -> Vec<KnownAxialFeature> {
        vec![
            KnownAxialFeature {
                feature_id: 10,
                axial_coordinate_m: -0.20e-3,
            },
            KnownAxialFeature {
                feature_id: 11,
                axial_coordinate_m: -0.20e-3,
            },
            KnownAxialFeature {
                feature_id: 20,
                axial_coordinate_m: 0.20e-3,
            },
            KnownAxialFeature {
                feature_id: 21,
                axial_coordinate_m: 0.20e-3,
            },
        ]
    }

    fn config() -> EstimatorConfig {
        EstimatorConfig {
            tick_period_s: 0.001,
            maximum_measurement_age_ticks: 10,
            maximum_burst_span_ticks: 1,
            minimum_calibrated_ray_count: 4,
            correlated_position_floor_m: [2.0e-6, 3.0e-6, 4.0e-6],
            correlated_axis_floor_rad: 2.0e-3,
            ..EstimatorConfig::default()
        }
    }

    fn point_at(axial_coordinate_m: f64) -> [f64; 3] {
        add(CENTER, scale(AXIS, axial_coordinate_m))
    }

    fn measurement(feature_id: u32, axial_coordinate_m: f64, head_id: u32) -> FeatureMeasurement {
        FeatureMeasurement {
            object_id: OBJECT_ID,
            feature_id,
            head_id,
            calibrated_ray_count: 3,
            capture_tick: 100,
            available_tick: 102,
            measured_point_world_m: point_at(axial_coordinate_m),
            covariance_diagonal_m2: [2.0e-6_f64.powi(2); 3],
            confidence: 1.0,
        }
    }

    fn nominal_measurements() -> Vec<FeatureMeasurement> {
        vec![
            measurement(10, -0.20e-3, 1),
            measurement(11, -0.20e-3, 2),
            measurement(20, 0.20e-3, 1),
            measurement(21, 0.20e-3, 2),
        ]
    }

    fn assert_close(left: f64, right: f64, tolerance: f64) {
        assert!(
            (left - right).abs() <= tolerance,
            "{left:.12e} != {right:.12e} within {tolerance:.3e}"
        );
    }

    #[test]
    fn nominal_axisymmetric_pose_is_valid_and_roll_is_unobservable() {
        let mut estimator =
            ObservedPoseEstimator::new(config(), OBJECT_ID, four_feature_model()).unwrap();
        let estimate = estimator.update(103, &nominal_measurements());

        assert_eq!(estimate.validity, EstimateValidity::Valid);
        let pose = estimate.usable_pose().unwrap();
        for dimension in 0..3 {
            assert_close(pose.center_world_m[dimension], CENTER[dimension], 1.0e-12);
            assert_close(pose.axis_world_unit[dimension], AXIS[dimension], 1.0e-12);
        }
        assert!(!pose.roll_observable);
        assert_eq!(estimate.accepted_feature_count, 4);
        assert_eq!(estimate.head_count, 2);
        assert_eq!(estimate.observation_source_count, 2);
        assert_eq!(estimate.calibrated_ray_count, 12);
        assert_eq!(estimate.measurement_age_ticks, Some(3));
        assert_eq!(estimate.maximum_capture_to_available_latency_ticks, Some(2));
    }

    #[test]
    fn constructor_rejects_feature_model_below_configured_axial_span() {
        let mut cfg = config();
        cfg.minimum_axial_lever_arm_m = 0.400e-3 + 1.0e-12;

        let error = ObservedPoseEstimator::new(cfg, OBJECT_ID, four_feature_model())
            .expect_err("0.400 mm feature span must not satisfy a larger configured minimum");

        assert_eq!(error.field, "feature_model");
        assert_eq!(
            error.reason,
            "axial station span is below minimum_axial_lever_arm_m"
        );
    }

    #[test]
    fn input_order_does_not_change_any_report_field() {
        let measurements = nominal_measurements();
        let mut reversed = measurements.clone();
        reversed.reverse();
        let mut first =
            ObservedPoseEstimator::new(config(), OBJECT_ID, four_feature_model()).unwrap();
        let mut second =
            ObservedPoseEstimator::new(config(), OBJECT_ID, four_feature_model()).unwrap();

        let first_report = first.update(103, &measurements);
        let second_report = second.update(103, &reversed);
        assert_eq!(first_report, second_report);
        assert_eq!(
            serde_json::to_string(&first_report).unwrap(),
            serde_json::to_string(&second_report).unwrap()
        );
    }

    #[test]
    fn invalid_confidence_cannot_shrink_measurement_uncertainty() {
        for confidence in [-1.0, 0.0, 1.01, 100.0] {
            let mut estimator =
                ObservedPoseEstimator::new(config(), OBJECT_ID, four_feature_model()).unwrap();
            let mut observations = nominal_measurements();
            for observation in &mut observations {
                observation.confidence = confidence;
            }
            let estimate = estimator.update(103, &observations);
            assert_eq!(
                estimate.validity,
                EstimateValidity::InvalidMeasurementConfidence
            );
            assert!(estimate.usable_pose().is_none());
            assert!(estimate.uncertainty.is_none());
        }
    }

    #[test]
    fn replayed_exposures_cannot_erase_prediction_or_restore_validity() {
        let mut estimator =
            ObservedPoseEstimator::new(config(), OBJECT_ID, four_feature_model()).unwrap();
        assert!(estimator.update(103, &nominal_measurements()).is_valid());
        let predicted = estimator.predict_commanded_translation(105, [1.0e-6, 0.0, 0.0]);
        assert!(predicted.is_valid());
        let mut replay = nominal_measurements();
        for observation in &mut replay {
            observation.available_tick = 106;
        }
        for tick in [106, 107] {
            let rejected = estimator.update(tick, &replay);
            assert_eq!(
                rejected.validity,
                EstimateValidity::ObservationTimestampRegression
            );
            assert!(rejected.usable_pose().is_none());
            let state = estimator.last_state.as_ref().unwrap();
            assert_eq!(Some(state.pose), predicted.pose);
            assert_eq!(Some(state.uncertainty), predicted.uncertainty);
            assert_eq!(state.state_tick, 105);
        }
        let mut fresh = nominal_measurements();
        for observation in &mut fresh {
            observation.capture_tick = 108;
            observation.available_tick = 110;
            observation.measured_point_world_m[0] += 1.0e-6;
        }
        assert!(estimator.update(110, &fresh).is_valid());
    }

    #[test]
    fn delayed_new_exposures_before_motion_cannot_replace_predicted_pose() {
        let mut estimator =
            ObservedPoseEstimator::new(config(), OBJECT_ID, four_feature_model()).unwrap();
        assert!(estimator.update(103, &nominal_measurements()).is_valid());
        assert!(estimator
            .predict_commanded_translation(105, [1.0e-6, 0.0, 0.0])
            .is_valid());
        let mut delayed = nominal_measurements();
        for observation in &mut delayed {
            observation.capture_tick = 104;
            observation.available_tick = 106;
        }
        let rejected = estimator.update(106, &delayed);
        assert_eq!(
            rejected.validity,
            EstimateValidity::ObservationTimestampRegression
        );
        assert!(rejected.usable_pose().is_none());
    }

    #[test]
    fn observation_update_cannot_move_controller_clock_backwards() {
        let mut estimator =
            ObservedPoseEstimator::new(config(), OBJECT_ID, four_feature_model()).unwrap();
        assert!(estimator.update(105, &nominal_measurements()).is_valid());
        let rejected = estimator.update(104, &nominal_measurements());
        assert_eq!(
            rejected.validity,
            EstimateValidity::ObservationTimestampRegression
        );
        assert!(rejected.usable_pose().is_none());
        assert_eq!(estimator.last_state.as_ref().unwrap().state_tick, 105);
    }

    #[test]
    fn duplicate_exposure_is_rejected_even_if_delivery_tick_changes() {
        let mut observations = nominal_measurements();
        let mut replayed = observations[0].clone();
        replayed.available_tick += 1;
        observations.push(replayed);
        let mut estimator =
            ObservedPoseEstimator::new(config(), OBJECT_ID, four_feature_model()).unwrap();

        let estimate = estimator.update(104, &observations);

        assert_eq!(estimate.validity, EstimateValidity::DuplicateMeasurement);
        assert!(estimate
            .validity_detail
            .contains("duplicate object/feature/head/capture exposure"));
        assert!(estimate.pose.is_none());
    }

    #[test]
    fn correlated_floor_never_averages_away() {
        let mut dense = Vec::new();
        for head in 0..50 {
            dense.extend([
                measurement(10, -0.20e-3, head),
                measurement(11, -0.20e-3, head),
                measurement(20, 0.20e-3, head),
                measurement(21, 0.20e-3, head),
            ]);
        }
        let cfg = config();
        let floor_m = cfg.correlated_position_floor_m;
        let axis_floor_rad = cfg.correlated_axis_floor_rad;
        let mut estimator =
            ObservedPoseEstimator::new(cfg, OBJECT_ID, four_feature_model()).unwrap();
        let estimate = estimator.update(103, &dense);
        let uncertainty = estimate.uncertainty.unwrap();

        for (dimension, floor) in floor_m.iter().enumerate() {
            assert!(uncertainty.center_sigma_m[dimension] >= *floor);
        }
        for sigma in uncertainty.axis_tangent_sigma_rad {
            assert!(sigma >= axis_floor_rad);
        }

        // Repeated burst updates replace the optical solution; they do not
        // information-fuse the same shared calibration floor as independent.
        let first_variance = uncertainty.center_variance_m2;
        for burst in 1..=20 {
            let capture_tick = 100 + burst;
            let mut repeated = nominal_measurements();
            for observation in &mut repeated {
                observation.capture_tick = capture_tick;
                observation.available_tick = capture_tick + 2;
            }
            let repeated_estimate = estimator.update(capture_tick + 3, &repeated);
            let repeated_uncertainty = repeated_estimate.uncertainty.unwrap();
            for (dimension, floor) in floor_m.iter().enumerate() {
                assert!(repeated_uncertainty.center_sigma_m[dimension] >= *floor);
            }
        }
        // A dense 50-head burst can have lower independent variance than a
        // four-sample burst, but neither result can cross the shared floor.
        assert!(first_variance
            .iter()
            .zip(floor_m)
            .all(|(variance, floor)| *variance >= floor * floor));
    }

    #[test]
    fn one_observed_axial_station_is_rejected_as_rank_deficient_geometry() {
        let mut cfg = config();
        cfg.minimum_measurement_count = 2;
        cfg.minimum_feature_count = 2;
        let mut estimator =
            ObservedPoseEstimator::new(cfg, OBJECT_ID, four_feature_model()).unwrap();
        let observations = vec![measurement(10, -0.20e-3, 1), measurement(11, -0.20e-3, 2)];

        let estimate = estimator.update(103, &observations);
        assert_eq!(
            estimate.validity,
            EstimateValidity::InsufficientAxialStations
        );
        assert!(estimate.pose.is_none());
    }

    #[test]
    fn isolated_outlier_is_rejected_deterministically() {
        let model: Vec<KnownAxialFeature> = (0..6)
            .map(|feature_id| KnownAxialFeature {
                feature_id,
                axial_coordinate_m: if feature_id < 3 { -0.20e-3 } else { 0.20e-3 },
            })
            .collect();
        let mut cfg = config();
        cfg.minimum_measurement_count = 5;
        cfg.minimum_feature_count = 5;
        let mut observations: Vec<FeatureMeasurement> = model
            .iter()
            .map(|feature| {
                measurement(
                    feature.feature_id,
                    feature.axial_coordinate_m,
                    1 + feature.feature_id,
                )
            })
            .collect();
        observations[5].measured_point_world_m[0] += 120.0e-6;
        let mut estimator = ObservedPoseEstimator::new(cfg, OBJECT_ID, model).unwrap();

        let estimate = estimator.update(103, &observations);
        assert_eq!(estimate.validity, EstimateValidity::Valid);
        assert_eq!(estimate.outlier_count, 1);
        assert_eq!(estimate.rejected_measurement_count, 1);
        assert_eq!(
            estimate.rejected_measurements[0].reason,
            MeasurementRejectionReason::StatisticalOutlier
        );
        assert_close(estimate.pose.unwrap().center_world_m[0], CENTER[0], 1.0e-12);
    }

    #[test]
    fn stale_burst_is_explicitly_invalid() {
        let mut estimator =
            ObservedPoseEstimator::new(config(), OBJECT_ID, four_feature_model()).unwrap();
        let estimate = estimator.update(200, &nominal_measurements());
        assert_eq!(estimate.validity, EstimateValidity::StaleMeasurements);
        assert_eq!(estimate.rejected_measurement_count, 4);
        assert!(estimate.pose.is_none());
    }

    #[test]
    fn repeated_features_from_one_head_count_as_one_source() {
        let observations: Vec<FeatureMeasurement> = four_feature_model()
            .into_iter()
            .map(|feature| measurement(feature.feature_id, feature.axial_coordinate_m, 7))
            .collect();
        let mut estimator =
            ObservedPoseEstimator::new(config(), OBJECT_ID, four_feature_model()).unwrap();
        let estimate = estimator.update(103, &observations);

        assert_eq!(estimate.validity, EstimateValidity::Valid);
        assert_eq!(estimate.head_count, 1);
        assert_eq!(estimate.observation_source_count, 1);
        assert_eq!(estimate.view_count, 1);
    }

    #[test]
    fn commanded_translation_predicts_pose_and_grows_uncertainty() {
        let mut cfg = config();
        cfg.maximum_measurement_age_ticks = 20;
        let mut estimator =
            ObservedPoseEstimator::new(cfg, OBJECT_ID, four_feature_model()).unwrap();
        let observed = estimator.update(103, &nominal_measurements());
        let before = observed.uncertainty.unwrap().center_variance_m2;
        let predicted = estimator.predict_commanded_translation(108, [50.0e-6, 0.0, 0.0]);

        assert_eq!(predicted.validity, EstimateValidity::Valid);
        assert_close(
            predicted.pose.unwrap().center_world_m[0],
            CENTER[0] + 50.0e-6,
            1.0e-12,
        );
        let after = predicted.uncertainty.unwrap().center_variance_m2;
        assert!(after
            .iter()
            .zip(before)
            .all(|(after, before)| *after > before));
        assert_eq!(predicted.prediction_count, 1);
    }

    #[test]
    fn held_transform_sigma_is_added_once_in_quadrature() {
        let mut cfg = config();
        cfg.maximum_measurement_age_ticks = 20;
        let mut estimator =
            ObservedPoseEstimator::new(cfg, OBJECT_ID, four_feature_model()).unwrap();
        let observed = estimator.update(103, &nominal_measurements());
        let before = observed.uncertainty.unwrap().center_variance_m2;
        let held_sigma = [4.0e-6, 5.0e-6, 6.0e-6];

        let inflated =
            estimator.inflate_translation_uncertainty(103, held_sigma, "guarded grasp attachment");

        assert_eq!(inflated.validity, EstimateValidity::Valid);
        let after = inflated.uncertainty.unwrap().center_variance_m2;
        for dimension in 0..3 {
            assert_close(
                after[dimension],
                before[dimension] + held_sigma[dimension].powi(2),
                1.0e-24,
            );
        }
        assert!(inflated
            .validity_detail
            .contains("guarded grasp attachment"));
    }

    #[test]
    fn accepted_calibration_reference_residual_increases_translation_uncertainty_once() {
        let mut cfg = config();
        cfg.maximum_measurement_age_ticks = 20;
        let mut estimator =
            ObservedPoseEstimator::new(cfg, OBJECT_ID, four_feature_model()).unwrap();
        let observed = estimator.update(103, &nominal_measurements());
        let before = observed.uncertainty.unwrap().center_variance_m2;
        let reference_residual_m = 6.0e-6;
        let conservative_sigma_m = reference_residual_m / 3.0;

        let inflated = estimator.inflate_translation_uncertainty(
            103,
            [conservative_sigma_m; 3],
            "accepted macro calibration-reference residual",
        );

        assert_eq!(inflated.validity, EstimateValidity::Valid);
        let after = inflated.uncertainty.unwrap().center_variance_m2;
        for dimension in 0..3 {
            assert_close(
                after[dimension],
                before[dimension] + conservative_sigma_m.powi(2),
                1.0e-24,
            );
            assert!(after[dimension] > before[dimension]);
        }
        assert!(inflated
            .validity_detail
            .contains("accepted macro calibration-reference residual"));
    }

    #[test]
    fn residual_scale_uses_five_dof_axisymmetric_state() {
        assert_eq!(AXISYMMETRIC_POSE_DOF, 5);
        assert_eq!(reduced_pose_residual_degrees_of_freedom(4), 7);
        assert_eq!(reduced_pose_residual_degrees_of_freedom(2), 1);
    }

    #[test]
    fn conservative_diagonal_bounds_omitted_pose_coupling() {
        let sigma: [f64; AXISYMMETRIC_POSE_DOF] = [2.0e-6, 3.0e-6, 4.0e-6, 8.0e-3, 12.0e-3];
        // Positive-semidefinite covariance with position/axis and tangent-axis
        // coupling: half independent diagonal plus one shared rank-one mode.
        let covariance: [[f64; AXISYMMETRIC_POSE_DOF]; AXISYMMETRIC_POSE_DOF] =
            std::array::from_fn(|row| {
                std::array::from_fn(|column| {
                    0.5 * sigma[row] * sigma[column]
                        + if row == column {
                            0.5 * sigma[row].powi(2)
                        } else {
                            0.0
                        }
                })
            });
        let bound = conservative_covariance_diagonal(covariance);
        for index in 0..AXISYMMETRIC_POSE_DOF {
            assert!(bound[index] >= covariance[index][index]);
        }

        for probe in [
            [1.0, -2.0, 0.5, 1.0e-4, -2.0e-4],
            [-3.0, 0.25, 2.0, -3.0e-4, 1.0e-4],
            [0.0, 1.0, -1.0, 2.0e-4, 2.0e-4],
        ] {
            let full = (0..AXISYMMETRIC_POSE_DOF)
                .map(|row| {
                    (0..AXISYMMETRIC_POSE_DOF)
                        .map(|column| probe[row] * covariance[row][column] * probe[column])
                        .sum::<f64>()
                })
                .sum::<f64>();
            let diagonal = (0..AXISYMMETRIC_POSE_DOF)
                .map(|index| bound[index] * probe[index].powi(2))
                .sum::<f64>();
            assert!(diagonal + 1.0e-30 >= full, "{diagonal:.12e} < {full:.12e}");
        }
    }

    #[test]
    fn innovation_cancels_shared_correlated_floor_once() {
        let position_floor_m = [3.0e-6; 3];
        let axis_floor_rad = 2.0e-3;
        let uncertainty = ReducedPoseUncertainty {
            center_variance_m2: [13.0e-12; 3],
            center_sigma_m: [13.0e-12_f64.sqrt(); 3],
            axis_tangent_basis_world: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            axis_tangent_variance_rad2: [9.0e-6; 2],
            axis_tangent_sigma_rad: [3.0e-3; 2],
            correlated_position_floor_m: position_floor_m,
            correlated_axis_floor_rad: axis_floor_rad,
        };
        let prior = AxisymmetricPose5d {
            center_world_m: [0.0; 3],
            axis_world_unit: [0.0, 0.0, 1.0],
            roll_observable: false,
        };
        let translation_sigma = 8.0e-12_f64.sqrt();
        let observed = AxisymmetricPose5d {
            center_world_m: [translation_sigma, 0.0, 0.0],
            ..prior
        };
        let innovation = pose_innovation(prior, uncertainty, observed, uncertainty, true);
        // Each state has 4 um^2 independent variance plus the shared 9 um^2
        // floor. The shared term cancels, leaving 8 um^2 for the difference.
        assert_close(innovation.normalized_norm, 1.0, 1.0e-12);
        assert_close(
            shared_floor_difference_variance(13.0e-12, 13.0e-12, 3.0e-6, 3.0e-6),
            8.0e-12,
            1.0e-26,
        );

        let independent_heads = pose_innovation(prior, uncertainty, observed, uncertainty, false);
        assert_close(
            independent_heads.normalized_norm,
            (8.0_f64 / 26.0).sqrt(),
            1.0e-12,
        );

        let axis_sigma = 10.0e-6_f64.sqrt();
        let rotated = AxisymmetricPose5d {
            center_world_m: prior.center_world_m,
            axis_world_unit: [axis_sigma.sin(), 0.0, axis_sigma.cos()],
            roll_observable: false,
        };
        let axis_innovation = pose_innovation(prior, uncertainty, rotated, uncertainty, true);
        assert_close(axis_innovation.axis_angle_rad, axis_sigma, 1.0e-14);
        assert_close(axis_innovation.normalized_norm, 1.0, 1.0e-12);
    }
}
