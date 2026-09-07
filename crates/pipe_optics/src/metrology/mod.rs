//! Metres, seconds, radians, kelvin and pixels; `a_from_b` maps b into a.
//! Reconstruction has no access to simulation truth. See `synthetic` for the
//! explicitly reduced, seeded image-feature/intensity acquisition model.
mod config;
mod contract;
mod part;
mod products;
pub use part::*;
mod solve;
mod structured;
pub mod synthetic;
pub mod verification;
pub use config::*;
pub use contract::*;
pub use products::*;
pub use solve::*;
pub use structured::*;

use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MetrologyError {
    InvalidConfiguration(String),
    InvalidObservation,
    InvalidCalibration,
    DuplicateObservation,
    InsufficientViews,
    DegenerateGeometry,
    OutsideField,
    InconsistentObservations,
    IncompatibleTiming,
    MotionDuringSequence,
    LowSignal,
    Saturation,
    UnsupportedSurface,
    Occluded,
    UnobservablePose,
    VerificationLeakage,
}
impl std::fmt::Display for MetrologyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for MetrologyError {}

pub(crate) fn positive(x: f64) -> bool {
    x.is_finite() && x > 0.0
}
pub(crate) fn nonnegative(x: f64) -> bool {
    x.is_finite() && x >= 0.0
}
pub(crate) fn config_error(s: &str) -> MetrologyError {
    MetrologyError::InvalidConfiguration(s.into())
}
