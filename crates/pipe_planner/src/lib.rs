//! Deterministic task planning for the pipe micro-assembly cell.
//!
//! This crate deliberately stops at the task-executive boundary. Collision,
//! rigid-body, optical, and arm models produce [`SensorFrame`] values; the
//! executive turns those observations into guarded hardware commands. Keeping
//! this boundary small makes the exact same state machine usable by a native
//! simulator, a WASM build, and eventually a real low-cost controller.

mod core_adapter;
mod executive;
mod model;
mod scenario;

pub use core_adapter::*;
pub use executive::GearboxTaskExecutive;
pub use model::*;
pub use scenario::MicroGearboxScenario;
