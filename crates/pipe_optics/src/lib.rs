//! Low-cost optical metrology simulation for the Pipe assembly cell.
//!
//! The crate deliberately uses no external dependencies, which keeps native and
//! `wasm32-unknown-unknown` builds small and reproducible.  Units are SI unless a
//! field name explicitly says otherwise (pixels and radians are called out).

#![forbid(unsafe_code)]

mod camera;
mod math;
mod noise;
mod precision;
mod reconstruction;
mod scene;
mod sensing;

pub use camera::{
    BrownConrady, CalibratedCamera, CalibrationDrift, CameraIntrinsics, DriftRandomWalk, ImageSize,
    PinholeCamera, ProjectedPoint,
};
pub use math::{Mat3, Ray, RigidTransform, Vec2, Vec3};
pub use precision::{
    random_triangulation_precision, remaining_independent_rms_budget,
    symmetric_triangulation_angle_rad, PrecisionModelError, PrecisionModelInput,
    PrecisionPrediction, RandomTriangulationPrecision,
};
pub use reconstruction::{
    fuse_points, triangulate_rays, Covariance3, FusedPoint, PointEstimate, QualityMetrics,
    Triangulation,
};
pub use scene::{
    Aabb, Cylinder, Geometry, Hit, Material, MeshError, Primitive, Scene, Sphere, Triangle,
};
pub use sensing::{
    DepthReturn, DepthSample, Fiducial, FiducialObservation, MissingReturn, ScanConfig, ScanFrame,
    ScanStats, StructuredLightRig,
};
