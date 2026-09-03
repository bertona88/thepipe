#![forbid(unsafe_code)]

use pipe_sim::{
    PointMotionRuntime as NativePointMotionRuntime, ReferenceSimulator as NativeSimulator,
    ScenarioSpec, REPORT_SCHEMA_VERSION, SCENE_SCHEMA_VERSION,
};
use pipe_sim_core::{ManipulatorId, Vec3};
use wasm_bindgen::prelude::*;

pub const WASM_API_SCHEMA_VERSION: u32 = REPORT_SCHEMA_VERSION;
const DEFAULT_MAX_CYCLES: u32 = 12_000;
const DEFAULT_MAX_POINT_MOTION_STEPS: u32 = 20_000;

fn js_error(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&error.to_string())
}

/// Browser-safe wrapper around the same deterministic simulator used by the
/// headless CLI. JSON is used intentionally so the website can retain report
/// snapshots across WASM schema upgrades without exposing Rust object layouts.
#[wasm_bindgen(js_name = ReferenceSimulator)]
pub struct WasmReferenceSimulator {
    inner: NativeSimulator,
}

/// Standalone M1b browser runtime. This deliberately excludes the legacy
/// gearbox executive so it cannot overwrite a Cartesian calibration command.
#[wasm_bindgen(js_name = PointMotionSimulator)]
pub struct WasmPointMotionSimulator {
    inner: NativePointMotionRuntime,
}

#[wasm_bindgen(js_class = PointMotionSimulator)]
impl WasmPointMotionSimulator {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Result<WasmPointMotionSimulator, JsValue> {
        Ok(Self {
            inner: NativePointMotionRuntime::new().map_err(js_error)?,
        })
    }

    #[wasm_bindgen(js_name = setToolTarget)]
    pub fn set_tool_target(
        &mut self,
        manipulator_id: u32,
        x_m: f64,
        y_m: f64,
        z_m: f64,
    ) -> Result<u64, JsValue> {
        self.inner
            .submit_tool_target(ManipulatorId(manipulator_id), Vec3::new(x_m, y_m, z_m))
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = setCalibrationTarget)]
    pub fn set_calibration_target(&mut self) -> Result<u64, JsValue> {
        self.inner.submit_calibration_target().map_err(js_error)
    }

    #[wasm_bindgen(js_name = stepJson)]
    pub fn step_json(&mut self) -> Result<String, JsValue> {
        self.inner.step().map_err(js_error)?;
        self.inner.scene_frame_json(false).map_err(js_error)
    }

    #[wasm_bindgen(js_name = runUntilSettledJson)]
    pub fn run_until_settled_json(&mut self, max_steps: u32) -> Result<String, JsValue> {
        let limit = if max_steps == 0 {
            DEFAULT_MAX_POINT_MOTION_STEPS
        } else {
            max_steps
        };
        self.inner
            .run_until_settled(limit)
            .and_then(|report| report.to_json(false))
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = runCalibrationCycleJson)]
    pub fn run_calibration_cycle_json(
        &mut self,
        max_steps_per_leg: u32,
    ) -> Result<String, JsValue> {
        let limit = if max_steps_per_leg == 0 {
            DEFAULT_MAX_POINT_MOTION_STEPS
        } else {
            max_steps_per_leg
        };
        self.inner
            .run_calibration_cycle(limit)
            .and_then(|report| report.to_json(false))
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = reportJson)]
    pub fn report_json(&self, pretty: bool) -> Result<String, JsValue> {
        self.inner.report().to_json(pretty).map_err(js_error)
    }

    #[wasm_bindgen(js_name = sceneDescriptionJson)]
    pub fn scene_description_json(&self) -> Result<String, JsValue> {
        self.inner.scene_description_json(false).map_err(js_error)
    }

    #[wasm_bindgen(js_name = sceneFrameJson)]
    pub fn scene_frame_json(&self) -> Result<String, JsValue> {
        self.inner.scene_frame_json(false).map_err(js_error)
    }

    #[wasm_bindgen(getter, js_name = active)]
    pub fn active(&self) -> bool {
        self.inner.is_active()
    }
}

#[wasm_bindgen(js_class = ReferenceSimulator)]
impl WasmReferenceSimulator {
    #[wasm_bindgen(constructor)]
    pub fn new(scenario: Option<String>) -> Result<WasmReferenceSimulator, JsValue> {
        let scenario = scenario.as_deref().unwrap_or("nominal");
        let inner = NativeSimulator::from_scenario_name(scenario).map_err(js_error)?;
        Ok(Self { inner })
    }

    /// Advance exactly one 20 ms executive cycle and return its full snapshot.
    #[wasm_bindgen(js_name = stepJson)]
    pub fn step_json(&mut self) -> Result<String, JsValue> {
        self.inner
            .step()
            .map_err(js_error)?
            .to_json(false)
            .map_err(js_error)
    }

    /// Run until assembly completes/aborts, using 12k cycles when zero is passed.
    #[wasm_bindgen(js_name = runToCompletionJson)]
    pub fn run_to_completion_json(&mut self, max_cycles: u32) -> Result<String, JsValue> {
        let limit = if max_cycles == 0 {
            DEFAULT_MAX_CYCLES
        } else {
            max_cycles
        };
        self.inner
            .run_to_completion(limit)
            .map_err(js_error)?
            .to_json(false)
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = reportJson)]
    pub fn report_json(&self, pretty: bool) -> Result<String, JsValue> {
        self.inner.report_json(pretty).map_err(js_error)
    }

    /// Static geometry/topology/configuration. Fetch once per simulator reset.
    #[wasm_bindgen(js_name = sceneDescriptionJson)]
    pub fn scene_description_json(&self) -> Result<String, JsValue> {
        self.inner.scene_description_json(false).map_err(js_error)
    }

    /// Current physical/estimated/commanded layers without advancing time.
    #[wasm_bindgen(js_name = sceneFrameJson)]
    pub fn scene_frame_json(&self) -> Result<String, JsValue> {
        self.inner.scene_frame_json(false).map_err(js_error)
    }

    #[wasm_bindgen(getter, js_name = terminal)]
    pub fn terminal(&self) -> bool {
        self.inner.is_terminal()
    }

    #[wasm_bindgen(getter, js_name = completed)]
    pub fn completed(&self) -> bool {
        self.inner.is_completed()
    }

    #[wasm_bindgen(getter, js_name = scenario)]
    pub fn scenario(&self) -> String {
        self.inner.scenario_spec().name.clone()
    }
}

#[wasm_bindgen(js_name = runReferenceScenario)]
pub fn run_reference_scenario(scenario: &str, max_cycles: u32) -> Result<String, JsValue> {
    let mut simulator = NativeSimulator::from_scenario_name(scenario).map_err(js_error)?;
    let limit = if max_cycles == 0 {
        DEFAULT_MAX_CYCLES
    } else {
        max_cycles
    };
    simulator
        .run_to_completion(limit)
        .map_err(js_error)?
        .to_json(false)
        .map_err(js_error)
}

#[wasm_bindgen(js_name = availableScenariosJson)]
pub fn available_scenarios_json() -> String {
    let values = ScenarioSpec::available()
        .iter()
        .map(|name| format!("\"{name}\""))
        .collect::<Vec<_>>()
        .join(",");
    format!("[{values}]")
}

#[wasm_bindgen(js_name = reportSchemaVersion)]
pub fn report_schema_version() -> u32 {
    REPORT_SCHEMA_VERSION
}

#[wasm_bindgen(js_name = sceneSchemaVersion)]
pub fn scene_schema_version() -> u32 {
    SCENE_SCHEMA_VERSION
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scenario_listing_is_json() {
        assert_eq!(
            available_scenarios_json(),
            "[\"gearbox_baseline_v1\",\"nominal\",\"collision\",\"occlusion\",\"insertion-force\",\"fault-suite\"]"
        );
    }

    #[test]
    fn schema_versions_match() {
        assert_eq!(report_schema_version(), pipe_sim::REPORT_SCHEMA_VERSION);
        assert_eq!(scene_schema_version(), pipe_sim::SCENE_SCHEMA_VERSION);
    }
}
