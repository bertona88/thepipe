#![forbid(unsafe_code)]

use pipe_sim::{
    ReferenceSimulator as NativeSimulator, ScenarioSpec, REPORT_SCHEMA_VERSION,
    SCENE_SCHEMA_VERSION,
};
use wasm_bindgen::prelude::*;

pub const WASM_API_SCHEMA_VERSION: u32 = REPORT_SCHEMA_VERSION;
const DEFAULT_MAX_CYCLES: u32 = 12_000;

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
