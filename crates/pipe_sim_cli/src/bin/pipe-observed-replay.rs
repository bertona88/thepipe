//! Headless engineering recording; stdout is the versioned replay envelope.
use std::io::{self, Write};
use std::process::ExitCode;

use pipe_sim::observed_manipulation::{
    M1eFault, ObservedManipulationRuntime, BASELINE_M1F_SCENARIO_JSON,
};

fn run() -> Result<bool, String> {
    let mut args = std::env::args().skip(1);
    let mut source = None;
    let mut scenario = None;
    let mut fault = None;
    while let Some(key) = args.next() {
        if key == "--help" {
            println!("pipe-observed-replay --source-revision FULL_SHA [--scenario PATH] [--fault NAME]\nDefault: fixed-head M1f. Exact samples every 100 ticks plus decisions. JSON to stdout.");
            return Ok(true);
        }
        let slot = match key.as_str() {
            "--source-revision" => &mut source,
            "--scenario" => &mut scenario,
            "--fault" => &mut fault,
            _ => return Err(format!("unknown argument: {key}")),
        };
        if slot.is_some() {
            return Err(format!("duplicate argument: {key}"));
        }
        let value = args
            .next()
            .filter(|v| !v.starts_with("--"))
            .ok_or_else(|| format!("missing value for {key}"))?;
        *slot = Some(value);
    }
    let source = source.ok_or("--source-revision is required")?;
    if source.len() != 40 || !source.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("source revision must be a full 40-character Git SHA".into());
    }
    let json = match scenario {
        Some(path) => std::fs::read_to_string(path).map_err(|e| e.to_string())?,
        None => BASELINE_M1F_SCENARIO_JSON.to_owned(),
    };
    let fault = fault
        .unwrap_or_else(|| "none".into())
        .parse::<M1eFault>()
        .map_err(|e| e.to_string())?;
    let mut runtime =
        ObservedManipulationRuntime::from_scenario_json(&json, fault).map_err(|e| e.to_string())?;
    runtime
        .enable_replay(100, 20_000)
        .map_err(|e| e.to_string())?;
    let report = runtime.run_cycle().map_err(|e| e.to_string())?;
    let accepted = report.expected_outcome_observed
        && report
            .acceptance_gates
            .iter()
            .filter(|gate| gate.applicable)
            .all(|gate| gate.passed);
    // JSON-encoded argv preserves spaces and literal characters unambiguously.
    let command =
        serde_json::to_string(&std::env::args().collect::<Vec<_>>()).map_err(|e| e.to_string())?;
    let replay = runtime
        .replay(&source, &command)
        .map_err(|e| e.to_string())?;
    serde_json::to_writer(io::stdout().lock(), &replay).map_err(|e| e.to_string())?;
    io::stdout()
        .lock()
        .write_all(b"\n")
        .map_err(|e| e.to_string())?;
    Ok(accepted)
}

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(2),
        Err(error) => {
            eprintln!("pipe-observed-replay: {error}");
            ExitCode::FAILURE
        }
    }
}
