use std::env;
use std::process::ExitCode;

use pipe_sim::optical_codesign_report;

fn main() -> ExitCode {
    match run() {
        Ok(model_feasible) => {
            if model_feasible {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(2)
            }
        }
        Err(error) => {
            eprintln!("pipe-optical-codesign: {error}");
            ExitCode::from(1)
        }
    }
}

fn run() -> Result<bool, String> {
    let mut pretty = true;
    for argument in env::args().skip(1) {
        match argument.as_str() {
            "--compact" => pretty = false,
            "--help" | "-h" => {
                eprintln!("usage: pipe-optical-codesign [--compact]");
                return Ok(true);
            }
            other => return Err(format!("unknown argument '{other}'")),
        }
    }

    let report = optical_codesign_report().map_err(|error| error.to_string())?;
    println!(
        "{}",
        report.to_json(pretty).map_err(|error| error.to_string())?
    );
    eprintln!(
        "{}: {}; hardware qualification {}",
        report.study_id, report.overall_status, report.hardware_qualification_status
    );
    Ok(report.overall_status == "model_feasible_hardware_qualification_required")
}
