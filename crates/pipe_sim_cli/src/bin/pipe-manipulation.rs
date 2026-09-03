use std::env;
use std::process::ExitCode;

use pipe_sim::SimpleManipulationRuntime;

const DEFAULT_MAX_STEPS_PER_ACTION: u32 = 20_000;

fn main() -> ExitCode {
    match run() {
        Ok(completed) => {
            if completed {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(2)
            }
        }
        Err(error) => {
            eprintln!("pipe-manipulation: {error}");
            ExitCode::from(1)
        }
    }
}

fn run() -> Result<bool, String> {
    let mut pretty = true;
    let mut max_steps_per_action = DEFAULT_MAX_STEPS_PER_ACTION;
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--compact" => pretty = false,
            "--max-steps-per-action" => {
                let value = args
                    .next()
                    .ok_or("--max-steps-per-action requires a positive integer")?;
                max_steps_per_action = value
                    .parse::<u32>()
                    .map_err(|_| format!("invalid --max-steps-per-action value '{value}'"))?;
                if max_steps_per_action == 0 {
                    return Err("--max-steps-per-action must be positive".to_owned());
                }
            }
            "--help" | "-h" => {
                return Err(
                    "usage: pipe-manipulation [--max-steps-per-action N] [--compact]".to_owned(),
                );
            }
            other => return Err(format!("unknown argument '{other}'")),
        }
    }

    let mut runtime = SimpleManipulationRuntime::new().map_err(|error| error.to_string())?;
    let report = runtime
        .run_cycle(max_steps_per_action)
        .map_err(|error| error.to_string())?;
    println!(
        "{}",
        report.to_json(pretty).map_err(|error| error.to_string())?
    );
    Ok(report.status == "complete")
}
