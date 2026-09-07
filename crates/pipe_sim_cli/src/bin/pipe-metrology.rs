use pipe_sim::metrology::run_verification;
use pipe_sim::point_motion::PointMotionRuntime;
use std::{env, fs};
fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut config = None;
    let mut revision = None;
    let mut repeats = 12;
    let mut baseline = false;
    let mut machine = false;
    let mut summary = false;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--config" => config = Some(args.next().ok_or("--config requires path")?),
            "--source-revision" => {
                revision = Some(args.next().ok_or("--source-revision requires SHA")?)
            }
            "--repeats" => {
                repeats = args
                    .next()
                    .ok_or("--repeats requires count")?
                    .parse::<u32>()?
            }
            "--print-config" => baseline = true,
            "--machine" => machine = true,
            "--summary" => summary = true,
            "--help" => {
                println!("pipe-metrology [--config PATH] --source-revision SHA [--repeats N] [--summary]\npipe-metrology --print-config\npipe-metrology --machine --source-revision SHA");
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if machine {
        if config.is_some() || baseline || summary {
            return Err(
                "--machine cannot combine with --config, --print-config or --summary".into(),
            );
        }
        let source = revision.ok_or("--machine requires --source-revision")?;
        if source.len() != 40 || !source.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err("full source commit SHA required".into());
        }
        let mut runtime = PointMotionRuntime::new()?;
        let mut optics = runtime.metrology_candidate()?;
        let mut frame = serde_json::to_value(runtime.acquire_metrology(&mut optics)?)?;
        frame["source_revision"] = source.into();
        println!("{}", serde_json::to_string_pretty(&frame)?);
        return Ok(());
    }
    let c = if let Some(path) = config {
        serde_json::from_str(&fs::read_to_string(path)?)?
    } else {
        pipe_sim::metrology::baseline_config()
    };
    if baseline {
        println!("{}", serde_json::to_string_pretty(&c)?);
        return Ok(());
    }
    let report = run_verification(
        c,
        &revision.ok_or("--source-revision is required")?,
        repeats,
    )?;
    let mut output = serde_json::to_value(report)?;
    if summary {
        for name in ["passive", "structured"] {
            output[name]
                .as_object_mut()
                .ok_or("invalid summary")?
                .remove("samples");
        }
        for item in output["sensitivity"]
            .as_array_mut()
            .ok_or("invalid sensitivity summary")?
        {
            item.as_object_mut()
                .ok_or("invalid sensitivity entry")?
                .remove("samples");
        }
        output["sample_details_omitted"] = true.into();
    }
    println!("{}", serde_json::to_string_pretty(&output)?);
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}
