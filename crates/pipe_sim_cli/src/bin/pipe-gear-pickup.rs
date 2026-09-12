use pipe_sim::gear_pickup::{run_gear_pickup, PickupFault};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 5 || args.len() > 6 {
        return Err("usage: pipe-gear-pickup CONFIG SOURCE_REVISION SOURCE_TREE_SHA256 OUTPUT [missing-part|stale|timeout|unsupported-release]".into());
    }
    if args[2].len() != 40 || !args[2].bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("SOURCE_REVISION must be actual 40-digit Git revision".into());
    }
    if args[3].len() != 64 || !args[3].bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("SOURCE_TREE_SHA256 must be actual 64-digit SHA256".into());
    }
    let fault = match args.get(5).map(String::as_str) {
        None => PickupFault::None,
        Some("missing-part") => PickupFault::MissingPartObservation,
        Some("stale") => PickupFault::StaleObservation,
        Some("timeout") => PickupFault::MotionTimeout,
        Some("unsupported-release") => PickupFault::UnsupportedRelease,
        Some(_) => return Err("unknown fault".into()),
    };
    let report = run_gear_pickup(
        &std::fs::read_to_string(&args[1])?,
        &args[2],
        &args[3],
        fault,
    )?;
    std::fs::write(&args[4], serde_json::to_vec_pretty(&report)?)?;
    eprintln!(
        "{}: {:?}; {} authoritative samples",
        report.status,
        report.refusal,
        report.samples.len()
    );
    Ok(())
}
