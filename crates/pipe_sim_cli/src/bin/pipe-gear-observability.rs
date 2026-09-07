use pipe_sim::gear_observability::{run_gear_study, GearStudyConfig};
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}
fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut revision = None;
    let mut config = GearStudyConfig::default();
    let mut print = false;
    let mut summary = false;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--config" => {
                config = serde_json::from_str(&std::fs::read_to_string(
                    args.next().ok_or("missing config path")?,
                )?)?
            }
            "--source-revision" => revision = Some(args.next().ok_or("missing revision")?),
            "--print-config" => print = true,
            "--summary" => summary = true,
            _ => return Err(format!("unknown argument: {a}").into()),
        }
    }
    if print {
        println!("{}", serde_json::to_string_pretty(&config)?);
        return Ok(());
    }
    let report = run_gear_study(config, &revision.ok_or("--source-revision required")?)?;
    let mut value = serde_json::to_value(report)?;
    if summary {
        let mut configs = serde_json::Map::new();
        for candidate in value["candidates"]
            .as_array_mut()
            .ok_or("invalid candidates")?
        {
            let hash = candidate["config_sha256"]
                .as_str()
                .ok_or("missing config hash")?
                .to_string();
            let fields = candidate.as_object_mut().ok_or("invalid candidate")?;
            fields.remove("samples");
            configs
                .entry(hash)
                .or_insert(fields.remove("optical_config").ok_or("missing optics")?);
        }
        value["sample_details_omitted"] = true.into();
        value["optical_configurations_by_sha256"] = configs.into();
    }
    println!("{}", serde_json::to_string_pretty(&value)?);
    Ok(())
}
