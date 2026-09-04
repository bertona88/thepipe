use std::env;
use std::ffi::{OsStr, OsString};
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use pipe_sim::observed_manipulation::{
    M1eFault, ObservedManipulationReport, ObservedManipulationRuntime,
};

#[derive(Debug, PartialEq, Eq)]
struct Options {
    scenario_path: Option<PathBuf>,
    fault: M1eFault,
    report_path: Option<PathBuf>,
    pretty: bool,
}

#[derive(Debug, PartialEq, Eq)]
enum ParsedCommand {
    Run(Options),
    Help,
}

fn usage() -> String {
    format!(
        "usage: pipe-observed-manipulation [--scenario PATH] [--fault NAME] \
         [--report PATH] [--compact]\n\
         \n\
         Runs the deterministic M1e observed-state manipulation scenario.\n\
         With no --scenario, the versioned embedded baseline is used.\n\
         Faults: {}",
        M1eFault::available().join(", ")
    )
}

fn parse_args<I, S>(args: I) -> Result<ParsedCommand, String>
where
    I: IntoIterator<Item = S>,
    S: Into<OsString>,
{
    let mut scenario_path = None;
    let mut fault = M1eFault::None;
    let mut fault_set = false;
    let mut report_path = None;
    let mut pretty = true;
    let mut compact_set = false;
    let mut args = args.into_iter().map(Into::into);

    while let Some(argument) = args.next() {
        if argument == "--scenario" {
            reject_duplicate(scenario_path.is_some(), "--scenario")?;
            scenario_path = Some(PathBuf::from(required_value(&mut args, "--scenario")?));
        } else if argument == "--fault" {
            reject_duplicate(fault_set, "--fault")?;
            let value = required_value(&mut args, "--fault")?;
            let value = value
                .into_string()
                .map_err(|value| format!("--fault requires a UTF-8 value, found {value:?}"))?;
            fault = value
                .parse::<M1eFault>()
                .map_err(|error| error.to_string())?;
            fault_set = true;
        } else if argument == "--report" {
            reject_duplicate(report_path.is_some(), "--report")?;
            report_path = Some(PathBuf::from(required_value(&mut args, "--report")?));
        } else if argument == "--compact" {
            reject_duplicate(compact_set, "--compact")?;
            pretty = false;
            compact_set = true;
        } else if argument == "--help" || argument == "-h" {
            return Ok(ParsedCommand::Help);
        } else if let Some(other) = argument.to_str() {
            return Err(format!("unknown argument '{other}'\n{}", usage()));
        } else {
            return Err(format!(
                "argument is not valid UTF-8: {argument:?}\n{}",
                usage()
            ));
        }
    }

    Ok(ParsedCommand::Run(Options {
        scenario_path,
        fault,
        report_path,
        pretty,
    }))
}

fn required_value<I>(args: &mut I, option: &str) -> Result<OsString, String>
where
    I: Iterator<Item = OsString>,
{
    let value = args
        .next()
        .ok_or_else(|| format!("{option} requires a value"))?;
    if value.as_encoded_bytes().first() == Some(&b'-') && value != "-" {
        return Err(format!(
            "{option} requires a value, found option '{}'",
            value.to_string_lossy()
        ));
    }
    Ok(value)
}

fn reject_duplicate(already_set: bool, option: &str) -> Result<(), String> {
    if already_set {
        Err(format!("{option} may be specified only once"))
    } else {
        Ok(())
    }
}

fn build_runtime(options: &Options) -> Result<ObservedManipulationRuntime, String> {
    match options.scenario_path.as_deref() {
        Some(path) => {
            let scenario_json = fs::read_to_string(path)
                .map_err(|error| format!("failed to read scenario {}: {error}", path.display()))?;
            ObservedManipulationRuntime::from_scenario_json(&scenario_json, options.fault)
                .map_err(|error| error.to_string())
        }
        None => ObservedManipulationRuntime::new(options.fault).map_err(|error| error.to_string()),
    }
}

fn write_report(path: &Path, json: &str) -> Result<(), String> {
    if path.as_os_str() == "-" {
        return Ok(());
    }
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent).map_err(|error| {
            format!(
                "failed to create report directory {}: {error}",
                parent.display()
            )
        })?;
    }
    let mut bytes = Vec::with_capacity(json.len() + 1);
    bytes.extend_from_slice(json.as_bytes());
    bytes.push(b'\n');
    fs::write(path, bytes)
        .map_err(|error| format!("failed to write report {}: {error}", path.display()))
}

fn reject_same_scenario_and_report(options: &Options) -> Result<(), String> {
    let (Some(scenario), Some(report)) = (
        options.scenario_path.as_deref(),
        options.report_path.as_deref(),
    ) else {
        return Ok(());
    };
    if report.as_os_str() == OsStr::new("-") {
        return Ok(());
    }

    let canonical_match = match (fs::canonicalize(scenario), fs::canonicalize(report)) {
        (Ok(scenario), Ok(report)) => scenario == report,
        _ => false,
    };
    if scenario == report || canonical_match || same_file_identity(scenario, report) {
        return Err(format!(
            "scenario and report paths must differ: {}",
            scenario.display()
        ));
    }
    Ok(())
}

#[cfg(unix)]
fn same_file_identity(left: &Path, right: &Path) -> bool {
    use std::os::unix::fs::MetadataExt;

    match (fs::metadata(left), fs::metadata(right)) {
        (Ok(left), Ok(right)) => left.dev() == right.dev() && left.ino() == right.ino(),
        _ => false,
    }
}

#[cfg(not(unix))]
fn same_file_identity(_left: &Path, _right: &Path) -> bool {
    false
}

fn write_line(mut writer: impl Write, value: &str) -> io::Result<()> {
    writer.write_all(value.as_bytes())?;
    writer.write_all(b"\n")?;
    writer.flush()
}

fn write_stdout(value: &str) -> Result<(), String> {
    write_line(io::stdout().lock(), value)
        .map_err(|error| format!("failed to write stdout: {error}"))
}

fn error_exit(error: &str) -> ExitCode {
    let _ = write_line(
        io::stderr().lock(),
        &format!("pipe-observed-manipulation: {error}"),
    );
    ExitCode::from(1)
}

fn run(options: Options) -> Result<bool, String> {
    reject_same_scenario_and_report(&options)?;
    let mut runtime = build_runtime(&options)?;
    let report = runtime.run_cycle().map_err(|error| error.to_string())?;
    let accepted = report_meets_expected_outcome(&report, options.fault);
    let json = report
        .to_json(options.pretty)
        .map_err(|error| error.to_string())?;

    write_stdout(&json)?;
    if let Some(path) = options.report_path.as_deref() {
        write_report(path, &json)?;
    }

    Ok(accepted)
}

/// A named fault is an acceptance test of its fail-closed classification.  A
/// nominal run has the stronger contract: every controller gate must pass and
/// the post-terminal, evaluation-only physical score must also be inside the
/// declared tolerance without unplanned penetration.  The latter never feeds
/// back into the controller or its deterministic report hash.
fn report_meets_expected_outcome(report: &ObservedManipulationReport, fault: M1eFault) -> bool {
    if !report.expected_outcome_observed {
        return false;
    }
    if fault != M1eFault::None {
        return report
            .acceptance_gates
            .iter()
            .filter(|gate| gate.applicable)
            .all(|gate| gate.passed);
    }
    report.acceptance_gates.iter().all(|gate| gate.passed)
        && report.evaluation_only_truth.as_ref().is_some_and(|score| {
            score.within_declared_seat_tolerances
                && score.maximum_unplanned_penetration_m == 0.0
                && score.peak_grip_force_proxy_n <= report.metrics.maximum_grip_force_proxy_n
                && score.peak_insertion_force_proxy_n
                    <= report.metrics.maximum_insertion_force_proxy_n
                && score.physical_release_verified
                && report.metrics.retreat_confirmed
        })
}

fn main() -> ExitCode {
    match parse_args(env::args_os().skip(1)) {
        Ok(ParsedCommand::Help) => match write_stdout(&usage()) {
            Ok(()) => ExitCode::SUCCESS,
            Err(error) => error_exit(&error),
        },
        Ok(ParsedCommand::Run(options)) => match run(options) {
            Ok(true) => ExitCode::SUCCESS,
            Ok(false) => ExitCode::from(2),
            Err(error) => error_exit(&error),
        },
        Err(error) => error_exit(&error),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn owned(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| (*value).to_owned()).collect()
    }

    #[test]
    fn defaults_to_embedded_nominal_pretty_report() {
        assert_eq!(
            parse_args(Vec::<String>::new()).unwrap(),
            ParsedCommand::Run(Options {
                scenario_path: None,
                fault: M1eFault::None,
                report_path: None,
                pretty: true,
            })
        );
    }

    #[test]
    fn parses_scenario_fault_report_and_compact_flags() {
        assert_eq!(
            parse_args(owned(&[
                "--scenario",
                "scenario.json",
                "--fault",
                "insertion-jam",
                "--report",
                "out/report.json",
                "--compact",
            ]))
            .unwrap(),
            ParsedCommand::Run(Options {
                scenario_path: Some(PathBuf::from("scenario.json")),
                fault: M1eFault::InsertionJam,
                report_path: Some(PathBuf::from("out/report.json")),
                pretty: false,
            })
        );
    }

    #[test]
    fn help_is_a_successful_command() {
        assert_eq!(parse_args(owned(&["--help"])).unwrap(), ParsedCommand::Help);
        assert!(usage().contains("optical_dropout"));
    }

    #[test]
    fn rejects_missing_option_values_and_unknown_flags() {
        assert_eq!(
            parse_args(owned(&["--scenario"])).unwrap_err(),
            "--scenario requires a value"
        );
        assert!(parse_args(owned(&["--fault", "--compact"]))
            .unwrap_err()
            .contains("found option '--compact'"));
        assert!(parse_args(owned(&["--laser-mode"]))
            .unwrap_err()
            .contains("unknown argument '--laser-mode'"));
    }

    #[test]
    fn rejects_unknown_faults_and_duplicate_options() {
        let unknown = parse_args(owned(&["--fault", "laser_dragons"])).unwrap_err();
        assert!(unknown.contains("unknown M1e fault 'laser_dragons'"));
        assert!(unknown.contains("optical_dropout"));

        assert_eq!(
            parse_args(owned(&[
                "--scenario",
                "first.json",
                "--scenario",
                "second.json",
            ]))
            .unwrap_err(),
            "--scenario may be specified only once"
        );
        assert_eq!(
            parse_args(owned(&["--compact", "--compact"])).unwrap_err(),
            "--compact may be specified only once"
        );
    }

    #[cfg(unix)]
    #[test]
    fn preserves_non_utf8_paths_and_rejects_non_utf8_flags_and_faults() {
        use std::os::unix::ffi::OsStringExt;

        let path = OsString::from_vec(b"scenario-\xff.json".to_vec());
        let parsed = parse_args(vec![OsString::from("--scenario"), path.clone()]).unwrap();
        let ParsedCommand::Run(options) = parsed else {
            panic!("scenario option must parse as a run");
        };
        assert_eq!(options.scenario_path, Some(PathBuf::from(path)));

        let invalid = OsString::from_vec(vec![0xff]);
        assert!(parse_args(vec![invalid.clone()])
            .unwrap_err()
            .contains("argument is not valid UTF-8"));
        assert!(parse_args(vec![OsString::from("--fault"), invalid])
            .unwrap_err()
            .contains("--fault requires a UTF-8 value"));
    }

    #[test]
    fn rejects_a_report_that_would_overwrite_its_scenario() {
        let options = Options {
            scenario_path: Some(PathBuf::from("scenario.json")),
            fault: M1eFault::None,
            report_path: Some(PathBuf::from("scenario.json")),
            pretty: false,
        };
        assert!(reject_same_scenario_and_report(&options)
            .unwrap_err()
            .contains("scenario and report paths must differ"));
    }

    #[test]
    fn write_line_propagates_output_failures() {
        struct BrokenWriter;

        impl Write for BrokenWriter {
            fn write(&mut self, _buffer: &[u8]) -> io::Result<usize> {
                Err(io::Error::new(io::ErrorKind::BrokenPipe, "closed"))
            }

            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }

        assert_eq!(
            write_line(BrokenWriter, "report").unwrap_err().kind(),
            io::ErrorKind::BrokenPipe
        );
        let mut output = Vec::new();
        write_line(&mut output, "report").unwrap();
        assert_eq!(output, b"report\n");
    }

    #[test]
    fn nominal_exit_acceptance_includes_gates_and_terminal_physical_score() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
        let mut report = runtime.run_cycle().unwrap();
        assert!(report_meets_expected_outcome(&report, M1eFault::None));

        report.acceptance_gates[0].passed = false;
        assert!(!report_meets_expected_outcome(&report, M1eFault::None));
        report.acceptance_gates[0].passed = true;
        report
            .evaluation_only_truth
            .as_mut()
            .unwrap()
            .within_declared_seat_tolerances = false;
        assert!(!report_meets_expected_outcome(&report, M1eFault::None));
    }

    #[test]
    fn named_fault_exit_acceptance_still_requires_applicable_safety_gates() {
        let mut runtime = ObservedManipulationRuntime::new(M1eFault::OpticalDropout).unwrap();
        let mut report = runtime.run_cycle().unwrap();
        assert!(report_meets_expected_outcome(
            &report,
            M1eFault::OpticalDropout
        ));

        let gate = report
            .acceptance_gates
            .iter_mut()
            .find(|gate| gate.applicable)
            .expect("fault reports retain applicable safety gates");
        gate.passed = false;
        assert!(!report_meets_expected_outcome(
            &report,
            M1eFault::OpticalDropout
        ));
    }
}
