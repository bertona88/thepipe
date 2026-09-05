use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Output, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};

use serde_json::Value;

fn cli() -> Command {
    Command::new(env!("CARGO_BIN_EXE_pipe-observed-manipulation"))
}

fn baseline_scenario_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/observed_manipulation_m1e_v1.json")
}

fn report(output: &Output) -> Value {
    serde_json::from_slice(&output.stdout).unwrap_or_else(|error| {
        panic!(
            "stdout was not a JSON report: {error}; stderr={}",
            String::from_utf8_lossy(&output.stderr)
        )
    })
}

fn stderr(output: &Output) -> String {
    String::from_utf8_lossy(&output.stderr).into_owned()
}

struct TestDirectory {
    path: PathBuf,
}

impl TestDirectory {
    fn new(label: &str) -> Self {
        static NEXT_ID: AtomicU64 = AtomicU64::new(0);
        let base = option_env!("CARGO_TARGET_TMPDIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/cli-test-tmp")
            });
        let path = base.join(format!(
            "pipe-observed-manipulation-{label}-{}-{}",
            std::process::id(),
            NEXT_ID.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&path).expect("create isolated CLI test directory");
        Self { path }
    }
}

impl Drop for TestDirectory {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.path);
    }
}

#[test]
fn nominal_process_replay_is_byte_identical_and_exits_zero() {
    let run = || {
        cli()
            .args(["--scenario"])
            .arg(baseline_scenario_path())
            .args(["--fault", "none", "--report", "-", "--compact"])
            .output()
            .expect("run observed-manipulation CLI")
    };

    let first = run();
    let second = run();

    assert_eq!(first.status.code(), Some(0), "{}", stderr(&first));
    assert_eq!(second.status.code(), Some(0), "{}", stderr(&second));
    assert_eq!(first.stdout, second.stdout);
    let value = report(&first);
    assert_eq!(value["status"], "complete");
    assert_eq!(value["expected_outcome_observed"], true);
    assert_eq!(
        value["controller_report_sha256"].as_str().map(str::len),
        Some(64)
    );
}

#[test]
fn expected_named_fault_is_a_zero_exit_failed_safe_report() {
    let output = cli()
        .args(["--fault", "optical_dropout", "--compact"])
        .output()
        .expect("run expected fault through observed-manipulation CLI");

    assert_eq!(output.status.code(), Some(0), "{}", stderr(&output));
    let value = report(&output);
    assert_eq!(value["status"], "failed_safe");
    assert_eq!(value["terminal_reason"], "optical_dropout");
    assert_eq!(value["expected_outcome_observed"], true);
}

#[test]
fn unexpected_nominal_outcome_exits_two_and_still_emits_json() {
    let directory = TestDirectory::new("unexpected-outcome");
    let scenario_path = directory.path.join("low-grip-limit.json");
    let mut scenario: Value = serde_json::from_slice(
        &fs::read(baseline_scenario_path()).expect("read baseline scenario"),
    )
    .expect("parse baseline scenario");
    scenario["grasp"]["maximum_grip_force_n"] = serde_json::json!(0.050);
    fs::write(
        &scenario_path,
        serde_json::to_vec_pretty(&scenario).expect("serialize modified scenario"),
    )
    .expect("write modified scenario");

    let output = cli()
        .args(["--scenario"])
        .arg(&scenario_path)
        .arg("--compact")
        .output()
        .expect("run unexpected nominal outcome");

    assert_eq!(output.status.code(), Some(2), "{}", stderr(&output));
    let value = report(&output);
    assert_eq!(value["status"], "failed_safe");
    assert_eq!(value["terminal_reason"], "contact_force_limit");
    assert_eq!(value["expected_outcome_observed"], false);
}

#[test]
fn scenario_and_report_may_not_resolve_to_the_same_file() {
    let directory = TestDirectory::new("same-file");
    let scenario_path = directory.path.join("scenario.json");
    let original = fs::read(baseline_scenario_path()).expect("read baseline scenario");
    fs::write(&scenario_path, &original).expect("copy baseline scenario");
    let aliased_report_path = directory.path.join(".").join("scenario.json");

    let output = cli()
        .arg("--scenario")
        .arg(&scenario_path)
        .arg("--report")
        .arg(&aliased_report_path)
        .arg("--compact")
        .output()
        .expect("run same-file guard");

    assert_eq!(output.status.code(), Some(1), "{}", stderr(&output));
    assert!(output.stdout.is_empty());
    assert!(stderr(&output).contains("scenario and report paths must differ"));
    assert_eq!(
        fs::read(&scenario_path).expect("scenario survives rejected invocation"),
        original
    );
}

#[test]
fn report_write_failure_preserves_stdout_json_and_exits_one() {
    let directory = TestDirectory::new("report-error");
    let unwritable_report = directory.path.join("report-is-a-directory");
    fs::create_dir(&unwritable_report).expect("create directory at report path");

    let output = cli()
        .args(["--fault", "optical_dropout", "--compact", "--report"])
        .arg(&unwritable_report)
        .output()
        .expect("run report-write failure");

    assert_eq!(output.status.code(), Some(1), "{}", stderr(&output));
    assert_eq!(report(&output)["expected_outcome_observed"], true);
    assert!(stderr(&output).contains("failed to write report"));
}

#[test]
fn successful_report_file_contains_the_stdout_json() {
    let directory = TestDirectory::new("report-copy");
    let report_path = directory.path.join("fault-report.json");

    let output = cli()
        .args(["--fault", "optical_dropout", "--compact", "--report"])
        .arg(&report_path)
        .output()
        .expect("run report-file output");

    assert_eq!(output.status.code(), Some(0), "{}", stderr(&output));
    assert_eq!(
        fs::read(&report_path).expect("read report file"),
        output.stdout
    );
}

#[test]
fn closed_stdout_is_a_handled_exit_one_not_a_panic() {
    let mut child = cli()
        .args(["--fault", "optical_dropout", "--compact"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn observed-manipulation CLI");
    drop(child.stdout.take().expect("take child's stdout pipe"));

    let output = child.wait_with_output().expect("wait for CLI");
    let error = stderr(&output);
    assert_eq!(output.status.code(), Some(1), "{error}");
    assert!(error.contains("failed to write stdout"), "{error}");
    assert!(!error.contains("panicked"), "{error}");
}
