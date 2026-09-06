use std::process::Command;

fn binary() -> Command {
    Command::new(env!("CARGO_BIN_EXE_pipe-metrology"))
}
fn revision() -> String {
    let output = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .output()
        .unwrap();
    assert!(output.status.success());
    String::from_utf8(output.stdout).unwrap().trim().to_string()
}

#[test]
fn metrology_cli_is_deterministic_and_does_not_turn_partial_coverage_into_success() {
    let revision = revision();
    let args = [
        "--source-revision",
        revision.as_str(),
        "--repeats",
        "4",
        "--summary",
    ];
    let first = binary().args(args).output().unwrap();
    assert!(
        first.status.success(),
        "{}",
        String::from_utf8_lossy(&first.stderr)
    );
    let second = binary().args(args).output().unwrap();
    assert_eq!(first.stdout, second.stdout);
    let report: serde_json::Value = serde_json::from_slice(&first.stdout).unwrap();
    assert_eq!(report["source_revision"], revision);
    assert_eq!(report["config_sha256"].as_str().unwrap().len(), 64);
    assert_eq!(report["sample_details_omitted"], true);
    assert!(report["structured"]["samples"].is_null());
    assert_eq!(
        report["structured"]["local_target_met_in_simulation"],
        false
    );
    assert!(
        report["structured"]["accepted"].as_u64().unwrap()
            < report["structured"]["attempted"].as_u64().unwrap()
    );
    assert!(report["accuracy_claim"]
        .as_str()
        .unwrap()
        .contains("no physical accuracy"));
    assert_eq!(
        report["configuration"]["calibration"]["provenance"],
        "synthetic_nominal"
    );
}

#[test]
fn cli_rejects_missing_revision_and_unknown_arguments() {
    for args in [vec![], vec!["--machine"], vec!["--invent-success"]] {
        let output = binary().args(args).output().unwrap();
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
    }
    let config = binary().arg("--print-config").output().unwrap();
    assert!(config.status.success());
    let config: serde_json::Value = serde_json::from_slice(&config.stdout).unwrap();
    assert_eq!(config["tube_id_m"], 0.1);
}
