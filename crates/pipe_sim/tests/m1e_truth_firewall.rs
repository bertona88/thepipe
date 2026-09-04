use pipe_sim::observed_manipulation::{
    M1eFault, ObservedManipulationReport, ObservedManipulationRuntime,
};

const CONTROLLER_SOURCE: &str = include_str!("../src/observed_manipulation/controller.rs");
const ESTIMATOR_SOURCE: &str = include_str!("../src/observed_manipulation/estimator.rs");
const PLANT_SOURCE: &str = include_str!("../src/observed_manipulation/plant.rs");
const RUNTIME_SOURCE: &str = include_str!("../src/observed_manipulation/runtime.rs");

/// Remove comments and string/byte-string/raw-string contents while
/// preserving byte positions and Rust punctuation. Firewall checks should be
/// driven by compiled source, not by documentation prose or diagnostic text.
fn code_only(source: &str) -> String {
    let bytes = source.as_bytes();
    let mut result = bytes.to_vec();
    let mut index = 0;

    while index < bytes.len() {
        if bytes[index..].starts_with(b"//") {
            let start = index;
            index += 2;
            while index < bytes.len() && bytes[index] != b'\n' {
                index += 1;
            }
            result[start..index].fill(b' ');
            continue;
        }
        if bytes[index..].starts_with(b"/*") {
            let start = index;
            index += 2;
            let mut depth = 1_u32;
            while index < bytes.len() && depth > 0 {
                if bytes[index..].starts_with(b"/*") {
                    depth += 1;
                    index += 2;
                } else if bytes[index..].starts_with(b"*/") {
                    depth -= 1;
                    index += 2;
                } else {
                    index += 1;
                }
            }
            assert_eq!(depth, 0, "unterminated block comment in Rust source");
            result[start..index].fill(b' ');
            continue;
        }

        if let Some((quote, hashes)) = raw_string_start(bytes, index) {
            let start = index;
            index = quote + 1;
            loop {
                assert!(
                    index < bytes.len(),
                    "unterminated raw string in Rust source"
                );
                if bytes[index] == b'"'
                    && bytes
                        .get(index + 1..index + 1 + hashes)
                        .is_some_and(|tail| tail.iter().all(|byte| *byte == b'#'))
                {
                    index += 1 + hashes;
                    break;
                }
                index += 1;
            }
            result[start..index].fill(b' ');
            continue;
        }

        let quote = if bytes[index] == b'"' {
            Some(index)
        } else if bytes[index] == b'b' && bytes.get(index + 1) == Some(&b'"') {
            Some(index + 1)
        } else {
            None
        };
        if let Some(quote) = quote {
            let start = index;
            index = quote + 1;
            let mut escaped = false;
            loop {
                assert!(index < bytes.len(), "unterminated string in Rust source");
                let byte = bytes[index];
                index += 1;
                if escaped {
                    escaped = false;
                } else if byte == b'\\' {
                    escaped = true;
                } else if byte == b'"' {
                    break;
                }
            }
            result[start..index].fill(b' ');
            continue;
        }

        index += 1;
    }

    String::from_utf8(result).expect("blanking literal/comment bytes preserves UTF-8")
}

fn raw_string_start(bytes: &[u8], start: usize) -> Option<(usize, usize)> {
    let mut index = start;
    if bytes.get(index) == Some(&b'b') {
        index += 1;
    }
    if bytes.get(index) != Some(&b'r') {
        return None;
    }
    index += 1;
    let hash_start = index;
    while bytes.get(index) == Some(&b'#') {
        index += 1;
    }
    (bytes.get(index) == Some(&b'"')).then_some((index, index - hash_start))
}

fn identifier_count(source: &str, identifier: &str) -> usize {
    source
        .split(|character: char| !(character.is_ascii_alphanumeric() || character == '_'))
        .filter(|token| *token == identifier)
        .count()
}

fn compact(source: &str) -> String {
    source
        .chars()
        .filter(|character| !character.is_ascii_whitespace())
        .collect()
}

fn function_item<'a>(source: &'a str, name: &str) -> &'a str {
    let signature = format!("fn {name}");
    let function_start = source
        .find(&signature)
        .unwrap_or_else(|| panic!("missing function {name}"));
    let body_start = source[function_start..]
        .find('{')
        .map(|offset| function_start + offset)
        .unwrap_or_else(|| panic!("missing body for function {name}"));
    let mut depth = 0_u32;
    for (offset, byte) in source.as_bytes()[body_start..].iter().enumerate() {
        match byte {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return &source[function_start..=body_start + offset];
                }
            }
            _ => {}
        }
    }
    panic!("unterminated body for function {name}");
}

fn controller_transcript(report: &ObservedManipulationReport) -> serde_json::Value {
    let mut value = serde_json::to_value(report).expect("report serializes");
    value
        .as_object_mut()
        .expect("report serializes as an object")
        .remove(&["evaluation", "only", "truth"].join("_"));
    value
}

#[test]
fn controller_and_estimator_cannot_name_truth_owning_types_or_the_plant_module() {
    let forbidden_identifiers = [
        ["pipe", "sim", "core"].join("_"),
        ["Observed", "Plant"].concat(),
        ["Plant", "Evaluation", "Metrics"].concat(),
        ["Simu", "lation"].concat(),
        ["Rigid", "Body"].concat(),
        ["Scene", "Frame"].concat(),
        ["Depth", "Sample"].concat(),
        ["evaluation", "metrics"].join("_"),
        ["truth", "ring", "center", "points"].join("_"),
    ];

    for (module, source) in [
        ("controller", CONTROLLER_SOURCE),
        ("estimator", ESTIMATOR_SOURCE),
    ] {
        let code = code_only(source);
        for forbidden in &forbidden_identifiers {
            assert_eq!(
                identifier_count(&code, forbidden),
                0,
                "{module} compiled source names truth-owned identifier {forbidden}"
            );
        }
        assert!(
            !compact(&code).contains("super::plant"),
            "{module} compiled source imports the private plant module"
        );
    }
}

#[test]
fn plant_exports_raw_contact_packets_not_controller_contact_verdicts() {
    let code = code_only(PLANT_SOURCE);
    let raw_packet = ["Contact", "Packet"].concat();
    assert!(
        identifier_count(&code, &raw_packet) > 0,
        "plant must retain a raw hardware-plausible contact boundary"
    );

    for forbidden in [
        ["Contact", "State"].concat(),
        ["Classified", "Contact", "Evidence"].concat(),
    ] {
        assert_eq!(
            identifier_count(&code, &forbidden),
            0,
            "plant compiled source publishes controller-semantic verdict {forbidden}"
        );
    }
}

#[test]
fn runtime_truth_query_is_unique_terminal_and_excluded_from_control_gates_and_hash() {
    let code = code_only(RUNTIME_SOURCE);
    let evaluation_query = ["evaluation", "metrics"].join("_");
    assert_eq!(
        identifier_count(&code, &evaluation_query),
        1,
        "runtime may contain exactly one post-run plant truth query"
    );

    let terminal_name = ["terminal", "evaluation"].join("_");
    let terminal_item = function_item(&code, &terminal_name);
    assert_eq!(identifier_count(terminal_item, &evaluation_query), 1);
    let terminal = compact(terminal_item);
    let guard = terminal
        .find("if!self.is_terminal(){returnNone;}")
        .expect("terminal evaluation must return before a nonterminal truth query");
    let truth_call = terminal
        .find(&format!("self.plant.{evaluation_query}()"))
        .expect("terminal evaluation must use the single explicit truth query");
    assert!(
        guard < truth_call,
        "terminal guard must precede truth access"
    );

    assert_eq!(
        identifier_count(&code, &terminal_name),
        2,
        "terminal evaluation must have one definition and one report-only call site"
    );
    let report_item = function_item(&code, "report");
    assert_eq!(identifier_count(report_item, &terminal_name), 1);
    let report = compact(report_item);
    assert!(report.contains(&format!(
        "{}:self.{terminal_name}()",
        ["evaluation", "only", "truth"].join("_")
    )));

    for function in ["acceptance_gates", "controller_hash"] {
        let item = function_item(&code, function);
        assert_eq!(
            identifier_count(item, &evaluation_query),
            0,
            "{function} must not consume post-run truth"
        );
        assert!(
            !compact(item).contains("self.plant"),
            "{function} must not query the plant"
        );
    }

    for forbidden in [
        ["pipe", "sim", "core"].join("_"),
        ["socket", "pose"].join("_"),
        ["peg", "body"].join("_"),
        ["physical", "tool", "pose"].join("_"),
        ["active", "arm"].join("_"),
        ["truth", "ring", "center", "points"].join("_"),
        ["private", "insertion", "contact"].join("_"),
        ["private", "jaw", "contact", "channels"].join("_"),
        ["initial", "peg", "error", "m"].join("_"),
        ["initial", "socket", "error", "m"].join("_"),
        ["initial", "tool", "command", "error", "m"].join("_"),
        ["initial", "peg", "axis", "tilt", "rad"].join("_"),
        ["initial", "socket", "axis", "tilt", "rad"].join("_"),
        ["initial", "tool", "axis", "tilt", "rad"].join("_"),
    ] {
        assert_eq!(
            identifier_count(&code, &forbidden),
            0,
            "runtime compiled source bypasses its sanitized boundary via {forbidden}"
        );
    }
}

#[test]
fn seeded_controller_transcript_replays_for_success_and_fail_closed_paths() {
    for fault in [M1eFault::None, M1eFault::InconsistentObservation] {
        let mut first_runtime = ObservedManipulationRuntime::new(fault).unwrap();
        let mut second_runtime = ObservedManipulationRuntime::new(fault).unwrap();
        let first = first_runtime.run_cycle().unwrap();
        let second = second_runtime.run_cycle().unwrap();

        assert_eq!(
            controller_transcript(&first),
            controller_transcript(&second),
            "controller transcript changed for seeded {} replay",
            fault.id()
        );
        assert_eq!(
            first.controller_report_sha256,
            second.controller_report_sha256,
            "controller report hash changed for seeded {} replay",
            fault.id()
        );
        assert_eq!(first.truth_firewall.controller_truth_access_count, 0);
        assert_eq!(second.truth_firewall.controller_truth_access_count, 0);
    }
}
