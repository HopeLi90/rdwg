//! End-to-end tests for the `dwgparse` **binary** — the outermost layer.
//!
//! Everything else in the suite exercises the library. These tests drive the
//! compiled executable the way a user does, which is the only way to catch
//! regressions in argument handling, exit codes, and which stream the
//! diagnostics go to (`stdout` for the report, `stderr` for errors).
//!
//! `env!("CARGO_BIN_EXE_dwgparse")` is the path cargo built for this target,
//! so no hard-coded `target/debug/...` guesswork is involved.

use std::process::Command;

const BIN: &str = env!("CARGO_BIN_EXE_dwgparse");

const SAMPLE_2000: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/sample_2000.dwg");
const SAMPLE_2004: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/example_2004.dwg");
const SAMPLE_2007: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/example_2007.dwg");

fn run(args: &[&str]) -> (i32, String, String) {
    let out = Command::new(BIN)
        .args(args)
        .output()
        .expect("the dwgparse binary must be runnable");
    (
        out.status.code().expect("binary exited with a signal"),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// The report has one section per concern; all of them must be present, and
/// none of them may be the "(none)" placeholder.
fn assert_full_report(stdout: &str, version: &str) {
    assert!(
        stdout.contains(&format!("version        : {version}")),
        "missing version line for {version}:\n{stdout}"
    );
    for section in [
        "--- Layers (table records) ---",
        "--- Blocks (BLOCK_HEADER records) ---",
        "--- Entity geometry ---",
        "--- Extended attributes",
    ] {
        assert!(stdout.contains(section), "missing section {section}:\n{stdout}");
    }
    assert!(
        stdout.contains("object count   : "),
        "missing object count:\n{stdout}"
    );
    // A drawing that parses must not report empty tables.
    assert!(
        !stdout.contains("--- Layers (table records) ---\n(none)"),
        "layer table came out empty:\n{stdout}"
    );
    assert!(
        !stdout.contains("--- Entity geometry ---\n(none)"),
        "no geometry was printed:\n{stdout}"
    );
}

#[test]
fn cli_reports_a_full_summary_for_ac1015() {
    let (code, stdout, stderr) = run(&[SAMPLE_2000]);
    assert_eq!(code, 0, "stderr: {stderr}");
    assert_full_report(&stdout, "AC1015");
}

#[test]
fn cli_reports_a_full_summary_for_ac1021() {
    let (code, stdout, stderr) = run(&[SAMPLE_2007]);
    assert_eq!(code, 0, "stderr: {stderr}");
    assert_full_report(&stdout, "AC1021");
}

#[test]
fn cli_reports_a_full_summary_for_ac1018() {
    let (code, stdout, stderr) = run(&[SAMPLE_2004]);
    assert_eq!(code, 0, "stderr: {stderr}");
    assert_full_report(&stdout, "AC1018");
}

/// AC1018 is the R2004 section-tree container. AC1024/AC1027/AC1032 (R2010,
/// R2013, R2018…2024, incl. CAD2020) *reuse that same container*, so the page
/// tree, XOR header and ACD sections must all decode for every member of the
/// family — the file is no longer rejected on a wrong "unsupported container"
/// guess. The object stream these containers hand back reuses the R2007 object
/// encoder with one R2010+ header change (a `UMC` handle-stream size plus a
/// `BOT` type instead of `BS`/`RL`), which the decoder is now version-gated
/// for, so every member of the family decodes real objects end to end.
#[test]
fn cli_opens_the_r2004_container_family() {
    let targets: &[(&str, &str)] = &[
        ("AC1024", "tests/data/example_2010.dwg"),
        ("AC1027", "tests/data/example_2013.dwg"),
        ("AC1032", "tests/data/example_2018.dwg"),
    ];
    for (version, path) in targets {
        let full = concat!(env!("CARGO_MANIFEST_DIR"), "/");
        let (code, stdout, stderr) = run(&[&(full.to_owned() + path)]);
        assert_eq!(code, 0, "{version} container should open ({stderr})");
        assert!(
            stdout.contains(&format!("version        : {version}")),
            "{version} version line missing:\n{stdout}"
        );
        // The container yields real section bytes and the version-aware object
        // decoder turns them into objects; an empty/undecoded stream would
        // surface as an error rather than a clean exit.
        assert!(
            !stderr.contains("no objects") && stdout.contains("object count"),
            "{version} reported no objects after container decode:\n{stderr}"
        );
    }
}

#[test]
fn cli_rejects_an_unknown_version_stamp() {
    let mut bytes = std::fs::read(SAMPLE_2007).expect("fixture");
    bytes[0..6].copy_from_slice(b"AC1037");
    let mut path = std::env::temp_dir();
    path.push(format!("dwgparse_cli_ac1037_{}.dwg", std::process::id()));
    std::fs::write(&path, &bytes).expect("write");

    let (code, _, stderr) = run(&[path.to_str().unwrap()]);
    let _ = std::fs::remove_file(&path);

    assert_eq!(code, 1);
    assert!(stderr.contains("AC1037"), "stderr: {stderr}");
    assert!(
        stderr.contains("AC1032"),
        "the error should list AC1032 as supported so a 2020 user is not stuck: {stderr}"
    );
}

#[test]
fn cli_reports_a_missing_file() {
    let (code, _, stderr) = run(&["/nonexistent/path/to/nowhere.dwg"]);
    assert_eq!(code, 1);
    assert!(
        stderr.contains("cannot read"),
        "expected a read error, got: {stderr}"
    );
}

#[test]
fn cli_output_is_byte_identical_across_runs() {
    // Guards against iteration-order dependence leaking into the report (e.g.
    // printing straight out of a HashMap).
    for path in [SAMPLE_2000, SAMPLE_2007] {
        let (c1, o1, _) = run(&[path]);
        let (c2, o2, _) = run(&[path]);
        assert_eq!(c1, c2);
        assert_eq!(o1, o2, "CLI output is not deterministic for {path}");
    }
}
