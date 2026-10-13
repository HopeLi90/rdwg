//! Version-stamp routing — in particular: **does AutoCAD 2020 work?**
//!
//! AutoCAD 2020 writes the **AC1032** stamp. There is no 2020-specific DWG
//! format: Autodesk introduced the "DWG 2018 format" and then reused it
//! unchanged for AutoCAD 2018, 2019, **2020**, 2021, 2022, 2023 and 2024. So
//! "support CAD 2020" is exactly the same problem as "support AC1032", and
//! AC1032 is one of the stamps this parser already routes.
//!
//! Two containers, not one:
//!
//! ```text
//! AC1015  2000 / 2001 / 2002   flat sections
//! AC1018  2004 / 2005 / 2006   R2004 section tree ("AcFssFcAJMB")
//! AC1021  2007 / 2008 / 2009   R2007 page container (RS + LZ77)
//! AC1024  2010 / 2011 / 2012   R2004 section tree (reused)
//! AC1027  2013 … 2017          R2004 section tree (reused)
//! AC1032  2018 … 2024          R2004 section tree (reused)  <- CAD 2020
//! ```
//!
//! The page container is *not* universal: AC1021 is the only release that uses
//! it. Implementing the R2004 section tree therefore unlocked R2004, R2010,
//! R2013 and R2018–2024 in one go.
//!
//! The container is only half the job. Object *fields* are version-guarded too,
//! and the block that moves most is LibreDWG's `common_entity_data.spec`
//! (`is_xdic_missing` at R_2004a, `material_flags`/`shadow_flags` and the
//! per-object string stream at R_2007a, three visual-style bits at R_2010b,
//! `has_ds_data` at R_2013). Decoding every release with the AC1021 layout
//! silently produces plausible garbage, so `EntityVariant` carries the release
//! through the entity decoders.
//!
//! The tests below prove all of that with **real** fixtures of every release —
//! the same drawing re-saved by 2004/2007/2010/2013/2018 — instead of the
//! re-stamping trick this file used to rely on. Re-stamping an AC1021 file as
//! AC1032 no longer proves anything: the two stamps use different containers,
//! so it would fail for a reason that has nothing to do with the stamp.

use dwgparse::DwgFile;

/// The AC1021 fixture, kept for the "unknown stamps are rejected" case (which
/// only needs *some* well-formed file to corrupt) and as the reference the
/// other releases are compared against.
const SAMPLE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/example_2007.dwg");

/// One real drawing, saved by five different releases.
const SAMPLE_2004: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/example_2004.dwg");
const SAMPLE_2010: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/example_2010.dwg");
const SAMPLE_2013: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/example_2013.dwg");
const SAMPLE_2018: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/example_2018.dwg");

const FIXTURES: [(&str, &str); 5] = [
    ("AC1018", SAMPLE_2004),
    ("AC1021", SAMPLE),
    ("AC1024", SAMPLE_2010),
    ("AC1027", SAMPLE_2013),
    ("AC1032", SAMPLE_2018),
];

/// Everything a caller can observe about the *entities*, stringified.
///
/// `Geometry` deliberately does not derive `PartialEq` (it holds `f64`, where
/// `==` is the wrong tool and `NaN` breaks the equivalence relation), so
/// compare the `Debug` output instead: that prints each double with
/// shortest-round-trip precision, making this a **bit-for-bit** comparison, not
/// an approximate one.
///
/// Only geometry-bearing objects are compared, and the result is sorted: the
/// five fixtures are the same drawing re-saved by different releases, and each
/// release writes a different number of *non-entity* objects (dictionaries,
/// MLINESTYLEs, …) in a different order — 735/540/536/481/474 objects for
/// 2004/2007/2010/2013/2018.
///
/// String payloads are blanked out, because the drawing itself was edited
/// between saves: the 2007 file stores the MTEXT `108\U+00B0` while the 2018
/// file stores `108°` (LibreDWG's `dwg2dxf` export agrees). Coordinates, flags
/// and vertex counts are still compared bit-for-bit.
fn fingerprint(dwg: &DwgFile) -> Vec<String> {
    let mut out: Vec<String> = dwg
        .objects
        .iter()
        .filter(|o| o.geometry.is_some())
        .map(|o| scrub(&format!("{:?}|{:?}", o.type_name, o.geometry.as_ref().unwrap())))
        .collect();
    out.sort();
    out
}

/// Empty every `"…"` payload in a `Debug` rendering, leaving numbers and
/// variant names intact.
fn scrub(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars();
    let mut in_str = false;
    while let Some(c) = chars.next() {
        if c == '"' {
            in_str = !in_str;
            out.push('"');
        } else if in_str {
            if c == '\\' {
                chars.next();
            }
        } else {
            out.push(c);
        }
    }
    out
}

fn parse(path: &str) -> DwgFile {
    DwgFile::parse(path).unwrap_or_else(|e| panic!("{} failed to parse: {e}", path))
}

#[test]
fn ac1032_is_the_stamp_of_autocad_2020() {
    // Sanity-check the premise this whole file rests on: 2020 -> AC1032.
    //
    // Autodesk's DWG version history:
    //   AC1015  2000-2002      AC1021  2007-2009      AC1027  2013-2017
    //   AC1018  2004-2006      AC1024  2010-2012      AC1032  2018-2024
    //                                                (2020 is in here)
    //   AC1037  2024+          (a newer, different format — not supported)
    let dwg = parse(SAMPLE_2018);
    assert_eq!(dwg.version, "AC1032");
    assert!(dwg.object_count > 0, "an AC1032 drawing must yield objects");
}

#[test]
fn autocad_2018_decodes_the_same_drawing_as_ac1021() {
    // A *genuine* AC1032 drawing — the strongest available evidence that
    // "CAD 2020 support" is real, since AC1032 is what 2020 writes.
    let base = parse(SAMPLE);
    let r2018 = parse(SAMPLE_2018);

    assert_eq!(r2018.version, "AC1032");
    assert_eq!(r2018.layers.len(), base.layers.len(), "same layer table");
    assert_eq!(
        fingerprint(&r2018),
        fingerprint(&base),
        "geometry, text and layer bindings must be bit-identical across releases"
    );
}

#[test]
fn ac1018_uses_the_section_tree_container_and_parses() {
    // A *real* R2004 drawing. This used to be rejected with "uses the
    // 'AcFssFcAJMB' section-tree container, which this parser does not
    // implement"; the section tree is now decoded, and because AC1024/AC1027/
    // AC1032 reuse it, this is also the code path AutoCAD 2020 takes.
    let dwg = parse(SAMPLE_2004);
    assert_eq!(dwg.version, "AC1018");
    assert!(dwg.object_count > 0, "an AC1018 drawing must yield objects");
    // Layer names on R2004 are inline `TV`s on the field stream — there is no
    // per-object string stream below R2007, so an empty name here is the
    // regression this guards.
    assert!(
        dwg.layers.iter().any(|l| !l.name.is_empty()),
        "R2004 layer names must decode"
    );
}

#[test]
fn every_release_decodes_the_same_drawing() {
    // The same drawing saved by 2004, 2007, 2010, 2013 and 2018. The object
    // *fields* are version-guarded (see the module docs), so this is the test
    // that catches a mis-drawn `EntityVariant` boundary: get one bit wrong and
    // one release silently decodes garbage coordinates.
    let base = parse(SAMPLE);

    for (stamp, path) in FIXTURES {
        let dwg = parse(path);
        assert_eq!(dwg.version, stamp);
        assert_eq!(dwg.layers.len(), base.layers.len(), "{stamp}: layer table");
        assert_eq!(fingerprint(&dwg), fingerprint(&base), "{stamp} diverged");
    }
}

#[test]
fn unknown_stamps_are_still_rejected() {
    // AC1037 is AutoCAD 2024+, a genuinely newer format we have never verified;
    // guessing at it would silently produce garbage, so it must fail loudly.
    for stamp in ["AC1037", "AC1012", "AC1014", "AC1500", "GARBAG"] {
        let mut bytes = std::fs::read(SAMPLE).expect("fixture");
        bytes[0..6].copy_from_slice(stamp.as_bytes());
        let mut path = std::env::temp_dir();
        path.push(format!("dwgparse_reject_{stamp}.dwg"));
        std::fs::write(&path, &bytes).expect("write");

        let err = DwgFile::parse(path.to_str().unwrap())
            .expect_err("an unknown version stamp must be rejected");
        assert!(
            err.contains("unsupported DWG version"),
            "{stamp} was rejected for the wrong reason: {err}"
        );
        // The message must tell a 2020 user which stamps *are* accepted.
        assert!(err.contains("AC1032"), "error should name AC1032: {err}");

        let _ = std::fs::remove_file(&path);
    }
}
