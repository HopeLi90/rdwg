//! Cross-layer invariants — the properties that must hold for *every* file the
//! parser accepts, asserted against **both** supported dialects.
//!
//! This is the "vertical" half of the joint test matrix: each assertion spans
//! several layers of the stack at once, so a break can be localised by *which*
//! invariant failed.
//!
//! | invariant | layers it ties together |
//! |---|---|
//! | `object_count == objects.len()` | object table → public metadata |
//! | handles unique, `index` matches position | handle stream → object table |
//! | `object_by_handle` round-trips every handle | handle index → object table |
//! | `entities().count() == geometry.is_some()` count | object decoding → public API |
//! | every coordinate finite and in range | field stream (`BD`/`RD`/`DD`) → geometry |
//! | radius / text height positive | field decoding → semantics |
//! | `layer_handle` resolves, `layer` agrees | object → layer table → API |
//! | POLYLINE vertices non-empty, bulges aligned | handle vector → vertex assembly → geometry |
//! | no `VERTEX_*` leaks into `entities()` | assembly → API |
//! | parsing twice is bit-identical | whole stack (determinism) |
//!
//! "Horizontal" coverage comes from running the *same* checks over the R2000
//! and the R2007 fixture: a regression that only affects one dialect shows up
//! as one failing test, not a silent behaviour change.

use std::collections::{HashMap, HashSet};

use dwgparse::{DwgFile, Geometry};

const SAMPLES: [(&str, &str); 2] = [
    (
        "AC1015/R2000",
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/sample_2000.dwg"),
    ),
    (
        "AC1021/R2007",
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/example_2007.dwg"),
    ),
];

/// Coordinates come out of `BD` / `RD` / `DD` fields. A field-order or
/// bit-offset mistake almost never produces a plausible wrong number — it
/// produces `NaN`, an infinity, or something absurd like 1e300. Bounding the
/// magnitude is what turns "it didn't crash" into "it decoded correctly".
const COORD_LIMIT: f64 = 1e12;

fn finite(v: f64, field: &str, ctx: &str) {
    assert!(v.is_finite(), "{ctx}: {field} is not finite ({v})");
}

fn bounded(v: f64, field: &str, ctx: &str) {
    finite(v, field, ctx);
    assert!(
        v.abs() <= COORD_LIMIT,
        "{ctx}: {field} = {v} is outside any plausible drawing extent"
    );
}

fn bounded_pt(p: (f64, f64, f64), field: &str, ctx: &str) {
    bounded(p.0, &format!("{field}.x"), ctx);
    bounded(p.1, &format!("{field}.y"), ctx);
    bounded(p.2, &format!("{field}.z"), ctx);
}

fn check_all(dwg: &DwgFile, label: &str) {
    // --- object table -------------------------------------------------------
    assert!(!dwg.objects.is_empty(), "{label}: no objects decoded");
    assert_eq!(
        dwg.object_count,
        dwg.objects.len(),
        "{label}: object_count disagrees with the object table"
    );
    assert!(!dwg.layers.is_empty(), "{label}: no layers decoded");

    // --- handle stream ------------------------------------------------------
    let mut handles: HashSet<u64> = HashSet::new();
    for (i, o) in dwg.objects.iter().enumerate() {
        assert_eq!(o.index, i, "{label}: object index does not match position");
        assert!(
            handles.insert(o.handle),
            "{label}: duplicate handle 0x{:x} (index {i}, type {})",
            o.handle,
            o.type_name
        );
        // Every indexed handle must round-trip back to the same object.
        let back = dwg
            .object_by_handle(o.handle)
            .unwrap_or_else(|| panic!("{label}: handle 0x{:x} does not resolve", o.handle));
        assert_eq!(back.index, i, "{label}: handle index points at the wrong object");
    }

    // --- layer table --------------------------------------------------------
    let mut names: HashMap<&str, u64> = HashMap::new();
    for l in &dwg.layers {
        if let Some(prev) = names.insert(l.name.as_str(), l.handle) {
            assert_eq!(
                prev, l.handle,
                "{label}: duplicate layer name {:?} with two handles",
                l.name
            );
        }
        assert!(
            dwg.layer_by_name(&l.name).is_some(),
            "{label}: layer {:?} not findable by name",
            l.name
        );
    }

    // --- entity projection --------------------------------------------------
    let with_geometry = dwg.objects.iter().filter(|o| o.geometry.is_some()).count();
    assert_eq!(
        dwg.entities().count(),
        with_geometry,
        "{label}: entities() disagrees with the number of decoded geometries"
    );
    assert!(with_geometry > 0, "{label}: no geometry at all was decoded");

    for o in dwg.objects.iter().filter(|o| o.geometry.is_some()) {
        let ctx = format!("{label} #{} {}", o.index, o.type_name);

        // Layer binding must be internally consistent.
        if let Some(h) = o.layer_handle {
            assert!(
                dwg.layer_name(h).is_some(),
                "{ctx}: layer_handle 0x{h:x} does not resolve to a layer"
            );
        }
        if let Some(l) = &o.layer {
            assert_eq!(
                Some(l.handle),
                o.layer_handle,
                "{ctx}: cached layer disagrees with layer_handle"
            );
        }

        match o.geometry.as_ref().unwrap() {
            Geometry::Line { start, end } => {
                bounded_pt(*start, "start", &ctx);
                bounded_pt(*end, "end", &ctx);
            }
            Geometry::Circle { center, radius } => {
                bounded_pt(*center, "center", &ctx);
                bounded(*radius, "radius", &ctx);
                assert!(*radius > 0.0, "{ctx}: circle radius is not positive ({radius})");
            }
            Geometry::Arc {
                center,
                radius,
                start_angle,
                end_angle,
            } => {
                bounded_pt(*center, "center", &ctx);
                bounded(*radius, "radius", &ctx);
                assert!(*radius > 0.0, "{ctx}: arc radius is not positive ({radius})");
                bounded(*start_angle, "start_angle", &ctx);
                bounded(*end_angle, "end_angle", &ctx);
            }
            Geometry::Point { x, y, z } => {
                bounded(*x, "x", &ctx);
                bounded(*y, "y", &ctx);
                bounded(*z, "z", &ctx);
            }
            Geometry::LwPolyline { points, .. } => {
                assert!(!points.is_empty(), "{ctx}: LWPOLYLINE with no vertices");
                for (i, (x, y)) in points.iter().enumerate() {
                    bounded(*x, &format!("pt[{i}].x"), &ctx);
                    bounded(*y, &format!("pt[{i}].y"), &ctx);
                }
            }
            Geometry::Text {
                ins_pt,
                height,
                value,
            } => {
                bounded(ins_pt.0, "ins_pt.x", &ctx);
                bounded(ins_pt.1, "ins_pt.y", &ctx);
                bounded(*height, "height", &ctx);
                assert!(*height > 0.0, "{ctx}: text height is not positive ({height})");
                assert!(!value.is_empty(), "{ctx}: TEXT with empty value");
            }
            Geometry::MText {
                ins_pt,
                height,
                value,
            } => {
                bounded_pt(*ins_pt, "ins_pt", &ctx);
                bounded(*height, "height", &ctx);
                assert!(*height > 0.0, "{ctx}: mtext height is not positive ({height})");
                assert!(!value.is_empty(), "{ctx}: MTEXT with empty value");
            }
            Geometry::Insert {
                ins_pt,
                block_name,
                scale,
            } => {
                bounded_pt(*ins_pt, "ins_pt", &ctx);
                bounded(scale.0, "scale.x", &ctx);
                bounded(scale.1, "scale.y", &ctx);
                bounded(scale.2, "scale.z", &ctx);
                assert!(!block_name.is_empty(), "{ctx}: INSERT with empty block name");
            }
            Geometry::Polyline {
                vertices, bulges, ..
            } => {
                // The whole point of handle-vector assembly: a POLYLINE that
                // came out with zero vertices is a failed assembly, not an
                // empty polyline.
                assert!(!vertices.is_empty(), "{ctx}: POLYLINE with no vertices");
                assert_eq!(
                    vertices.len(),
                    bulges.len(),
                    "{ctx}: bulges are not 1:1 with vertices"
                );
                for (i, p) in vertices.iter().enumerate() {
                    bounded_pt(*p, &format!("vertex[{i}]"), &ctx);
                }
                for (i, b) in bulges.iter().enumerate() {
                    finite(*b, &format!("bulge[{i}]"), &ctx);
                }
            }
            Geometry::Ellipse {
                center,
                major_axis,
                axis_ratio,
                start_angle,
                end_angle,
            } => {
                bounded_pt(*center, "center", &ctx);
                bounded_pt(*major_axis, "major_axis", &ctx);
                bounded(*axis_ratio, "axis_ratio", &ctx);
                bounded(*start_angle, "start_angle", &ctx);
                bounded(*end_angle, "end_angle", &ctx);
                // A degenerate axis vector means the field order slipped.
                let len = (major_axis.0.powi(2) + major_axis.1.powi(2) + major_axis.2.powi(2)).sqrt();
                assert!(len > 0.0, "{ctx}: ellipse major axis has zero length");
            }
            Geometry::Face3d { corners, .. } => {
                for (i, c) in corners.iter().enumerate() {
                    bounded_pt(*c, &format!("corner[{i}]"), &ctx);
                }
            }
            Geometry::Solid { corners, elevation, .. } => {
                for (i, c) in corners.iter().enumerate() {
                    bounded_pt(*c, &format!("corner[{i}]"), &ctx);
                }
                bounded(*elevation, "elevation", &ctx);
            }
        }
    }

    // --- POLYLINE assembly --------------------------------------------------
    for o in &dwg.objects {
        let Some(pl) = &o.polyline else { continue };
        let ctx = format!("{label} #{} {}", o.index, o.type_name);
        let Some(Geometry::Polyline { vertices, .. }) = &o.geometry else {
            panic!("{ctx}: has a polyline header but no Polyline geometry");
        };
        // The handle vector also carries `VERTEX_PFACE_FACE` records (which are
        // index lists, not points), so it may legitimately be longer.
        assert!(
            pl.vertex_handles.len() >= vertices.len(),
            "{ctx}: {} vertex handles but {} assembled vertices",
            pl.vertex_handles.len(),
            vertices.len()
        );
        // Every referenced vertex handle must actually exist.
        for h in &pl.vertex_handles {
            assert!(
                dwg.object_by_handle(*h).is_some(),
                "{ctx}: vertex handle 0x{h:x} does not resolve"
            );
        }
    }

    // Vertices are consumed by their parent polyline; leaking them as
    // standalone entities would double-count them in every downstream consumer.
    assert_eq!(
        dwg.entities().filter(|o| o.type_name.starts_with("VERTEX")).count(),
        0,
        "{label}: VERTEX objects must not appear in entities()"
    );
    assert_eq!(
        dwg.entities().filter(|o| o.type_name == "SEQEND").count(),
        0,
        "{label}: SEQEND must not appear in entities()"
    );

    // --- layer-scoped queries ----------------------------------------------
    let mut on_layers = 0usize;
    for l in &dwg.layers {
        on_layers += dwg.entities_on_layer(l).count();
    }
    assert!(
        on_layers <= dwg.entities().count(),
        "{label}: entities_on_layer counted {on_layers} over {} entities",
        dwg.entities().count()
    );
}

#[test]
fn invariants_hold_for_ac1015_r2000() {
    let (label, path) = &SAMPLES[0];
    let dwg = DwgFile::parse(path).expect("sample_2000.dwg must parse");
    check_all(&dwg, label);
}

#[test]
fn invariants_hold_for_ac1021_r2007() {
    let (label, path) = &SAMPLES[1];
    let dwg = DwgFile::parse(path).expect("example_2007.dwg must parse");
    check_all(&dwg, label);
}

/// Both dialects must expose the same *shape* of results: the same invariant
/// suite passes on both, so a dialect-specific regression cannot hide.
#[test]
fn both_dialects_produce_comparable_shapes() {
    let parsed: Vec<(&str, DwgFile)> = SAMPLES
        .iter()
        .map(|(label, path)| {
            (
                *label,
                DwgFile::parse(path).unwrap_or_else(|e| panic!("{label}: {e}")),
            )
        })
        .collect();

    for (label, dwg) in &parsed {
        assert!(!dwg.layers.is_empty(), "{label}: no layers");
        assert!(dwg.entities().count() > 0, "{label}: no entities");
        // At least one of the basic entity types every real drawing has.
        let kinds: HashSet<&str> = dwg.entities().map(|o| o.type_name.as_str()).collect();
        assert!(
            kinds.contains("LINE") || kinds.contains("LWPOLYLINE") || kinds.contains("POLYLINE_2D"),
            "{label}: neither LINE nor any polyline decoded: {kinds:?}"
        );
    }
}

/// Parsing is a pure function of the file: no interior mutability, no
/// iteration-order dependence, no reliance on `HashMap` ordering.
#[test]
fn parsing_is_deterministic() {
    for (label, path) in SAMPLES {
        let a = DwgFile::parse(path).expect("parse");
        let b = DwgFile::parse(path).expect("parse");
        let fp = |d: &DwgFile| -> Vec<String> {
            d.objects
                .iter()
                .map(|o| format!("{:?}|{:?}|{:?}|{:?}", o.handle, o.type_name, o.geometry, o.text))
                .collect()
        };
        assert_eq!(fp(&a), fp(&b), "{label}: two parses of the same file diverged");
        assert_eq!(a.layers.len(), b.layers.len(), "{label}: layer count diverged");
    }
}
