//! End-to-end tests for the extended entity coverage on `example_2007.dwg`
//! (AC1021 / R2007): classic POLYLINE (+ VERTEX assembly), ELLIPSE, 3DFACE,
//! SOLID / TRACE.
//!
//! Every numeric expectation below was produced by the parser and then
//! cross-checked field-by-field (max coordinate difference < 5e-11) against the
//! equivalent ground-truth DXF export (`/tmp/truth_2007.dxf`, generated with
//! `dwg2dxf`). So these assertions are not guesses — they lock geometrically
//! correct decoding, not just "it didn't crash".

use dwgparse::{DwgFile, Geometry};

const SAMPLE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/example_2007.dwg");

/// Loose-but-tight tolerance: the binary and the DXF both store IEEE-754
/// doubles of the same source values, so the true difference is ~1e-11. 1e-6
/// still catches any regression that shifts a coordinate by more than a handful
/// of ULPs without being so tight that harmless rounding in the hardcoded
/// literals trips the test.
const TOL: f64 = 1e-6;

fn approx(a: f64, b: f64) -> bool {
    (a - b).abs() < TOL
}

fn approx_pt(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
    approx(a.0, b.0) && approx(a.1, b.1) && approx(a.2, b.2)
}

#[test]
fn version_routing_r2007() {
    let dwg = DwgFile::parse(SAMPLE).expect("example_2007 should parse");
    // AC1021 == R2007; routed through the R2007 (page-container) decoder.
    assert_eq!(dwg.version, "AC1021");
    assert!(dwg.object_count > 200, "expected a sizeable object table");
}

#[test]
fn decodes_ellipse_geometry() {
    let dwg = DwgFile::parse(SAMPLE).expect("example_2007 should parse");
    let ellipses: Vec<_> = dwg.entities().filter(|o| o.type_name == "ELLIPSE").collect();
    assert_eq!(ellipses.len(), 1, "fixture has exactly one ELLIPSE");

    match &ellipses[0].geometry {
        Some(Geometry::Ellipse {
            center,
            major_axis,
            axis_ratio,
            start_angle,
            end_angle,
        }) => {
            assert!(approx_pt(*center, (3532.1668210256, 2904.2609165161, 0.0)), "center = {center:?}");
            assert!(approx_pt(*major_axis, (-1220.8656432199, 355.8017603696, 0.0)), "major = {major_axis:?}");
            assert!(approx(*axis_ratio, 0.5120328286), "ratio = {axis_ratio}");
            assert!(approx(*start_angle, 0.0), "start = {start_angle}");
            assert!(approx(*end_angle, 6.2831853072), "end = {end_angle}");
        }
        other => panic!("expected Ellipse, got {:?}", other),
    }
}

#[test]
fn decodes_two_3d_faces() {
    let dwg = DwgFile::parse(SAMPLE).expect("example_2007 should parse");
    let faces: Vec<_> = dwg.entities().filter(|o| o.type_name == "3DFACE").collect();
    assert_eq!(faces.len(), 2, "fixture has exactly two 3DFACE entities");

    let mut seen_flat = false;   // #208: all z == 0
    let mut seen_raised = false; // #209: all z == 1
    for f in &faces {
        match &f.geometry {
            Some(Geometry::Face3d { corners, .. }) => {
                // The two faces are distinguished by their z elevation.
                let z0 = corners[0].2;
                if approx(z0, 0.0) {
                    assert!(approx_pt(corners[0], (-2199.2082707151, 10997.2517870047, 0.0)));
                    assert!(approx_pt(corners[1], (-1219.8990846747, 12535.4347443136, 0.0)));
                    assert!(approx_pt(corners[2], (-54.0547453293, 11789.6490908331, 0.0)));
                    assert!(approx_pt(corners[3], (-1359.8003969662, 10344.6892730587, 0.0)));
                    seen_flat = true;
                } else if approx(z0, 1.0) {
                    assert!(approx_pt(corners[0], (-54.0547453293, 11789.6490908331, 1.0)));
                    assert!(approx_pt(corners[1], (-1359.8003969662, 10344.6892730587, 1.0)));
                    assert!(approx_pt(corners[2], (-473.7587524535, 10344.6892730587, 1.0)));
                    assert!(approx_pt(corners[3], (-54.0547453293, 11789.6490908331, 1.0)));
                    seen_raised = true;
                } else {
                    panic!("unexpected 3DFACE z elevation: {}", z0);
                }
            }
            other => panic!("expected Face3d, got {:?}", other),
        }
    }
    assert!(seen_flat && seen_raised, "both 3DFACE elevations must appear");
}

#[test]
fn decodes_fifteen_solids() {
    let dwg = DwgFile::parse(SAMPLE).expect("example_2007 should parse");
    let solids: Vec<_> = dwg.entities().filter(|o| o.type_name == "SOLID").collect();
    assert_eq!(solids.len(), 15, "fixture has exactly 15 SOLID entities");

    // Pick the SOLID whose first corner is (622.8621..., 10028.767...) and verify
    // all four corners (planar, z == elevation).
    let target = solids
        .iter()
        .find(|s| match &s.geometry {
            Some(Geometry::Solid { corners, .. }) => approx_pt(corners[0], (622.8621358549, 10028.7673513476, 0.0)),
            _ => false,
        })
        .expect("the reference SOLID must be present");
    match &target.geometry {
        Some(Geometry::Solid { corners, elevation }) => {
            assert!(approx(*elevation, 0.0));
            assert!(approx_pt(corners[0], (622.8621358549, 10028.7673513476, 0.0)));
            assert!(approx_pt(corners[1], (623.6623947804, 10028.5349075982, 0.0)));
            assert!(approx_pt(corners[2], (622.5649340695, 10026.2503526965, 0.0)));
            assert!(approx_pt(corners[3], (622.5649340695, 10026.2503526965, 0.0)));
            // SOLID is a triangle here: corners 3 and 4 coincide.
            assert!(approx_pt(corners[2], corners[3]), "triangle: c3 == c4");
        }
        other => panic!("expected Solid, got {:?}", other),
    }
}

#[test]
fn assembles_polyline_3d_vertices() {
    let dwg = DwgFile::parse(SAMPLE).expect("example_2007 should parse");
    let pl: Vec<_> = dwg.entities().filter(|o| o.type_name == "POLYLINE_3D").collect();
    assert_eq!(pl.len(), 1, "fixture has exactly one 3D polyline");

    match &pl[0].geometry {
        Some(Geometry::Polyline { is_3d, vertices, .. }) => {
            assert!(*is_3d, "this polyline must be flagged 3D");
            assert_eq!(vertices.len(), 4, "four VERTEX_3D must be stitched on");
            // The assembled vertex chain must include the vertex at z = 5.0
            // (proves handle-vector assembly, not just sequential grouping).
            let has_z5 = vertices.iter().any(|v| approx(v.2, 5.0));
            assert!(has_z5, "3D polyline must contain the z=5.0 vertex: {vertices:?}");
            // And the specific 4 vertices, in order.
            assert!(approx_pt(vertices[2], (4263.0089829732, 5198.7767139096, 5.0)));
        }
        other => panic!("expected Polyline, got {:?}", other),
    }
}

#[test]
fn assembles_polyface_mesh_vertices() {
    let dwg = DwgFile::parse(SAMPLE).expect("example_2007 should parse");
    let pf: Vec<_> = dwg.entities().filter(|o| o.type_name == "POLYLINE_PFACE").collect();
    assert_eq!(pf.len(), 1, "fixture has exactly one polyface mesh");

    match &pf[0].geometry {
        Some(Geometry::Polyline { is_3d, vertices, .. }) => {
            assert!(!*is_3d, "polyface mesh is planar (z from elevation)");
            assert_eq!(vertices.len(), 6, "six VERTEX_PFACE points must be stitched on");
            // The VERTEX_PFACE_FACE (0x0e) records carry only indices, no point;
            // they must NOT appear as vertices.
            assert!(vertices.iter().all(|v| !(approx(v.0, 0.0) && approx(v.1, 0.0) && approx(v.2, 0.0))),
                    "face-index records must be excluded: {vertices:?}");
            assert!(approx_pt(vertices[0], (7589.9073116575, 3459.3382354665, 0.0)));
            assert!(approx_pt(vertices[5], (8160.6464484093, 4219.9035582767, 0.0)));
        }
        other => panic!("expected Polyline, got {:?}", other),
    }
}

#[test]
fn decodes_eighteen_mtexts() {
    let dwg = DwgFile::parse(SAMPLE).expect("example_2007 should parse");
    let mtexts: Vec<_> = dwg.entities().filter(|o| o.type_name == "MTEXT").collect();
    assert_eq!(mtexts.len(), 18, "fixture has exactly 18 MTEXT entities");

    // Find the MTEXT at (830.416..., 10749.311...) and verify it end-to-end.
    // The *text string* is the strongest signal here: it can only decode
    // correctly if the preceding field order (ins_pt / extrusion / x_axis_dir /
    // rect_width / rect_height / text_height / attachment / flow_dir / extents)
    // is right, so this guards the `SINCE(R_2007a)` `rect_height` branch too.
    let target = mtexts
        .iter()
        .find(|m| match &m.geometry {
            Some(Geometry::MText { ins_pt, .. }) => {
                approx_pt(*ins_pt, (830.4162903179, 10749.3113709571, 0.0))
            }
            _ => false,
        })
        .expect("the reference MTEXT must be present");
    match &target.geometry {
        Some(Geometry::MText { ins_pt, height, value }) => {
            assert!(approx_pt(*ins_pt, (830.4162903179, 10749.3113709571, 0.0)), "{ins_pt:?}");
            assert!(approx(*height, 2.5), "height = {height}");
            assert_eq!(value, "1504,68");
        }
        other => panic!("expected MText, got {:?}", other),
    }

    // A second one, with a multi-line / escape-bearing payload.
    let multi = mtexts
        .iter()
        .find(|m| matches!(&m.geometry, Some(Geometry::MText { value, .. }) if value.contains("\\P")))
        .expect("an MTEXT with paragraph escapes must exist");
    match &multi.geometry {
        Some(Geometry::MText { value, .. }) => {
            assert_eq!(value, "Teksto granda nur por testi.\\PAlia linio.\\PAlia pli.");
        }
        other => panic!("expected MText, got {:?}", other),
    }
}

#[test]
fn polyline_and_vertex_counts_are_consistent() {
    let dwg = DwgFile::parse(SAMPLE).expect("example_2007 should parse");
    // After assembly the standalone VERTEX_* objects should be gone from the
    // entity set (their points were folded into the POLYLINE geometry), so we
    // should see zero VERTEX_* entities in `entities()`.
    let stray_vertices = dwg
        .entities()
        .filter(|o| o.type_name.starts_with("VERTEX"))
        .count();
    assert_eq!(stray_vertices, 0, "vertex points must be folded into polylines");

    // And we should have exactly one of each polyline flavour.
    let n3d = dwg.entities().filter(|o| o.type_name == "POLYLINE_3D").count();
    let npf = dwg.entities().filter(|o| o.type_name == "POLYLINE_PFACE").count();
    assert_eq!((n3d, npf), (1, 1));
}
