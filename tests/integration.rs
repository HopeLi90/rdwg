//! End-to-end tests against the real `sample_2000.dwg` fixture.
//!
//! The fixture is LibreDWG's own `test/test-data/sample_2000.dwg` (AC1015 /
//! R2000). Its geometry, layers and extended-attribute (XData) contents were
//! cross-checked against the equivalent `.dxf` in the same repository, so the
//! assertions below double as a regression guard for the whole parser.

use dwgparse::{DwgFile, Geometry, ParsedObject, XDataValue};

const SAMPLE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/sample_2000.dwg");

/// Dump the (type_code, type_name, count) histogram of the whole object table.
///
/// This doubles as a regression guard: it locks the object-type breakdown of the
/// fixture and makes any silent misclassification (e.g. a wrong `type_name`
/// mapping) visible on the next `cargo test -- --nocapture` run.
#[test]
fn object_type_histogram() {
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");
    use std::collections::BTreeMap;
    let mut hist: BTreeMap<u16, (String, usize)> = BTreeMap::new();
    for o in &dwg.objects {
        let e = hist.entry(o.type_code).or_insert_with(|| (o.type_name.clone(), 0));
        e.1 += 1;
    }
    for (code, (name, count)) in &hist {
        println!("  type 0x{code:02X} {name:<14} x{count}");
    }
    // Lock the known, stable structure of this fixture.
    assert_eq!(dwg.layers.len(), 2);
    let xrecord_count = dwg.objects.iter().filter(|o| o.type_name == "XRECORD").count();
    assert_eq!(xrecord_count, 5, "expected exactly 5 XRECORD objects");
}

#[test]
fn parses_basic_file_metadata() {
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");
    assert_eq!(dwg.version, "AC1015");
    // 61 objects across the entity/object streams.
    assert_eq!(dwg.object_count, 61);
    // Two layers are defined in this drawing.
    assert_eq!(dwg.layers.len(), 2);
}

/// BLOCK_HEADER (0x31) records define the drawing's blocks. An `INSERT` stores
/// only the block_header *handle*, so these names are what make an INSERT
/// readable.
///
/// Verified bit-exactly: three BLOCK_HEADERs at handles 0x1F, 0x50 and 0x55 —
/// the same three handles `sample_2000.dxf` lists as BLOCK_RECORDs.
///
/// NOTE on `*Paper_Space`: the *binary* names both paper-space layout blocks
/// `*Paper_Space` (length byte 0x0C = 12, no trailing digit). Only LibreDWG's
/// DXF *export* renames the second one to `*Paper_Space0` to keep DXF names
/// unique. So the expected vector below is the true on-disk content, and it is
/// deliberately one name different from the .dxf.
#[test]
fn extracts_block_header_names() {
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");
    let blocks: Vec<(u64, &str)> = dwg
        .objects
        .iter()
        .filter_map(|o| o.block_name.as_ref().map(|n| (o.handle, n.as_str())))
        .collect();
    assert_eq!(blocks.len(), 3, "expected 3 BLOCK_HEADER records, got {blocks:?}");

    // Handles must match the three BLOCK_RECORD handles in the .dxf.
    let mut handles: Vec<u64> = blocks.iter().map(|(h, _)| *h).collect();
    handles.sort_unstable();
    assert_eq!(handles, vec![0x1F, 0x50, 0x55], "BLOCK_HEADER handles");

    let mut names: Vec<&str> = blocks.iter().map(|(_, n)| *n).collect();
    names.sort_unstable();
    assert_eq!(
        names,
        vec!["*Model_Space", "*Paper_Space", "*Paper_Space"],
        "block names (binary ground truth; .dxf disambiguates the 3rd)"
    );

    // And the handle -> name lookup must resolve through the public API.
    assert_eq!(dwg.block_name(0x1F), Some("*Model_Space"));
    assert_eq!(dwg.block_name(0x50), Some("*Paper_Space"));
    assert_eq!(dwg.block_name(0x55), Some("*Paper_Space"));
    assert_eq!(dwg.block_name(0xDEADBEEF), None, "unknown handle -> None");
}

#[test]
fn extracts_layer_table() {
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");
    let by_handle: std::collections::HashMap<u64, &dwgparse::LayerInfo> =
        dwg.layers.iter().map(|l| (l.handle, l)).collect();

    // Layer "0" — the default layer, color 7 (white/black).
    let l0 = by_handle.get(&0x10).expect("layer 0x10 must exist");
    assert_eq!(l0.name, "0");
    assert_eq!(l0.color_index, 7);

    // Layer "Tavolo 1", color 2 (yellow).
    let lt = by_handle.get(&0x81).expect("layer 0x81 must exist");
    assert_eq!(lt.name, "Tavolo 1");
    assert_eq!(lt.color_index, 2);

    // Both layers reference the "Continuous" linetype (handle 0x16) via the
    // handle stream — exercises common-table-object handle parsing.
    assert_eq!(l0.ltype_handle, Some(0x16));
    assert_eq!(lt.ltype_handle, Some(0x16));

    // Both layers resolve by name lookup through the helper API.
    assert_eq!(dwg.layer_name(0x10), Some("0"));
    assert_eq!(dwg.layer_name(0x81), Some("Tavolo 1"));

    // Lock the LAYER `flag0` bit decode. Both records carry flag0 = 0x03F0,
    // which per LibreDWG's dwg.spec means: frozen/off/frozen_in_new/locked all
    // clear, plot_flag set (bit 0x10), and lineweight (flag0 & 0x03E0) >> 5.
    // NOTE: the .dxf exports 370 = -3 (BYLAYER) for these layers; that is
    // AutoCAD's lineweight representation and does not round-trip through the
    // 5-bit field, so 31 is the correct *binary* decode.
    for l in [l0, lt] {
        assert_eq!(l.flags, 0x03F0, "layer {:?} flag0", l.name);
        assert!(!l.frozen && !l.off && !l.frozen_in_new && !l.locked, "{}", l.name);
        assert!(l.plot_flag, "{} plot flag must be set", l.name);
        assert_eq!(l.lineweight, 31, "{} lineweight", l.name);
    }

    // The generic handle -> object lookup must find both table records.
    let obj0 = dwg.object_by_handle(0x10).expect("object 0x10");
    assert_eq!(obj0.type_name, "LAYER");
    assert_eq!(obj0.layer.as_ref().map(|l| l.name.as_str()), Some("0"));
    assert!(dwg.object_by_handle(0xDEADBEEF).is_none());
}

#[test]
fn extracts_circle_geometry() {
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");
    let circle = dwg
        .entities()
        .find(|o| o.type_name == "CIRCLE")
        .expect("a CIRCLE must be present");
    // Exact values from `sample_2000.dxf` (handle 0x8D, groups 10/20/30/40).
    match &circle.geometry {
        Some(Geometry::Circle { center, radius }) => {
            assert!((center.0 - 199.6812627452187).abs() < 1e-9, "cx = {}", center.0);
            assert!((center.1 - 24.69575912391304).abs() < 1e-9, "cy = {}", center.1);
            assert!(center.2.abs() < 1e-12, "cz = {}", center.2);
            assert!((*radius - 20.0).abs() < 1e-9, "r = {}", radius);
        }
        other => panic!("expected a Circle, got {:?}", other),
    }
    assert_eq!(circle.handle, 0x8D);
    // The circle lives on layer "Tavolo 1".
    assert_eq!(circle.layer_handle, Some(0x81));
    assert_eq!(dwg.layer_name(circle.layer_handle.unwrap()), Some("Tavolo 1"));
}

#[test]
fn extracts_text_geometry_and_value() {
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");
    let text = dwg
        .entities()
        .find(|o| o.type_name == "TEXT")
        .expect("a TEXT entity must be present");
    // Exact values from `sample_2000.dxf` (handle 0x8E, groups 10/20/40/1).
    match &text.geometry {
        Some(Geometry::Text {
            ins_pt,
            height,
            value,
            ..
        }) => {
            assert!((ins_pt.0 - -50.31873725478131).abs() < 1e-9, "x = {}", ins_pt.0);
            assert!((ins_pt.1 - 134.695759123913).abs() < 1e-9, "y = {}", ins_pt.1);
            assert!((*height - 5.0).abs() < 1e-9, "height = {}", height);
            assert_eq!(value, "Jen teksto simpla, cxu ne?");
        }
        other => panic!("expected Text, got {:?}", other),
    }
    assert_eq!(text.handle, 0x8E);
    assert_eq!(text.text.as_deref(), Some("Jen teksto simpla, cxu ne?"));
}

#[test]
fn extracts_line_endpoints() {
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");
    let lines: Vec<&dwgparse::ParsedObject> = dwg
        .entities()
        .filter(|o| o.type_name == "LINE")
        .collect();
    assert_eq!(lines.len(), 3, "expected exactly three LINE entities");

    // All three LINEs, matched against `sample_2000.dxf`
    // (handles 0x90/0x91/0x92, groups 10/20 -> 11/21).
    let expected = [
        (0x90u64, 0x81u64, (99.68126274521871, 94.69575912391304), (199.6812627452187, 94.69575912391304)),
        (0x91, 0x10, (199.6812627452187, 194.695759123913), (199.6812627452187, 94.69575912391304)),
        (0x92, 0x10, (99.68126274521871, 94.69575912391304), (199.6812627452187, 194.695759123913)),
    ];
    for (handle, _layer, s, e) in expected {
        let ln = lines
            .iter()
            .find(|l| l.handle == handle)
            .unwrap_or_else(|| panic!("LINE handle 0x{handle:X} must exist"));
        match &ln.geometry {
            Some(Geometry::Line { start, end }) => {
                assert!((start.0 - s.0).abs() < 1e-9, "0x{handle:X} sx = {}", start.0);
                assert!((start.1 - s.1).abs() < 1e-9, "0x{handle:X} sy = {}", start.1);
                assert!((end.0 - e.0).abs() < 1e-9, "0x{handle:X} ex = {}", end.0);
                assert!((end.1 - e.1).abs() < 1e-9, "0x{handle:X} ey = {}", end.1);
                // z is stored as a "z_is_zero" bit in the binary, not as a
                // coordinate, so both endpoints must come back as exactly 0.
                assert_eq!(start.2, 0.0, "0x{handle:X} sz");
                assert_eq!(end.2, 0.0, "0x{handle:X} ez");
            }
            other => panic!("expected Line, got {:?}", other),
        }
    }

    // Layer assignment: 0x90 is on "Tavolo 1", 0x91/0x92 are on layer "0".
    let by_handle = |h: u64| {
        lines
            .iter()
            .find(|l| l.handle == h)
            .and_then(|l| l.layer_handle)
            .and_then(|lh| dwg.layer_name(lh))
            .map(str::to_string)
    };
    assert_eq!(by_handle(0x90).as_deref(), Some("Tavolo 1"));
    assert_eq!(by_handle(0x91).as_deref(), Some("0"));
    assert_eq!(by_handle(0x92).as_deref(), Some("0"));
}

#[test]
fn extracts_lwpolyline_vertices() {
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");
    let pl = dwg
        .entities()
        .find(|o| o.type_name == "LWPOLYLINE")
        .expect("an LWPOLYLINE must be present");
    match &pl.geometry {
        Some(Geometry::LwPolyline { flag, points, bulges }) => {
            // DWG flag bit 512 == "closed"; DXF exports that as group 70 = 1.
            assert_eq!(*flag, 0x200);
            assert_eq!(points.len(), 4, "the polyline has four vertices");

            // Full vertex check against `sample_2000.dxf` (groups 10/20).
            // This is what actually validates the `FIELD_2DD_VECTOR` decoding:
            // vertex 0 is raw 2RD, vertices 1..n are DD deltas vs the previous
            // vertex. Getting that wrong silently shifts every later vertex, so
            // the count alone is not enough to prove correctness.
            let expected = [
                (-50.3187372547813, -85.30424087608697),
                (49.6812627452187, -85.30424087608697),
                (49.6812627452187, 54.69575912391305),
                (-50.3187372547813, 54.69575912391305),
            ];
            for (i, (ex, ey)) in expected.iter().enumerate() {
                let (x, y) = points[i];
                // Tight tolerance on purpose: for this fixture the DD deltas
                // round-trip exactly, so a loose 1e-3 would hide a regression
                // that shifts vertices by a few ULPs.
                assert!(
                    (x - *ex).abs() < 1e-9 && (y - *ey).abs() < 1e-9,
                    "vertex {i}: got ({x:?}, {y:?}), want ({ex:?}, {ey:?})"
                );
            }
        }
        other => panic!("expected LwPolyline, got {:?}", other),
    }
}

#[test]
fn extracts_extended_attributes() {
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");
    // At least the five XRECORD objects carry XData in this fixture.
    assert!(
        dwg.extended_attribute_count() >= 5,
        "expected >= 5 objects with extended attributes, got {}",
        dwg.extended_attribute_count()
    );

    // Locate the XRECORD at handle 0x98 (index 55), which holds the
    // SHADEPLOT-related XData — a non-trivial resbuf chain using the control
    // string group code 102 plus 16-bit ints.
    let xrec = dwg
        .objects
        .iter()
        .find(|o| o.handle == 0x98)
        .expect("XRECORD 0x98 must exist");
    assert!(!xrec.xdata.is_empty(), "XRECORD 0x98 must carry XData");

    let codes: Vec<i16> = xrec.xdata.iter().map(|x| x.code).collect();
    assert!(codes.contains(&102), "expected SHADEPLOT control strings (102)");
    assert!(codes.contains(&70), "expected 16-bit ints (70)");

    // Verify one concrete control-string value.
    let has_shadeplot = xrec.xdata.iter().any(|x| {
        matches!(&x.value, XDataValue::Str(s) if s == "SHADEPLOT")
    });
    assert!(has_shadeplot, "XData must contain the SHADEPLOT marker string");

    // And a concrete int: SHADEPLOTCUSTOMDPI = 300 (group 70).
    let has_customdpi = xrec.xdata.iter().any(|x| {
        matches!(&x.value, XDataValue::Int16(v) if *v == 300)
    });
    assert!(has_customdpi, "XData must contain SHADEPLOTCUSTOMDPI = 300");
}

#[test]
fn layer_scoped_entity_queries() {
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");

    // 1. Layer lookup by name.
    let layer = dwg
        .layer_by_name("Tavolo 1")
        .expect("layer 'Tavolo 1' must exist");
    assert!(dwg.layer_by_name("No Such Layer").is_none());

    // 2. Entities scoped to that layer must all carry geometry and all be
    //    linked by handle (never by name) to the layer we asked for.
    let on_layer: Vec<&ParsedObject> = dwg.entities_on_layer(layer).collect();
    assert!(!on_layer.is_empty(), "'Tavolo 1' must own entities");
    for o in &on_layer {
        assert_eq!(o.layer_handle, Some(layer.handle));
        assert!(o.geometry.is_some(), "entity 0x{:X} has no geometry", o.handle);
    }

    // 3. Every entity of the file is reachable through some layer, i.e. the
    //    entity->layer handle link never dangles in this fixture.
    let known: std::collections::HashSet<u64> = dwg.layers.iter().map(|l| l.handle).collect();
    for o in dwg.entities() {
        let h = o.layer_handle.expect("entity must reference a layer");
        assert!(known.contains(&h), "entity 0x{:X} -> unknown layer 0x{h:X}", o.handle);
    }

    // 4. Handle -> layer name round-trip agrees with the layer record itself.
    assert_eq!(dwg.layer_name(layer.handle), Some("Tavolo 1"));
}

#[test]
fn xrecord_handle_resolution_is_relative() {
    // The XRECORD at handle 0x95 (index 52) references a handle via code 340.
    let dwg = DwgFile::parse(SAMPLE).expect("sample should parse");
    let xrec = dwg
        .objects
        .iter()
        .find(|o| o.handle == 0x95)
        .expect("XRECORD 0x95 must exist");
    // Its XData group 340 must resolve to handle 0x6A (106) — the well-known
    // absolute handle present in the parsed object table.
    let resolved = xrec.xdata.iter().find(|x| x.code == 340);
    assert!(resolved.is_some(), "XRECORD 0x95 must have a group-340 entry");
    if let Some(XDataValue::Handle(h)) = resolved.map(|x| &x.value) {
        assert_eq!(*h, 106, "group 340 resolves to handle 106 (0x6A)");
    } else {
        panic!("group 340 entry is not a Handle");
    }
}
