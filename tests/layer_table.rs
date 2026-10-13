//! LAYER table records (type 0x33) checked against LibreDWG's `dwg2dxf`
//! export for every supported release.
//!
//! Entities are covered by `tests/crosscheck_geom.py`, but the **layer table**
//! is decoded by a completely separate code path — objects, not entities — and
//! it has its own set of version gates. Two of them were wrong at various
//! points, and both produce *plausible* garbage rather than an error, so they
//! need a test that pins real values:
//!
//! 1. **`has_ds_data(B)` at `SINCE(R_2013)`.** `common_object_handle_data.spec`
//!    puts one more bit on an *object's* field stream from AC1027 on:
//!
//!    ```text
//!    num_reactors(BL)      SINCE(R_13b1)
//!    is_xdic_missing(B)    SINCE(R_2004a)
//!    has_ds_data(B)        SINCE(R_2013)     <- easy to miss
//!    ```
//!
//!    Entities get the same bit from `common_entity_data.spec`, which is why
//!    the geometry cross-check never noticed it. Without it, AC1027/AC1032
//!    reported every layer as `color=97 lw=0` instead of `62=7/2/4/7/7`.
//!
//! 2. **`COMMON_TABLE_FLAGS` is `UNTIL (R_2004)`, not `UNTIL (R_2000)`.**
//!    AC1018 still reads the *old* three fields after the name —
//!    `is_xref_ref(B)`, `is_xref_resolved(BS)`, `is_xref_dep(B)` — 4 bits in
//!    total, not the single `BS` that R2007+ uses. Reading only the `BS` costs
//!    2 bits and shifts `flag0`, the CMC colour and the linetype handle.
//!
//! Expected values come from `dwg2dxf` on the same fixtures: DXF `62` is the
//! colour, `370 = -3` is the DWG-internal lineweight 31, and DXF `70` is
//! `frozen | frozen_in_new<<1 | locked<<2`.

use dwgparse::DwgFile;

const DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/");

/// A row of the expected layer table, in handle order.
struct Row {
    handle: u64,
    name: &'static str,
    /// DXF 62.
    color: i16,
    /// DXF 370 == -3, which is lineweight 31 in the DWG encoding.
    lineweight: u8,
    /// DXF 70 bits this parser exposes.
    frozen: bool,
    locked: bool,
    plot: bool,
}

/// The same drawing saved by five releases. Only one thing differs: AutoCAD
/// 2004 stores the system layer without the leading `*`.
fn expected(version: &str) -> Vec<Row> {
    let adsk = if version == "AC1018" {
        "ADSK_SYSTEM_LIGHTS"
    } else {
        "*ADSK_SYSTEM_LIGHTS"
    };
    // AC1018 alone carries `70 = 5` (frozen + locked) on the system layer.
    let system_flags = version == "AC1018";
    vec![
        Row {
            handle: 0x10,
            name: "0",
            color: 7,
            lineweight: 31,
            frozen: false,
            locked: false,
            plot: true,
        },
        Row {
            handle: 0x89,
            name: "Tavolo 2",
            color: 2,
            lineweight: 31,
            frozen: false,
            locked: false,
            plot: true,
        },
        Row {
            handle: 0x8A,
            name: "Tavolo 3",
            color: 4,
            lineweight: 31,
            frozen: false,
            locked: false,
            plot: true,
        },
        Row {
            handle: 0x37F,
            name: "Defpoints",
            color: 7,
            lineweight: 31,
            frozen: false,
            locked: false,
            plot: false,
        },
        Row {
            handle: 0x75D,
            name: adsk,
            color: 7,
            lineweight: 31,
            frozen: system_flags,
            locked: system_flags,
            plot: true,
        },
    ]
}

const FIXTURES: [(&str, &str); 5] = [
    ("AC1018", "example_2004.dwg"),
    ("AC1021", "example_2007.dwg"),
    ("AC1024", "example_2010.dwg"),
    ("AC1027", "example_2013.dwg"),
    ("AC1032", "example_2018.dwg"),
];

#[test]
fn layer_table_matches_libredwg_truth_for_every_release() {
    for (stamp, file) in FIXTURES {
        let path = format!("{DIR}{file}");
        let dwg = DwgFile::parse(&path).unwrap_or_else(|e| panic!("{stamp} ({file}): {e}"));
        assert_eq!(dwg.version, stamp, "{file}: version stamp");

        let mut got: Vec<_> = dwg.layers.iter().collect();
        got.sort_by_key(|l| l.handle);
        let want = expected(stamp);

        assert_eq!(
            got.len(),
            want.len(),
            "{stamp}: layer count (LibreDWG exports {})",
            want.len()
        );
        for (l, w) in got.iter().zip(want.iter()) {
            assert_eq!(l.handle, w.handle, "{stamp}: layer handle");
            assert_eq!(l.name, w.name, "{stamp} layer {:#x}: name", l.handle);
            assert_eq!(l.color_index, w.color, "{stamp} layer {:?}: DXF 62", l.name);
            assert_eq!(
                l.lineweight, w.lineweight,
                "{stamp} layer {:?}: DXF 370 (-3 == lineweight 31)",
                l.name
            );
            assert_eq!(l.frozen, w.frozen, "{stamp} layer {:?}: DXF 70 frozen", l.name);
            assert_eq!(l.locked, w.locked, "{stamp} layer {:?}: DXF 70 locked", l.name);
            assert_eq!(l.plot_flag, w.plot, "{stamp} layer {:?}: plot flag", l.name);
        }
    }
}

/// The `UNTIL (R_2004)` gate of `COMMON_TABLE_FLAGS`, isolated.
///
/// A 2-bit shortfall shows up as a `flag0` full of bits that belong to the
/// neighbouring fields. On this drawing the only layer whose `flag0` differs
/// from a plain "plot + default lineweight" is the ADSK system layer, and only
/// on AC1018 — which is exactly what makes it a usable witness.
#[test]
fn ac1018_keeps_the_three_field_xref_flags_of_until_r2004() {
    let path = format!("{DIR}example_2004.dwg");
    let dwg = DwgFile::parse(&path).expect("AC1018 parses");
    let adsk = dwg
        .layers
        .iter()
        .find(|l| l.handle == 0x75D)
        .expect("system layer");
    // DXF 70 == 5 == frozen(1) | locked(4).
    assert!(adsk.frozen && adsk.locked, "AC1018: 70 == 5, got {:?}", adsk);
    // …and the same layer is *not* frozen on any later release.
    for (stamp, file) in FIXTURES.iter().skip(1) {
        let later = DwgFile::parse(&format!("{DIR}{file}")).expect("parses");
        let l = later
            .layers
            .iter()
            .find(|l| l.handle == 0x75D)
            .expect("system layer");
        assert!(
            !l.frozen && !l.locked,
            "{stamp}: system layer must not inherit the AC1018 70=5, got {:?}",
            l
        );
    }
}
