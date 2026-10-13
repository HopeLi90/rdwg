//! `XRECORD` XData extraction, pinned against LibreDWG's field-level trace.
//!
//! `dwg2dxf` does not export `XRECORD`s, so the ground truth here comes from
//! `dwgread -v4` (`DWG_LOGLEVEL_HANDLE`), which logs every field as it is read.
//! On **all five** modern fixtures the drawing carries six identical
//! `XRECORD`s — handles `0x26B`…`0x270` — each decoding to exactly:
//!
//! ```text
//! num_reactors: 1 [BL 0]
//! is_xdic_missing: 1 [B 0]
//! xdata_size: 8 [BL 0]
//!   xdata[0] type: 270 [RS]   xdata[0]: 1 [RS 270]
//!   xdata[1] type: 271 [RS]   xdata[1]: 1 [RS 271]
//! cloning: 1 [BS 280]
//! num_objid_handles: 0 [BL]
//! ```
//!
//! Two separate defects used to hide behind this one record, and both produced
//! *plausible* output rather than an error, which is why they need a test:
//!
//! 1. **The object prologue.** `dwg_decode_object` reads
//!    `num_reactors(BL)` / `is_xdic_missing(B)` / `has_ds_data(B, R2013)` for
//!    *every* object, `XRECORD` included. Skipping it left `xdata_size` at 0
//!    and every modern file reported "no XData at all".
//!
//! 2. **The bit offset.** Every XData payload is a whole-byte read, and
//!    `bit_read_RC` is bit-aligned — it leaves `bit` where it found it. So a
//!    block that starts at bit 1 also *ends* at bit 1. Zeroing `bit` when the
//!    block closed dropped one bit and shifted the following `BS cloning`:
//!    `-24448` instead of `1`.

use dwgparse::{DwgFile, XDataValue};

const DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/");

/// The six `XRECORD` handles of this drawing, in handle order.
const XRECORD_HANDLES: [u64; 6] = [0x26B, 0x26C, 0x26D, 0x26E, 0x26F, 0x270];

const FIXTURES: [(&str, &str); 5] = [
    ("AC1018", "example_2004.dwg"),
    ("AC1021", "example_2007.dwg"),
    ("AC1024", "example_2010.dwg"),
    ("AC1027", "example_2013.dwg"),
    ("AC1032", "example_2018.dwg"),
];

/// The `XRECORD` type code, used to find the records without relying on the
/// name mapping.
const TYPE_XRECORD: u16 = 79;

#[test]
fn every_release_extracts_the_same_xdata_from_every_xrecord() {
    for (stamp, file) in FIXTURES {
        let path = format!("{DIR}{file}");
        let dwg = DwgFile::parse(&path).unwrap_or_else(|e| panic!("{stamp} ({file}): {e}"));

        for handle in XRECORD_HANDLES {
            let obj = dwg
                .object_by_handle(handle)
                .unwrap_or_else(|| panic!("{stamp}: no object at handle {handle:#x}"));
            assert_eq!(
                obj.type_code, TYPE_XRECORD,
                "{stamp}: handle {handle:#x} must be an XRECORD, got {} ({})",
                obj.type_code, obj.type_name
            );

            // `xdata_size` is 8 bytes: exactly the resbuf `[270=1, 271=1]`.
            // The trailing `cloning` (DXF 280) is a *separate* field that
            // follows the resbuf — it is NOT part of the XData list.
            assert_eq!(
                obj.xdata.len(),
                2,
                "{stamp} handle {handle:#x}: expected [270, 271], got {:?}",
                obj.xdata
            );
            assert_eq!(obj.xdata[0].code, 270, "{stamp} handle {handle:#x}: group 270");
            assert_eq!(
                obj.xdata[0].value,
                XDataValue::Int16(1),
                "{stamp} handle {handle:#x}: xdata[0] must be 1"
            );
            assert_eq!(obj.xdata[1].code, 271, "{stamp} handle {handle:#x}: group 271");
            assert_eq!(
                obj.xdata[1].value,
                XDataValue::Int16(1),
                "{stamp} handle {handle:#x}: xdata[1] must be 1"
            );
            // The SINCE(R_2000b) `cloning` field, DXF 280 == 1, kept apart
            // from the XData resbuf. A wrong bit offset here shows up as a
            // large garbage value (e.g. -24448 instead of 1).
            assert_eq!(
                obj.cloning,
                Some(1),
                "{stamp} handle {handle:#x}: cloning (DXF 280) must be 1"
            );
        }
    }
}

/// AC1015 is the control case: its XDATA happens to start byte-aligned, so it
/// never exposed the bit-offset bug. `sample_2000.dwg` carries the
/// `SHADEPLOT` chain instead of the 270/271 pair.
#[test]
fn ac1015_shadeplot_chain_still_decodes() {
    let path = format!("{DIR}sample_2000.dwg");
    let dwg = DwgFile::parse(&path).expect("AC1015 parses");
    assert_eq!(dwg.version, "AC1015");

    let obj = dwg
        .object_by_handle(0x98)
        .expect("XRECORD 0x98 (the SHADEPLOT settings record)");
    assert_eq!(obj.type_code, TYPE_XRECORD);

    let got: Vec<(i16, String)> = obj
        .xdata
        .iter()
        .map(|x| (x.code, format!("{:?}", x.value)))
        .collect();
    let want: Vec<(i16, String)> = vec![
        (102, "Str(\"SHADEPLOT\")".to_string()),
        (70, "Int16(0)".to_string()),
        (102, "Str(\"SHADEPLOTRESLEVEL\")".to_string()),
        (70, "Int16(2)".to_string()),
        (102, "Str(\"SHADEPLOTCUSTOMDPI\")".to_string()),
        (70, "Int16(300)".to_string()),
    ];
    assert_eq!(got, want, "AC1015 XRECORD 0x98 xdata (resbuf only; cloning is separate)");
    // The `cloning` flag (DXF 280) follows the resbuf as its own field.
    assert_eq!(obj.cloning, Some(1), "AC1015 XRECORD 0x98 cloning must be 1");
}

/// No release may silently fall back to "no XData". A regression here is the
/// prologue defect: `xdata_size` reads 0 and everything downstream is empty.
#[test]
fn modern_releases_are_never_empty() {
    for (stamp, file) in FIXTURES {
        let dwg = DwgFile::parse(&format!("{DIR}{file}")).expect("parses");
        let with_xdata = dwg.objects.iter().filter(|o| !o.xdata.is_empty()).count();
        assert!(
            with_xdata >= XRECORD_HANDLES.len(),
            "{stamp}: only {with_xdata} objects carry XData, expected at least {}",
            XRECORD_HANDLES.len()
        );
        for o in &dwg.objects {
            if o.type_code != TYPE_XRECORD {
                continue;
            }
            assert!(
                !o.xdata.is_empty(),
                "{stamp}: XRECORD #{} handle {:#x} decoded empty — \
                 the object prologue is probably missing again",
                o.index,
                o.handle
            );
        }
    }
}
