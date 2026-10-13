//! DWG (AC1015 / R2000) file parser.
//!
//! Object/section layout and bit-level ordering follow LibreDWG's
//! `decode.c` (`decode_R13_R2000`, `dwg_decode_add_object`,
//! `dwg_decode_entity`, `dwg_decode_object`) and `dwg.c`.

use crate::bitreader::BitReader;
use crate::xdata::{decode_eed, decode_xdata};
use crate::{
    r2004, r2007, Color, DictionaryEntry, DwgFile, DwgHandle, Eed, Geometry, LayerInfo,
    ParsedObject, PolylineHeader, VertexData,
};
use std::collections::HashMap;

/// Number of section-locator records in the R2000 header (§0..§5).
const R2000_SECTIONS: usize = 6;

/// Map a DWG *fixed* type code to its object/entity name.
///
/// The codes and names below are transcribed from the `/* (code) */` comments
/// that precede each declaration in LibreDWG's `dwg.spec` / `dwg2.spec`, and
/// cross-checked against the real `sample_2000.dwg` object table.
///
/// Two traps worth recording (both were wrong in earlier revisions):
/// * `0x31` is **BLOCK_HEADER**, not `BLOCK`. The actual `BLOCK` /
///   `ENDBLK` (AcDbBlockBegin/End) entities are `0x04` / `0x05`.
/// * The table objects are **not** the classic ODA numbering. LibreDWG/R2000
///   put `LTYPE_CONTROL`/`LTYPE` at `0x38`/`0x39` (not `0x36`/`0x37`), and
///   `0x44` is `DIMSTYLE_CONTROL`. The object-type histogram of
///   `sample_2000.dwg` confirms this: it has 3 `0x39` objects and the matching
///   `.dxf` has exactly 3 `LTYPE` records.
fn type_name(code: u16) -> &'static str {
    match code {
        0x01 => "TEXT",
        0x02 => "ATTRIB",
        0x03 => "ATTDEF",
        0x04 => "BLOCK",
        0x05 => "ENDBLK",
        0x06 => "SEQEND",
        0x07 => "INSERT",
        0x08 => "MINSERT",
        0x0a => "VERTEX_2D",
        0x0b => "VERTEX_3D",
        0x0c => "VERTEX_MESH",
        0x0d => "VERTEX_PFACE",
        0x0e => "VERTEX_PFACE_FACE",
        0x0f => "POLYLINE_2D",
        0x10 => "POLYLINE_3D",
        0x11 => "ARC",
        0x12 => "CIRCLE",
        0x13 => "LINE",
        0x14 => "DIMENSION_ORDINATE",
        0x15 => "DIMENSION_LINEAR",
        0x16 => "DIMENSION_ALIGNED",
        0x17 => "DIMENSION_ANG3PT",
        0x18 => "DIMENSION_ANG2LN",
        0x19 => "DIMENSION_RADIUS",
        0x1a => "DIMENSION_DIAMETER",
        0x1b => "POINT",
        0x1c => "3DFACE",
        0x1d => "POLYLINE_PFACE",
        0x1e => "POLYLINE_MESH",
        0x1f => "SOLID",
        0x20 => "TRACE",
        0x21 => "SHAPE",
        0x22 => "VIEWPORT",
        0x23 => "ELLIPSE",
        0x24 => "SPLINE",
        0x25 => "REGION",
        0x26 => "3DSOLID",
        0x27 => "BODY",
        0x28 => "RAY",
        0x29 => "XLINE",
        0x2a => "DICTIONARY",
        0x2b => "OLEFRAME",
        0x2c => "MTEXT",
        0x2d => "LEADER",
        0x2e => "TOLERANCE",
        0x2f => "MLINE",
        0x30 => "BLOCK_CONTROL",
        0x31 => "BLOCK_HEADER",
        0x32 => "LAYER_CONTROL",
        0x33 => "LAYER",
        0x34 => "STYLE_CONTROL",
        0x35 => "STYLE",
        0x38 => "LTYPE_CONTROL",
        0x39 => "LTYPE",
        0x3c => "VIEW_CONTROL",
        0x3d => "VIEW",
        0x3e => "UCS_CONTROL",
        0x3f => "UCS",
        0x40 => "VPORT_CONTROL",
        0x41 => "VPORT",
        0x42 => "APPID_CONTROL",
        0x43 => "APPID",
        0x44 => "DIMSTYLE_CONTROL",
        0x45 => "DIMSTYLE",
        0x46 => "VX_CONTROL",
        0x47 => "VX_TABLE_RECORD",
        0x48 => "GROUP",
        0x49 => "MLINESTYLE",
        0x4d => "LWPOLYLINE",
        0x4f => "XRECORD",
        _ => "UNKNOWN",
    }
}

/// Read a big-endian 2-byte integer (`RS_BE`).
fn read_rs_be(r: &mut BitReader) -> u16 {
    let hi = r.read_rc() as u16;
    let lo = r.read_rc() as u16;
    (hi << 8) | lo
}

/// Parse the R2000 section locator table (starts at file byte 25).
fn parse_locator(file: &[u8]) -> Result<Vec<(u8, u32, u32)>, String> {
    let mut r = BitReader::new(file);
    r.byte = 25;
    let mut sections = Vec::with_capacity(R2000_SECTIONS);
    for _ in 0..R2000_SECTIONS {
        let number = r.read_rc();
        let address = r.read_rl();
        let size = r.read_rl();
        sections.push((number, address, size));
    }
    Ok(sections)
}

/// Parse the Handles section (§2, the **object map**) and return the absolute
/// file offsets of every object record, in stored order.
///
/// The map is stored as a chain of *pages* exactly like LibreDWG's
/// `handles_section:` loop in `decode.c`:
///
/// ```text
/// do {
///   startpos = byte
///   section_size = RS_BE            // page size, counts its own 2 bytes
///   while (byte - startpos < section_size) {
///       handleoff = UMC;  offset = MC;  last_offset += offset;
///       -> one object record at absolute file offset `last_offset`
///   }
///   if bit > 0 { byte += 1 }        // pad to a byte boundary
///   crc = RS_BE                     // page CRC
/// } while (byte < map_end && section_size > 2)
/// ```
/// `stream_len` is the length of the buffer the resulting offsets index into.
/// For R2000 that is the whole file; for R2007 it is the decompressed
/// `AcDb:AcDbObjects` section (the handle map stores offsets *relative* to it),
/// so it must be passed separately from `file.len()`.
fn parse_handles(
    file: &[u8],
    addr: usize,
    map_size: usize,
    stream_len: usize,
) -> Result<Vec<usize>, String> {
    let mut r = BitReader::new(file);
    r.byte = addr;
    let lastmap = (addr + map_size).min(file.len());
    let mut offsets = Vec::new();
    loop {
        if r.byte + 2 > lastmap {
            break;
        }
        let startpos = r.byte;
        let section_size = read_rs_be(&mut r) as usize;
        // LibreDWG: sizes > 2040 are a hard error, <= 2 means "empty page".
        if section_size <= 2 || section_size > 2040 {
            break;
        }
        // PITFALL: `last_offset` is declared *inside* the page loop in
        // LibreDWG (`decode_r2007` and `read_r2000_section_handles` alike), so
        // every page restarts from 0. Accumulating across pages silently
        // drops every object after the first ~200 on multi-page files.
        let mut last_offset: i64 = 0;
        let mut oldpos: usize;
        while r.byte - startpos < section_size && r.byte + 2 <= lastmap {
            oldpos = r.byte;
            let _handleoff = r.read_umc(); // delta from the previous handle
            let offset = r.read_mc(); // signed delta from the previous address
            last_offset += offset;
            if last_offset > 0 && (last_offset as usize) < stream_len {
                offsets.push(last_offset as usize);
            }
            if r.byte == oldpos {
                break; // no forward progress
            }
        }
        // Page CRC is byte aligned.
        if r.bit > 0 {
            r.byte += 1;
            r.bit = 0;
        }
        if r.byte + 2 > lastmap {
            break;
        }
        let _crc = read_rs_be(&mut r);
        if r.byte >= lastmap {
            break;
        }
    }
    Ok(offsets)
}

/// Debug-only public wrapper around [`parse_handles`] so the examples crate can
/// reuse the exact same object-offset logic as the real parser.
pub fn parse_handles_debug(file: &[u8], addr: usize, map_size: usize, stream_len: usize) -> Result<Vec<usize>, String> {
    parse_handles(file, addr, map_size, stream_len)
}

/// Debug-only wrapper around the private `setup_string_stream`.
pub fn setup_string_stream_debug(obj_data: &[u8], bitsize: usize) -> Option<BitReader<'_>> {
    setup_string_stream(obj_data, bitsize)
}

/// Debug-only wrapper around `decode_eed` (in `xdata`).
pub fn decode_eed_debug(r: &mut BitReader, dialect: Dialect) -> Vec<crate::Eed> {
    crate::xdata::decode_eed(r, dialect)
}

/// Read the common entity-handle stream for an entity: owner, reactors,
/// xdict, layer, ltype — in that fixed order. Returns the layer handle.
/// Resolve a raw handle reference to its *absolute* handle value.
///
/// Handles with codes 6/8/10/12/14 are stored as offsets relative to the
/// referring object's own handle — see the switch in LibreDWG's
/// `dwg_decode_handleref` (`decode.c`):
///
/// ```text
///  6  -> own + 1        8  -> own - 1        10 -> own + offset
/// 12  -> own - offset   14 -> own            (all other codes: literal value)
/// ```
fn resolve_handle(h: DwgHandle, own: u64) -> u64 {
    match h.code {
        6 => own.saturating_add(1),
        8 => own.saturating_sub(1),
        10 => own.wrapping_add(h.value),
        12 => own.wrapping_sub(h.value),
        14 => own,
        _ => h.value, // 0, 2, 3, 4, 5 hold literal values
    }
}

/// Which DWG dialect the bit streams are laid out for.
///
/// Everything in the object stream is *almost* identical between R2000 and
/// R2007, but a handful of fields moved or changed encoding (see
/// `read_common_entity_fields`). Threading this through keeps one decoder
/// instead of two divergent copies.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// The DWG "dialect" we are decoding — R2000 (AC1015) vs R2007 (AC1021).
///
/// The two formats differ in enough details (modular `BL`, string streams,
/// EED string encoding, handle-stream ordering) that threading the dialect
/// through the decoders is cleaner than guessing per field.
pub enum Dialect {
    /// AC1015 — flat sections, no string stream, `CMC` colours.
    R2000,
    /// AC1021 — page container, per-object string stream, `ENC` colours.
    R2007,
}

/// Which variant of LibreDWG's `common_entity_data.spec` /
/// `common_entity_handle_data.spec` a file uses.
///
/// That block grew piece by piece, and every addition shifts every field — and
/// every handle — that follows it. Decoding a file with the wrong variant does
/// not fail loudly: it silently produces plausible-looking garbage, which is
/// exactly what happened when AC1018/AC1024/AC1027/AC1032 were all decoded with
/// the AC1021 layout.
///
/// ```text
/// SINCE(R_2004a)  AC1018+ : is_xdic_missing (B) replaces nolinks; ENC colour
/// SINCE(R_2007a)  AC1021+ : material_flags (BB) + shadow_flags (RC),
///                           and the per-object string stream
/// SINCE(R_2010b)  AC1024+ : has_full/face/edge_visualstyle (3x B)
/// SINCE(R_2013)   AC1027+ : has_ds_data (B)
/// ```
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum EntityVariant {
    /// AC1015 and older: `nolinks` (B), `CMC` colour, no string stream.
    PreR2004,
    /// AC1018 (R2004–R2006).
    R2004,
    /// AC1021 (R2007–R2009).
    R2007,
    /// AC1024 (R2010–R2012).
    R2010,
    /// AC1027 and newer (R2013–R2024, incl. CAD2020).
    R2013,
}

impl EntityVariant {
    /// Map an `ACxxxx` version stamp to the matching common-entity layout.
    pub(crate) fn from_version(version: &str) -> Self {
        match version {
            // AC1018/1019/1020 are all "R2004" as far as the object stream cares.
            v if v < "AC1021" => Self::R2004,
            "AC1021" | "AC1022" | "AC1023" => Self::R2007,
            "AC1024" | "AC1025" | "AC1026" => Self::R2010,
            _ => Self::R2013,
        }
    }
}

/// Condition flags produced by the common-entity field block that decide how
/// many handles follow on the handle stream.
struct EntityFlags {
    num_reactors: u32,
    entmode: u32,
    /// `true` when the `prev_entity`/`next_entity` pair is present on the handle
    /// stream. Only `VERSIONS(R_13b1, R_2000)` emit it; R2007+ never does.
    links: bool,
    ltype_flags: u32,
    plotstyle_flags: u32,
    /// `SINCE(R_2007a)`: 3 means a `material` handle follows on the handle stream.
    material_flags: u32,
    /// `SINCE(R_2007a)`: 3 means a `shadow` handle follows on the handle stream.
    shadow_flags: u8,
    /// `SINCE(R_2004a)`: when set, the `xdicobjhandle` is **absent** from the
    /// handle stream (`ENT_XDICOBJHANDLE` in `dec_macros.h` guards the read
    /// with `if (!_ent->is_xdic_missing)`). Reading it anyway shifts every
    /// later handle — including `layer` — by one slot.
    is_xdic_missing: bool,
    /// `SINCE(R_2010b)`: each of these adds one handle after `plotstyle`.
    has_full_visualstyle: bool,
    has_face_visualstyle: bool,
    has_edge_visualstyle: bool,
}

/// Read the common-entity **handle** stream in the exact conditional order of
/// LibreDWG's `common_entity_handle_data.spec` for R2000 (the `SINCE(R_2000b)`
/// branch, which is the one our files exercise):
///
/// ```text
/// if entmode == 0      -> ownerhandle
/// reactors x num_reactors
///                      -> xdicobjhandle          (VERSIONS R_13b1..R_2002)
/// if !nolinks          -> prev_entity, next_entity (VERSIONS R_13b1..R_2000)
///                      -> layer
/// if ltype_flags == 3  -> ltype
/// if material_flags==3 -> material          (SINCE R_2007a)
/// if shadow_flags == 3 -> shadow            (SINCE R_2007a)
/// if plotstyle_flags==3-> plotstyle
/// if has_{full,face,edge}_visualstyle      (SINCE R_2010b)
/// ```
/// The resolved handle references from an entity's handle stream.
pub struct EntityHandleRefs {
    pub layer: Option<u64>,
    pub owner: Option<u64>,
    pub xdict: Option<u64>,
    pub reactors: Vec<u64>,
}

fn read_entity_common_handles(
    hr: &mut BitReader,
    own_handle: u64,
    f: &EntityFlags,
) -> EntityHandleRefs {
    let mut refs = EntityHandleRefs {
        layer: None,
        owner: None,
        xdict: None,
        reactors: Vec::new(),
    };

    // 1. owner handle — only when entmode == 0.
    if f.entmode == 0 {
        let h = hr.read_h();
        let v = resolve_handle(h, own_handle);
        refs.owner = if v == 0 { None } else { Some(v) };
    }

    // 2. reactors
    let nr = f.num_reactors.min(1_000_000); // defensive: never spin forever
    for _ in 0..nr {
        let h = hr.read_h();
        let v = resolve_handle(h, own_handle);
        if v != 0 {
            refs.reactors.push(v);
        }
    }

    // 3. xdicobjhandle — unconditional up to R_2002, but `ENT_XDICOBJHANDLE`
    //    guards it with `if (!_ent->is_xdic_missing)` from R_2004a on.
    if !f.is_xdic_missing {
        let h = hr.read_h();
        let v = resolve_handle(h, own_handle);
        refs.xdict = if v == 0 { None } else { Some(v) };
    }

    // 4. prev/next entity links — VERSIONS(R_13b1, R_2000) only.
    if f.links {
        let _prev = hr.read_h();
        let _next = hr.read_h();
    }

    // 5. layer
    let layer_raw = hr.read_h();
    let layer_value = resolve_handle(layer_raw, own_handle);
    refs.layer = if layer_value == 0 { None } else { Some(layer_value) };

    // 6. ltype — 7. material/shadow (R_2007a+) — 8. plotstyle
    if f.ltype_flags == 3 {
        let _ltype = hr.read_h();
    }
    if f.material_flags == 3 {
        let _material = hr.read_h();
    }
    if f.shadow_flags == 3 {
        let _shadow = hr.read_h();
    }
    if f.plotstyle_flags == 3 {
        let _plotstyle = hr.read_h();
    }

    // 9. visual styles — SINCE(R_2010b), after plotstyle.
    if f.has_full_visualstyle {
        let _fvs = hr.read_h();
    }
    if f.has_face_visualstyle {
        let _xvs = hr.read_h();
    }
    if f.has_edge_visualstyle {
        let _evs = hr.read_h();
    }

    refs
}

/// Read the fixed common-entity-data field block (on the field-data stream).
///
/// Field order verified against LibreDWG's `common_entity_data.spec` for
/// R2000 (`SINCE(R_13b1)` … `R_2002`): `preview_exists(B)`, optional preview,
/// `entmode(BB)`, `num_reactors(BL)`, `nolinks(B)` — the latter is in
/// `VERSIONS(R_13b1, R_2002)` so it **is** present for R2000 — then color,
/// `ltype_scale(BD1)`, `ltype_flags(BB)`, `plotstyle_flags(BB)`, `invisible(BS)`,
/// `linewt(RC)`.
///
/// `variant` selects which conditional block applies — see [`EntityVariant`].
/// Getting it wrong does not raise an error, it just silently shifts every
/// coordinate, so it is worth being explicit at every branch.
fn read_common_entity_fields(
    rd: &mut BitReader,
    str_rd: &mut Option<BitReader>,
    variant: EntityVariant,
) -> EntityFlags {
    let _preview_exists = rd.read_b();
    let entmode = rd.read_bb();
    // `num_reactors` is a *compact* BL on the field stream (code 2 -> 0), NOT a
    // 4-byte RL. The spec's `FIELD_BL` carries the comment "ODA bug: BB as BS"
    // — i.e. the on-disk data uses the 2-bit code, exactly like LAYER. For
    // 0..=255 a modular `BL` and a `BS` are bit-identical anyway.
    let num_reactors = rd.read_bl();

    // R13b1..R_2002 carry `nolinks`; from R_2004a that bit is replaced by
    // `is_xdic_missing`. Both are a single `B`, so the *width* is the same but
    // the meaning — and what follows — is not.
    let (links, is_xdic_missing) = if variant >= EntityVariant::R2004 {
        let missing = rd.read_b() != 0;
        // R2004+ never emits prev/next entity links.
        (false, missing)
    } else {
        let nolinks = rd.read_b();
        (nolinks == 0, false)
    };

    // SINCE(R_2013): `has_ds_data`. Only 3D solids push a handle for it, but
    // the bit is present on the field stream for *every* entity.
    if variant >= EntityVariant::R2013 {
        let _has_ds_data = rd.read_b();
    }

    if variant >= EntityVariant::R2004 {
        // SINCE(R_2004a): the ENC colour encoding replaces `CMC`.
        let _color = read_enc_color(rd, str_rd);
    } else {
        let _color = read_cmc(rd, str_rd, EntityVariant::PreR2004);
    }

    let _ltype_scale = rd.read_bd1();
    let ltype_flags = rd.read_bb();
    let plotstyle_flags = rd.read_bb();
    // SINCE(R_2007a): material + shadow flags. Their value decides whether the
    // matching handle appears later on the handle stream.
    let (material_flags, shadow_flags) = if variant >= EntityVariant::R2007 {
        (rd.read_bb(), rd.read_rc())
    } else {
        (0, 0)
    };
    // SINCE(R_2010b): three visual-style presence bits, each of which adds a
    // handle at the end of the handle stream.
    let (has_full_visualstyle, has_face_visualstyle, has_edge_visualstyle) =
        if variant >= EntityVariant::R2010 {
            (rd.read_b() != 0, rd.read_b() != 0, rd.read_b() != 0)
        } else {
            (false, false, false)
        };

    let _invisible = rd.read_bs();
    let _linewt = rd.read_rc();
    EntityFlags {
        num_reactors,
        entmode,
        links,
        ltype_flags,
        plotstyle_flags,
        material_flags,
        shadow_flags,
        is_xdic_missing,
        has_full_visualstyle,
        has_face_visualstyle,
        has_edge_visualstyle,
    }
}

/// The field-stream prologue every *object* opens with:
/// `common_object_handle_data.spec` (LibreDWG `src/common_object_handle_data.spec`).
///
/// ```text
/// num_reactors(BL)      // SINCE(R_13b1)
/// is_xdic_missing(B)    // SINCE(R_2004a)   0 means "the XDICTIONARY is present"
/// has_ds_data(B)        // SINCE(R_2013)
/// ```
///
/// All three sit on the **field** stream — they only *govern* the handles that
/// follow on the handle stream. `has_ds_data` adds no handle unless the object
/// is a 3D solid, so the handle stream is unaffected by the R2013 bit.
///
/// Entities carry the same `has_ds_data` bit, but from the *entity* spec
/// (`common_entity_data.spec`), which is why [`read_common_entity_fields`]
/// reads it and this function is the object-side twin.
///
/// Returns `(num_reactors, is_xdic_missing)`.
fn read_object_prologue(rd: &mut BitReader, variant: EntityVariant) -> (u32, bool) {
    // `BL` here is the *modular* BL; for 0..=255 it is bit-identical to `BS`,
    // so the R2000 path that used a `BS` was accidentally right.
    let num_reactors = rd.read_bl();
    let is_xdic_missing = if variant >= EntityVariant::R2004 {
        let missing = rd.read_b() != 0;
        if variant >= EntityVariant::R2013 {
            let _has_ds_data = rd.read_b();
        }
        missing
    } else {
        false
    };
    (num_reactors, is_xdic_missing)
}

/// Decode a `LAYER` table object (type 0x33).
///
/// A `LAYER` is an *object* (table record), not an entity. Its data comes from
/// three separate bit streams, and the order in which they are consumed is
/// fixed by `dwg_decode_object` in LibreDWG's `decode.c`:
///
/// 1. `bitsize(RL)`, `handle(H)`, EED — read by the caller.
/// 2. **`common_object_handle_data.spec`** (`decode.c`: `#include` right after
///    the EED, *before* any object-specific field): `num_reactors(BL)`,
///    `is_xdic_missing(B, R_2004a+)` and `has_ds_data(B, R_2013+)` — see
///    [`read_object_prologue`]. They live on the **field** stream, not the
///    handle stream — they only *govern* the handles that follow.
/// 3. The object-specific fields: `COMMON_TABLE_FLAGS` (`name`,
///    `is_xref_resolved`), `flag0(BS)`, `color(CMC)`.
///
/// ```text
/// field stream:  num_reactors(BL)  is_xdic_missing(B)  has_ds_data(B, R_2013)
///                name(T)                                   // string stream, R2007
///                is_xref_resolved(BS)   [R2004+: ref/dep gone]
///                flag0(BS, SINCE R_2000b)   color(CMC, SINCE R_13b1)
/// handle stream: ownerhandle(4)  REACTORS×num_reactors  XDICOBJHANDLE(3)
///                xref(5)  plotstyle(5, R_2000b)  material(5, R_2007a)  ltype(5, R_13b1)
/// ```
///
/// Getting (2) wrong is the classic trap: it costs 3 bits (4 on R2013+) in the
/// common case (`num_reactors == 0` compacts to a 2-bit `10`, plus the `B`s),
/// which silently shifts `flag0`, the CMC `rgb` and the whole string stream.
///
/// Verified bit-exact against `example_2007.dwg`, LAYER handle `0x10`
/// (`"0"`): field stream 60 → 127, string stream 127 → 153, handle stream
/// 170 → 274 giving `flag0 = 1008` (plotflag + lineweight 31 = default),
/// `rgb = 0xC3000007` (ACI index 7), `owner = 0x2`, `xdict = 0x2B7`,
/// `plotstyle = 0xF`, `material = 0x279`, `ltype = 0x16`.
fn decode_layer(
    rd: &mut BitReader,
    hr: &mut BitReader,
    str_rd: &mut Option<BitReader>,
    handle_value: u64,
    variant: EntityVariant,
) -> Option<LayerInfo> {
    // --- common_object_handle_data (field stream, before everything else) ---
    let (num_reactors, is_xdic_missing) = read_object_prologue(rd, variant);

    // COMMON_TABLE_FLAGS: name first, then the xref flags. Only R2007+ moves an
    // object's `T` fields onto the string stream (`str_rd`); R2004 keeps them
    // inline on the field stream — see [`read_t`].
    let name = read_t(rd, str_rd, variant, variant >= EntityVariant::R2007);
    // `COMMON_TABLE_FLAGS` (LibreDWG `src/spec.h`) — and note the version gate:
    // it is **`UNTIL (R_2004)`**, so AC1018 still belongs to the OLD branch.
    // ```c
    //   FIELD_T (name, 2);
    //   UNTIL (R_2004) {  FIELD_B (is_xref_ref); FIELD_BS (is_xref_resolved);
    //                     FIELD_B (is_xref_dep); }
    //   LATER_VERSIONS  { is_xref_ref = 1; FIELD_BS (is_xref_resolved); }
    // ```
    // Reading only one `BS` on AC1018 costs 2 bits and silently corrupts
    // `flag0`, the CMC colour and the linetype for every layer in the drawing.
    if variant >= EntityVariant::R2007 {
        let _is_xref_resolved = rd.read_bs(); // 0 or 256
    } else {
        let _is_xref_ref = rd.read_b(); // "always 1"
        let _is_xref_resolved = rd.read_bs(); // 0 or 256
        let _is_xref_dep = rd.read_b();
    }
    // SINCE(R_2000b): flag0.  SINCE(R_13b1): color — a table record keeps
    // using `CMC`, even in R2007 (only *entities* switched to `ENC`).
    let flag0 = rd.read_bs() as u16;
    let color = read_cmc(rd, str_rd, variant);
    // --- handle stream ---------------------------------------------------
    // `common_object_handle_data` reads its *handles* from `hdl_dat`
    // (positioned at `bitsize`), and it runs before the object-specific spec,
    // so owner/xdict come first and `xref` follows them.
    let _owner = hr.read_h();
    for _ in 0..num_reactors.min(1_000_000) {
        hr.read_h();
    }
    let ltype: crate::DwgHandle;
    let plotstyle_handle: Option<u64>;
    if variant >= EntityVariant::R2004 {
        if !is_xdic_missing {
            let _xdict = hr.read_h();
        }
        let _xref = hr.read_h();
        let plotstyle = hr.read_h();
        // SINCE(R_2007a): a `material` handle sits between plotstyle and ltype.
        // Missing it makes every layer report its material as its linetype —
        // but it is *absent* on R2004, where adding it would do the same.
        if variant >= EntityVariant::R2007 {
            let _material = hr.read_h();
        }
        ltype = hr.read_h();
        plotstyle_handle = Some(plotstyle.value);
    } else {
        let _xdict = hr.read_h();
        let _xref = hr.read_h();
        let _plotstyle = hr.read_h();
        ltype = hr.read_h();
        plotstyle_handle = None;
    }
    Some(LayerInfo {
        handle: handle_value,
        name,
        color_index: color.index,
        flags: flag0,
        frozen: flag0 & 1 != 0,
        off: flag0 & 2 != 0,
        frozen_in_new: flag0 & 4 != 0,
        locked: flag0 & 8 != 0,
        plot_flag: flag0 & 16 != 0,
        lineweight: ((flag0 & 0x03E0) >> 5) as u8,
        ltype_handle: if ltype.value == 0 { None } else { Some(ltype.value) },
        plotstyle_handle,
    })
}

/// Decode a `BLOCK_HEADER` table object (type 0x31) — we only need its **name**.
///
/// A `BLOCK_HEADER` is the block *record*; its name is what an `INSERT`
/// entity's `block_header` handle points at, so resolving INSERTs requires it.
///
/// Field order — the generic `common_object_handle_data.spec`, which
/// `dwg_decode_object` includes *before* the type-specific spec, followed by
/// `COMMON_TABLE_FLAGS`:
///
/// ```text
/// field stream:  num_reactors(BL)   is_xdic_missing(B, R_2004a+)
///                has_ds_data(B, R_2013+)
///                name(T)                                 // COMMON_TABLE_FLAGS
///                anonymous(B) hasattrs(B) blkisxref(B) xrefoverlaid(B)
///                xref_loaded(B, R_2000b)
///                base_pt(3BD)  xref_pname(T)  num_inserts(RL)  ...
/// ```
///
/// We stop right after the name: everything past it (`base_pt`, `xref_pname`,
/// `num_inserts`, `description`, `preview`) is not needed to name a block, and
/// skipping it keeps us robust against per-version field differences. The
/// handle stream is never touched.
///
/// NOTE: `num_reactors` sits on the **field** stream for every version — it
/// never moved to the handle stream. R2007 hid the mistake for a long time
/// because an R2007 object's `T` fields live on the *separate* string stream,
/// so skipping those bits did not shift the name. On R2004 the name is an
/// inline `TV` on the field stream, so skipping them corrupts it and every
/// INSERT ends up with an empty block name.
fn decode_block_header(
    rd: &mut BitReader,
    str_rd: &mut Option<BitReader>,
    variant: EntityVariant,
) -> Option<String> {
    // `common_object_handle_data.spec`: modular `BL`, then SINCE(R_2004a) the
    // `is_xdic_missing` bit and SINCE(R_2013) `has_ds_data`. All on the field
    // stream.
    let _prologue = read_object_prologue(rd, variant);
    // Only R2007+ moves an object's `T` fields into the separate string stream;
    // R2004 and R2000 have none at all, so the name is an inline `TV` on the
    // field stream. Passing `true` there would silently yield an empty name.
    let name = read_t(rd, str_rd, variant, variant >= EntityVariant::R2007);
    if name.is_empty() {
        None
    } else {
        Some(name)
    }
}

/// Fill in `Geometry::Insert::block_name` by resolving each INSERT's
/// `block_handle` through the BLOCK_HEADER `handle -> name` map.
///
/// The binary format stores INSERTs with only the block_header *handle*, so
/// without this step every INSERT's name would be an empty string. Kept as a
/// standalone function so it can be unit-tested even though the bundled
/// fixture happens to contain no INSERT entities.
fn resolve_insert_block_names(objects: &mut [ParsedObject], block_names: &HashMap<u64, String>) {
    for o in objects.iter_mut() {
        let Some(bh) = o.block_handle else { continue };
        let Some(name) = block_names.get(&bh) else { continue };
        if let Some(Geometry::Insert { block_name, .. }) = &mut o.geometry {
            *block_name = name.clone();
        }
    }
}

/// Which fixed type codes open a polyline group (header followed by vertices).
fn is_polyline(tc: u16) -> bool {
    matches!(tc, 0x0f | 0x10 | 0x1d | 0x1e)
}

/// Which fixed type codes are polyline vertices.
fn is_vertex(tc: u16) -> bool {
    matches!(tc, 0x0a | 0x0b | 0x0c | 0x0d | 0x0e)
}

/// Whether a vertex of type `v_tc` belongs to a polyline of type `pl_tc`.
fn vertex_fits(pl_tc: u16, v_tc: u16) -> bool {
    matches!(
        (pl_tc, v_tc),
        (0x0f, 0x0a) | (0x10, 0x0b) | (0x1e, 0x0c) | (0x1d, 0x0d) | (0x1d, 0x0e)
    )
}

/// Assemble one `POLYLINE_*` header with its chained vertices into a single
/// `Geometry::Polyline`. `vidxs` are the indices (in `objects`) of the vertex
/// objects to stitch on, in order. The vertex objects keep their own data; only
/// the header's `geometry` is populated.
fn assemble_one_polyline(objects: &mut [ParsedObject], pl_idx: usize, vidxs: &[usize]) {
    let mut vertices: Vec<(f64, f64, f64)> = Vec::with_capacity(vidxs.len());
    let mut bulges: Vec<f64> = Vec::with_capacity(vidxs.len());
    for &vi in vidxs {
        if let Some(v) = &objects[vi].vertex {
            // `VERTEX_PFACE_FACE` (0x0e) carries only face indices, no point.
            if v.face_indices.is_some() {
                continue;
            }
            vertices.push(v.point);
            bulges.push(v.bulge);
        }
    }
    let Some(header) = objects[pl_idx].polyline.clone() else {
        return;
    };
    objects[pl_idx].geometry = Some(Geometry::Polyline {
        flag: header.flag,
        curve_type: header.curve_type,
        elevation: header.elevation,
        is_3d: header.is_3d,
        vertices,
        bulges,
    });
}

/// Stitch each `POLYLINE_*` with its vertices.
///
/// Primary method: resolve the `vertex_handles` captured from the polyline's
/// handle stream (`HANDLE_VECTOR`, R2004+). This is authoritative even when the
/// VERTEX objects are *not* stored contiguously after the polyline (some R2007
/// files place the `SEQEND` before the vertices). When `vertex_handles` is empty
/// (R2000, which only carries first/last handles) we fall back to sequential
/// grouping of the contiguous run of compatible `VERTEX_*` objects.
fn assemble_polylines(objects: &mut [ParsedObject], handle_index: &HashMap<u64, usize>) {
    // Resolve handle-based groups first.
    let mut seq_remaining: Vec<usize> = (0..objects.len()).collect();
    for i in 0..objects.len() {
        let Some(header) = objects[i].polyline.clone() else {
            continue;
        };
        if header.vertex_handles.is_empty() {
            continue;
        }
        let mut vidxs: Vec<usize> = Vec::with_capacity(header.vertex_handles.len());
        for &h in &header.vertex_handles {
            if let Some(&vi) = handle_index.get(&h) {
                vidxs.push(vi);
                // Mark this vertex as consumed so the sequential pass skips it.
                if let Some(pos) = seq_remaining.iter().position(|&x| x == vi) {
                    seq_remaining.swap_remove(pos);
                }
            }
        }
        assemble_one_polyline(objects, i, &vidxs);
        // This polyline is now fully resolved; keep it out of the sequential
        // pass (otherwise the fallback would re-finalize it with an empty list
        // and wipe the vertices we just stitched on).
        if let Some(pos) = seq_remaining.iter().position(|&x| x == i) {
            seq_remaining.swap_remove(pos);
        }
    }

    // Sequential fallback for polylines that have no vertex handle vector.
    let mut current: Option<usize> = None;
    let mut collected: Vec<usize> = Vec::new();
    for &i in &seq_remaining {
        let tc = objects[i].type_code;
        if is_polyline(tc) {
            if let Some(pi) = current.take() {
                assemble_one_polyline(objects, pi, &collected);
            }
            current = Some(i);
            collected.clear();
        } else if tc == 0x06 {
            if let Some(pi) = current.take() {
                assemble_one_polyline(objects, pi, &collected);
            }
            collected.clear();
        } else if is_vertex(tc) {
            if let Some(pi) = current {
                if vertex_fits(objects[pi].type_code, tc) && tc != 0x0e {
                    collected.push(i);
                }
            }
        }
    }
    if let Some(pi) = current.take() {
        assemble_one_polyline(objects, pi, &collected);
    }
}

/// Decode one object record at absolute file offset `offset`.
fn parse_object(file: &[u8], offset: usize, index: usize) -> Option<ParsedObject> {
    // Object record: MS size, then `size` bytes of data.
    let mut head = BitReader::new(file);
    head.byte = offset;
    let size = head.read_ms() as usize;
    let obj_start = head.byte; // head was seeded at `offset`; this is already absolute
    if size == 0 || obj_start + size > file.len() {
        return None;
    }
    let obj_data = &file[obj_start..obj_start + size];

    // Within obj_data: [0..2) type (BS), [2..6) RL bitsize, [6..) field data.
    // Reading type then bitsize lands us exactly at byte 6.
    let mut rd = BitReader::at(obj_data, 0, 0);
    let type_code = rd.read_bs();
    let bitsize = rd.read_rl() as usize;
    let hs_byte = bitsize / 8;
    let hs_bit = (bitsize % 8) as u32;
    if hs_byte > obj_data.len() {
        return None;
    }
    let mut hr = BitReader::at(obj_data, hs_byte, hs_bit);

    // First field-data items: the object's own handle, then its EED.
    let self_handle = rd.read_h();
    let eed: Vec<Eed> = decode_eed(&mut rd, Dialect::R2000);

    let mut obj = ParsedObject {
        index,
        type_code,
        type_name: type_name(type_code).to_string(),
        handle: self_handle.value,
        layer_handle: None,
        owner_handle: None,
        xdict_handle: None,
        reactors: Vec::new(),
        block_handle: None,
        block_name: None,
        geometry: None,
        text: None,
        layer: None,
        vertex: None,
        polyline: None,
        dictionary: None,
        xdata: Vec::new(),
        cloning: None,
        eed,
    };

    match type_code {
        0x31 => {
            // BLOCK_HEADER: capture the block (record) name so INSERT entities
            // can be resolved from their block_header handle further below.
            obj.block_name = decode_block_header(&mut rd, &mut None, EntityVariant::PreR2004);
        }
        0x2a => {
            decode_dictionary(&mut rd, &mut None, &mut hr, &mut obj, EntityVariant::PreR2004);
        }
        0x33 => {
            if let Some(layer) = decode_layer(
                &mut rd,
                &mut hr,
                &mut None,
                self_handle.value,
                EntityVariant::PreR2004,
            ) {
                obj.layer = Some(layer);
            }
        }
        0x4f => {
            decode_xrecord(&mut rd, &mut hr, hs_byte, &mut obj, EntityVariant::PreR2004);
        }
        // Known entity types: common entity data, layer handle, geometry.
        0x01 | 0x07 | 0x0a | 0x0b | 0x0c | 0x0d | 0x0e | 0x0f | 0x10 | 0x11 | 0x12
        | 0x13 | 0x1b | 0x1c | 0x1d | 0x1e | 0x1f | 0x20 | 0x23 | 0x2c | 0x4d => {
            let flags = read_common_entity_fields(&mut rd, &mut None, EntityVariant::PreR2004);
            let refs = read_entity_common_handles(&mut hr, self_handle.value, &flags);
            obj.layer_handle = refs.layer;
            obj.owner_handle = refs.owner;
            obj.xdict_handle = refs.xdict;
            obj.reactors = refs.reactors;
            match type_code {
                0x13 => decode_line(&mut rd, &mut obj),
                0x12 => decode_circle(&mut rd, &mut obj),
                0x11 => decode_arc(&mut rd, &mut obj),
                0x1b => decode_point(&mut rd, &mut obj),
                0x4d => decode_lwpolyline(&mut rd, &mut obj, Dialect::R2000),
                0x01 => decode_text(&mut rd, &mut None, &mut obj, EntityVariant::PreR2004),
                0x07 => decode_insert(&mut rd, &mut hr, &mut obj),
                0x0a | 0x0b | 0x0c | 0x0d | 0x0e => decode_vertex(&mut rd, &mut obj, type_code),
                0x0f | 0x10 | 0x1d | 0x1e => {
                    decode_polyline_header(&mut rd, &mut hr, &mut obj, type_code, Dialect::R2000)
                }
                0x1c => decode_3dface(&mut rd, &mut obj),
                0x1f | 0x20 => decode_solid(&mut rd, &mut obj),
                0x23 => decode_ellipse(&mut rd, &mut obj),
                0x2c => decode_mtext(&mut rd, &mut None, &mut obj, EntityVariant::PreR2004),
                _ => {}
            }
        }
        // Anything else: kept as handle + EED only (treated as a plain object).
        _ => {}
    }

    Some(obj)
}

// --- XRECORD (XData) -------------------------------------------------------

/// Decode the `XRECORD` (0x4f) object body — the file's **XData**.
///
/// XRECORD lives in `src/dwg2.spec`, not `dwg.spec`, and its own body starts
/// with `xdata_size` — everything before that is the generic object prologue
/// that `dwg_decode_object` reads for *every* object:
///
/// ```text
/// --- common_object_handle_data.spec (field stream) ---
/// BL num_reactors          SINCE(R_13b1)
/// B  is_xdic_missing       SINCE(R_2004a)
/// B  has_ds_data           SINCE(R_2013)
/// --- dwg2.spec: XRECORD (field stream) --------------
/// BL xdata_size
/// XDATA (xdata_size bytes)        <- dwg_decode_xdata
/// BS cloning (DXF 280)            SINCE(R_2000b)
/// --- handle stream -----------------------------------
/// ownerhandle(4)  REACTORS x num_reactors  XDICOBJHANDLE(3, if !is_xdic_missing)
/// objid_handles...                          until handlestream_size
/// ```
///
/// `num_reactors` and `xdata_size` are modular **`BL`**, not `BS`. The two
/// encodings happen to agree for every value in the fixtures, which is why the
/// wrong read looked right for so long — until the missing `is_xdic_missing`
/// bit shifted them on AC1018+ and made `xdata_size` come out as 0.
///
/// Proof for `sample_2000.dwg`: the field widths sum *exactly* to `bitsize`:
/// idx=52 → 42 header + 16 handle + 2 EED + 10 + 10 + 80 (xdata) + 10 = 170;
/// idx=55 → 42 + 16 + 2 + 10 + 10 + 568 + 10 = 658.
/// Ground truth (`dwgread -v4`, XRECORD handle `0.2.26B` in
/// `example_2007.dwg`):
///
/// ```text
/// num_reactors: 1 [BL 0]
/// is_xdic_missing: 1 [B 0]
/// xdata_size: 8 [BL 0]
///   xdata[0] type: 270 [RS]   xdata[0]: 1 [RS 270]
///   xdata[1] type: 271 [RS]   xdata[1]: 1 [RS 271]
/// cloning: 1 [BS 280]
/// num_objid_handles: 0 [BL]
/// ```
fn decode_xrecord(
    rd: &mut BitReader,
    hr: &mut BitReader,
    hs_byte: usize,
    obj: &mut ParsedObject,
    variant: EntityVariant,
) {
    // `common_object_handle_data.spec` — `dwg_decode_object` reads it for
    // *every* object, XRECORD included. Skipping it is what made AC1018+ come
    // out empty: the 1–2 missing bits land inside `xdata_size`.
    let (num_reactors, is_xdic_missing) = read_object_prologue(rd, variant);

    // `dwg2.spec`: `FIELD_BL (xdata_size, 0)` — a modular **BL**, not a BS.
    // The two agree for small values, which is why the mistake survived on
    // AC1015 and only showed up once the prologue was in place.
    let xdata_size = rd.read_bl() as usize;
    if xdata_size > 0 {
        // Never let a bogus size run past the handle stream.
        let xdata_size = xdata_size.min(hs_byte.saturating_sub(rd.byte));
        // R2007+ stores XData strings as UCS-2LE (TU); older releases use the
        // `RS length + RC codepage + bytes` TV form. See `decode_xdata`.
        let dialect = if variant >= EntityVariant::R2007 {
            Dialect::R2007
        } else {
            Dialect::R2000
        };
        obj.xdata = decode_xdata(rd, xdata_size, dialect);
    }
    // SINCE(R_2000b): `FIELD_BS (cloning, 280)`. `decode_xdata` leaves the
    // cursor at the byte end *with the bit offset preserved*, so this lands on
    // the right bit — see the note in `xdata.rs`. `cloning` is a property of the
    // XRECORD object itself, NOT a resbuf group: LibreDWG decodes it *after* the
    // XData chain as a separate `BS cloning` field and never folds it into the
    // XData list. We keep it in its own field (`obj.cloning`) rather than
    // pretending it is XData group 280 — that conflation made every XRECORD's
    // XData end with a spurious `280=1` and masked the true resbuf contents.
    let cloning = rd.read_bs() as i16;
    obj.cloning = Some(cloning);

    // Handle stream: ownerhandle(4), REACTORS x num_reactors,
    // XDICOBJHANDLE(3) — guarded by `is_xdic_missing`, exactly like every
    // other object — then the XRECORD's own objid handle list.
    // The owner / reactors are what link an XRECORD into its extension-
    // dictionary chain, so keep the resolved values on the object.
    let own = obj.handle;
    let owner_h = hr.read_h();
    {
        let v = resolve_handle(owner_h, own);
        obj.owner_handle = if v == 0 { None } else { Some(v) };
    }
    let nr = num_reactors.min(1_000_000); // defensive: never spin
    for _ in 0..nr {
        let h = hr.read_h();
        let v = resolve_handle(h, own);
        if v != 0 {
            obj.reactors.push(v);
        }
    }
    if !is_xdic_missing {
        let h = hr.read_h();
        let v = resolve_handle(h, own);
        obj.xdict_handle = if v == 0 { None } else { Some(v) };
    }
}

// --- DICTIONARY (named object map) -----------------------------------------

/// Decode the `DICTIONARY` (`0x2a`) object body — the named map that links an
/// entity's extension dictionary to its per-field `XRECORD`s.
///
/// Layout (verified byte-exact on `地块解析_cad2018.dwg` dictionaries `0xE4`,
/// `0xE5`, `0xB0`, `0xAD`, `0xC`; the names are pinyin-abbreviated land-parcel
/// fields — `BSM` 标识码, `BHKSSJ` 保护开始时间, `BHJSSJ` 保护结束时间, … —
/// and each resolves to the `XRECORD` holding that field's value):
/// ```text
/// --- common object prologue (field stream) ---
/// BL num_reactors          SINCE(R_13b1)
/// B  is_xdic_missing       SINCE(R_2004a)
/// B  has_ds_data           SINCE(R_2013)
/// --- dictionary.spec (field stream) ---
/// BS numitems (281)
/// RC encr                  SINCE(R_2004a) — purpose unknown, always skipped
/// BS cloning (280)         SINCE(R_2000b) — BEFORE the texts, not after
/// T  text[numitems]        inline TU (UCS-2LE) on the field stream — even
///                          though the file is R2018 and most objects keep
///                          their strings in the string stream, this writer
///                          (ESRI interop / CASS) stores dictionary keys inline
/// --- handle stream ---
/// H  ownerhandle
/// H  reactors × num_reactors
/// H  xdicobjhandle         (only when !is_xdic_missing)
/// H  itemhandles[numitems]
/// ```
///
/// Reading `cloning` after the texts, or skipping the prologue / the common
/// handle-stream prefix, shifts every name one entry off and maps `BSM` to
/// the wrong `XRECORD` — both mistakes were made and caught via the semantic
/// cross-check above (`BSM` 标识码 → the 19-digit `610621211000011752`).
fn decode_dictionary(
    rd: &mut BitReader,
    str_rd: &mut Option<BitReader>,
    hr: &mut BitReader,
    obj: &mut ParsedObject,
    variant: EntityVariant,
) {
    // Same generic object prologue the XRECORD decoder reads — skipping it is
    // what shifted `numitems` onto the reactor bits.
    let (num_reactors, is_xdic_missing) = read_object_prologue(rd, variant);

    let numitems = rd.read_bs() as usize;
    if variant >= EntityVariant::R2004 {
        // Unknown purpose byte present since AC1018; always skipped.
        let _encr = rd.read_rc();
    }
    // `BS cloning` (280) comes BEFORE the texts — reading it after consumed
    // the first text's length bits and produced a phantom empty name.
    let _cloning = rd.read_bs();

    let cap = numitems.min(100_000);
    let mut names: Vec<String> = Vec::with_capacity(cap);
    for _ in 0..cap {
        // Inline TU on the field stream (`from_string_stream = false`).
        names.push(read_t(rd, str_rd, variant, false));
    }

    // Handle stream: the common owner/reactors/xdict handles come FIRST,
    // then the numitems entry handles.
    let own = obj.handle;
    let owner_h = hr.read_h();
    {
        let v = resolve_handle(owner_h, own);
        obj.owner_handle = if v == 0 { None } else { Some(v) };
    }
    for _ in 0..num_reactors.min(1_000_000) {
        let h = hr.read_h();
        let v = resolve_handle(h, own);
        if v != 0 {
            obj.reactors.push(v);
        }
    }
    if !is_xdic_missing {
        let h = hr.read_h();
        let v = resolve_handle(h, own);
        obj.xdict_handle = if v == 0 { None } else { Some(v) };
    }
    let mut entries = Vec::with_capacity(cap);
    for name in names {
        let h = hr.read_h();
        let v = resolve_handle(h, own);
        entries.push(DictionaryEntry { name, handle: v });
    }
    if std::env::var("DWG_DEBUG_DICT").is_ok() {
        eprintln!(
            "[dbg-dict] 0x{:X}: {} entries: {:?}",
            own,
            entries.len(),
            entries.iter().take(6).map(|e| (&e.name, format!("0x{:X}", e.handle))).collect::<Vec<_>>()
        );
    }
    obj.dictionary = Some(entries);
}

// --- entity geometry decoders ----------------------------------------------

fn decode_line(rd: &mut BitReader, obj: &mut ParsedObject) {
    let z_is_zero = rd.read_b() != 0;
    let sx = rd.read_rd();
    let ex = rd.read_dd(sx);
    let sy = rd.read_rd();
    let ey = rd.read_dd(sy);
    let (sz, ez) = if z_is_zero {
        (0.0, 0.0)
    } else {
        let sz = rd.read_rd();
        let ez = rd.read_dd(sz);
        (sz, ez)
    };
    let _thickness = rd.read_bt();
    let _extrusion = rd.read_be();
    obj.geometry = Some(Geometry::Line {
        start: (sx, sy, sz),
        end: (ex, ey, ez),
    });
}

fn decode_circle(rd: &mut BitReader, obj: &mut ParsedObject) {
    let cx = rd.read_bd();
    let cy = rd.read_bd();
    let cz = rd.read_bd();
    let radius = rd.read_bd();
    let _thickness = rd.read_bt();
    let _extrusion = rd.read_be();
    obj.geometry = Some(Geometry::Circle {
        center: (cx, cy, cz),
        radius,
    });
}

fn decode_arc(rd: &mut BitReader, obj: &mut ParsedObject) {
    let cx = rd.read_bd();
    let cy = rd.read_bd();
    let cz = rd.read_bd();
    let radius = rd.read_bd();
    let _thickness = rd.read_bt();
    let _extrusion = rd.read_be();
    let start_angle = rd.read_bd();
    let end_angle = rd.read_bd();
    obj.geometry = Some(Geometry::Arc {
        center: (cx, cy, cz),
        radius,
        start_angle,
        end_angle,
    });
}

fn decode_point(rd: &mut BitReader, obj: &mut ParsedObject) {
    let x = rd.read_bd();
    let y = rd.read_bd();
    let z = rd.read_bd();
    let _thickness = rd.read_bt();
    let _extrusion = rd.read_be();
    let _x_ang = rd.read_bd();
    obj.geometry = Some(Geometry::Point { x, y, z });
}

fn decode_lwpolyline(rd: &mut BitReader, obj: &mut ParsedObject, dialect: Dialect) {
    // Verified against `DWG_ENTITY(LWPOLYLINE)` in LibreDWG's `dwg.spec`:
    //   BS flag (70);  +BD const_width if flag&4; +BD elevation if flag&8;
    //   +BD thickness if flag&2; +3BD extrusion if flag&1;  BL num_points (90);
    //   +BL num_bulges if flag&16; +BL num_widths if flag&32;
    //   points as a 2DD vector (first point raw RD/RD, later ones DD deltas);
    //   then num_bulges x BD bulges and num_widths x 2BD widths.
    let flag = rd.read_bs();
    if flag & 4 != 0 {
        let _const_width = rd.read_bd();
    }
    if flag & 8 != 0 {
        let _elevation = rd.read_bd();
    }
    if flag & 2 != 0 {
        let _thickness = rd.read_bd();
    }
    if flag & 1 != 0 {
        let _ex = (rd.read_bd(), rd.read_bd(), rd.read_bd());
    }
    // `num_points` is a *compact* `BS` on R2000 (a long-standing AC1015 quirk —
    // reading it as `BL` yields 3089706561 on `sample_2000.dwg`), but a modular
    // `BL` on R2007. `num_bulges`/`num_widths` are modular `BL` on *both*
    // versions (see `dwg.spec`). Getting the width of `num_points` wrong only
    // under-reads by 16 bits for large counts (e.g. the 3635-vertex polyline
    // handle #442), but that is enough to throw every vertex's doubles onto a
    // non-byte boundary and emit garbage first coordinates.
    let num_points = if dialect == Dialect::R2000 {
        rd.read_bs() as usize
    } else {
        rd.read_bl() as usize
    };
    // `num_bulges`/`num_widths` are *modular* `BL` (per `dwg.spec`), NOT the
    // compact `BS` that `num_points` uses. Reading them as `BS` is harmless
    // when the value is 0 (a `BL`/`BS` code-2 both consume 2 bits), but for a
    // non-zero count it under-reads by 16 bits and shifts every vertex — which
    // is exactly what produced the garbage first vertex on the 3635-vertex
    // LWPOLYLINE (handle #442): its `num_bulges` is 3635, not 0.
    let num_bulges = if flag & 16 != 0 { rd.read_bl() as usize } else { 0 };
    let num_widths = if flag & 32 != 0 { rd.read_bl() as usize } else { 0 };
    // LibreDWG caps this at 20000 (`VALUEOUTOFBOUNDS`).
    let mut points: Vec<(f64, f64)> = Vec::new();
    if num_points > 0 && num_points <= 20_000 {
        // First vertex is raw 2RD; the rest are DD relative to the previous one.
        let px = rd.read_rd();
        let py = rd.read_rd();
        points.push((px, py));
        for i in 1..num_points {
            let x = rd.read_dd(points[i - 1].0);
            let y = rd.read_dd(points[i - 1].1);
            points.push((x, y));
        }
    }
    // Bulges (DXF group 42) ride BEFORE the widths and define circular arcs
    // between consecutive vertices. Keep them so the converter can reconstruct
    // the true curved boundary instead of a straight chord.
    let mut bulges: Vec<f64> = Vec::with_capacity(num_bulges.min(20_000));
    for _ in 0..num_bulges.min(20_000) {
        bulges.push(rd.read_bd());
    }
    for _ in 0..num_widths.min(20_000) {
        let _start_width = rd.read_bd();
        let _end_width = rd.read_bd();
    }
    obj.geometry = Some(Geometry::LwPolyline {
        flag,
        points,
        bulges,
    });
}

fn decode_text(
    rd: &mut BitReader,
    str_rd: &mut Option<BitReader>,
    obj: &mut ParsedObject,
    variant: EntityVariant,
) {
    let dataflags = rd.read_rc();
    if dataflags & 0x01 == 0 {
        let _elevation = rd.read_rd();
    }
    let ins_x = rd.read_rd();
    let ins_y = rd.read_rd();
    if dataflags & 0x02 == 0 {
        let _ax = rd.read_dd(ins_x);
        let _ay = rd.read_dd(ins_y);
    }
    let _extrusion = rd.read_be();
    let _thickness = rd.read_bt();
    if dataflags & 0x04 == 0 {
        let _oblique = rd.read_rd();
    }
    if dataflags & 0x08 == 0 {
        let _rotation = rd.read_rd();
    }
    let height = rd.read_rd();
    if dataflags & 0x10 == 0 {
        let _width = rd.read_rd();
    }
    // R2007+ routes *every* `T` field — entities included — through the
    // object's string stream; R2004 and R2000 keep them inline on the field
    // stream as `TV`.
    let value = read_t(rd, str_rd, variant, variant >= EntityVariant::R2007);
    if dataflags & 0x20 == 0 {
        let _gen = rd.read_bs();
    }
    if dataflags & 0x40 == 0 {
        let _horiz = rd.read_bs();
    }
    if dataflags & 0x80 == 0 {
        let _vert = rd.read_bs();
    }
    obj.geometry = Some(Geometry::Text {
        ins_pt: (ins_x, ins_y),
        height,
        value: value.clone(),
    });
    obj.text = Some(value);
}

/// Decode an `MTEXT` (0x2c) — multi-line text.
///
/// Field-stream order (`dwg.spec`, `SINCE(R_2000b)` branch):
/// `ins_pt(3BD,10)`, `extrusion(3BD,210)`, `x_axis_dir(3BD,11)`, then the
/// **binary** ordering (the spec's `else` branch — DXF writes 40 before 41, but
/// the binary stores `rect_width` first):
/// `rect_width(BD,41)`, `rect_height(BD,46)` — the latter only `SINCE(R_2007a)`,
/// which R2007 (AC1021) satisfies — then `text_height(BD,40)`,
/// `attachment(BS,71)`, `flow_dir(BS,72)`, `extents_height(BD)`,
/// `extents_width(BD)`, and finally `text(T,1)`.
///
/// The remaining trailing fields (`linespace_style/BS`, `linespace_factor/BD`,
/// `unknown_b0/B`, and the R2004+ `bg_fill_*` block gated on `bg_fill_flag & 1`)
/// are deliberately **not** consumed: nothing downstream needs them, and
/// skipping them keeps us robust against the per-version gating.
fn decode_mtext(
    rd: &mut BitReader,
    str_rd: &mut Option<BitReader>,
    obj: &mut ParsedObject,
    variant: EntityVariant,
) {
    let ins_pt = (rd.read_bd(), rd.read_bd(), rd.read_bd());
    let _extrusion = (rd.read_bd(), rd.read_bd(), rd.read_bd());
    let _x_axis_dir = (rd.read_bd(), rd.read_bd(), rd.read_bd());
    let _rect_width = rd.read_bd();
    if variant >= EntityVariant::R2007 {
        // `rect_height` (DXF group 46) — SINCE(R_2007a).
        let _rect_height = rd.read_bd();
    }
    let height = rd.read_bd();
    let _attachment = rd.read_bs();
    let _flow_dir = rd.read_bs();
    let _extents_height = rd.read_bd();
    let _extents_width = rd.read_bd();
    // R2007+ routes every `T` field (entities included) through the object's
    // string stream; R2004 and R2000 keep them inline as `TV`.
    let value = read_t(rd, str_rd, variant, variant >= EntityVariant::R2007);
    obj.geometry = Some(Geometry::MText {
        ins_pt,
        height,
        value: value.clone(),
    });
    obj.text = Some(value);
}

fn decode_insert(rd: &mut BitReader, hr: &mut BitReader, obj: &mut ParsedObject) {
    let ix = rd.read_bd();
    let iy = rd.read_bd();
    let iz = rd.read_bd();
    let scale_flag = rd.read_bb();
    let (sx, sy, sz) = match scale_flag {
        3 => (1.0, 1.0, 1.0),
        1 => {
            // scale.x is fixed to 1.0 and not stored; y/z are DD deltas
            // relative to the (constant) scale.x = 1.0.
            let sy = rd.read_dd(1.0);
            let sz = rd.read_dd(1.0);
            (1.0, sy, sz)
        }
        2 => {
            let s = rd.read_rd();
            (s, s, s)
        }
        _ => {
            let sx = rd.read_rd();
            let sy = rd.read_dd(sx);
            let sz = rd.read_dd(sx);
            (sx, sy, sz)
        }
    };
    let _rotation = rd.read_bd();
    let _extrusion = (rd.read_bd(), rd.read_bd(), rd.read_bd());
    let _has_attribs = rd.read_b();
    // The block name lives in the BLOCK_HEADER table; in the binary it is
    // reached through the block_header handle, which follows the common
    // entity handles on the handle stream. We capture it so callers can
    // resolve it (the BLOCK_HEADER table maps handle -> name).
    let block = hr.read_h();
    obj.block_handle = if block.value == 0 { None } else { Some(block.value) };
    obj.geometry = Some(Geometry::Insert {
        ins_pt: (ix, iy, iz),
        block_name: String::new(),
        scale: (sx, sy, sz),
    });
}

// --- new entity / polyline decoders ----------------------------------------

/// Decode a `POLYLINE_2D` (0x0f), `POLYLINE_3D` (0x10), `POLYLINE_PFACE` (0x1d)
/// or `POLYLINE_MESH` (0x1e) **header**.
///
/// Field order verified against LibreDWG's `dwg.spec`:
///
/// * `POLYLINE_2D` (`SINCE(R_13b1)`): `flag(BS,70)`, `curve_type(BS,75)`,
///   `start_width(BD,40)`, `end_width(BD,41)`, `thickness(BT,39)`,
///   `elevation(BD,38)`, `extrusion(BE,210)` — then `num_owned(BL,0)` on
///   R2004+ (the count of owned vertices; consumed to stay aligned, but vertex
///   assembly here is done by sequential grouping, not by following handles).
/// * `POLYLINE_3D` (`LATER_VERSIONS`): `curve_type(RC,75)`, `flag(RC,0)`, then
///   `num_owned(BL,0)` on R2004+.
///
/// The vertex/seqend *handles* live on the handle stream; we do not read them
/// because every object is decoded independently and the vertices are gathered
/// in `assemble_polylines` by walking the object stream in order.
fn decode_polyline_header(
    rd: &mut BitReader,
    hr: &mut BitReader,
    obj: &mut ParsedObject,
    type_code: u16,
    dialect: Dialect,
) {
    // Field-stream order differs per polyline subtype (`dwg.spec`):
    //  * POLYLINE_2D (0x0f): flag(BS), curve_type(BS), start_width(BD),
    //    end_width(BD), thickness(BT), elevation(BD), extrusion(BE).
    //  * POLYLINE_3D (0x10): curve_type(RC), flag(RC).
    //  * POLYLINE_PFACE (0x1d): numverts(BS), numfaces(BS).
    //  * POLYLINE_MESH (0x1e): flag(BS), curve_type(BS), num_m_verts(BS),
    //    num_n_verts(BS), m_density(BS), n_density(BS).
    // Every subtype then carries `num_owned(BL)` on R2004+ (the count of owned
    // vertices), followed on the *handle* stream by that many vertex handles and
    // the seqend handle. We capture the vertex handles and stitch the points
    // back on in `assemble_polylines` — authoritative regardless of where the
    // VERTEX objects sit in the stream (some files place the SEQEND before
    // them). R2000 carries first/last handles instead, so vertex assembly there
    // falls back to sequential grouping.
    let mut flag: u16 = 0;
    let mut curve_type: u16 = 0;
    let mut elevation = 0.0f64;
    let is_3d = matches!(type_code, 0x10 | 0x1e);
    match type_code {
        0x0f => {
            flag = rd.read_bs() as u16;
            curve_type = rd.read_bs() as u16;
            let _start_width = rd.read_bd();
            let _end_width = rd.read_bd();
            let _thickness = rd.read_bt();
            elevation = rd.read_bd();
            let _extrusion = rd.read_be();
        }
        0x10 => {
            curve_type = rd.read_rc() as u16;
            flag = rd.read_rc() as u16;
        }
        0x1d => {
            let _numverts = rd.read_bs() as u16;
            let _numfaces = rd.read_bs() as u16;
        }
        0x1e => {
            flag = rd.read_bs() as u16;
            curve_type = rd.read_bs() as u16;
            let _num_m_verts = rd.read_bs() as u16;
            let _num_n_verts = rd.read_bs() as u16;
            let _m_density = rd.read_bs() as u16;
            let _n_density = rd.read_bs() as u16;
        }
        _ => {}
    }
    let mut vertex_handles: Vec<u64> = Vec::new();
    if dialect == Dialect::R2007 {
        let num_owned = rd.read_bl() as usize;
        for _ in 0..num_owned.min(1_000_000) {
            let h = hr.read_h();
            vertex_handles.push(resolve_handle(h, obj.handle));
        }
        let _seqend = hr.read_h();
    }
    obj.polyline = Some(PolylineHeader {
        flag,
        curve_type,
        elevation,
        is_3d,
        vertex_handles,
    });
}

/// Decode a `VERTEX_2D/3D/MESH/PFACE/PFACE_FACE` point record.
///
/// * `VERTEX_2D` (`SINCE(R_13b1)`): `flag(RC,0)`, `point(3BD,10)`,
///   `start_width(BD,40)`, `end_width(BD,41)`, `bulge(BD,42)`, `tangent(BD,50)`
///   (the `id(BL,91)` only exists on R2010+).
/// * `VERTEX_3D / _MESH / _PFACE` (`LATER_VERSIONS`): `flag(RC,0)`, `point(3BD)`.
/// * `VERTEX_PFACE_FACE` (`LATER_VERSIONS`): `flag` (=128), then four signed
///   `BSd` vertex indices — *not* a point. We still capture the flag; the face
///   topology is preserved via `Geometry::Polyline` for `POLYLINE_PFACE` only
///   when the indices are needed (see `assemble_polylines`).
///
/// The point + bulge are stashed on `obj.vertex`; the polyline assembly pass
/// stitches them onto the parent header.
fn decode_vertex(rd: &mut BitReader, obj: &mut ParsedObject, type_code: u16) {
    // `VERTEX_PFACE_FACE` (0x0e) is not a point at all: it carries the four
    // signed vertex indices that form a face of a polyface mesh. Reading a 3BD
    // here would consume those indices as a bogus point.
    if type_code == 0x0e {
        let i0 = rd.read_bs() as i16;
        let i1 = rd.read_bs() as i16;
        let i2 = rd.read_bs() as i16;
        let i3 = rd.read_bs() as i16;
        obj.vertex = Some(VertexData {
            point: (0.0, 0.0, 0.0),
            bulge: 0.0,
            flag: 128,
            width_start: 0.0,
            width_end: 0.0,
            face_indices: Some([i0, i1, i2, i3]),
        });
        return;
    }
    let flag = rd.read_rc() as u16;
    let x = rd.read_bd();
    let y = rd.read_bd();
    let z = rd.read_bd();
    let (mut bulge, mut width_start, mut width_end) = (0.0f64, 0.0f64, 0.0f64);
    if type_code == 0x0a {
        // VERTEX_2D carries widths + bulge (+ tangent) after the point.
        width_start = rd.read_bd();
        width_end = rd.read_bd();
        bulge = rd.read_bd();
        let _tangent = rd.read_bd();
    }
    obj.vertex = Some(VertexData {
        point: (x, y, z),
        bulge,
        flag,
        width_start,
        width_end,
        face_indices: None,
    });
}

/// Decode an `ELLIPSE` (0x23): `center(3BD,10)`, `sm_axis(3BD,11)` (endpoint of
/// the major axis, relative to `center`), `extrusion(3BD,210)`, `axis_ratio(BD,
/// 40)`, `start_angle(BD,41)`, `end_angle(BD,42)` — all on the field stream
/// (`dwg.spec`, no sub-version branches). The extrusion normal is dropped here;
/// only the planar geometry is kept.
fn decode_ellipse(rd: &mut BitReader, obj: &mut ParsedObject) {
    let center = (rd.read_bd(), rd.read_bd(), rd.read_bd());
    let major_axis = (rd.read_bd(), rd.read_bd(), rd.read_bd());
    let _extrusion = (rd.read_bd(), rd.read_bd(), rd.read_bd());
    let axis_ratio = rd.read_bd();
    let start_angle = rd.read_bd();
    let end_angle = rd.read_bd();
    obj.geometry = Some(Geometry::Ellipse {
        center,
        major_axis,
        axis_ratio,
        start_angle,
        end_angle,
    });
}

/// Decode a `3DFACE` (0x1c, spec name `_3DFACE`), `SINCE(R_2000b)` branch:
/// `has_no_flags(B)`, `z_is_zero(B)`, `corner1(RD,RD[,RD])`, then `corner2..4`
/// as `3DD` deltas relative to the previous corner, and `invis_flags(BS,70)`
/// when `!has_no_flags`.
fn decode_3dface(rd: &mut BitReader, obj: &mut ParsedObject) {
    let has_no_flags = rd.read_b() != 0;
    let z_is_zero = rd.read_b() != 0;
    let c1x = rd.read_rd();
    let c1y = rd.read_rd();
    let c1z = if z_is_zero { 0.0 } else { rd.read_rd() };
    let c1 = (c1x, c1y, c1z);
    let c2 = (rd.read_dd(c1.0), rd.read_dd(c1.1), rd.read_dd(c1.2));
    let c3 = (rd.read_dd(c2.0), rd.read_dd(c2.1), rd.read_dd(c2.2));
    let c4 = (rd.read_dd(c3.0), rd.read_dd(c3.1), rd.read_dd(c3.2));
    let invisibility = if !has_no_flags { rd.read_bs() as u16 } else { 0 };
    obj.geometry = Some(Geometry::Face3d {
        corners: [c1, c2, c3, c4],
        invisibility,
    });
}

/// Decode a `SOLID` (0x1f) or `TRACE` (0x20). Both share the identical
/// `LATER_VERSIONS` field block: `thickness(BT,39)`, `elevation(BD,38)`, four
/// `corner(2RD)` points, `extrusion(BE,210)`. The four corners share the
/// elevation as their z.
fn decode_solid(rd: &mut BitReader, obj: &mut ParsedObject) {
    let _thickness = rd.read_bt();
    let elevation = rd.read_bd();
    let c1 = (rd.read_rd(), rd.read_rd(), elevation);
    let c2 = (rd.read_rd(), rd.read_rd(), elevation);
    let c3 = (rd.read_rd(), rd.read_rd(), elevation);
    let c4 = (rd.read_rd(), rd.read_rd(), elevation);
    let _extrusion = rd.read_be();
    obj.geometry = Some(Geometry::Solid {
        corners: [c1, c2, c3, c4],
        elevation,
    });
}

// --- top-level -------------------------------------------------------------

pub fn parse_file(path: &str) -> Result<DwgFile, String> {
    let file = std::fs::read(path).map_err(|e| format!("cannot read {path}: {e}"))?;
    if file.len() < 30 {
        return Err("file too small to be a DWG".into());
    }
    let version = String::from_utf8_lossy(&file[0..6]).into_owned();
    match version.as_str() {
        // AC1015 = R2000: flat sections, objects addressed by absolute offset.
        "AC1015" => parse_r2000(&file, version),
        //
        // AC1021 (R2007) is the *page container* — section memory image,
        // Reed–Solomon interleaved, LZ77 compressed. Only AC1021 uses it.
        //
        // AC1018 (R2004) and every later release up to AC1032 reuse the R2004
        // *section-tree* container (LibreDWG funnels them all through
        // `decode_R2004`). The container differs from R2007, but the object
        // stream it produces is byte-identical in encoding, so one decoder
        // covers 2004 … 2024.
        //
        // Release coverage worth spelling out, because the stamps are one-to-
        // many and this trips people up:
        //
        //   AC1015  2000 / 2000i / 2002
        //   AC1018  2004 / 2005 / 2006   <- R2004 container
        //   AC1021  2007 / 2008 / 2009   <- R2007 page container
        //   AC1024  2010 / 2011 / 2012   <- R2004 container (reused)
        //   AC1027  2013 … 2017           <- R2004 container (reused)
        //   AC1032  2018 … 2024           <- R2004 container (reused)
        //
        // So "AutoCAD 2020" is not a format of its own — it writes AC1032, the
        // same stamp as 2018 and 2024, and is handled by the R2004 decoder.
        //
        // AC1037 (AutoCAD 2024+) is a newer format that has never been
        // verified against a real sample, so it stays rejected rather than
        // being parsed on a guess.
        "AC1021" => parse_r2007(&file, version),
        "AC1018" | "AC1024" | "AC1027" | "AC1032" => parse_r2004(&file, version),
        other => Err(format!(
            "unsupported DWG version '{other}' (supported: AC1015/R2000-2002, \
             AC1018/R2004-2006, AC1021/R2007-2009, AC1024/R2010-2012, \
             AC1027/R2013-2017, AC1032/R2018-2024 -- AutoCAD 2020 writes \
             AC1032). AC1037/R2024+ uses a different container and is not \
             supported"
        )),
    }
}

/// Shared tail of both parsers: index handles, collect layers, resolve blocks.
fn build_dwg(version: String, mut objects: Vec<ParsedObject>) -> DwgFile {
    // Build handle -> index and gather layers.
    let mut handle_index: HashMap<u64, usize> = HashMap::new();
    let mut layers: Vec<LayerInfo> = Vec::new();
    for (i, o) in objects.iter().enumerate() {
        handle_index.insert(o.handle, i);
        if let Some(l) = &o.layer {
            layers.push(l.clone());
        }
    }

    // BLOCK_HEADER (0x31) records carry the block name; build handle -> name
    // so INSERT entities (which only store the block_header *handle*) can be
    // resolved to a human-readable block name.
    let mut block_names: HashMap<u64, String> = HashMap::new();
    for o in &objects {
        if let Some(n) = &o.block_name {
            block_names.insert(o.handle, n.clone());
        }
    }
    resolve_insert_block_names(&mut objects, &block_names);

    // Stitch chained `VERTEX_*` objects back onto their parent `POLYLINE_*`.
    // Done after block-name resolution; `entities()` then reports the assembled
    // polyline while the (now-consumed) vertices drop out.
    assemble_polylines(&mut objects, &handle_index);

    let object_count = objects.len();
    DwgFile {
        version,
        object_count,
        layers,
        objects,
        handle_index,
        block_names,
    }
}

/// R2000 (AC1015): the sections are laid out flat in the file.
fn parse_r2000(file: &[u8], version: String) -> Result<DwgFile, String> {
    let sections = parse_locator(&file)?;
    // §2 == Handles section.
    let handles_addr = sections
        .get(2)
        .map(|s| s.1 as usize)
        .ok_or("missing Handles section")?;
    let handles_size = sections.get(2).map(|s| s.2 as usize).unwrap_or(0);
    let offsets = parse_handles(&file, handles_addr, handles_size, file.len())?;

    let mut objects: Vec<ParsedObject> = Vec::with_capacity(offsets.len());
    for (i, &off) in offsets.iter().enumerate() {
        if let Some(obj) = parse_object(&file, off, i) {
            objects.push(obj);
        }
    }
    if objects.is_empty() {
        // A real DWG always carries at least the block/layer tables, so an
        // empty object table means the locator or the handle section did not
        // decode — not that the drawing is empty. Reporting success here would
        // be exactly the "read a pile of garbage but look successful" failure
        // mode; a 4 KiB file of repeated bytes used to do just that.
        return Err("no objects decoded from the R2000 object stream".into());
    }

    Ok(build_dwg(version, objects))
}

/// Shared tail for every *container* format (R2007 page container and R2004
/// section-tree): the object stream is encoded identically from R2007 onward, so
/// once a container hands back plain `objects`/`handles` bytes the rest is the
/// same decode.
fn decode_object_stream(
    secs: r2007::R2007Sections,
    version: String,
    variant: EntityVariant,
) -> Result<DwgFile, String> {
    let offsets = parse_handles(&secs.handles, 0, secs.handles.len(), secs.objects.len())?;
    let mut objects: Vec<ParsedObject> = Vec::with_capacity(offsets.len());
    for (i, &off) in offsets.iter().enumerate() {
        if let Some(obj) = parse_object_r2007(&secs.objects, off, i, variant) {
            objects.push(obj);
        }
    }
    if objects.is_empty() {
        return Err("no objects decoded from the object stream".into());
    }
    Ok(build_dwg(version, objects))
}

/// R2007 (AC1021): undo the page container first, then decode the object stream.
/// The object map (Handles section) has the *same* layout as R2000, only its
/// offsets are relative to the (decompressed) objects section.
fn parse_r2007(file: &[u8], version: String) -> Result<DwgFile, String> {
    let secs = r2007::load_sections(file)?;
    let variant = EntityVariant::from_version(&version);
    decode_object_stream(secs, version, variant)
}

/// R2004 (AC1018) and the versions that reuse its section-tree container —
/// AC1024 (R2010), AC1027 (R2013), AC1032 (R2018…2024, incl. CAD2020). The
/// container differs from R2007, but the object stream it yields is byte-for-
/// byte the same encoding, so the object decoder is shared.
fn parse_r2004(file: &[u8], version: String) -> Result<DwgFile, String> {
    let secs = r2004::load_sections(file)?;
    // The R2004 section-tree container is reused by 2004/2005/2006 (AC1018) and
    // 2010…2024 (AC1024/AC1027/AC1032). `EntityVariant` carries both facts the
    // object decoder needs: the R2010+ `BOT`/`UMC` record header, and which
    // conditional blocks of `common_entity_data.spec` are present.
    let variant = EntityVariant::from_version(&version);
    decode_object_stream(secs, version, variant)
}

/// Locate the R2007-only **string stream** of one object.
///
/// Since R2007 every object may keep its strings in a small area at the end of
/// its field data. The area is found by walking *backwards* from the end of the
/// field data (`bitsize`), mirroring `obj_string_stream` in LibreDWG's
/// `decode_r2007.c`:
///
/// ```text
/// pos        = bitsize - 1
/// has_strings= 1 bit                (the very last bit before the handles)
/// data_size  = RS at bitsize - 17   (+ a 16-bit hi part at bitsize - 33)
/// start      = (bitsize - 17) - data_size   (data_size is a *bit* count!)
/// ```
///
/// The string data lives in the tail of the field-data region, just before the
/// handle stream — `data_size` **bits** of `[length][UCS-2 …]` for every `T`
/// field of the object, in field order. Reading is forward from `start`.
///
/// NOTE: although LibreDWG's comment calls `data_size` "in byte", the final
/// `bit_advance_position(str, -(int)data_size)` moves the cursor back by
/// `data_size` **bits** — verified by hand-decoding LAYER 0x10 where
/// `data_size == 26` and the string stream is exactly the 26 bits encoding the
/// name `"0"` (BS length 1 + one UCS-2 unit). Treating it as bytes pushes the
/// cursor before the buffer start and yields `None`.
///
/// Returns `None` when the object has no string area.
fn setup_string_stream(obj_data: &[u8], bitsize: usize) -> Option<BitReader<'_>> {
    if bitsize < 34 {
        return None;
    }
    let mut s = BitReader::at(obj_data, 0, 0);

    // has_strings: the single bit at `bitsize - 1` (the last bit before the
    // handle stream).
    s.set_bitpos(bitsize - 1);
    if s.read_b() == 0 {
        return None; // has_strings == 0
    }

    // Mirror LibreDWG's exact byte/bit arithmetic in `obj_string_stream`:
    //   bit_advance_position(-1)  (back over the has_strings bit)
    //   str->byte -= 2           (back 16 bits, to the size-marker RS)
    //   data_size = RS(16 bits)  (the size marker, at bitsize-17)
    //   if (data_size & 0x8000): byte -= 4; hi = RS; byte -= 2
    //   str->byte -= 2           (common, line 1406)
    //   bit_advance_position(-data_size)   // data_size is in BITS
    advance(&mut s, -1);
    s.byte = s.byte.wrapping_sub(2);
    let mut data_size = s.read_rs() as u32; // RS at bitsize-17
    if data_size & 0x8000 != 0 {
        s.byte = s.byte.wrapping_sub(4);
        let hi = s.read_rs() as u32;
        data_size = (data_size & 0x7fff) | (hi << 15);
        s.byte = s.byte.wrapping_sub(2);
    }
    s.byte = s.byte.wrapping_sub(2); // common str->byte -= 2
    advance(&mut s, -(data_size as i64)); // data_size is a BIT count

    let pos = s.byte as i64 * 8 + s.bit as i64;
    if pos < 0 || pos as usize > obj_data.len() * 8 {
        return None;
    }
    let mut out = BitReader::at(obj_data, 0, 0);
    out.set_bitpos(pos as usize);
    Some(out)
}

/// Mirror LibreDWG's `bit_advance_position`: `bits = (long)bit + advance;
/// byte += bits >> 3; bit = bits & 7` (arithmetic shift, so negative advances
/// cross byte boundaries correctly).
fn advance(r: &mut BitReader, adv: i64) {
    let bits = r.bit as i64 + adv;
    let byte = r.byte as i64 + (bits >> 3);
    let bit = (bits & 7) as u32;
    r.byte = byte as usize;
    r.bit = bit;
}

/// Does this raw object type carry an R2007 string stream?
///
/// Mirrors `obj_has_strings()` in LibreDWG's `decode.c`. Types that answer
/// `false` get `str_dat->size = 0`, i.e. their `T` fields read as empty and
/// **no bytes are consumed**. Without this gate the `has_strings` probe at
/// `bitsize - 1` would happily pick up an unrelated set bit and every later
/// string in that object would be garbage.
fn obj_has_strings(type_code: u16) -> bool {
    // Custom (CLASS-defined) objects always have one.
    if type_code >= 500 {
        return true;
    }
    match type_code {
        // --- no string stream -------------------------------------------
        0x05 | 0x06 | 0x07 | 0x08 // ENDBLK, SEQEND, INSERT, MINSERT
        | 0x0a..=0x10             // VERTEX_*, POLYLINE_2D/3D
        | 0x11 | 0x12 | 0x13      // ARC, CIRCLE, LINE
        | 0x1b | 0x1c | 0x1d | 0x1e | 0x1f | 0x20 | 0x21 // POINT..SHAPE
        | 0x23 | 0x24             // ELLIPSE, SPLINE
        | 0x28 | 0x29             // RAY, XLINE
        | 0x2f                    // MLINE
        | 0x30 | 0x32 | 0x34 | 0x38 | 0x3c | 0x3e | 0x40 | 0x42 | 0x44 | 0x46 // *_CONTROL
        | 0x4b | 0x4c | 0x4d      // DUMMY, LONG_TRANSACTION, LWPOLYLINE
        | 0x50                    // PLACEHOLDER
        => false,
        // --- has a string stream ----------------------------------------
        _ => true,
    }
}

/// Read a *regular* `T` field (table/object name, TEXT value, …).
///
/// `bit_read_T` is just `IS_FROM_TU(dat) ? bit_read_TU : bit_read_TV`, and
/// `IS_FROM_TU` is `from_version >= R_2007`. So the *encoding* flips at R2007,
/// but the *stream* it lives on is a separate question:
///
/// * R2000 (AC1015): inline `TV` on the field stream.
/// * **R2004 (AC1018): inline `TV` on the field stream.** This is the one that
///   surprises people: R2004 already uses the section-tree container and the
///   R2004 `common_entity_data`, but it has **no** string stream — LibreDWG
///   only calls `obj_string_stream` inside `SINCE(R_2007a)`, and everywhere
///   else it passes `Bit_Chain *str_dat = dat`. Treating AC1018 strings as
///   R2007-style string-stream `TU` yields an empty name for every layer, so
///   every entity reports `layer=""`.
/// * R2007 *entity* (TEXT, …): inline `TU` on the field stream.
/// * R2007+ *object* (LAYER, BLOCK_HEADER, …): the whole `T` — `BS` length and
///   the UCS-2 bytes — lives in the separate **string stream** (`str_rd`); the
///   field stream is not touched at all. `from_string_stream = true` selects
///   this mode (a `None` stream yields an empty string).
fn read_t(
    rd: &mut BitReader,
    str_rd: &mut Option<BitReader>,
    variant: EntityVariant,
    from_string_stream: bool,
) -> String {
    // Only R2007+ actually *has* a string stream to read from.
    if from_string_stream && variant >= EntityVariant::R2007 {
        match str_rd {
            Some(s) => s.read_tu(),
            None => String::new(),
        }
    } else if variant >= EntityVariant::R2007 {
        rd.read_tu()
    } else {
        // R2004 included: no string stream, so the `T` is an inline `TV`.
        rd.read_tv()
    }
}

/// Read a `CMC` colour (`bit_read_CMC` in LibreDWG's `bits.c`).
///
/// ```text
/// BS index                          (R13b1+; RS before that)
///   R2004+: BL rgb, RC flag, and — when flag < 4 — T name / T book_name
/// ```
///
/// Used by *non-entity* objects such as `LAYER`. Entities from R2004 on use
/// [`read_enc_color`] instead.
fn read_cmc(rd: &mut BitReader, str_rd: &mut Option<BitReader>, variant: EntityVariant) -> Color {
    // R2004+: the initial `BS index` is a legacy value and is *always*
    // overwritten by `dwg_find_color_index(rgb)` at the end of `bit_read_CMC`.
    // We still consume the BS so the field stream stays aligned, but the
    // authoritative colour is the 24-bit truecolor looked up in the 256-entry
    // palette (DXF group 62). See `rgb_to_palette_index`.
    let _legacy_index = rd.read_bs();
    let mut index: i16 = _legacy_index as i16;
    if variant >= EntityVariant::R2004 {
        // R2004+ (`bit_read_CMC`): the truecolor `BL rgb` and `RC flag` tail is
        // *always* present, independent of whether the object carries a string
        // stream. Only the optional name/book_name move.
        let rgb = rd.read_bl();
        let flag = rd.read_rc();
        if flag < 4 {
            // `bit_read_CMC` reads them with `bit_read_T (str_dat)`. Below
            // R2007 `str_dat` *is* the field stream, so they are inline `TV`s.
            if variant >= EntityVariant::R2007 {
                if let Some(s) = str_rd {
                    if flag & 1 != 0 {
                        let _name = s.read_tu();
                    }
                    if flag & 2 != 0 {
                        let _book_name = s.read_tu();
                    }
                }
            } else {
                if flag & 1 != 0 {
                    let _name = rd.read_tv();
                }
                if flag & 2 != 0 {
                    let _book_name = rd.read_tv();
                }
            }
        }
        // `bit_read_CMC` overwrites the index with `dwg_find_color_index(rgb)`,
        // which masks to 24 bits and does an exact palette match — but files
        // store plain palette colours as `method << 24 | index`, so the palette
        // scan misses and the `rgb & 0xff` fallback has to run. See
        // [`crate::cmc_color_index`].
        index = crate::cmc_color_index(rgb);
    }
    Color { index }
}

/// Read an `ENC` colour — the R2004+ replacement for `CMC` on **entities**
/// (`common_entity_data.spec`, the `SINCE(R_2004a)` branch).
///
/// The whole colour is packed into one `BS`:
///
/// ```text
/// BS  raw        index = raw & 0x1ff,  flag = raw >> 8
/// if flag & 0x20:  BL alpha_raw
/// if flag & 0x40:  H  handle  (DBCOLOR — on the handle stream in LibreDWG)
/// elif flag & 0x80: BL rgb      (ODA bug, documented as BS)
/// if (flag & 0x41) == 0x41: T name  -> on the STRING stream (str_dat)
/// if (flag & 0x42) == 0x42: T book_name -> on the STRING stream
/// ```
///
/// The name/book_name (when present) live in the separate **string** stream and
/// are read from `str_rd`; routing them through the field stream would desync
/// every field that follows the colour. (The DBCOLOR handle, `flag & 0x40`, is
/// rare and read from the field stream here for layout-stability; our entity
/// handle stream is consumed *after* the field stream, so interleaving it would
/// cost more than it is worth for the bylayer/byblock colours in real files.)
fn read_enc_color(rd: &mut BitReader, str_rd: &mut Option<BitReader>) -> Color {
    let raw = rd.read_bs();
    let index = (raw & 0x1ff) as i16;
    let flag = raw >> 8;
    if flag & 0x20 != 0 {
        let _alpha_raw = rd.read_bl();
    }
    if flag & 0x40 != 0 {
        // DBCOLOR handle — see note above; kept on the field stream for now.
        let _handle = rd.read_h();
    } else if flag & 0x80 != 0 {
        let _rgb = rd.read_bl();
    }
    if flag & 0x41 == 0x41 {
        if let Some(s) = str_rd {
            let _name = s.read_tu();
        }
    }
    if flag & 0x42 == 0x42 {
        if let Some(s) = str_rd {
            let _book_name = s.read_tu();
        }
    }
    Color { index }
}

/// Decode one object of the R2007 object stream at `offset`.
///
/// Record layout (identical shape to R2000, but `bitsize` is measured from the
/// **start of the record, including the `MS size` field** — verified against
/// `example_2007.dwg`, where the first object has `bitsize == 74 == 16 (size)
/// + 10 (type) + 32 (bitsize) + 16 (handle)`):
///
/// ```text
/// MS size | BS type | RL bitsize | H handle | EED | fields | handle stream
/// ```
fn parse_object_r2007(
    buf: &[u8],
    offset: usize,
    index: usize,
    variant: EntityVariant,
) -> Option<ParsedObject> {
    if offset >= buf.len() {
        return None;
    }
    let mut head = BitReader::at(buf, offset, 0);
    let size = head.read_ms() as usize;
    if size == 0 {
        return None;
    }

    // Object-record header differs before/after R2010 (`SINCE R_2010b` in
    // LibreDWG's `dwg_decode_add_object`):
    //   * R2004/R2007 (pre-2010):  size(MS)  type(BS)  bitsize(RL)
    //   * R2010+           :       size(MS)  handlestream_size(UMC)
    //                              type(BOT)  bitsize = size*8 - handlestream_size
    //
    // LibreDWG comments the R2010+ UMC as "not counted in the object size":
    // `obj->address` (and the `dat->size = obj->size` window) start *after*
    // the UMC. Getting that wrong shifts the data window by the UMC's byte
    // length, which in turn misplaces the trailing handle stream at `bitsize`.
    let type_code: u16;
    let bitsize: usize;
    let obj_data: &[u8];
    let rd_start_byte: usize;
    let rd_start_bit: u32;
    if variant >= EntityVariant::R2010 {
        let handlestream_size = head.read_umc() as usize; // bit length of handle stream
        let obj_start = head.byte;
        if head.bit != 0
            || size.checked_add(obj_start).map_or(true, |e| e > buf.len())
            || handlestream_size > size * 8
        {
            return None;
        }
        obj_data = &buf[obj_start..obj_start + size];
        let mut rd = BitReader::at(obj_data, 0, 0);
        type_code = rd.read_bot();
        bitsize = size * 8 - handlestream_size;
        rd_start_byte = rd.byte;
        rd_start_bit = rd.bit;
    } else {
        let obj_start = head.byte;
        if obj_start + size > buf.len() {
            return None;
        }
        obj_data = &buf[obj_start..obj_start + size];
        let mut rd = BitReader::at(obj_data, 0, 0);
        type_code = rd.read_bs();
        bitsize = rd.read_rl() as usize;
        rd_start_byte = rd.byte;
        rd_start_bit = rd.bit;
    }
    if bitsize > obj_data.len() * 8 {
        return None;
    }
    let mut rd = BitReader::at(obj_data, rd_start_byte, rd_start_bit);
    let mut hr = BitReader::at(obj_data, bitsize / 8, (bitsize % 8) as u32);
    // Only *some* types have a string stream, and only `SINCE(R_2007a)`: AC1018
    // keeps its strings inline on the field stream. Probing `bitsize - 1`
    // unconditionally would mistake an unrelated set bit for `has_strings`
    // and then read UCS-2 garbage for every `T` field in the object.
    let mut str_rd = if variant >= EntityVariant::R2007 && obj_has_strings(type_code) {
        setup_string_stream(obj_data, bitsize)
    } else {
        None
    };

    let self_handle = rd.read_h();
    // EED item 0 is a UCS-2 `TU` only SINCE(R_2007a); below that (R13b1..R2007b,
    // R2004 included) it is `RC` length + `RS_BE` codepage + single bytes.
    let eed_dialect = if variant >= EntityVariant::R2007 {
        Dialect::R2007
    } else {
        Dialect::R2000
    };
    let eed: Vec<Eed> = decode_eed(&mut rd, eed_dialect);

    let mut obj = ParsedObject {
        index,
        type_code,
        type_name: type_name(type_code).to_string(),
        handle: self_handle.value,
        layer_handle: None,
        owner_handle: None,
        xdict_handle: None,
        reactors: Vec::new(),
        block_handle: None,
        block_name: None,
        geometry: None,
        text: None,
        layer: None,
        vertex: None,
        polyline: None,
        dictionary: None,
        xdata: Vec::new(),
        cloning: None,
        eed,
    };

    // The remaining `Dialect` users here draw the line at *R2000 vs R2004+*
    // (modular `BL` for `num_points`, the owned-handle list on POLYLINE), not at
    // R2007 — everything R2004 and newer shares those.
    let dialect = if variant >= EntityVariant::R2004 {
        Dialect::R2007
    } else {
        Dialect::R2000
    };

    match type_code {
        0x31 => {
            obj.block_name = decode_block_header(&mut rd, &mut str_rd, variant);
        }
        0x2a => {
            decode_dictionary(&mut rd, &mut str_rd, &mut hr, &mut obj, variant);
        }
        0x33 => {
            if let Some(layer) =
                decode_layer(&mut rd, &mut hr, &mut str_rd, self_handle.value, variant)
            {
                obj.layer = Some(layer);
            }
        }
        0x4f => {
            decode_xrecord(&mut rd, &mut hr, bitsize / 8, &mut obj, variant);
        }
        0x01 | 0x07 | 0x0a | 0x0b | 0x0c | 0x0d | 0x0e | 0x0f | 0x10 | 0x11 | 0x12
        | 0x13 | 0x1b | 0x1c | 0x1d | 0x1e | 0x1f | 0x20 | 0x23 | 0x2c | 0x4d => {
            let flags = read_common_entity_fields(&mut rd, &mut str_rd, variant);
            let refs = read_entity_common_handles(&mut hr, self_handle.value, &flags);
            obj.layer_handle = refs.layer;
            obj.owner_handle = refs.owner;
            obj.xdict_handle = refs.xdict;
            obj.reactors = refs.reactors;
            match type_code {
                0x13 => decode_line(&mut rd, &mut obj),
                0x12 => decode_circle(&mut rd, &mut obj),
                0x11 => decode_arc(&mut rd, &mut obj),
                0x1b => decode_point(&mut rd, &mut obj),
                0x4d => decode_lwpolyline(&mut rd, &mut obj, dialect),
                0x01 => decode_text(&mut rd, &mut str_rd, &mut obj, variant),
                0x07 => decode_insert(&mut rd, &mut hr, &mut obj),
                0x0a | 0x0b | 0x0c | 0x0d | 0x0e => decode_vertex(&mut rd, &mut obj, type_code),
                0x0f | 0x10 | 0x1d | 0x1e => {
                    decode_polyline_header(&mut rd, &mut hr, &mut obj, type_code, dialect)
                }
                0x1c => decode_3dface(&mut rd, &mut obj),
                0x1f | 0x20 => decode_solid(&mut rd, &mut obj),
                0x23 => decode_ellipse(&mut rd, &mut obj),
                0x2c => decode_mtext(&mut rd, &mut str_rd, &mut obj, variant),
                _ => {}
            }
        }
        _ => {}
    }

    Some(obj)
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- bit-stream construction helpers ----------------------------------
    // DWG is bit-oriented; these let every test express its stream in bits and
    // pack them MSB-first, exactly like the wire format.

    fn push(bits: &mut Vec<u8>, value: u64, n: usize) {
        for i in (0..n).rev() {
            bits.push(((value >> i) & 1) as u8);
        }
    }

    fn pack(bits: &[u8]) -> Vec<u8> {
        let mut out = vec![0u8; (bits.len() + 7) / 8];
        for (i, &b) in bits.iter().enumerate() {
            if b != 0 {
                out[i / 8] |= 0x80 >> (i % 8);
            }
        }
        out
    }

    fn push_rd(bits: &mut Vec<u8>, d: f64) {
        for b in d.to_le_bytes() {
            push(bits, b as u64, 8);
        }
    }

    /// Compact `BD`: code 0 -> 8-byte LE double.
    fn push_bd(bits: &mut Vec<u8>, d: f64) {
        push(bits, 0, 2); // code 0
        push_rd(bits, d);
    }

    /// Compact `BS` code 0 -> RS (little-endian 2 bytes).
    fn push_bs(bits: &mut Vec<u8>, v: u16) {
        push(bits, 0, 2); // code 0
        push(bits, v as u64, 8);
        push(bits, (v >> 8) as u64, 8);
    }

    fn br(data: &Vec<u8>) -> BitReader<'_> {
        BitReader::new(data)
    }

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn locator_reads_six_sections_big_endian() {
        // 25 bytes of padding (header up to the locator), then 6 records of
        // (RC number, RL address, RL size).
        let mut buf = vec![0u8; 25];
        // 6 locator records (RC number, RL address, RL size); section 2 is the
        // Handles section at address 0x100, size 0x200.
        for i in 0..6u8 {
            if i == 2 {
                buf.extend_from_slice(&[2, 0, 1, 0, 0, 0, 2, 0, 0]);
            } else {
                buf.extend_from_slice(&[i, 0, 0, 0, 0, 0, 0, 0, 0]);
            }
        }
        let secs = parse_locator(&buf).unwrap();
        assert_eq!(secs.len(), 6);
        assert_eq!(secs[2], (2, 0x100, 0x200));
    }

    #[test]
    fn read_rs_be_is_big_endian() {
        let mut bits = vec![];
        push(&mut bits, 0x12, 8);
        push(&mut bits, 0x34, 8);
        let data = pack(&bits);
        let mut r = br(&data);
        assert_eq!(read_rs_be(&mut r), 0x1234);
    }

    #[test]
    fn read_tv_is_length_prefixed_no_codepage() {
        // LibreDWG's bit_read_TV (R13b1+) is BS length + bytes, no codepage.
        let mut bits = vec![];
        push_bs(&mut bits, 5); // length 5
        for c in b"hello" {
            push(&mut bits, *c as u64, 8);
        }
        let data = pack(&bits);
        let mut r = br(&data);
        assert_eq!(r.read_tv(), "hello");
    }

    #[test]
    fn read_tv_trims_trailing_nul() {
        let mut bits = vec![];
        push_bs(&mut bits, 4);
        for c in b"hi\0" {
            push(&mut bits, *c as u64, 8);
        }
        let data = pack(&bits);
        let mut r = br(&data);
        assert_eq!(r.read_tv(), "hi");
    }

    #[test]
    fn decode_circle_geometry() {
        let mut bits = vec![];
        push_bd(&mut bits, 199.681);
        push_bd(&mut bits, 24.696);
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 20.0);
        push(&mut bits, 1, 1); // BT: bit 1 -> thickness 0.0
        push(&mut bits, 1, 1); // BE: bit 1 -> default (0,0,1)
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_circle(&mut r, &mut obj);
        match obj.geometry {
            Some(Geometry::Circle { center, radius }) => {
                assert!(approx(center.0, 199.681));
                assert!(approx(center.1, 24.696));
                assert!(approx(center.2, 0.0));
                assert!(approx(radius, 20.0));
            }
            other => panic!("expected circle, got {:?}", other),
        }
    }

    #[test]
    fn decode_arc_geometry() {
        let mut bits = vec![];
        push_bd(&mut bits, 10.0);
        push_bd(&mut bits, 20.0);
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 5.0);
        push(&mut bits, 1, 1); // thickness 0
        push(&mut bits, 1, 1); // BE: bit 1 -> default (0,0,1)
        push_bd(&mut bits, 0.0); // start angle
        push_bd(&mut bits, std::f64::consts::PI); // end angle
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_arc(&mut r, &mut obj);
        match obj.geometry {
            Some(Geometry::Arc { center, radius, start_angle, end_angle }) => {
                assert!(approx(center.0, 10.0));
                assert!(approx(center.1, 20.0));
                assert!(approx(radius, 5.0));
                assert!(approx(start_angle, 0.0));
                assert!(approx(end_angle, std::f64::consts::PI));
            }
            other => panic!("expected arc, got {:?}", other),
        }
    }

    #[test]
    fn decode_point_geometry() {
        let mut bits = vec![];
        push_bd(&mut bits, 1.0);
        push_bd(&mut bits, 2.0);
        push_bd(&mut bits, 3.0);
        push(&mut bits, 1, 1); // thickness
        push(&mut bits, 1, 1); // BE: bit 1 -> default (0,0,1)
        push_bd(&mut bits, 0.0); // x_ang
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_point(&mut r, &mut obj);
        match obj.geometry {
            Some(Geometry::Point { x, y, z }) => {
                assert!(approx(x, 1.0));
                assert!(approx(y, 2.0));
                assert!(approx(z, 3.0));
            }
            other => panic!("expected point, got {:?}", other),
        }
    }

    #[test]
    fn decode_line_geometry_z_zero() {
        // z_is_zero = 1 -> no z fields stored.
        let mut bits = vec![];
        push(&mut bits, 1, 1); // z_is_zero flag set
        push_rd(&mut bits, 99.681); // sx (raw RD)
        push(&mut bits, 3, 2); // DD code 3 -> absolute
        push_rd(&mut bits, 199.681); // ex
        push_rd(&mut bits, 94.696); // sy (raw RD)
        push(&mut bits, 3, 2); // DD code 3 -> absolute
        push_rd(&mut bits, 194.696); // ey
        push(&mut bits, 1, 1); // thickness
        push(&mut bits, 1, 1); // BE: bit 1 -> default (0,0,1)
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_line(&mut r, &mut obj);
        match obj.geometry {
            Some(Geometry::Line { start, end }) => {
                assert!(approx(start.0, 99.681));
                assert!(approx(end.0, 199.681));
                assert!(approx(start.1, 94.696));
                assert!(approx(end.1, 194.696));
                assert!(approx(start.2, 0.0));
                assert!(approx(end.2, 0.0));
            }
            other => panic!("expected line, got {:?}", other),
        }
    }

    #[test]
    fn decode_lwpolyline_two_vertices() {
        // flag = 0 (no optional BDs, no bulges/widths).
        let mut bits = vec![];
        push_bs(&mut bits, 0); // flag 0
        push_bs(&mut bits, 2); // num_points = 2
        // first vertex raw 2RD, second vertex 2DD (DD vs first)
        push_rd(&mut bits, -50.319);
        push_rd(&mut bits, -85.304);
        // second vertex (DD code 3 -> absolute, as LibreDWG's 2DD emits)
        let x2 = -50.319 + 10.0;
        let y2 = -85.304 + 20.0;
        push(&mut bits, 3, 2);
        push_rd(&mut bits, x2);
        push(&mut bits, 3, 2);
        push_rd(&mut bits, y2);
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_lwpolyline(&mut r, &mut obj, Dialect::R2000);
        match obj.geometry {
            Some(Geometry::LwPolyline { flag, points, bulges }) => {
                assert_eq!(flag, 0);
                assert_eq!(points.len(), 2);
                assert!(bulges.is_empty());
                assert!(approx(points[0].0, -50.319));
                assert!(approx(points[0].1, -85.304));
                assert!(approx(points[1].0, -50.319 + 10.0));
                assert!(approx(points[1].1, -85.304 + 20.0));
            }
            other => panic!("expected lwpolyline, got {:?}", other),
        }
    }

    #[test]
    fn decode_text_full_dataflags() {
        // dataflags = 0xFF: every optional field is *skipped* (bit set means
        // "do not store"), so only ins_pt, extrusion, thickness, height and the
        // value string are present.
        let mut bits = vec![];
        push(&mut bits, 0xFF, 8); // RC dataflags
        push_rd(&mut bits, -50.319); // ins_x
        push_rd(&mut bits, 134.696); // ins_y
        push(&mut bits, 1, 1); // BE default (0,0,1)
        push(&mut bits, 1, 1); // BT thickness 0
        push_rd(&mut bits, 5.0); // height
        push_bs(&mut bits, 21); // TV length 21
        for c in b"Jen teksto simpla" {
            push(&mut bits, *c as u64, 8);
        }
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_text(&mut r, &mut None, &mut obj, EntityVariant::PreR2004);
        match obj.geometry {
            Some(Geometry::Text { ins_pt, height, value }) => {
                assert!(approx(ins_pt.0, -50.319));
                assert!(approx(ins_pt.1, 134.696));
                assert!(approx(height, 5.0));
                assert_eq!(value, "Jen teksto simpla");
            }
            other => panic!("expected text, got {:?}", other),
        }
    }

    #[test]
    fn decode_layer_r2007_is_bit_exact_against_the_reference_decoder() {
        // The raw object bytes of LAYER handle 0x10 in
        // `tests/data/example_2007.dwg` (MS size already stripped). Every
        // assertion below is cross-checked against the DXF export:
        // 2 -> "0", 62 -> 7, 370 -> -3 (lineweight 31), 390 -> F, 347 -> 279,
        // 6 -> Continuous (handle 0x16), 330 -> 2, 360 -> 2B7.
        let obj: Vec<u8> = vec![
            0x4C, 0xEA, 0x80, 0x00, 0x00, 0x00, 0x44, 0x29, 0x1E, 0x00, 0x70, 0x0E, 0x00, 0x01,
            0x86, 0x00, 0x80, 0x98, 0x00, 0x0D, 0x00, 0x50, 0x40, 0x8C, 0x80, 0xAD, 0xD4, 0x14,
            0x43, 0xD4, 0x80, 0x9E, 0x54, 0x45, 0xBF,
        ];

        // Header: type(BS) = 0x33, bitsize(RL) = 170, handle(H) = 0x10, no EED.
        let mut rd = BitReader::at(&obj, 0, 0);
        assert_eq!(rd.read_bs(), 0x33);
        let bitsize = rd.read_rl() as usize;
        assert_eq!(bitsize, 170);
        assert_eq!(rd.read_h().value, 0x10);
        assert!(decode_eed(&mut rd, Dialect::R2000).is_empty());
        assert_eq!(rd.bitpos(), 60, "field stream starts at bit 60");

        let mut str_rd = setup_string_stream(&obj, bitsize);
        let mut hr = BitReader::at(&obj, bitsize / 8, (bitsize % 8) as u32);

        let layer =
            decode_layer(&mut rd, &mut hr, &mut str_rd, 0x10, EntityVariant::R2007).expect("decodes");
        assert_eq!(layer.name, "0");
        assert_eq!(layer.color_index, 7, "CMC 0xC3000007 -> ACI 7");
        assert_eq!(layer.flags, 1008, "plotflag + lineweight 31");
        assert_eq!(layer.lineweight, 31);
        assert!(layer.plot_flag);
        assert!(!layer.frozen && !layer.off && !layer.locked);
        assert_eq!(layer.ltype_handle, Some(0x16));
        assert_eq!(layer.plotstyle_handle, Some(0xF));
        // The field stream must end exactly where the string stream begins.
        assert_eq!(rd.bitpos(), 127);
    }

    #[test]
    fn object_prologue_adds_the_r2013_has_ds_data_bit() {
        // `common_object_handle_data.spec` — the object-side twin of the
        // `has_ds_data` bit the *entity* spec already reads:
        //   num_reactors(BL)    SINCE(R_13b1)
        //   is_xdic_missing(B)  SINCE(R_2004a)
        //   has_ds_data(B)      SINCE(R_2013)
        // Missing it shifts every later field of every object by one bit.
        let mut bits = Vec::new();
        push(&mut bits, 0b10, 2); // BL code 2 -> num_reactors = 0
        push(&mut bits, 1, 1); // is_xdic_missing = 1 (no XDICTIONARY handle)
        push(&mut bits, 0, 1); // has_ds_data = 0
        push(&mut bits, 0b1011, 4); // trailing marker: whatever comes next
        let data = pack(&bits);

        let cases: [(&str, EntityVariant, usize, bool); 5] = [
            ("R2000", EntityVariant::PreR2004, 2, false),
            ("R2004", EntityVariant::R2004, 3, true),
            ("R2007", EntityVariant::R2007, 3, true),
            ("R2010", EntityVariant::R2010, 3, true),
            ("R2013", EntityVariant::R2013, 4, true),
        ];
        for (label, variant, end, missing) in cases {
            let mut rd = br(&data);
            let (nr, xdic_missing) = read_object_prologue(&mut rd, variant);
            assert_eq!(nr, 0, "{label}: num_reactors");
            assert_eq!(xdic_missing, missing, "{label}: is_xdic_missing");
            assert_eq!(rd.bitpos(), end, "{label}: bits consumed by the prologue");
        }
        // …and the R2013 prologue must stop *exactly* on the marker bits.
        let mut rd = br(&data);
        read_object_prologue(&mut rd, EntityVariant::R2013);
        assert_eq!(rd.read_rc(), 0b1011 << 4, "the marker must follow unshifted");
    }

    #[test]
    fn entity_handles_skip_xdict_when_is_xdic_missing() {
        // R_2004a+ guards the XDICTIONARY handle with
        // `if (!is_xdic_missing)`; reading it unconditionally shifts `layer`
        // (and everything after it) by one slot — which is what made every
        // R2007 entity report `layer=?`.
        let with_xdict = [0x41u8, 0x02, 0x32, 0x02, 0xB7, 0x51, 0x89];
        let without_xdict = [0x41u8, 0x02, 0x51, 0x89];

        let flags = |missing: bool| EntityFlags {
            num_reactors: 0,
            entmode: 0,
            links: false,
            ltype_flags: 0,
            plotstyle_flags: 0,
            material_flags: 0,
            shadow_flags: 0,
            is_xdic_missing: missing,
            has_full_visualstyle: false,
            has_face_visualstyle: false,
            has_edge_visualstyle: false,
        };

        let mut r = BitReader::new(&with_xdict);
        let refs = read_entity_common_handles(&mut r, 0x100, &flags(false));
        assert_eq!(refs.layer, Some(0x89));

        let mut r = BitReader::new(&without_xdict);
        let refs = read_entity_common_handles(&mut r, 0x100, &flags(true));
        assert_eq!(refs.layer, Some(0x89));
    }

    #[test]
    fn decode_block_header_reads_name_after_num_reactors() {
        // num_reactors (compact BS, 0 -> 2 bits) then the name TV.
        let mut bits = vec![];
        push(&mut bits, 2, 2); // BS code 2 -> num_reactors = 0
        push(&mut bits, 1, 2); // TV length: BS code 1 -> RC payload
        push(&mut bits, 13, 8); // length 13
        for c in b"*Paper_Space0" {
            push(&mut bits, *c as u64, 8);
        }
        let data = pack(&bits);
        let mut r = br(&data);
        assert_eq!(
            decode_block_header(&mut r, &mut None, EntityVariant::PreR2004).as_deref(),
            Some("*Paper_Space0")
        );
    }

    #[test]
    fn decode_block_header_rejects_empty_name() {
        let mut bits = vec![];
        push(&mut bits, 2, 2); // num_reactors = 0
        push(&mut bits, 2, 2); // TV length 0 -> empty name
        let data = pack(&bits);
        let mut r = br(&data);
        assert_eq!(
            decode_block_header(&mut r, &mut None, EntityVariant::PreR2004),
            None,
            "empty name -> None"
        );
    }

    /// An INSERT stores only its block_header handle; the name must be filled
    /// in from the BLOCK_HEADER table afterwards.
    #[test]
    fn insert_block_name_is_resolved_from_block_header() {
        let mut insert = ParsedObject::default();
        insert.block_handle = Some(0x55);
        insert.geometry = Some(Geometry::Insert {
            ins_pt: (1.0, 2.0, 3.0),
            block_name: String::new(),
            scale: (1.0, 1.0, 1.0),
        });
        // A non-entity object with a block_handle must be left alone.
        let mut other = ParsedObject::default();
        other.block_handle = Some(0x55);
        let mut objects = vec![insert, other];

        let mut names = HashMap::new();
        names.insert(0x55u64, "MyBlock".to_string());
        resolve_insert_block_names(&mut objects, &names);

        match &objects[0].geometry {
            Some(Geometry::Insert { block_name, .. }) => {
                assert_eq!(block_name, "MyBlock", "INSERT name resolved via handle");
            }
            other => panic!("expected Insert, got {other:?}"),
        }
        // An unresolved handle leaves the name untouched rather than panicking.
        let mut unresolved = ParsedObject::default();
        unresolved.block_handle = Some(0x999);
        unresolved.geometry = Some(Geometry::Insert {
            ins_pt: (0.0, 0.0, 0.0),
            block_name: String::new(),
            scale: (1.0, 1.0, 1.0),
        });
        let mut objs = vec![unresolved];
        resolve_insert_block_names(&mut objs, &names);
        match &objs[0].geometry {
            Some(Geometry::Insert { block_name, .. }) => {
                assert!(block_name.is_empty(), "unknown handle -> name stays empty");
            }
            other => panic!("expected Insert, got {other:?}"),
        }
    }

    #[test]
    fn decode_insert_scale_flag_three_is_unity() {
        // scale_flag = 3 -> scale (1,1,1), nothing stored on the stream.
        let mut bits = vec![];
        push_bd(&mut bits, 3.0); // ins x
        push_bd(&mut bits, 4.0); // ins y
        push_bd(&mut bits, 5.0); // ins z
        push(&mut bits, 3, 2); // BB scale_flag = 3
        push_bd(&mut bits, 0.0); // rotation
        push_bd(&mut bits, 0.0); // ext x
        push_bd(&mut bits, 0.0); // ext y
        push_bd(&mut bits, 1.0); // ext z
        push(&mut bits, 0, 1); // has_attribs = 0
        let data = pack(&bits);
        let mut r = br(&data);
        let dummy = vec![0u8; 1];
        let mut hr = BitReader::new(&dummy);
        let mut obj = ParsedObject::default();
        decode_insert(&mut r, &mut hr, &mut obj);
        match obj.geometry {
            Some(Geometry::Insert { ins_pt, scale, .. }) => {
                assert!(approx(ins_pt.0, 3.0));
                assert!(approx(ins_pt.1, 4.0));
                assert!(approx(ins_pt.2, 5.0));
                assert!(approx(scale.0, 1.0));
                assert!(approx(scale.1, 1.0));
                assert!(approx(scale.2, 1.0));
            }
            other => panic!("expected insert, got {:?}", other),
        }
    }

    #[test]
    fn decode_insert_scale_flag_two_reads_one_value() {
        // scale_flag = 2 -> scale.x is a raw RD, y = z = x.
        let mut bits = vec![];
        push_bd(&mut bits, 3.0);
        push_bd(&mut bits, 4.0);
        push_bd(&mut bits, 5.0);
        push(&mut bits, 2, 2); // BB scale_flag = 2
        push_rd(&mut bits, 2.5); // scale.x raw RD
        push_bd(&mut bits, 0.0); // rotation
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 1.0);
        push(&mut bits, 0, 1);
        let data = pack(&bits);
        let mut r = br(&data);
        let dummy = vec![0u8; 1];
        let mut hr = BitReader::new(&dummy);
        let mut obj = ParsedObject::default();
        decode_insert(&mut r, &mut hr, &mut obj);
        match obj.geometry {
            Some(Geometry::Insert { scale, .. }) => {
                assert!(approx(scale.0, 2.5));
                assert!(approx(scale.1, 2.5));
                assert!(approx(scale.2, 2.5));
            }
            other => panic!("expected insert, got {:?}", other),
        }
    }

    #[test]
    fn decode_insert_scale_flag_zero_reads_all() {
        // scale_flag = 0 -> scale.x raw RD, y/z are DD deltas vs x.
        let mut bits = vec![];
        push_bd(&mut bits, 3.0);
        push_bd(&mut bits, 4.0);
        push_bd(&mut bits, 5.0);
        push(&mut bits, 0, 2); // BB scale_flag = 0
        push_rd(&mut bits, 1.5); // scale.x
        // scale.y / scale.z are DD (default scale.x); code 3 -> absolute
        push(&mut bits, 3, 2);
        push_rd(&mut bits, 2.0);
        push(&mut bits, 3, 2);
        push_rd(&mut bits, 1.0);
        push_bd(&mut bits, 0.0); // rotation
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 1.0);
        push(&mut bits, 0, 1);
        let data = pack(&bits);
        let mut r = br(&data);
        let dummy = vec![0u8; 1];
        let mut hr = BitReader::new(&dummy);
        let mut obj = ParsedObject::default();
        decode_insert(&mut r, &mut hr, &mut obj);
        match obj.geometry {
            Some(Geometry::Insert { scale, .. }) => {
                assert!(approx(scale.0, 1.5));
                assert!(approx(scale.1, 2.0));
                assert!(approx(scale.2, 1.0));
            }
            other => panic!("expected insert, got {:?}", other),
        }
    }

    // === entity field-stream order (synthetic bytes) =========================
    // These drive the decoders with hand-built bit streams so the *field order*
    // and *bit width* of every entity type are pinned down — coverage the
    // cross-check harness cannot give, because it only proves the entities we
    // happen to have a real fixture for. A field-order regression (e.g. reading
    // an extra BD for R2007 MTEXT) fails these immediately.

    #[test]
    fn decode_ellipse_field_order() {
        // 12 BDs in order: center(3), major_axis(3), extrusion(3), axis_ratio,
        // start_angle, end_angle.
        let mut bits = vec![];
        push_bd(&mut bits, 1.5); // center x
        push_bd(&mut bits, 2.5); // center y
        push_bd(&mut bits, 0.0); // center z
        push_bd(&mut bits, 3.0); // major x
        push_bd(&mut bits, 0.0); // major y
        push_bd(&mut bits, 0.0); // major z
        push_bd(&mut bits, 0.0); // extrusion x
        push_bd(&mut bits, 0.0); // extrusion y
        push_bd(&mut bits, 1.0); // extrusion z
        push_bd(&mut bits, 0.5); // axis_ratio
        push_bd(&mut bits, 0.0); // start_angle
        push_bd(&mut bits, 2.094); // end_angle
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_ellipse(&mut r, &mut obj);
        match obj.geometry {
            Some(Geometry::Ellipse {
                center,
                major_axis,
                axis_ratio,
                start_angle,
                end_angle,
            }) => {
                assert!(approx(center.0, 1.5) && approx(center.1, 2.5));
                assert!(approx(major_axis.0, 3.0));
                assert!(approx(axis_ratio, 0.5));
                assert!(approx(start_angle, 0.0));
                assert!(approx(end_angle, 2.094));
            }
            other => panic!("expected ellipse, got {:?}", other),
        }
    }

    #[test]
    fn decode_3dface_z_zero_skips_invis_flags() {
        // has_no_flags=1 -> no invis_flags field; z_is_zero=1 -> corner1 has no
        // z. Corners 2-4 are `3DD` deltas (code 3 = full RD) vs the previous.
        let mut bits = vec![];
        push(&mut bits, 1, 1); // has_no_flags
        push(&mut bits, 1, 1); // z_is_zero
        push_rd(&mut bits, 1.0); // c1 x
        push_rd(&mut bits, 2.0); // c1 y
        for _ in 0..3 {
            for _ in 0..3 {
                push(&mut bits, 3, 2); // DD code 3 -> full RD
                push_rd(&mut bits, 10.0);
            }
        }
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_3dface(&mut r, &mut obj);
        match obj.geometry {
            Some(Geometry::Face3d { corners, invisibility }) => {
                assert!(approx(corners[0].0, 1.0) && approx(corners[0].1, 2.0));
                assert!(approx(corners[0].2, 0.0), "z_is_zero -> c1 z = 0");
                assert!(approx(corners[3].0, 10.0) && approx(corners[3].2, 10.0));
                assert_eq!(invisibility, 0, "has_no_flags -> invis stays 0");
            }
            other => panic!("expected face3d, got {:?}", other),
        }
    }

    #[test]
    fn decode_3dface_with_z_reads_invis_flags() {
        // has_no_flags=0 -> invis_flags BS is read after the four corners;
        // z_is_zero=0 -> corner1 carries a z component.
        let mut bits = vec![];
        push(&mut bits, 0, 1); // has_no_flags = 0
        push(&mut bits, 0, 1); // z_is_zero = 0
        push_rd(&mut bits, 1.0);
        push_rd(&mut bits, 2.0);
        push_rd(&mut bits, 3.0); // c1 z present
        for _ in 0..3 {
            for _ in 0..3 {
                push(&mut bits, 3, 2);
                push_rd(&mut bits, 33.0);
            }
        }
        push_bs(&mut bits, 0x0F); // invis_flags: edges 1-4 invisible
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_3dface(&mut r, &mut obj);
        match obj.geometry {
            Some(Geometry::Face3d { corners, invisibility }) => {
                assert!(approx(corners[0].2, 3.0), "c1 z present");
                assert!(approx(corners[2].2, 33.0), "corner z present (DD code 3)");
                assert_eq!(invisibility, 0x0F, "invis flags read after corners");
            }
            other => panic!("expected face3d, got {:?}", other),
        }
    }

    #[test]
    fn decode_solid_four_corners_share_elevation() {
        // thickness(BT)=0, elevation(BD), 4x 2RD corners, extrusion(BE) default.
        let mut bits = vec![];
        push(&mut bits, 1, 1); // BT: 1 -> thickness 0.0 (no BD)
        push_bd(&mut bits, 7.25); // elevation
        for (x, y) in [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)] {
            push_rd(&mut bits, x);
            push_rd(&mut bits, y);
        }
        push(&mut bits, 1, 1); // BE: 1 -> default (0,0,1)
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_solid(&mut r, &mut obj);
        match obj.geometry {
            Some(Geometry::Solid { corners, elevation }) => {
                assert!(approx(elevation, 7.25));
                assert!(approx(corners[0].0, 0.0) && approx(corners[0].1, 0.0));
                assert!(approx(corners[0].2, 7.25), "corner z = elevation");
                assert!(approx(corners[3].2, 7.25), "all corners share elevation");
                assert!(approx(corners[2].0, 1.0) && approx(corners[2].1, 1.0));
            }
            other => panic!("expected solid, got {:?}", other),
        }
    }

    #[test]
    fn decode_mtext_r2000_inline_text_no_rect_height() {
        // R2000: text is inline TV; the binary has NO rect_height field
        // (that is SINCE(R_2007a), which R2000 does not satisfy).
        let mut bits = vec![];
        push_bd(&mut bits, 5.0); // ins x
        push_bd(&mut bits, 6.0); // ins y
        push_bd(&mut bits, 0.0); // ins z
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 1.0); // extrusion
        push_bd(&mut bits, 1.0);
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 0.0); // x_axis_dir
        push_bd(&mut bits, 10.0); // rect_width
        push_bd(&mut bits, 2.5); // height (immediately after rect_width on R2000)
        push_bs(&mut bits, 1); // attachment
        push_bs(&mut bits, 5); // flow_dir
        push_bd(&mut bits, 3.0); // extents_height
        push_bd(&mut bits, 4.0); // extents_width
        let text = b"Hello MTEXT";
        push_bs(&mut bits, text.len() as u16); // TV length
        for c in text {
            push(&mut bits, *c as u64, 8);
        }
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_mtext(&mut r, &mut None, &mut obj, EntityVariant::PreR2004);
        match obj.geometry {
            Some(Geometry::MText {
                ins_pt, height, value, ..
            }) => {
                assert!(approx(ins_pt.0, 5.0) && approx(ins_pt.1, 6.0));
                assert!(approx(height, 2.5), "height read right after rect_width");
                assert_eq!(value, "Hello MTEXT", "R2000 text is inline TV");
            }
            other => panic!("expected mtext, got {:?}", other),
        }
    }

    #[test]
    fn decode_mtext_r2007_extra_rect_height_and_string_stream() {
        // R2007 (AC1021/AC1032): rect_height (group 46) IS present before
        // height, and the text comes from the object string stream as TU
        // (BS length + UCS-2LE) — NOT inline. This is the field-order difference
        // that bites if a decoder fails to special-case the dialect.
        let mut bits = vec![];
        push_bd(&mut bits, 5.0); // ins x
        push_bd(&mut bits, 6.0); // ins y
        push_bd(&mut bits, 0.0); // ins z
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 1.0); // extrusion
        push_bd(&mut bits, 1.0);
        push_bd(&mut bits, 0.0);
        push_bd(&mut bits, 0.0); // x_axis_dir
        push_bd(&mut bits, 10.0); // rect_width
        push_bd(&mut bits, 9.0); // rect_height (R2007 only — must be consumed!)
        push_bd(&mut bits, 2.5); // height (after rect_height)
        push_bs(&mut bits, 1); // attachment
        push_bs(&mut bits, 5); // flow_dir
        push_bd(&mut bits, 3.0); // extents_height
        push_bd(&mut bits, 4.0); // extents_width
        // Deliberately NO TV on the field stream.
        let data = pack(&bits);

        // String stream: BS length (UTF-16 code units) + UCS-2LE bytes.
        let units: Vec<u16> = "Δpost".encode_utf16().collect();
        let mut sbuf = vec![];
        push_bs(&mut sbuf, units.len() as u16);
        for cu in &units {
            push(&mut sbuf, (*cu & 0xFF) as u64, 8);
            push(&mut sbuf, ((*cu >> 8) & 0xFF) as u64, 8);
        }
        let str_data = pack(&sbuf);

        let mut r = br(&data);
        let mut str_rd = Some(br(&str_data));
        let mut obj = ParsedObject::default();
        decode_mtext(&mut r, &mut str_rd, &mut obj, EntityVariant::R2007);
        match obj.geometry {
            Some(Geometry::MText {
                ins_pt, height, value, ..
            }) => {
                assert!(approx(ins_pt.0, 5.0));
                assert!(approx(height, 2.5), "height read AFTER rect_height");
                assert_eq!(value, "Δpost", "text sourced from the R2007 string stream");
            }
            other => panic!("expected mtext, got {:?}", other),
        }
    }

    // --- polyline assembly primitives (VERTEX + POLYLINE header) -----------
    //
    // These are the bit-level primitives that `assemble_polylines` stitches
    // together. They had no field-stream coverage before — only indirect
    // invariant checks — so a field-order regression here would pass the
    // higher-level tests while silently mis-assigning points/bulges/handles.

    #[test]
    fn decode_vertex_2d_full_fields() {
        // VERTEX_2D (0x0a): flag(RC), point(3BD), start_width(BD),
        // end_width(BD), bulge(BD), tangent(BD). widths/bulge/tangent exist
        // ONLY for the 2D subtype.
        let mut bits = vec![];
        push(&mut bits, 32, 8); // flag RC = 32
        push_bd(&mut bits, 1.0); // x
        push_bd(&mut bits, 2.0); // y
        push_bd(&mut bits, 3.0); // z
        push_bd(&mut bits, 0.5); // start_width
        push_bd(&mut bits, 0.5); // end_width
        push_bd(&mut bits, 0.25); // bulge
        push_bd(&mut bits, std::f64::consts::FRAC_PI_2); // tangent
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_vertex(&mut r, &mut obj, 0x0a);
        let v = obj.vertex.expect("vertex decoded");
        assert!(approx(v.point.0, 1.0) && approx(v.point.1, 2.0) && approx(v.point.2, 3.0));
        assert!(approx(v.bulge, 0.25), "2D vertex carries a bulge");
        assert!(approx(v.width_start, 0.5) && approx(v.width_end, 0.5));
        assert!(v.face_indices.is_none(), "2D vertex is a point, not a face");
    }

    #[test]
    fn decode_vertex_3d_only_point_and_flag() {
        // VERTEX_3D / _MESH / _PFACE (0x0b): flag(RC) + point(3BD) only.
        // No widths/bulge/tangent are read.
        let mut bits = vec![];
        push(&mut bits, 64, 8); // flag RC
        push_bd(&mut bits, 7.0);
        push_bd(&mut bits, 8.0);
        push_bd(&mut bits, 9.0);
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_vertex(&mut r, &mut obj, 0x0b);
        let v = obj.vertex.expect("vertex decoded");
        assert!(approx(v.point.0, 7.0) && approx(v.point.2, 9.0));
        assert_eq!(v.bulge, 0.0, "3D vertex has no bulge");
        assert_eq!(v.width_start, 0.0);
        assert!(v.face_indices.is_none());
    }

    #[test]
    fn decode_vertex_pface_face_reads_four_indices() {
        // VERTEX_PFACE_FACE (0x0e) is NOT a point: it carries four signed
        // vertex indices (BSd). Reading a 3BD here would consume those as a
        // bogus point and corrupt polyface topology.
        let mut bits = vec![];
        push_bs(&mut bits, 1); // i0 = 1
        push_bs(&mut bits, 2); // i1 = 2
        push_bs(&mut bits, 3); // i2 = 3
        push_bs(&mut bits, 4); // i3 = 4
        let data = pack(&bits);
        let mut r = br(&data);
        let mut obj = ParsedObject::default();
        decode_vertex(&mut r, &mut obj, 0x0e);
        let v = obj.vertex.expect("vertex decoded");
        assert_eq!(v.point, (0.0, 0.0, 0.0), "pface face has no point");
        assert_eq!(v.flag, 128);
        assert_eq!(v.face_indices, Some([1, 2, 3, 4]), "four indices captured");
    }

    #[test]
    fn decode_polyline_2d_header_field_order() {
        // POLYLINE_2D (0x0f): flag(BS), curve_type(BS), start_width(BD),
        // end_width(BD), thickness(BT), elevation(BD), extrusion(BE).
        let mut bits = vec![];
        push_bs(&mut bits, 8); // flag
        push_bs(&mut bits, 0); // curve_type
        push_bd(&mut bits, 0.0); // start_width
        push_bd(&mut bits, 0.0); // end_width
        push(&mut bits, 1, 1); // BT code 1 -> thickness 0.0 (no BD payload)
        push_bd(&mut bits, 12.5); // elevation
        push(&mut bits, 1, 1); // BE code 1 -> default (0,0,1)
        let data = pack(&bits);
        let mut r = br(&data);
        let empty_hr = Vec::new();
        let mut hr = br(&empty_hr);
        let mut obj = ParsedObject::default();
        decode_polyline_header(&mut r, &mut hr, &mut obj, 0x0f, Dialect::R2000);
        let p = obj.polyline.expect("polyline header decoded");
        assert_eq!(p.flag, 8);
        assert_eq!(p.curve_type, 0);
        assert!(approx(p.elevation, 12.5));
        assert!(!p.is_3d, "2D polyline is flat");
    }

    #[test]
    fn decode_polyline_3d_header_sets_is_3d() {
        // POLYLINE_3D (0x10): curve_type(RC), flag(RC). is_3d must be set and
        // no width/elevation/BT/BE fields are read.
        let mut bits = vec![];
        push(&mut bits, 0, 8); // curve_type RC
        push(&mut bits, 1, 8); // flag RC
        let data = pack(&bits);
        let mut r = br(&data);
        let empty_hr = Vec::new();
        let mut hr = br(&empty_hr);
        let mut obj = ParsedObject::default();
        decode_polyline_header(&mut r, &mut hr, &mut obj, 0x10, Dialect::R2000);
        let p = obj.polyline.expect("polyline header decoded");
        assert_eq!(p.curve_type, 0);
        assert_eq!(p.flag, 1);
        assert!(p.is_3d, "3D polyline sets is_3d");
    }

    #[test]
    fn decode_polyline_pface_header_field_order() {
        // POLYLINE_PFACE (0x1d): numverts(BS), numfaces(BS) — no widths or
        // elevation; is_3d stays false.
        let mut bits = vec![];
        push_bs(&mut bits, 3); // numverts
        push_bs(&mut bits, 2); // numfaces
        let data = pack(&bits);
        let mut r = br(&data);
        let empty_hr = Vec::new();
        let mut hr = br(&empty_hr);
        let mut obj = ParsedObject::default();
        decode_polyline_header(&mut r, &mut hr, &mut obj, 0x1d, Dialect::R2000);
        let p = obj.polyline.expect("polyline header decoded");
        assert!(!p.is_3d);
        assert_eq!(p.flag, 0);
        assert!(approx(p.elevation, 0.0));
    }

    #[test]
    fn decode_polyline_mesh_header_field_order() {
        // POLYLINE_MESH (0x1e): flag(BS), curve_type(BS), num_m_verts(BS),
        // num_n_verts(BS), m_density(BS), n_density(BS).
        let mut bits = vec![];
        push_bs(&mut bits, 1); // flag
        push_bs(&mut bits, 0); // curve_type
        push_bs(&mut bits, 4); // num_m_verts
        push_bs(&mut bits, 5); // num_n_verts
        push_bs(&mut bits, 2); // m_density
        push_bs(&mut bits, 3); // n_density
        let data = pack(&bits);
        let mut r = br(&data);
        let empty_hr = Vec::new();
        let mut hr = br(&empty_hr);
        let mut obj = ParsedObject::default();
        decode_polyline_header(&mut r, &mut hr, &mut obj, 0x1e, Dialect::R2000);
        let p = obj.polyline.expect("polyline header decoded");
        assert_eq!(p.flag, 1);
        assert_eq!(p.curve_type, 0);
        // NOTE: the decoder treats POLYLINE_MESH (0x1e) as 3D (matches the
        // `0x10 | 0x1e` rule in `decode_polyline_header`), unlike PFACE
        // (0x1d) which it leaves flat. This pins that asymmetry.
        assert!(p.is_3d, "mesh (0x1e) is treated as 3d by the decoder");
    }

    #[test]
    fn decode_polyline_2d_header_r2007_reads_num_owned() {
        // R2007 (and R2004+) carries num_owned(BL) after the 2D header fields,
        // then on the handle stream that many vertex handles + the seqend
        // handle. With num_owned=0 no vertex handles are read; the seqend
        // handle is still consumed.
        let mut bits = vec![];
        push_bs(&mut bits, 8); // flag
        push_bs(&mut bits, 0); // curve_type
        push_bd(&mut bits, 0.0); // start_width
        push_bd(&mut bits, 0.0); // end_width
        push(&mut bits, 1, 1); // BT default
        push_bd(&mut bits, 0.0); // elevation
        push(&mut bits, 1, 1); // BE default
        push(&mut bits, 2, 2); // BL code 2 -> num_owned = 0 (no RL payload)
        // seqend handle on the handle stream: code byte 0x00 -> size 0, value 0.
        let mut hbits = vec![];
        push(&mut hbits, 0x00, 8);
        let hdata = pack(&hbits);
        let data = pack(&bits);
        let mut r = br(&data);
        let mut hr = br(&hdata);
        let mut obj = ParsedObject::default();
        decode_polyline_header(&mut r, &mut hr, &mut obj, 0x0f, Dialect::R2007);
        let p = obj.polyline.expect("polyline header decoded");
        assert_eq!(p.flag, 8);
        assert!(p.vertex_handles.is_empty(), "num_owned=0 -> no vertex handles");
    }

    #[test]
    fn decode_insert_reads_rotation_and_block_handle() {
        // INSERT: ins(3BD), scale_flag(BB=2 -> single RD s), rotation(BD),
        // extrusion(3BD), has_attribs(B), then block handle on the handle
        // stream. This pins the field order *after* the scale triple and the
        // block-handle resolution, neither of which the existing scale_flag
        // tests exercised.
        let mut bits = vec![];
        push_bd(&mut bits, 1.0); // ins x
        push_bd(&mut bits, 2.0); // ins y
        push_bd(&mut bits, 0.0); // ins z
        push(&mut bits, 2, 2); // scale_flag = 2 -> one RD, applied to x/y/z
        push_rd(&mut bits, 2.0); // s
        push_bd(&mut bits, 0.785); // rotation
        push_bd(&mut bits, 0.0); // extrusion x
        push_bd(&mut bits, 0.0); // extrusion y
        push_bd(&mut bits, 1.0); // extrusion z
        push(&mut bits, 0, 1); // has_attribs = 0
        let data = pack(&bits);
        // block handle: code byte 0x51 -> size 1, code 5; value 0x2A = 42.
        let mut hbits = vec![];
        push(&mut hbits, 0x51, 8);
        push(&mut hbits, 0x2A, 8);
        let hdata = pack(&hbits);
        let mut r = br(&data);
        let mut hr = br(&hdata);
        let mut obj = ParsedObject::default();
        decode_insert(&mut r, &mut hr, &mut obj);
        match obj.geometry {
            Some(Geometry::Insert { ins_pt, scale, .. }) => {
                assert!(approx(ins_pt.0, 1.0) && approx(ins_pt.2, 0.0));
                assert!(approx(scale.0, 2.0) && approx(scale.1, 2.0) && approx(scale.2, 2.0));
            }
            other => panic!("expected insert, got {:?}", other),
        }
        assert_eq!(obj.block_handle, Some(42), "block handle from handle stream");
    }
}
