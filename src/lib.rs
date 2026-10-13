//! `dwgparse` — a pure-std DWG CAD file parser, covering the mainstream
//! AutoCAD releases:
//!
//! | stamp    | AutoCAD releases                                | status |
//! |----------|-------------------------------------------------|--------|
//! | `AC1015` | 2000 / 2000i / 2002                             | supported (flat sections) |
//! | `AC1018` | 2004 / 2005 / 2006                              | **rejected** — different container |
//! | `AC1021` | 2007 / 2008 / 2009                              | supported (page container) |
//! | `AC1024` | 2010 / 2011 / 2012                              | supported |
//! | `AC1027` | 2013 / 2014 / 2015 / 2016 / 2017                | supported |
//! | `AC1032` | 2018 / 2019 / **2020** / 2021 / 2022 / 2023 / 2024 | supported |
//!
//! R2000 (`AC1015`) stores its sections flat and gets its own decoder. Every
//! R2007+ stamp shares one *page-container* layout (section memory image,
//! Reed–Solomon interleaved, LZ77 compressed) and routes through the R2007
//! decoder — only the six-byte stamp at offset 0 differs. Note that
//! **AutoCAD 2020 has no format of its own**: it writes `AC1032`, the same
//! stamp as 2018 and 2024.
//!
//! `AC1018` (R2004) is *not* part of that family: it stores sections in a
//! binary tree rooted at a header beginning with the 12-byte file-ID string
//! `AcFssFcAJMB`, which needs a separate container decoder (LibreDWG calls it
//! `decode_r2004`). It is rejected up front with an explanation rather than
//! being fed to the wrong decoder. `AC1037` (AutoCAD 2024+) is a newer format
//! and is rejected rather than parsed on a guess.
//!
//! It extracts:
//! * **Layer information** (`LAYER` table records: name, color, flags).
//! * **Entity geometry coordinates** — LINE, CIRCLE, ARC, POINT, LWPOLYLINE,
//!   TEXT, MTEXT, INSERT, plus **classic POLYLINE** (2D/3D/MESH/PFACE,
//!   assembled from its chained `VERTEX_*` objects), **ELLIPSE**, **3DFACE**,
//!   and **SOLID / TRACE**.
//! * **Extended attributes** — both the entity-level *EED* (Extended Entity
//!   Data, decoded right after each entity's own handle) and the *XData*
//!   stored in `XRECORD` objects. This was the primary requirement.
//!
//! The bit-level read primitives and the object/section layout follow
//! LibreDWG's `bits.c` / `decode.c` / `dwg.c` for R2000, which were studied in
//! detail before implementation.
//!
//! For the full design — file layout, bit primitives, the field/handle dual
//! stream model, API usage patterns and the list of format traps — see
//! `docs/技术设计方案.md`.

pub mod bitreader;
pub mod parser;
pub mod r2004;
pub mod r2007;
pub mod xdata;

use std::collections::HashMap;

/// Decode raw text bytes from a DWG string field into a Rust `String`, with
/// maximum compatibility for Chinese (and other CJK) environments.
///
/// DWG stores human-readable text in one of three ways depending on the release
/// and the field:
/// * **R2007+ entity strings** live in the object's *string stream* and are
///   UTF-16LE — those are handled by [`bitreader::BitReader::read_tu`] and are
///   *not* affected by this function.
/// * **R2000/R2004 `T` strings** (and EED/XData group-1000 `TV` chunks) are a
///   single-byte *code-page* string. On a Chinese-language AutoCAD these bytes
///   are **GBK** (code page 936); on an international setup they may be UTF-8,
///   and R2007+ writes them as UTF-8.
///
/// The previous code used `String::from_utf8_lossy`, which silently produced
/// mojibake for any GBK byte sequence (e.g. `张三` stored as GBK, then displayed
/// through a GBK codepage, shows up as `寮犱笁`). This helper instead tries
/// **strict UTF-8 first**; only when the bytes are *not* valid UTF-8 does it
/// fall back to **GBK** decoding. Valid UTF-8 input is returned untouched, so
/// behaviour for UTF-8 / UTF-16 sources is unchanged, while GBK-stored Chinese
/// is recovered correctly.
pub fn codepage_string(bytes: &[u8]) -> String {
    // Fast path: already valid UTF-8 (UTF-8 DWG, or plain ASCII).
    if let Ok(s) = std::str::from_utf8(bytes) {
        return s.to_string();
    }
    // Fallback: interpret as GBK (code page 936) — covers Chinese AutoCAD.
    let (decoded, _, _) = encoding_rs::GBK.decode(bytes);
    decoded.into_owned()
}

/// A DWG object handle reference (`bit_read_H`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DwgHandle {
    pub code: u8,
    pub value: u64,
}

/// A color value (`bit_read_CMC`). For R2000 only the `BS` index is stored.
#[derive(Debug, Clone, Copy, Default)]
pub struct Color {
    pub index: i16,
}

/// The standard AutoCAD 256-colour palette (indices 0..=255).
///
/// Verbatim from LibreDWG's `rgb_palette[256]` (`dwg.c`). R2004+ stores every
/// colour as a 32-bit truecolor (`CMC`/`ENC`); to recover the legacy DXF group
/// 62 index we hash the 24-bit `RGB` back through this palette with
/// [`rgb_to_palette_index`].
pub const RGB_PALETTE: [[u8; 3]; 256] = [
    [0x00, 0x00, 0x00], [0xFF, 0x00, 0x00], [0xFF, 0xFF, 0x00], [0x00, 0xFF, 0x00],
    [0x00, 0xFF, 0xFF], [0x00, 0x00, 0xFF], [0xFF, 0x00, 0xFF], [0xFF, 0xFF, 0xFF],
    [0x41, 0x41, 0x41], [0x80, 0x80, 0x80], [0xFF, 0x00, 0x00], [0xFF, 0xAA, 0xAA],
    [0xBD, 0x00, 0x00], [0xBD, 0x7E, 0x7E], [0x81, 0x00, 0x00], [0x81, 0x56, 0x56],
    [0x68, 0x00, 0x00], [0x68, 0x45, 0x45], [0x4F, 0x00, 0x00], [0x4F, 0x35, 0x35],
    [0xFF, 0x3F, 0x00], [0xFF, 0xBF, 0xAA], [0xBD, 0x2E, 0x00], [0xBD, 0x8D, 0x7E],
    [0x81, 0x1F, 0x00], [0x81, 0x60, 0x56], [0x68, 0x19, 0x00], [0x68, 0x4E, 0x45],
    [0x4F, 0x13, 0x00], [0x4F, 0x3B, 0x35], [0xFF, 0x7F, 0x00], [0xFF, 0xD4, 0xAA],
    [0xBD, 0x5E, 0x00], [0xBD, 0x9D, 0x7E], [0x81, 0x40, 0x00], [0x81, 0x6B, 0x56],
    [0x68, 0x34, 0x00], [0x68, 0x56, 0x45], [0x4F, 0x27, 0x00], [0x4F, 0x42, 0x35],
    [0xFF, 0xBF, 0x00], [0xFF, 0xEA, 0xAA], [0xBD, 0x8D, 0x00], [0xBD, 0xAD, 0x7E],
    [0x81, 0x60, 0x00], [0x81, 0x76, 0x56], [0x68, 0x4E, 0x00], [0x68, 0x5F, 0x45],
    [0x4F, 0x3B, 0x00], [0x4F, 0x49, 0x35], [0xFF, 0xFF, 0x00], [0xFF, 0xFF, 0xAA],
    [0xBD, 0xBD, 0x00], [0xBD, 0xBD, 0x7E], [0x81, 0x81, 0x00], [0x81, 0x81, 0x56],
    [0x68, 0x68, 0x00], [0x68, 0x68, 0x45], [0x4F, 0x4F, 0x00], [0x4F, 0x4F, 0x35],
    [0xBF, 0xFF, 0x00], [0xEA, 0xFF, 0xAA], [0x8D, 0xBD, 0x00], [0xAD, 0xBD, 0x7E],
    [0x60, 0x81, 0x00], [0x76, 0x81, 0x56], [0x4E, 0x68, 0x00], [0x5F, 0x68, 0x45],
    [0x3B, 0x4F, 0x00], [0x49, 0x4F, 0x35], [0x7F, 0xFF, 0x00], [0xD4, 0xFF, 0xAA],
    [0x5E, 0xBD, 0x00], [0x9D, 0xBD, 0x7E], [0x40, 0x81, 0x00], [0x6B, 0x81, 0x56],
    [0x34, 0x68, 0x00], [0x56, 0x68, 0x45], [0x27, 0x4F, 0x00], [0x42, 0x4F, 0x35],
    [0x3F, 0xFF, 0x00], [0xBF, 0xFF, 0xAA], [0x2E, 0xBD, 0x00], [0x8D, 0xBD, 0x7E],
    [0x1F, 0x81, 0x00], [0x60, 0x81, 0x56], [0x19, 0x68, 0x00], [0x4E, 0x68, 0x45],
    [0x13, 0x4F, 0x00], [0x3B, 0x4F, 0x35], [0x00, 0xFF, 0x00], [0xAA, 0xFF, 0xAA],
    [0x00, 0xBD, 0x00], [0x7E, 0xBD, 0x7E], [0x00, 0x81, 0x00], [0x56, 0x81, 0x56],
    [0x00, 0x68, 0x00], [0x45, 0x68, 0x45], [0x00, 0x4F, 0x00], [0x35, 0x4F, 0x35],
    [0x00, 0xFF, 0x3F], [0xAA, 0xFF, 0xBF], [0x00, 0xBD, 0x2E], [0x7E, 0xBD, 0x8D],
    [0x00, 0x81, 0x1F], [0x56, 0x81, 0x60], [0x00, 0x68, 0x19], [0x45, 0x68, 0x4E],
    [0x00, 0x4F, 0x13], [0x35, 0x4F, 0x3B], [0x00, 0xFF, 0x7F], [0xAA, 0xFF, 0xD4],
    [0x00, 0xBD, 0x5E], [0x7E, 0xBD, 0x9D], [0x00, 0x81, 0x40], [0x56, 0x81, 0x6B],
    [0x00, 0x68, 0x34], [0x45, 0x68, 0x56], [0x00, 0x4F, 0x27], [0x35, 0x4F, 0x42],
    [0x00, 0xFF, 0xBF], [0xAA, 0xFF, 0xEA], [0x00, 0xBD, 0x8D], [0x7E, 0xBD, 0xAD],
    [0x00, 0x81, 0x60], [0x56, 0x81, 0x76], [0x00, 0x68, 0x4E], [0x45, 0x68, 0x5F],
    [0x00, 0x4F, 0x3B], [0x35, 0x4F, 0x49], [0x00, 0xFF, 0xFF], [0xAA, 0xFF, 0xFF],
    [0x00, 0xBD, 0xBD], [0x7E, 0xBD, 0xBD], [0x00, 0x81, 0x81], [0x56, 0x81, 0x81],
    [0x00, 0x68, 0x68], [0x45, 0x68, 0x68], [0x00, 0x4F, 0x4F], [0x35, 0x4F, 0x4F],
    [0x00, 0xBF, 0xFF], [0xAA, 0xEA, 0xFF], [0x00, 0x8D, 0xBD], [0x7E, 0xAD, 0xBD],
    [0x00, 0x60, 0x81], [0x56, 0x76, 0x81], [0x00, 0x4E, 0x68], [0x45, 0x5F, 0x68],
    [0x00, 0x3B, 0x4F], [0x35, 0x49, 0x4F], [0x00, 0x7F, 0xFF], [0xAA, 0xD4, 0xFF],
    [0x00, 0x5E, 0xBD], [0x7E, 0x9D, 0xBD], [0x00, 0x40, 0x81], [0x56, 0x6B, 0x81],
    [0x00, 0x34, 0x68], [0x45, 0x56, 0x68], [0x00, 0x27, 0x4F], [0x35, 0x42, 0x4F],
    [0x00, 0x3F, 0xFF], [0xAA, 0xBF, 0xFF], [0x00, 0x2E, 0xBD], [0x7E, 0x8D, 0xBD],
    [0x00, 0x1F, 0x81], [0x56, 0x60, 0x81], [0x00, 0x19, 0x68], [0x45, 0x4E, 0x68],
    [0x00, 0x13, 0x4F], [0x35, 0x3B, 0x4F], [0x00, 0x00, 0xFF], [0xAA, 0xAA, 0xFF],
    [0x00, 0x00, 0xBD], [0x7E, 0x7E, 0xBD], [0x00, 0x00, 0x81], [0x56, 0x56, 0x81],
    [0x00, 0x00, 0x68], [0x45, 0x45, 0x68], [0x00, 0x00, 0x4F], [0x35, 0x35, 0x4F],
    [0x3F, 0x00, 0xFF], [0xBF, 0xAA, 0xFF], [0x2E, 0x00, 0xBD], [0x8D, 0x7E, 0xBD],
    [0x1F, 0x00, 0x81], [0x60, 0x56, 0x81], [0x19, 0x00, 0x68], [0x4E, 0x45, 0x68],
    [0x13, 0x00, 0x4F], [0x3B, 0x35, 0x4F], [0x7F, 0x00, 0xFF], [0xD4, 0xAA, 0xFF],
    [0x5E, 0x00, 0xBD], [0x9D, 0x7E, 0xBD], [0x40, 0x00, 0x81], [0x6B, 0x56, 0x81],
    [0x34, 0x00, 0x68], [0x56, 0x45, 0x68], [0x27, 0x00, 0x4F], [0x42, 0x35, 0x4F],
    [0xBF, 0x00, 0xFF], [0xEA, 0xAA, 0xFF], [0x8D, 0x00, 0xBD], [0xAD, 0x7E, 0xBD],
    [0x60, 0x00, 0x81], [0x76, 0x56, 0x81], [0x4E, 0x00, 0x68], [0x5F, 0x45, 0x68],
    [0x3B, 0x00, 0x4F], [0x49, 0x35, 0x4F], [0xFF, 0x00, 0xFF], [0xFF, 0xAA, 0xFF],
    [0xBD, 0x00, 0xBD], [0xBD, 0x7E, 0xBD], [0x81, 0x00, 0x81], [0x81, 0x56, 0x81],
    [0x68, 0x00, 0x68], [0x68, 0x45, 0x68], [0x4F, 0x00, 0x4F], [0x4F, 0x35, 0x4F],
    [0xFF, 0x00, 0xBF], [0xFF, 0xAA, 0xEA], [0xBD, 0x00, 0x8D], [0xBD, 0x7E, 0xAD],
    [0x81, 0x00, 0x60], [0x81, 0x56, 0x76], [0x68, 0x00, 0x4E], [0x68, 0x45, 0x5F],
    [0x4F, 0x00, 0x3B], [0x4F, 0x35, 0x49], [0xFF, 0x00, 0x7F], [0xFF, 0xAA, 0xD4],
    [0xBD, 0x00, 0x5E], [0xBD, 0x7E, 0x9D], [0x81, 0x00, 0x40], [0x81, 0x56, 0x6B],
    [0x68, 0x00, 0x34], [0x68, 0x45, 0x56], [0x4F, 0x00, 0x27], [0x4F, 0x35, 0x42],
    [0xFF, 0x00, 0x3F], [0xFF, 0xAA, 0xBF], [0xBD, 0x00, 0x2E], [0xBD, 0x7E, 0x8D],
    [0x81, 0x00, 0x1F], [0x81, 0x56, 0x60], [0x68, 0x00, 0x19], [0x68, 0x45, 0x4E],
    [0x4F, 0x00, 0x13], [0x4F, 0x35, 0x3B], [0x33, 0x33, 0x33], [0x50, 0x50, 0x50],
    [0x69, 0x69, 0x69], [0x82, 0x82, 0x82], [0xBE, 0xBE, 0xBE], [0xFF, 0xFF, 0xFF],
];

/// Map a 24-bit truecolor (`0xRRGGBB`) back to its AutoCAD palette index.
///
/// Mirrors LibreDWG's `dwg_find_color_index`: an exact linear scan over
/// [`RGB_PALETTE`]. Returns 256 when there is no exact match — the "BYLAYER"
/// sentinel in this parser's colour model (matches LibreDWG, which also
/// returns 256 for non-palette colours rather than picking the nearest).
pub fn rgb_to_palette_index(rgb: u32) -> i16 {
    let r = ((rgb >> 16) & 0xFF) as u8;
    let g = ((rgb >> 8) & 0xFF) as u8;
    let b = (rgb & 0xFF) as u8;
    for (i, c) in RGB_PALETTE.iter().enumerate() {
        if c[0] == r && c[1] == g && c[2] == b {
            return i as i16;
        }
    }
    256
}

/// Resolve an R2004+ `CMC` colour's ACI index from its raw `BL rgb`.
///
/// The low 24 bits of a `CMC` `rgb` are **not** always a truecolor. Real
/// AutoCAD files store a plain palette colour as `method << 24 | index`, where
/// `method` is the top byte:
///
/// | method | meaning                                    |
/// |--------|--------------------------------------------|
/// | `0xC0` | ByLayer                                    |
/// | `0xC1` | ByBlock                                    |
/// | `0xC2` | entity (default) — low bits really are RGB |
/// | `0xC3` | truecolor                                  |
/// | `0xC8` | none                                       |
///
/// Verified on `example_2007.dwg`: its layers carry `rgb` `0xC3000007`,
/// `0xC3000002` and `0xC3000004`, and the ground-truth DXF export reports
/// group 62 as 7, 2 and 4 — i.e. the low bits hold the *index*, not
/// `RGB_PALETTE[index]` (which would be `0xFFFFFF`, `0xFFFF00`, `0x00FFFF`).
/// A pure palette lookup therefore returns the 256 "no match" sentinel for
/// every single layer.
///
/// The fix follows LibreDWG's `bit_downconvert_CMC`, which falls back to
/// `index = rgb & 0xff` when the palette scan fails:
///
/// ```c
/// index = dwg_find_color_index (color->rgb);
/// if (color->index == 256)
///   color->index = color->rgb & 0xff;
/// ```
pub fn cmc_color_index(rgb: u32) -> i16 {
    match (rgb >> 24) as u8 {
        0xC0 => 256, // ByLayer
        0xC1 => 0,   // ByBlock
        0xC8 => 0,   // none
        // No method byte: the whole thing is a plain 0xRRGGBB.
        0 => rgb_to_palette_index(rgb),
        _ => {
            let idx = rgb_to_palette_index(rgb & 0x00FF_FFFF);
            if idx == 256 {
                (rgb & 0xFF) as i16
            } else {
                idx
            }
        }
    }
}

/// Parsed `LAYER` table record.
#[derive(Debug, Clone)]
pub struct LayerInfo {
    pub handle: u64,
    pub name: String,
    pub color_index: i16,
    pub flags: u16,
    pub frozen: bool,
    pub off: bool,
    pub frozen_in_new: bool,
    pub locked: bool,
    pub plot_flag: bool,
    pub lineweight: u8,
    pub ltype_handle: Option<u64>,
    pub plotstyle_handle: Option<u64>,
}

/// One decoded geometry primitive.
#[derive(Debug, Clone)]
pub enum Geometry {
    Line {
        start: (f64, f64, f64),
        end: (f64, f64, f64),
    },
    Circle {
        center: (f64, f64, f64),
        radius: f64,
    },
    Arc {
        center: (f64, f64, f64),
        radius: f64,
        start_angle: f64,
        end_angle: f64,
    },
    Point {
        x: f64,
        y: f64,
        z: f64,
    },
    LwPolyline {
        flag: u16,
        /// Vertices (2D). When `flag & 1` is set the polyline is a closed ring.
        points: Vec<(f64, f64)>,
        /// Per-vertex bulge (DXF group 42), aligned 1:1 with `points`; `0.0`
        /// means a straight segment. A non-zero bulge turns the segment into a
        /// circular arc — needed so curved parcel boundaries aren't flattened
        /// into a chord (which would look like a "missing segment" in GIS).
        bulges: Vec<f64>,
    },
    Text {
        ins_pt: (f64, f64),
        height: f64,
        value: String,
    },
    /// Multi-line text (`MTEXT`, type code `0x2c`). Unlike `Text`, the insertion
    /// point is a full 3D point and the entity carries a rotation given by the
    /// `x_axis_dir` vector (DXF group 11) rather than a scalar angle.
    MText {
        ins_pt: (f64, f64, f64),
        height: f64,
        value: String,
    },
    Insert {
        ins_pt: (f64, f64, f64),
        block_name: String,
        scale: (f64, f64, f64),
    },
    /// A (classic) POLYLINE and the vertices attached to it. DWG stores a
    /// polyline as a `POLYLINE_*` header object followed — in the object stream,
    /// contiguously — by a chain of `VERTEX_*` objects terminated by a `SEQEND`.
    /// The parser stitches those vertices back onto the header in
    /// [`crate::parser`]'s assembly pass. `is_3d` distinguishes a 3D polyline /
    /// mesh (z meaningful) from a 2D polyline (z comes from `elevation`).
    Polyline {
        flag: u16,
        curve_type: u16,
        elevation: f64,
        is_3d: bool,
        vertices: Vec<(f64, f64, f64)>,
        /// Per-vertex bulge, aligned 1:1 with `vertices` (0.0 when absent).
        bulges: Vec<f64>,
    },
    /// An ellipse: `center`, the endpoint of the major axis (a vector measured
    /// from `center`), and the minor/major `axis_ratio` (0..1). Angles in
    /// radians, measured CCW from the major axis.
    Ellipse {
        center: (f64, f64, f64),
        major_axis: (f64, f64, f64),
        axis_ratio: f64,
        start_angle: f64,
        end_angle: f64,
    },
    /// A 3D face — four corner points plus an edge-visibility bitmask.
    Face3d {
        corners: [(f64, f64, f64); 4],
        /// DXF group 70: which of the four edges are invisible.
        invisibility: u16,
    },
    /// A SOLID / TRACE — four corner points (planar, sharing `elevation`).
    Solid {
        corners: [(f64, f64, f64); 4],
        elevation: f64,
    },
}

/// Decoded `VERTEX_*` data, kept on the vertex object so the polyline assembly
/// pass can stitch points (and bulges) back onto their parent `POLYLINE_*`.
#[derive(Debug, Clone, Default)]
pub struct VertexData {
    pub point: (f64, f64, f64),
    /// DXF group 42 bulge (0.0 when the vertex has none).
    pub bulge: f64,
    /// DXF group 70 vertex flags.
    pub flag: u16,
    /// DXF groups 40/41 start/end width (only `VERTEX_2D` carries them).
    pub width_start: f64,
    pub width_end: f64,
    /// Only on `VERTEX_PFACE_FACE` (0x0e): the four signed vertex indices that
    /// form one face of a polyface mesh. Set to `None` for positional vertices.
    pub face_indices: Option<[i16; 4]>,
}

/// Decoded `POLYLINE_*` header fields. The actual vertices are assembled
/// afterwards from the chained `VERTEX_*` objects.
#[derive(Debug, Clone, Default)]
pub struct PolylineHeader {
    pub flag: u16,
    pub curve_type: u16,
    pub elevation: f64,
    /// `true` for `POLYLINE_3D` / `POLYLINE_MESH` (z is meaningful); `false`
    /// for `POLYLINE_2D` / `POLYLINE_PFACE` (z comes from `elevation`).
    pub is_3d: bool,
    /// The handles of the `VERTEX_*` objects owned by this polyline, in order.
    /// Filled from the polyline's handle stream (`HANDLE_VECTOR` on R2004+).
    /// Empty when the format provides only first/last handles (R2000), where
    /// vertex assembly falls back to sequential grouping.
    pub vertex_handles: Vec<u64>,
}

/// One EED data item (internal EED codes 0..=71).
#[derive(Debug, Clone)]
pub struct EedItem {
    pub code: u8,
    pub raw: Vec<u8>,
    pub text: Option<String>,
    pub point: Option<(f64, f64, f64)>,
    pub real: Option<f64>,
    pub int16: Option<i16>,
    pub int32: Option<i32>,
    pub int64: Option<i64>,
}

/// A decoded EED block: an application-id handle plus its data items.
#[derive(Debug, Clone)]
pub struct Eed {
    pub appid_handle: u64,
    pub items: Vec<EedItem>,
}

/// A decoded XData (`XRECORD`) value.
#[derive(Debug, Clone, PartialEq)]
pub enum XDataValue {
    Str(String),
    Real(f64),
    Int8(i8),
    Int16(i16),
    Int32(i32),
    Int64(i64),
    Point3((f64, f64, f64)),
    Binary(Vec<u8>),
    Handle(u64),
}

/// One XData (`XRECORD`) resbuf entry, keyed by its DXF group code.
#[derive(Debug, Clone)]
pub struct XDataItem {
    pub code: i16,
    pub value: XDataValue,
}

/// Everything we extracted from a single DWG object/entity.
#[derive(Debug, Clone, Default)]
pub struct ParsedObject {
    pub index: usize,
    pub type_code: u16,
    pub type_name: String,
    pub handle: u64,
    pub layer_handle: Option<u64>,
    /// Owner handle (resolved from the handle stream). For objects inside a
    /// block definition this is the owning `BLOCK_HEADER`; for XRECORDs in an
    /// extension-dictionary chain it is the dictionary / parent XRECORD.
    pub owner_handle: Option<u64>,
    /// Extension-dictionary handle (`XDICOBJHANDLE`) — the root of the
    /// object's associated XRECORD chain. Present on entities that carry
    /// linked extension data.
    pub xdict_handle: Option<u64>,
    /// Reactor handles (resolved), in stream order — used to walk
    /// reactor-linked XRECORD chains.
    pub reactors: Vec<u64>,
    /// Handle of the `BLOCK_HEADER` this entity references (only `INSERT`).
    pub block_handle: Option<u64>,
    /// Block name; set on `BLOCK_HEADER` (0x31) records that define a block.
    pub block_name: Option<String>,
    pub geometry: Option<Geometry>,
    pub text: Option<String>,
    pub layer: Option<LayerInfo>,
    /// Decoded `VERTEX_*` geometry, present on `VERTEX_2D/3D/MESH/PFACE/
    /// PFACE_FACE` objects. Not mirrored into [`DwgFile::entities`] — the
    /// parent `POLYLINE_*` consumes it during assembly.
    pub vertex: Option<VertexData>,
    /// Decoded `POLYLINE_*` header, present on `POLYLINE_2D/3D/PFACE/MESH`.
    pub polyline: Option<PolylineHeader>,
    /// Named entries of a `DICTIONARY` (`0x2a`) object, in stream order.
    /// This is what links an entity's extension dictionary to its
    /// per-field `XRECORD` attribute chain.
    pub dictionary: Option<Vec<DictionaryEntry>>,
    pub xdata: Vec<XDataItem>,
    /// `cloning` flag of an `XRECORD` (`DXF 280`), read *after* the XData resbuf
    /// chain. It is a property of the XRECORD object itself, **not** a resbuf
    /// group — LibreDWG decodes it as a separate `BS cloning` field and never
    /// folds it into the XData list, so we keep it apart too.
    pub cloning: Option<i16>,
    pub eed: Vec<Eed>,
}

/// One named entry of a `DICTIONARY` object (`0x2a`): the entry's key `name`
/// and the handle of the object it references (usually an `XRECORD` carrying
/// one attribute value, or a nested `DICTIONARY`).
#[derive(Debug, Clone)]
pub struct DictionaryEntry {
    pub name: String,
    pub handle: u64,
}

/// Top-level parse result.
#[derive(Debug)]
pub struct DwgFile {
    pub version: String,
    pub object_count: usize,
    pub layers: Vec<LayerInfo>,
    pub objects: Vec<ParsedObject>,
    handle_index: HashMap<u64, usize>,
    /// BLOCK_HEADER handle -> block name (`*Model_Space`, `*Paper_Space`, …).
    block_names: HashMap<u64, String>,
}

impl DwgFile {
    /// Parse a DWG file from disk.
    pub fn parse(path: &str) -> Result<DwgFile, String> {
        parser::parse_file(path)
    }

    /// Resolve a layer handle to its layer name (e.g. for reporting an
    /// entity's owning layer).
    pub fn layer_name(&self, handle: u64) -> Option<&str> {
        let idx = *self.handle_index.get(&handle)?;
        self.objects.get(idx).and_then(|o| o.layer.as_ref()).map(|l| l.name.as_str())
    }

    /// Look up any object (entity or table record) by its handle.
    pub fn object_by_handle(&self, handle: u64) -> Option<&ParsedObject> {
        let idx = *self.handle_index.get(&handle)?;
        self.objects.get(idx)
    }

    /// Find a layer by its exact name (DWG layer names are case-preserving).
    pub fn layer_by_name(&self, name: &str) -> Option<&LayerInfo> {
        self.layers.iter().find(|l| l.name == name)
    }

    /// All entities that live on `layer`.
    ///
    /// Convenience wrapper over `entities().filter(|o| o.layer_handle ==
    /// Some(layer.handle))`; the binary format links an entity to its layer
    /// purely by handle, never by name.
    pub fn entities_on_layer<'a>(
        &'a self,
        layer: &'a LayerInfo,
    ) -> impl Iterator<Item = &'a ParsedObject> {
        self.entities().filter(move |o| o.layer_handle == Some(layer.handle))
    }

    /// Resolve a `BLOCK_HEADER` handle to the block's name.
    ///
    /// An `INSERT` entity stores only its block_header *handle* in the binary
    /// format, so this is how its `block_name` is filled in.
    pub fn block_name(&self, handle: u64) -> Option<&str> {
        self.block_names.get(&handle).map(|s| s.as_str())
    }

    /// Iterate only the objects that carry geometry.
    pub fn entities(&self) -> impl Iterator<Item = &ParsedObject> {
        self.objects.iter().filter(|o| o.geometry.is_some())
    }

    /// Count of objects carrying any extended attribute (EED or XData).
    pub fn extended_attribute_count(&self) -> usize {
        self.objects
            .iter()
            .filter(|o| !o.eed.is_empty() || !o.xdata.is_empty())
            .count()
    }
}

// Re-export the bit reader in case callers want to craft streams for tests.
pub use bitreader::BitReader as BitChain;
pub use parser::Dialect;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cmc_index_is_stored_in_the_low_byte_not_as_palette_rgb() {
        // Real ACAD R2007 files store a plain palette colour as
        // `method << 24 | index` (method 0xC3), *not* as RGB_PALETTE[index].
        // These three come from example_2007.dwg and match DXF group 62.
        assert_eq!(cmc_color_index(0xC3000007), 7);
        assert_eq!(cmc_color_index(0xC3000002), 2);
        assert_eq!(cmc_color_index(0xC3000004), 4);

        // ... which is why a bare palette lookup cannot work:
        assert_eq!(rgb_to_palette_index(0xC3000007 & 0x00FF_FFFF), 256);
        // (index 7 really is 0xFFFFFF, so the stored value is not an RGB)
        assert_eq!(RGB_PALETTE[7], [0xFF, 0xFF, 0xFF]);
    }

    #[test]
    fn cmc_index_falls_back_to_palette_for_real_truecolors() {
        // A genuine 0xRRGGBB still resolves through the palette: 0x00FF00 is
        // index 3 (green), 0xFF0000 is index 1 (red).
        assert_eq!(cmc_color_index(0xC300FF00), 3);
        assert_eq!(cmc_color_index(0x0000FF00), 3);
        assert_eq!(cmc_color_index(0x00FF0000), 1);
        // Anything off-palette degrades to `rgb & 0xff`, exactly like
        // LibreDWG's bit_downconvert_CMC fallback.
        assert_eq!(cmc_color_index(0xC3010203), 0x03);
    }

    #[test]
    fn cmc_index_handles_the_special_methods() {
        assert_eq!(cmc_color_index(0xC0000000), 256); // ByLayer
        assert_eq!(cmc_color_index(0xC1000000), 0); // ByBlock
        assert_eq!(cmc_color_index(0xC8000000), 0); // none
    }
}
