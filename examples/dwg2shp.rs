//! `dwg2shp` — convert parsed DWG entities (via `dwgparse`) to ESRI
//! Shapefiles, preserving extended attributes (EED + XData) inside the DBF
//! attribute table.
//!
//! `cargo run --release --example dwg2shp -- <file.dwg> [out_dir]`
//!
//! Output (one shapefile per geometry class that has data):
//!   * <stem>_points.{shp,shx,dbf}
//!   * <stem>_lines.{shp,shx,dbf}
//!   * <stem>_polygons.{shp,shx,dbf}
//!   * <stem>_extattr.csv   — every object carrying EED/XData (spatial or not)
//!
//! Extended attributes are stored losslessly as UTF-8 text across DBF
//! columns `EXT_1, EXT_2, …` (each 250 bytes); reassemble by simple
//! concatenation in column order. The companion `.csv` keeps the same data
//! with no 250-byte limit, covering even non-spatial objects (STYLE, XRECORD).

use dwgparse::{DwgFile, Geometry};
use std::env;
use std::fs;
use std::path::Path;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Cat {
    Point,
    Line,
    Polygon,
}

/// DBF text encoding.
///
/// ESRI Shapefile DBF stores text as raw bytes with only a single "language
/// driver" (`LDID`) byte to hint the codepage, and most GIS readers (GDAL/OGR,
/// ArcMap) fall back to the *system* codepage when that hint is ambiguous —
/// which for a Chinese Windows means GBK, producing mojibake for UTF-8 bytes.
///
/// Because this tool mostly runs in a Chinese environment, the default is now
/// **GBK**, which every Chinese GIS reader (ArcMap, ArcGIS Pro, QGIS on Chinese
/// Windows) decodes natively — no `.cpg` dependency. UTF-8 is still available
/// via `--encoding utf8` when the downstream chain reliably honours `.cpg`.
///
/// * `Gbk` (default) — re-encode text as GBK (codepage 936) and set
///   `LDID = 0x57` ("Windows ANSI" → system cp936 on Chinese Windows). Verified
///   to display correctly in ArcMap; also emits `.cpg = GBK` for GDAL/QGIS/Pro.
/// * `Utf8` — write UTF-8 bytes, set `LDID = 0x57`, and emit a `.cpg` file
///   containing `UTF-8`. The `.cpg` is what GDAL/OGR and ArcGIS Pro actually
///   trust, so use this when targeting QGIS / ArcGIS Pro with a UTF-8 pipeline.
#[derive(Clone, Copy, PartialEq)]
enum Encoding {
    Utf8,
    Gbk,
}

impl Encoding {
    fn ldid(self) -> u8 {
        match self {
            Encoding::Utf8 => 0x57, // codepage 65001 (UTF-8); readers rely on .cpg
            // 0x57 = "Windows ANSI". ArcMap resolves this to the *system* ANSI
            // codepage (CP_ACP), which on a Chinese Windows is cp936 (GBK) — so
            // GBK bytes are decoded as GBK. This is the value verified to work in
            // ArcMap on a Chinese system. (0x7C is the explicit "cp936" marker and
            // is honoured by GDAL/QGIS/ArcGIS Pro, but some ArcMap builds ignore it
            // and fall back to CP_ACP — which is exactly what 0x57 pins down.)
            Encoding::Gbk => 0x57,
        }
    }
    fn cpg(self) -> &'static str {
        match self {
            Encoding::Utf8 => "UTF-8",
            Encoding::Gbk => "GBK",
        }
    }
    /// Encode `s` into this encoding, truncated/padded to exactly `len` bytes.
    fn field(self, s: &str, len: usize) -> Vec<u8> {
        match self {
            Encoding::Utf8 => {
                let mut b = s.as_bytes().to_vec();
                if b.len() > len {
                    let mut cut = len;
                    while cut > 0 && !s.is_char_boundary(cut) {
                        cut -= 1;
                    }
                    b.truncate(cut);
                }
                b.resize(len, 0x20);
                b
            }
            Encoding::Gbk => {
                // Encode char-by-char so we never split a 2-byte GBK sequence
                // at the field boundary.
                let mut out: Vec<u8> = Vec::with_capacity(len);
                for c in s.chars() {
                    let eb = gbk_bytes(c);
                    if out.len() + eb.len() > len {
                        break;
                    }
                    out.extend_from_slice(&eb);
                }
                out.resize(len, 0x20);
                out
            }
        }
    }
}

/// Encode a single character as GBK (codepage 936). Unmappable characters
/// collapse to a literal `?` (0x3F), matching classic DBF behaviour.
fn gbk_bytes(c: char) -> Vec<u8> {
    let s = c.to_string();
    let (bytes, _, _) = encoding_rs::GBK.encode(&s);
    bytes.to_vec()
}

/// Write `bytes` to `path`, reporting a clear, actionable error instead of
/// panicking on `.unwrap()`. The most common failure is `PermissionDenied`
/// because the file is currently open/locked in a GIS tool (QGIS / ArcMap) or a
/// spreadsheet — exactly the situation where a silent panic would leave the
/// user staring at *stale, still-garbled* output and concluding the fix failed.
fn write_file(path: &Path, bytes: &[u8]) {
    if let Err(e) = fs::write(path, bytes) {
        eprintln!(
            "  !! 写入失败: {}\n     原因: {}\n     该文件可能正被其他程序占用（QGIS / ArcGIS / Excel 等）。\n     请先关闭这些程序，再重新运行转换。",
            path.display(),
            e
        );
        std::process::exit(2);
    }
}

/// Flag bits that mark a `LWPOLYLINE`/`POLYLINE` as a *closed* ring.
///
/// * bit 0 (`0x01`) — the standard DXF group-70 closed flag, set by AutoCAD
///   when the polyline is genuinely closed (no duplicated endpoint stored).
/// * bit 9 (`0x200`) — observed in Chinese cadastral / CASS exports (e.g.
///   `地块解析_cad2018.dwg`, layer `YJJBNTBHTB`) where parcel boundaries carry
///   `0x200` as their closed marker even though the geometry is stored open
///   (last vertex ~9–15 m from the first). Honouring it recovers the missing
///   closing edge that the raw geometry otherwise drops.
const CLOSED_FLAG_BITS: u16 = 0x01 | 0x200;

/// **Polygon classification for a linear entity (LWPOLYLINE / POLYLINE).**
///
/// A polyline becomes a polygon parcel (面图斑) only on a *genuine* closed-area
/// signal — it must actually enclose an area. The signals, in priority order:
///
/// 1. An explicit `Closed` / `闭合` attribute:
///    * value 是/Yes/…  → polygon (authoritative, overrides everything);
///    * value 否/No/…    → line (authoritative — e.g. a 线状围墙 that happens
///      to carry the CASS `0x200` marker or to be geometrically closed);
/// 2. DWG closed-flag bits (`CLOSED_FLAG_BITS`): bit 0 is the standard AutoCAD
///    closed marker; bit 9 (`0x200`) is the CASS / southern-survey "面状"
///    marker used for building footprints, parcels and closed walls in the
///    test corpus — verified to be the reliable "area" signal across the
///    宅基地 / 地块解析_cad2018 / 张南岳 samples;
/// 3. Geometrically closed (last vertex ≈ first vertex).
///
/// Layers are **NOT** force-closed by default. A `--close-layers JMD,JZD`
/// switch still exists for files where the author truly wants every polyline
/// on those layers coerced into an area, but it is opt-in because blanket
/// layer forcing wrongly upgrades linear features (线状围墙, fences, road
/// centre-lines) that live on the same mixed layers (e.g. `JMD`) into
/// polygons. In the test data the polygon cases are already covered by the
/// `0x200` marker, so the default of no force-closing keeps 线状围墙 as lines
/// while 面状 building outlines stay polygons.
fn layer_force_polygon(layer: &str, close: &[String]) -> bool {
    let u = layer.to_uppercase();
    close.iter().any(|p| u.starts_with(&p.to_uppercase()))
}

/// True when `v` reads as an affirmative "closed" value, the way a DWG /
/// cadastral attribute expresses it: `是`, `Yes`, `Y`, `TRUE`, `1`, `on`,
/// `闭合`, etc.
fn is_closed_value(v: &str) -> bool {
    let t = v.trim().to_ascii_lowercase();
    matches!(
        t.as_str(),
        "是" | "yes" | "y" | "true" | "t" | "1" | "on" | "闭合" | "guihe" | "closed"
    ) || t.contains("yes")
}

/// Authoritative closed-state of a linear entity, decided by an explicit
/// `Closed` / `闭合` named attribute (resolved from the entity's extension
/// dictionary XRECORD chain into `attrs`):
///
/// * `Yes`  — the attribute exists and reads as 是/Yes/… → the entity **must**
///   become a polygon parcel (面状图斑), regardless of any DWG flag bits or
///   geometric closure.
/// * `No`   — the attribute exists but reads as 否/No/… → the entity **must**
///   stay a line (线状几何对象), even if its DWG closed flag (`0x200`) is set
///   or its vertices happen to coincide. This is the mis-classification the
///   tool used to make (flag/geometry overrode an explicit "No").
/// * `Absent` — no `Closed`/`闭合` attribute is present → fall back to the
///   usual heuristics (DWG closed flag bits, geometric closure, `--close-layers`).
///
/// Only an affirmative value means "closed"; any other present value (否, No,
/// 0, …) is treated as `No` so a stray/unknown value never silently upgrades a
/// line to a polygon.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ClosedState {
    Absent,
    Yes,
    No,
}

fn closed_state(attrs: &[(String, String)]) -> ClosedState {
    for (k, v) in attrs {
        let kk = k.trim().to_ascii_lowercase();
        if kk == "closed" || k == "闭合" || kk.contains("clos") {
            return if is_closed_value(v) {
                ClosedState::Yes
            } else {
                ClosedState::No
            };
        }
    }
    ClosedState::Absent
}

struct Feat {
    cat: Cat,
    layer: String,
    etype: String,
    handle: String,
    /// 原CAD实体编码 — the entity's original CAD handle (object id), so the
    /// SHP feature can be traced back to its source DWG entity.
    cad_code: String,
    /// CASS地物编码 — the CASS/South-CAD feature code carried in the entity's
    /// EED (first `code==0` text item, e.g. `141121` for a building outline,
    /// `144301` for a wall, `300000` for a parcel). Empty when the entity
    /// carries no EED feature code.
    cass_code: String,
    ext: String,
    txt: String,
    /// Named extension attributes resolved from the entity's extension-
    /// dictionary XRECORD chain: `(dictionary key, XRECORD value)` in chain
    /// order — e.g. `("BSM", "610621211000011752")` (标识码).
    attrs: Vec<(String, String)>,
    pts: Vec<(f64, f64)>,
}

/// Resolve an entity's extension-dictionary attribute chain into named
/// `(key, value)` pairs.
///
/// Association path (verified on `地块解析_cad2018.dwg`):
/// ```text
/// entity.xdict_handle            -> DICTIONARY  (e.g. 0xE4)
///   .entry "ESRI_Attributes"     -> DICTIONARY  (e.g. 0xE5)
///     .entries key -> XRECORD    (47 named records, one value each)
/// ```
/// Intermediate levels are followed recursively, so both shallow chains
/// (`xdict -> XRECORD`) and the nested `DICTIONARY -> DICTIONARY -> XRECORD`
/// form resolve. Cycles are broken by a visited set.
fn resolve_ext_attributes(dwg: &DwgFile, entity: &dwgparse::ParsedObject) -> Vec<(String, String)> {
    let mut out = Vec::new();
    let Some(xd) = entity.xdict_handle else {
        return out;
    };
    let mut visited: Vec<u64> = vec![entity.handle, xd];
    walk_dictionary(dwg, xd, &mut visited, &mut out);
    out
}

fn walk_dictionary(
    dwg: &DwgFile,
    dict_handle: u64,
    visited: &mut Vec<u64>,
    out: &mut Vec<(String, String)>,
) {
    let Some(dict) = dwg.object_by_handle(dict_handle) else {
        return;
    };
    let Some(entries) = &dict.dictionary else {
        return;
    };
    for e in entries {
        let Some(target) = dwg.object_by_handle(e.handle) else {
            continue;
        };
        if visited.contains(&e.handle) {
            continue;
        }
        visited.push(e.handle);
        if target.dictionary.is_some() {
            // Nested dictionary (e.g. "ESRI_Attributes" -> attribute dict).
            walk_dictionary(dwg, e.handle, visited, out);
        } else if target.type_code == 0x4f {
            // XRECORD: one attribute value.
            out.push((e.name.clone(), xrecord_value(target)));
        }
    }
}

/// Extract the attribute value an XRECORD carries: the last XData item wins,
/// because writers emit a filler empty string (group 1) before the real
/// numeric payload (groups 40/90/170) — e.g. `{1: "", 90: 30}` means `30`.
fn xrecord_value(x: &dwgparse::ParsedObject) -> String {
    match x.xdata.last() {
        Some(item) => match &item.value {
            dwgparse::XDataValue::Str(t) => t.clone(),
            dwgparse::XDataValue::Real(r) => {
                // Trim trailing zeros: 24252.77 stays, 0.1058 stays.
                let s = format!("{}", r);
                s
            }
            dwgparse::XDataValue::Int8(v) => v.to_string(),
            dwgparse::XDataValue::Int16(v) => v.to_string(),
            dwgparse::XDataValue::Int32(v) => v.to_string(),
            dwgparse::XDataValue::Int64(v) => v.to_string(),
            dwgparse::XDataValue::Point3(p) => format!("{},{},{}", p.0, p.1, p.2),
            dwgparse::XDataValue::Binary(b) => format!("<{}B>", b.len()),
            dwgparse::XDataValue::Handle(h) => format!("0x{:X}", h),
        },
        None => String::new(),
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let mut dwg_path = "tests/data/宅基地.dwg".to_string();
    let mut out_dir = "dwg2shp_out".to_string();
    // `--close-layers` is opt-in: by default NO layer is force-closed, so
    // mixed layers (e.g. JMD) keep their 线状围墙/线状 features as lines and
    // only genuinely-closed (0x200 / bit0 / geometric / Closed=Yes) entities
    // become polygons.
    let mut close_layers: Vec<String> = Vec::new();
    let mut encoding = Encoding::Gbk;
    let mut dwg_seen = false;

    let mut i = 1;
    while i < args.len() {
        let a = &args[i];
        if a == "--encoding" {
            if let Some(v) = args.get(i + 1) {
                encoding = match v.to_ascii_lowercase().as_str() {
                    "gbk" | "936" | "cp936" => Encoding::Gbk,
                    "utf8" | "utf-8" | "65001" => Encoding::Utf8,
                    other => {
                        eprintln!("warning: unknown --encoding '{other}', using gbk (Chinese env default)");
                        Encoding::Gbk
                    }
                };
            }
            i += 2;
            continue;
        } else if let Some(v) = a.strip_prefix("--encoding=") {
            encoding = match v.to_ascii_lowercase().as_str() {
                "gbk" | "936" | "cp936" => Encoding::Gbk,
                "utf8" | "utf-8" | "65001" => Encoding::Utf8,
                other => {
                    eprintln!("warning: unknown --encoding '{other}', using gbk (Chinese env default)");
                    Encoding::Gbk
                }
            };
            i += 1;
            continue;
        } else if a == "--close-layers" {
            if let Some(v) = args.get(i + 1) {
                close_layers = v
                    .split(',')
                    .map(|s| s.trim().to_string())
                    .filter(|s| !s.is_empty())
                    .collect();
            }
            i += 2;
            continue;
        } else if let Some(v) = a.strip_prefix("--close-layers=") {
            close_layers = v
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect();
            i += 1;
            continue;
        } else if !a.starts_with('-') {
            // positional: first free arg = dwg path, second = out dir
            if !dwg_seen {
                dwg_path = a.clone();
                dwg_seen = true;
            } else {
                out_dir = a.clone();
            }
            i += 1;
            continue;
        }
        i += 1;
    }

    let dwg = DwgFile::parse(&dwg_path).expect("failed to parse DWG");
    let stem = Path::new(&dwg_path)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("output")
        .to_string();
    fs::create_dir_all(&out_dir).expect("mkdir failed");

    let mut points = Vec::new();
    let mut lines = Vec::new();
    let mut polygons = Vec::new();
    // handle, cad_code, cass_code, layer, etype, geom(Y/N), attr
    let mut csv_rows: Vec<(String, String, String, String, String, String, String)> = Vec::new();
    let mut forced_poly = 0usize;
    let mut forced_layers: Vec<String> = Vec::new();

    for obj in &dwg.objects {
        let layer = obj
            .layer_handle
            .and_then(|h| dwg.layer_name(h))
            .unwrap_or("0")
            .to_string();
        let etype = obj.type_name.clone();
        let handle = format!("0x{:X}", obj.handle);
        let ext = ext_attr_string(obj);
        let cass_code = extract_cass_code(obj);
        let cad_code = handle.clone();
        let has_attr = !obj.eed.is_empty() || !obj.xdata.is_empty();

        // Entities owned by a BLOCK_HEADER are block-definition contents, not
        // standalone drawing entities — emitting them produces stray features
        // at the block-local coordinates (e.g. the `A$Cb5949cd7` copy of a
        // parcel boundary stored at (0..458, 0..1076) while the drawing uses
        // UTM coordinates). Standard CAD->GIS practice explodes block contents
        // only via their INSERT references, so skip them here.
        if let Some(own) = obj.owner_handle {
            if dwg
                .object_by_handle(own)
                .map(|o| o.type_code == 0x31 && !o.block_name.as_deref().unwrap_or("").starts_with('*'))
                .unwrap_or(false)
            {
                continue;
            }
        }

        if let Some(g) = &obj.geometry {
            // Resolve named extension attributes first so we can honour an
            // explicit `Closed` / `闭合` attribute when classifying the geometry.
            let attrs = resolve_ext_attributes(&dwg, obj);
            let closed_state = closed_state(&attrs);
            if let Some((cat, pts, txt, was_forced)) =
                geom_classify(g, &layer, &close_layers, closed_state)
            {
                if was_forced {
                    forced_poly += 1;
                    if !forced_layers.contains(&layer) {
                        forced_layers.push(layer.clone());
                    }
                }
                let f = Feat {
                    cat,
                    layer: layer.clone(),
                    etype: etype.clone(),
                    handle: handle.clone(),
                    cad_code: cad_code.clone(),
                    cass_code: cass_code.clone(),
                    ext: ext.clone(),
                    txt,
                    attrs,
                    pts,
                };
                match f.cat {
                    Cat::Point => points.push(f),
                    Cat::Line => lines.push(f),
                    Cat::Polygon => polygons.push(f),
                }
            }
        }
        if has_attr {
            let geom = if obj.geometry.is_some() {
                "Y".to_string()
            } else {
                "N".to_string()
            };
            csv_rows.push((
                handle.clone(),
                cad_code.clone(),
                cass_code.clone(),
                layer.clone(),
                etype.clone(),
                geom,
                ext.clone(),
            ));
        }
    }

    let base = Path::new(&out_dir).join(&stem);
    let bp = |s: &str| base.with_file_name(format!("{}_{}", stem, s));
    let mut written: Vec<String> = Vec::new();
    if !points.is_empty() {
        write_shapefile(&bp("points"), 1, &points, encoding);
        written.push(format!("{}", bp("points").to_string_lossy()));
    }
    if !lines.is_empty() {
        write_shapefile(&bp("lines"), 3, &lines, encoding);
        written.push(format!("{}", bp("lines").to_string_lossy()));
    }
    if !polygons.is_empty() {
        write_shapefile(&bp("polygons"), 5, &polygons, encoding);
        written.push(format!("{}", bp("polygons").to_string_lossy()));
    }

    let csv_path = base.with_file_name(format!("{}_extattr.csv", stem));
    write_csv(&csv_path, &csv_rows);

    println!("DWG version : {}", dwg.version);
    println!("Layers      : {}", dwg.layers.len());
    println!("Objects     : {}", dwg.object_count);
    println!("Points      : {}", points.len());
    println!("Lines       : {}", lines.len());
    println!("Polygons    : {}", polygons.len());
    println!("Ext-attr obj: {}", csv_rows.len());
    println!(
        "Close-layers: {:?}  (opt-in; polylines on these are force-closed to polygon)",
        close_layers
    );
    println!(
        "Force-closed: {} polyline(s) on layer(s) {:?}",
        forced_poly, forced_layers
    );
    println!(
        "DBF encoding : {}  (LDID=0x{:02X}, wrote <stem>.cpg)",
        encoding.cpg(),
        encoding.ldid()
    );
    println!("Output dir  : {}", out_dir);
    for w in &written {
        println!("  wrote {}.shp/.shx/.dbf", w);
    }
    println!("  wrote {}", csv_path.to_string_lossy());
}

/// Map a parsed geometry primitive to (category, 2D points, annotation text,
/// was_force_closed).
///
/// A `LWPOLYLINE`/`POLYLINE` is classified as follows, with an explicit
/// `Closed` / `闭合` named attribute taking priority over everything else:
///
///   * `ClosedState::Yes` — the attribute exists and reads as 是/Yes/… → the
///     entity is **always** a polygon parcel (面图斑), no matter what its DWG
///     flag bits or geometry say.
///   * `ClosedState::No` — the attribute exists but reads as 否/No/… → the
///     entity is **always** a line (线状几何对象); the DWG closed flag
///     (`0x200`) and geometric closure are deliberately ignored, because the
///     author explicitly said the entity is open.
///   * `ClosedState::Absent` — no such attribute → fall back to the usual
///     heuristics: its DWG closed flag is set (`flag & CLOSED_FLAG_BITS`, bit 0
///     = standard AutoCAD marker, bit 9 = CASS/cadastral marker), it is
///     *geometrically* closed (last vertex ≈ first vertex), or its `layer`
///     matches a `--close-layers` prefix.
///
/// A polyline that stays a line but is geometrically closed still gets its
/// closing vertex appended, so the line "图斑" reads as closed in GIS instead
/// of showing a missing segment. Bulges (DXF group 42) are honoured: a non-zero
/// bulge turns the segment into a circular arc, so curved boundaries are
/// reconstructed faithfully rather than flattened into a chord (which would
/// look like a dropped segment).
fn geom_classify(
    g: &Geometry,
    layer: &str,
    close: &[String],
    closed_state: ClosedState,
) -> Option<(Cat, Vec<(f64, f64)>, String, bool)> {
    let force = layer_force_polygon(layer, close);
    match g {
        Geometry::Point { x, y, .. } => Some((Cat::Point, vec![(*x, *y)], String::new(), false)),
        Geometry::Text { ins_pt, value, .. } => {
            Some((Cat::Point, vec![(ins_pt.0, ins_pt.1)], value.clone(), false))
        }
        Geometry::MText { ins_pt, value, .. } => {
            Some((Cat::Point, vec![(ins_pt.0, ins_pt.1)], value.clone(), false))
        }
        Geometry::Insert { ins_pt, .. } => {
            Some((Cat::Point, vec![(ins_pt.0, ins_pt.1)], String::new(), false))
        }
        Geometry::Line { start, end } => Some((
            Cat::Line,
            vec![(start.0, start.1), (end.0, end.1)],
            String::new(),
            false,
        )),
        Geometry::LwPolyline {
            flag,
            points,
            bulges,
        } => {
            let geom_closed = points
                .first()
                .zip(points.last())
                .map(|(a, b)| (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6)
                .unwrap_or(false);
            match closed_state {
                ClosedState::Yes => {
                    Some((Cat::Polygon, expand_polyline(points, bulges, true), String::new(), false))
                }
                ClosedState::No => {
                    // Explicit "No/否": stay a line, never force-close.
                    Some((Cat::Line, expand_polyline(points, bulges, false), String::new(), false))
                }
                ClosedState::Absent => {
                    let closed_flag = flag & CLOSED_FLAG_BITS != 0;
                    let closed = closed_flag || geom_closed;
                    if closed {
                        Some((Cat::Polygon, expand_polyline(points, bulges, true), String::new(), false))
                    } else if force {
                        Some((
                            Cat::Polygon,
                            expand_polyline(points, bulges, true),
                            String::new(),
                            true,
                        ))
                    } else {
                        // Stay a line, but still close the ring if it is
                        // geometrically closed so the SHP line reads as a closed
                        // loop (no missing edge).
                        Some((Cat::Line, expand_polyline(points, bulges, geom_closed), String::new(), false))
                    }
                }
            }
        }
        Geometry::Polyline {
            flag,
            vertices,
            bulges,
            ..
        } => {
            let pts: Vec<(f64, f64)> = vertices.iter().map(|v| (v.0, v.1)).collect();
            let geom_closed = pts
                .first()
                .zip(pts.last())
                .map(|(a, b)| (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6)
                .unwrap_or(false);
            match closed_state {
                ClosedState::Yes => {
                    Some((Cat::Polygon, expand_polyline(&pts, bulges, true), String::new(), false))
                }
                ClosedState::No => {
                    Some((Cat::Line, expand_polyline(&pts, bulges, false), String::new(), false))
                }
                ClosedState::Absent => {
                    let closed_flag = flag & CLOSED_FLAG_BITS != 0;
                    let closed = closed_flag || geom_closed;
                    if closed {
                        Some((Cat::Polygon, expand_polyline(&pts, bulges, true), String::new(), false))
                    } else if force {
                        Some((
                            Cat::Polygon,
                            expand_polyline(&pts, bulges, true),
                            String::new(),
                            true,
                        ))
                    } else {
                        Some((Cat::Line, expand_polyline(&pts, bulges, geom_closed), String::new(), false))
                    }
                }
            }
        }
        Geometry::Circle { center, radius } => {
            let pts = sample_circle(*center, *radius, 64);
            Some((Cat::Polygon, close_ring(pts), String::new(), false))
        }
        Geometry::Arc {
            center,
            radius,
            start_angle,
            end_angle,
        } => {
            let (full, pts) = sample_arc(*center, *radius, *start_angle, *end_angle);
            if full {
                Some((Cat::Polygon, close_ring(pts), String::new(), false))
            } else {
                Some((Cat::Line, pts, String::new(), false))
            }
        }
        Geometry::Ellipse {
            center,
            major_axis,
            axis_ratio,
            start_angle,
            end_angle,
        } => {
            let (full, pts) =
                sample_ellipse(*center, *major_axis, *axis_ratio, *start_angle, *end_angle);
            if full {
                Some((Cat::Polygon, close_ring(pts), String::new(), false))
            } else {
                Some((Cat::Line, pts, String::new(), false))
            }
        }
        Geometry::Face3d { corners, .. } => {
            let pts: Vec<(f64, f64)> = corners.iter().map(|c| (c.0, c.1)).collect();
            Some((Cat::Polygon, close_ring(pts), String::new(), false))
        }
        Geometry::Solid { corners, .. } => {
            let pts: Vec<(f64, f64)> = corners.iter().map(|c| (c.0, c.1)).collect();
            Some((Cat::Polygon, close_ring(pts), String::new(), false))
        }
    }
}

fn close_ring(mut pts: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    if pts.len() >= 2 {
        let (fx, fy) = pts[0];
        let (lx, ly) = *pts.last().unwrap();
        if (fx - lx).abs() > 1e-12 || (fy - ly).abs() > 1e-12 {
            pts.push((fx, fy));
        }
    }
    pts
}

/// Expand a polyline into its true vertex list, honouring per-segment bulges
/// (DXF group 42). A segment with bulge `b` is a circular arc; `b = 0` is a
/// straight chord. `closed` appends the first vertex so the result is a closed
/// ring (recovering the edge the raw geometry otherwise omits).
///
/// Callers must pass `bulges` aligned 1:1 with `points` (one entry per vertex;
/// the bulge at index `i` shapes the segment `points[i] -> points[i+1]`).
fn expand_polyline(
    points: &[(f64, f64)],
    bulges: &[f64],
    closed: bool,
) -> Vec<(f64, f64)> {
    if points.len() < 2 {
        return points.to_vec();
    }
    let n = points.len();
    let seg = if closed { n } else { n - 1 };
    let mut out: Vec<(f64, f64)> = Vec::with_capacity(n + seg * 4);
    for i in 0..seg {
        let p0 = points[i];
        let p1 = points[(i + 1) % n];
        let bulge = bulges.get(i).copied().unwrap_or(0.0);
        out.push(p0);
        if bulge.abs() > 1e-9 {
            out.extend(arc_points(p0, p1, bulge));
        }
    }
    if closed {
        // Explicitly close the ring back to the first vertex.
        let (fx, fy) = points[0];
        let (lx, ly) = *out.last().unwrap();
        if (fx - lx).abs() > 1e-9 || (fy - ly).abs() > 1e-9 {
            out.push((fx, fy));
        }
    } else {
        out.push(*points.last().unwrap());
    }
    out
}

/// Interior arc samples (excluding both endpoints) for the segment `p0 -> p1`
/// with the given bulge. `bulge = tan(theta/4)` where `theta` is the signed
/// included angle; positive bulge bulges to the left of `p0 -> p1`.
fn arc_points(p0: (f64, f64), p1: (f64, f64), bulge: f64) -> Vec<(f64, f64)> {
    let cx = p1.0 - p0.0;
    let cy = p1.1 - p0.1;
    let l = (cx * cx + cy * cy).sqrt();
    if l < 1e-12 {
        return Vec::new();
    }
    let theta = 4.0 * bulge.atan();
    if theta.abs() < 1e-9 {
        return Vec::new();
    }
    let ux = cx / l;
    let uy = cy / l;
    // unit perpendicular pointing to the left of p0->p1
    let (px, py) = (-uy, ux);
    let tanh = (theta / 2.0).tan();
    let cdist = (l / 2.0) / tanh;
    let ccx = (p0.0 + p1.0) / 2.0 + px * cdist;
    let ccy = (p0.1 + p1.1) / 2.0 + py * cdist;
    let r = (l / 2.0) / (theta / 2.0).sin();
    let a0 = (p0.1 - ccy).atan2(p0.0 - ccx);
    let a1 = (p1.1 - ccy).atan2(p1.0 - ccx);
    let mut sweep = a1 - a0;
    while sweep - theta > std::f64::consts::PI {
        sweep -= 2.0 * std::f64::consts::PI;
    }
    while sweep - theta < -std::f64::consts::PI {
        sweep += 2.0 * std::f64::consts::PI;
    }
    let steps = (((theta.abs() / (2.0 * std::f64::consts::PI)) * 64.0).ceil() as i32).max(2);
    let mut v = Vec::with_capacity(steps as usize);
    for k in 1..steps {
        let a = a0 + sweep * (k as f64) / (steps as f64);
        v.push((ccx + r * a.cos(), ccy + r * a.sin()));
    }
    v
}

fn sample_circle(c: (f64, f64, f64), r: f64, n: usize) -> Vec<(f64, f64)> {
    (0..n)
        .map(|i| {
            let t = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
            (c.0 + r * t.cos(), c.1 + r * t.sin())
        })
        .collect()
}

fn norm_span(s: f64) -> f64 {
    let mut s = s;
    while s <= -2.0 * std::f64::consts::PI {
        s += 2.0 * std::f64::consts::PI;
    }
    while s > 2.0 * std::f64::consts::PI {
        s -= 2.0 * std::f64::consts::PI;
    }
    s
}

fn sample_arc(
    c: (f64, f64, f64),
    r: f64,
    a0: f64,
    a1: f64,
) -> (bool, Vec<(f64, f64)>) {
    let span = norm_span(a1 - a0);
    if span.abs() < 1e-9 || (span - 2.0 * std::f64::consts::PI).abs() < 1e-9 {
        return (true, sample_circle(c, r, 64));
    }
    let n = ((span / (2.0 * std::f64::consts::PI) * 64.0).ceil() as i32).max(2);
    let pts: Vec<(f64, f64)> = (0..=n)
        .map(|i| {
            let t = a0 + span * i as f64 / n as f64;
            (c.0 + r * t.cos(), c.1 + r * t.sin())
        })
        .collect();
    (false, pts)
}

fn sample_ellipse(
    c: (f64, f64, f64),
    major: (f64, f64, f64),
    ratio: f64,
    a0: f64,
    a1: f64,
) -> (bool, Vec<(f64, f64)>) {
    let span = norm_span(a1 - a0);
    let full = span.abs() < 1e-9 || (span - 2.0 * std::f64::consts::PI).abs() < 1e-9;
    let mlen = (major.0 * major.0 + major.1 * major.1 + major.2 * major.2).sqrt();
    let (ux, uy) = if mlen > 0.0 {
        (major.0 / mlen, major.1 / mlen)
    } else {
        (1.0, 0.0)
    };
    // minor axis = major axis rotated 90° in the XY plane, scaled by axis_ratio
    let (px, py) = (-uy, ux);
    let span2 = if full { 2.0 * std::f64::consts::PI } else { span };
    let n = if full {
        64
    } else {
        ((span2 / (2.0 * std::f64::consts::PI) * 64.0).ceil() as i32).max(2)
    };
    let pts: Vec<(f64, f64)> = (0..=n)
        .map(|i| {
            let t = a0 + span2 * i as f64 / n as f64;
            let cx = major.0 * t.cos() + px * mlen * ratio * t.sin();
            let cy = major.1 * t.cos() + py * mlen * ratio * t.sin();
            (c.0 + cx, c.1 + cy)
        })
        .collect();
    (full, pts)
}

/// Build a single reversible-ish, human-readable string of all EED + XData.
fn ext_attr_string(obj: &dwgparse::ParsedObject) -> String {
    let mut s = String::new();
    for eed in &obj.eed {
        s.push_str(&format!("EED[appid=0x{:X}]", eed.appid_handle));
        for it in &eed.items {
            let v = if let Some(t) = &it.text {
                format!("\"{}\"", t.replace('"', "'"))
            } else if let Some(p) = &it.point {
                format!("({},{},{})", p.0, p.1, p.2)
            } else if let Some(r) = &it.real {
                format!("{:.6}", r)
            } else if let Some(v) = &it.int16 {
                v.to_string()
            } else if let Some(v) = &it.int32 {
                v.to_string()
            } else if let Some(v) = &it.int64 {
                v.to_string()
            } else {
                "?".to_string()
            };
            s.push_str(&format!(" {}={};", it.code, v));
        }
    }
    for xd in &obj.xdata {
        // `XDataValue` is not re-exported at the crate root, so format via its
        // `Debug` impl (it derives `Debug`). Output e.g. `Str("00025")`.
        s.push_str(&format!(" XData[{}={:?}];", xd.code, xd.value));
    }
    s
}

/// Extract the CASS 地物编码 (feature code) from an entity's EED / XData.
///
/// In CASS / South-CAD exports the feature code is the first EED string item
/// (group code 1000; parsed here as `code == 0` with a text value) — e.g.
/// `141121` (building outline), `144301` (wall), `300000` (parcel). Some
/// writers instead place it in XData as a `Str` value, so that is checked as a
/// fallback.
///
/// Returns the code string, or an empty string when the entity carries no EED
/// feature code at all (so the DBF column stays blank, per requirement).
fn extract_cass_code(obj: &dwgparse::ParsedObject) -> String {
    for eed in &obj.eed {
        for it in &eed.items {
            if let Some(t) = &it.text {
                let s = t.trim();
                if is_cass_code(s) {
                    return s.to_string();
                }
            }
        }
    }
    for xd in &obj.xdata {
        if let dwgparse::XDataValue::Str(t) = &xd.value {
            let s = t.trim();
            if is_cass_code(s) {
                return s.to_string();
            }
        }
    }
    String::new()
}

/// A CASS feature code is a run of ASCII digits, optionally followed by a
/// `-<n>` suffix (e.g. `141121`, `144301-1`). Length 3..=12.
fn is_cass_code(s: &str) -> bool {
    if s.is_empty() || s.len() > 12 {
        return false;
    }
    let mut dash_seen = false;
    for c in s.chars() {
        if c.is_ascii_digit() {
            continue;
        } else if c == '-' && !dash_seen {
            dash_seen = true;
        } else {
            return false;
        }
    }
    s.starts_with(|c: char| c.is_ascii_digit())
}

fn shp_header(
    shape_type: i32,
    xmin: f64,
    ymin: f64,
    xmax: f64,
    ymax: f64,
    file_len_words: i32,
) -> Vec<u8> {
    let mut b = Vec::with_capacity(100);
    b.extend_from_slice(&9994i32.to_be_bytes()); // file code
    for _ in 0..5 {
        b.extend_from_slice(&0i32.to_be_bytes()); // unused
    }
    b.extend_from_slice(&file_len_words.to_be_bytes());
    b.extend_from_slice(&1000i32.to_le_bytes()); // version
    b.extend_from_slice(&shape_type.to_le_bytes());
    for v in [xmin, ymin, xmax, ymax] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    for _ in 0..4 {
        b.extend_from_slice(&0.0f64.to_le_bytes()); // Zmin,Zmax,Mmin,Mmax
    }
    b
}

fn shape_content(shape_type: i32, pts: &[(f64, f64)]) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(&shape_type.to_le_bytes());
    if shape_type == 1 {
        if let Some(p) = pts.first() {
            b.extend_from_slice(&p.0.to_le_bytes());
            b.extend_from_slice(&p.1.to_le_bytes());
        }
    } else {
        let (mut xmn, mut ymn, mut xmx, mut ymx) = (
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        );
        for (x, y) in pts {
            xmn = xmn.min(*x);
            ymn = ymn.min(*y);
            xmx = xmx.max(*x);
            ymx = ymx.max(*y);
        }
        for v in [xmn, ymn, xmx, ymx] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        b.extend_from_slice(&1i32.to_le_bytes()); // numParts
        b.extend_from_slice(&(pts.len() as i32).to_le_bytes()); // numPoints
        b.extend_from_slice(&0i32.to_le_bytes()); // parts[0]
        for (x, y) in pts {
            b.extend_from_slice(&x.to_le_bytes());
            b.extend_from_slice(&y.to_le_bytes());
        }
    }
    b
}

fn write_shapefile(base: &Path, shape_type: i32, feats: &[Feat], encoding: Encoding) {
    let (mut xmin, mut ymin, mut xmax, mut ymax) = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for f in feats {
        for (x, y) in &f.pts {
            xmin = xmin.min(*x);
            ymin = ymin.min(*y);
            xmax = xmax.max(*x);
            ymax = ymax.max(*y);
        }
    }
    if !xmin.is_finite() {
        xmin = 0.0;
        ymin = 0.0;
        xmax = 0.0;
        ymax = 0.0;
    }

    let mut contents: Vec<Vec<u8>> = Vec::with_capacity(feats.len());
    let mut content_lens: Vec<i32> = Vec::with_capacity(feats.len());
    for f in feats {
        let c = shape_content(shape_type, &f.pts);
        content_lens.push((c.len() / 2) as i32);
        contents.push(c);
    }

    let shp_file_len = 50 + 4 * feats.len() as i32 + content_lens.iter().sum::<i32>();
    let mut shp = shp_header(shape_type, xmin, ymin, xmax, ymax, shp_file_len);
    for (i, c) in contents.iter().enumerate() {
        shp.extend_from_slice(&(i as i32 + 1).to_be_bytes()); // record number
        shp.extend_from_slice(&content_lens[i].to_be_bytes()); // content length
        shp.extend_from_slice(c);
    }

    let shx_file_len = 50 + 4 * feats.len() as i32;
    let mut shx = shp_header(shape_type, xmin, ymin, xmax, ymax, shx_file_len);
    let mut off = 50i32;
    for i in 0..feats.len() {
        shx.extend_from_slice(&off.to_be_bytes());
        shx.extend_from_slice(&content_lens[i].to_be_bytes());
        off += 4 + content_lens[i];
    }

    let dbf = build_dbf(feats, encoding);

    write_file(&base.with_extension("shp"), &shp);
    write_file(&base.with_extension("shx"), &shx);
    write_file(&base.with_extension("dbf"), &dbf);
    // Signal the DBF text encoding to GIS readers (GDAL/OGR, ArcGIS Pro trust
    // this file; it is what fixes garbled Chinese in QGIS).
    write_file(&base.with_extension("cpg"), encoding.cpg().as_bytes());
}

fn build_dbf(feats: &[Feat], enc: Encoding) -> Vec<u8> {
    let maxext = feats.iter().map(|f| f.ext.len()).max().unwrap_or(0);
    let n_ext = maxext.div_ceil(250).max(1);

    // Union of the extension-attribute keys across all features, in
    // first-seen order — one DBF column per dictionary key (e.g. `BSM`,
    // `BHKSSJ`, `BHJSSJ`), width = longest encoded value.
    let mut attr_names: Vec<String> = Vec::new();
    for f in feats {
        for (k, _) in &f.attrs {
            if !attr_names.iter().any(|n| n == k) {
                attr_names.push(k.clone());
            }
        }
    }
    let attr_width = |name: &str, feats: &[Feat]| -> usize {
        let mut w = 8usize;
        for f in feats {
            if let Some((_, v)) = f.attrs.iter().find(|(k, _)| k == name) {
                let byte_len = match enc {
                    Encoding::Utf8 => v.len(),
                    Encoding::Gbk => v.chars().map(|c| gbk_bytes(c).len()).sum(),
                };
                w = w.max(byte_len);
            }
        }
        w
    };

    let mut fields: Vec<(String, usize)> = vec![
        ("LAYER".into(), 20),
        ("ETYPE".into(), 14),
        ("HANDLE".into(), 12),
        ("CADCODE".into(), 12),
        ("CASSCODE".into(), 16),
        ("TXT".into(), 254),
    ];
    for name in &attr_names {
        fields.push((name.clone(), attr_width(name, feats)));
    }
    for i in 1..=n_ext {
        fields.push((format!("EXT_{}", i), 250));
    }
    let nfields = fields.len();
    let rec_len: i32 = 1 + fields.iter().map(|(_, l)| *l as i32).sum::<i32>();
    let header_len: i32 = 32 + 32 * nfields as i32 + 1;
    let nrec = feats.len() as i32;

    let mut b = Vec::new();
    b.push(0x03); // dBASE III+ without memo
    b.extend_from_slice(&[0, 0, 0]); // last update date (unused)
    b.extend_from_slice(&nrec.to_le_bytes());
    b.extend_from_slice(&(header_len as i16).to_le_bytes());
    b.extend_from_slice(&(rec_len as i16).to_le_bytes());
    b.extend_from_slice(&[0, 0]); // reserved
    b.push(0); // incomplete transaction
    b.push(0); // encryption
    b.extend_from_slice(&[0; 12]); // reserved (multi-user)
    b.push(0); // MDX flag
    b.push(enc.ldid()); // language driver (0x57=UTF-8 / 0x50=GBK)
    b.extend_from_slice(&[0, 0]); // reserved

    for (name, len) in &fields {
        let mut nm = [0u8; 11];
        let bytes = name.as_bytes();
        let n = bytes.len().min(10);
        nm[..n].copy_from_slice(&bytes[..n]);
        b.extend_from_slice(&nm);
        b.push(b'C'); // character field
        b.extend_from_slice(&[0, 0, 0, 0]); // field displacement (bytes 12-15)
        b.push(*len as u8);
        b.push(0); // decimal count (byte 17)
        b.extend_from_slice(&[0, 0]); // reserved (18-19)
        b.push(0); // work area id (20)
        b.push(0); // reserved (21)
        b.push(0); // MDX flag (22)
        b.extend_from_slice(&[0; 9]); // reserved (23-31); total descriptor = 32 bytes
    }
    b.push(0x0D); // field descriptor terminator

    for f in feats {
        b.push(0x20); // not deleted
        b.extend_from_slice(&enc.field(&f.layer, 20));
        b.extend_from_slice(&enc.field(&f.etype, 14));
        b.extend_from_slice(&enc.field(&f.handle, 12));
        b.extend_from_slice(&enc.field(&f.cad_code, 12));
        b.extend_from_slice(&enc.field(&f.cass_code, 16));
        b.extend_from_slice(&enc.field(&f.txt, 254));
        for name in &attr_names {
            let v = f
                .attrs
                .iter()
                .find(|(k, _)| k == name)
                .map(|(_, v)| v.as_str())
                .unwrap_or("");
            let w = fields
                .iter()
                .find(|(n, _)| n == name)
                .map(|(_, w)| *w)
                .unwrap_or(8);
            b.extend_from_slice(&enc.field(v, w));
        }
        for i in 0..n_ext {
            let chunk: String = f.ext.chars().skip(i * 250).take(250).collect();
            b.extend_from_slice(&enc.field(&chunk, 250));
        }
    }
    b.push(0x1A); // EOF marker
    b
}

fn write_csv(path: &Path, rows: &[(String, String, String, String, String, String, String)]) {
    let mut s = String::new();
    s.push_str("HANDLE,CADCODE,CASSCODE,LAYER,ETYPE,GEOM,ATTR\n");
    for (h, c, cc, l, e, g, a) in rows {
        s.push_str(&csv_field(h));
        s.push(',');
        s.push_str(&csv_field(c));
        s.push(',');
        s.push_str(&csv_field(cc));
        s.push(',');
        s.push_str(&csv_field(l));
        s.push(',');
        s.push_str(&csv_field(e));
        s.push(',');
        s.push_str(&csv_field(g));
        s.push(',');
        s.push_str(&csv_field(a));
        s.push('\n');
    }
    write_file(path, s.as_bytes());
}

fn csv_field(s: &str) -> String {
    if s.contains(',') || s.contains('"') || s.contains('\n') {
        let mut o = String::from("\"");
        o.push_str(&s.replace('"', "\"\""));
        o.push('"');
        o
    } else {
        s.to_string()
    }
}

#[cfg(test)]
mod closed_attr_tests {
    use super::*;
    use dwgparse::Geometry;

    fn lw(flag: u16, closed_geom: bool) -> Geometry {
        // A tiny 3-vertex ring; when `closed_geom` is true the last vertex
        // coincides with the first so it reads as geometrically closed.
        let pts = if closed_geom {
            vec![(0.0, 0.0), (10.0, 0.0), (0.0, 0.0)]
        } else {
            vec![(0.0, 0.0), (10.0, 0.0), (5.0, 5.0)]
        };
        let n = pts.len();
        Geometry::LwPolyline { flag, points: pts, bulges: vec![0.0; n] }
    }

    #[test]
    fn cass_code_pattern() {
        // Valid CASS feature codes: digits, optional "-<n>" suffix.
        assert!(is_cass_code("141121"));
        assert!(is_cass_code("144301-1"));
        assert!(is_cass_code("300000"));
        // Not feature codes: owner names, parcel numbers, empty.
        assert!(!is_cass_code("张三"));
        assert!(!is_cass_code("101120JC00025"));
        assert!(!is_cass_code(""));
        assert!(!is_cass_code("AB12"));
    }

    fn cat_of(g: &Geometry, close: &[String], st: ClosedState) -> Cat {
        geom_classify(g, "SOME_LAYER", close, st).unwrap().0
    }

    #[test]
    fn yes_attr_is_always_polygon() {
        // Even an open geometry with no flag must become a polygon when the
        // attribute explicitly says Yes.
        assert_eq!(cat_of(&lw(0, false), &[], ClosedState::Yes), Cat::Polygon);
    }

    #[test]
    fn no_attr_is_always_line() {
        // Explicit "No" must override BOTH the CASS closed flag (0x200) AND a
        // geometrically closed vertex loop. This is the bug that used to export
        // "Closed=否/No" entities as polygons.
        let g_closed_flag = lw(CLOSED_FLAG_BITS, false);
        assert_eq!(cat_of(&g_closed_flag, &[], ClosedState::No), Cat::Line);
        let g_geom_closed = lw(0, true);
        assert_eq!(cat_of(&g_geom_closed, &[], ClosedState::No), Cat::Line);
        let g_both = lw(CLOSED_FLAG_BITS, true);
        assert_eq!(cat_of(&g_both, &[], ClosedState::No), Cat::Line);
    }

    #[test]
    fn absent_attr_falls_back_to_heuristics() {
        // No Closed attribute: the DWG closed flag still wins.
        assert_eq!(cat_of(&lw(CLOSED_FLAG_BITS, false), &[], ClosedState::Absent), Cat::Polygon);
        // No Closed attribute: geometric closure still wins.
        assert_eq!(cat_of(&lw(0, true), &[], ClosedState::Absent), Cat::Polygon);
        // No Closed attribute, open, no forced layer -> line.
        assert_eq!(cat_of(&lw(0, false), &[], ClosedState::Absent), Cat::Line);
        // No Closed attribute, open, but layer in --close-layers -> polygon.
        assert_eq!(cat_of(&lw(0, false), &["SOME".into()], ClosedState::Absent), Cat::Polygon);
    }

    #[test]
    fn caxx200_marker_is_the_reliable_face_signal_on_mixed_layers() {
        // 张南岳: JMD mixes 面状 building outlines (flag 0x308, bit 0x200 set)
        // and 线状围墙 (flag 0x100, no 0x200). With the default of NO forced
        // layers, the 0x200 entities must still become polygons while the
        // 0x100 walls stay lines.
        let building = lw(CLOSED_FLAG_BITS, false); // open but carries 0x200
        assert_eq!(cat_of(&building, &[], ClosedState::Absent), Cat::Polygon);
        let wall = lw(0x100, false); // open, no 0x200, no closed attr
        assert_eq!(cat_of(&wall, &[], ClosedState::Absent), Cat::Line);
    }

    #[test]
    fn no_default_layer_force_keeps_linear_walls_as_lines() {
        // By default force-closing is OFF, so an open JMD polyline with no
        // 0x200 flag and no Closed attribute is a LINE (not a polygon), even
        // though it lives on a layer that used to be blanket-closed.
        let wall_on_jmd = lw(0x108, false);
        assert_eq!(
            geom_classify(&wall_on_jmd, "JMD", &[], ClosedState::Absent).unwrap().0,
            Cat::Line
        );
        // The same layer can still be explicitly force-closed when the user
        // opts in via --close-layers.
        assert_eq!(
            geom_classify(&wall_on_jmd, "JMD", &["JMD".into()], ClosedState::Absent).unwrap().0,
            Cat::Polygon
        );
    }

    #[test]
    fn closed_state_detection() {
        // Affirmative values -> Yes.
        assert_eq!(closed_state(&[("Closed".into(), "Yes".into())]), ClosedState::Yes);
        assert_eq!(closed_state(&[("闭合".into(), "是".into())]), ClosedState::Yes);
        assert_eq!(closed_state(&[("CLOSED".into(), "1".into())]), ClosedState::Yes);
        // Any present-but-not-affirmative value -> No (否 / No / 0 / unknown).
        assert_eq!(closed_state(&[("Closed".into(), "No".into())]), ClosedState::No);
        assert_eq!(closed_state(&[("闭合".into(), "否".into())]), ClosedState::No);
        assert_eq!(closed_state(&[("closed".into(), "0".into())]), ClosedState::No);
        assert_eq!(closed_state(&[("Closed".into(), "maybe".into())]), ClosedState::No);
        // No matching key -> Absent.
        assert_eq!(closed_state(&[("BSM".into(), "123".into())]), ClosedState::Absent);
        assert_eq!(closed_state(&[]), ClosedState::Absent);
    }
}
