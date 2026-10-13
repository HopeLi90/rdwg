//! CLI front-end for `dwgparse`.
//!
//! Usage:
//! ```text
//! dwgparse [path/to/file.dwg]
//! ```
//! With no argument it defaults to `sample_2000.dwg` next to the binary's
//! working directory.
//!
//! It prints:
//!   * the file version and total object count,
//!   * the parsed layer table (name / color / flags),
//!   * every entity's geometry coordinates,
//!   * a summary of the extended attributes (EED on entities, XData in
//!     XRECORD objects) — the primary requirement.

use dwgparse::{DwgFile, Geometry};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| "sample_2000.dwg".to_string());

    let dwg = match DwgFile::parse(&path) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
    };

    println!("==== DWG parse summary for {} ====", path);
    println!("version        : {}", dwg.version);
    println!("object count   : {}", dwg.object_count);
    println!("layers         : {}", dwg.layers.len());
    println!(
        "ext. attr obs  : {} (EED/XData present)",
        dwg.extended_attribute_count()
    );

    // --- Layers -----------------------------------------------------------
    println!("\n--- Layers (table records) ---");
    if dwg.layers.is_empty() {
        println!("(none)");
    }
    for l in &dwg.layers {
        let flags = format!(
            "{}{}{}{}{}",
            if l.frozen { "F" } else { "-" },
            if l.off { "O" } else { "-" },
            if l.frozen_in_new { "N" } else { "-" },
            if l.locked { "L" } else { "-" },
            if l.plot_flag { "P" } else { "-" },
        );
        println!(
            "  handle=0x{:X} name={:<14} color={:<4} lw={:<3} flags[{}] ltype=0x{:X}",
            l.handle, l.name, l.color_index, l.lineweight, flags,
            l.ltype_handle.unwrap_or(0)
        );
    }

    // --- Blocks -----------------------------------------------------------
    // An INSERT entity stores only its block_header *handle*, so the names come
    // from the BLOCK_HEADER (0x31) table records.
    println!("\n--- Blocks (BLOCK_HEADER records) ---");
    let blocks: Vec<&dwgparse::ParsedObject> = dwg
        .objects
        .iter()
        .filter(|o| o.block_name.is_some())
        .collect();
    if blocks.is_empty() {
        println!("(none)");
    }
    for b in &blocks {
        println!(
            "  handle=0x{:X} name={}",
            b.handle,
            b.block_name.as_deref().unwrap_or("?")
        );
    }

    // --- Entity geometry --------------------------------------------------
    println!("\n--- Entity geometry ---");
    let mut n = 0;
    for obj in dwg.entities() {
        n += 1;
        let layer = obj
            .layer_handle
            .map(|h| dwg.layer_name(h).unwrap_or("?"))
            .unwrap_or("?");
        print!("  [{:<10}] #{:<4} layer={:<14}", obj.type_name, obj.index, layer);
        match &obj.geometry {
            Some(Geometry::Line { start, end }) => {
                println!(
                    "LINE ({:.3},{:.3},{:.3}) -> ({:.3},{:.3},{:.3})",
                    start.0, start.1, start.2, end.0, end.1, end.2
                );
            }
            Some(Geometry::Circle { center, radius }) => {
                println!(
                    "CIRCLE c=({:.3},{:.3},{:.3}) r={:.3}",
                    center.0, center.1, center.2, radius
                );
            }
            Some(Geometry::Arc {
                center,
                radius,
                start_angle,
                end_angle,
            }) => {
                println!(
                    "ARC c=({:.3},{:.3},{:.3}) r={:.3} a=[{:.3},{:.3}]",
                    center.0, center.1, center.2, radius, start_angle, end_angle
                );
            }
            Some(Geometry::Point { x, y, z }) => {
                println!("POINT ({:.3},{:.3},{:.3})", x, y, z);
            }
            Some(Geometry::LwPolyline { flag, points, bulges: _ }) => {
                println!(
                    "LWPOLYLINE flag=0x{:X} verts={} first={:?}",
                    flag,
                    points.len(),
                    points.first()
                );
            }
            Some(Geometry::Text {
                ins_pt,
                height,
                value,
            }) => {
                println!(
                    "TEXT @({:.3},{:.3}) h={:.3} \"{}\"",
                    ins_pt.0, ins_pt.1, height, value
                );
            }
            Some(Geometry::MText {
                ins_pt,
                height,
                value,
            }) => {
                println!(
                    "MTEXT @({:.3},{:.3},{:.3}) h={:.3} \"{}\"",
                    ins_pt.0, ins_pt.1, ins_pt.2, height, value
                );
            }
            Some(Geometry::Insert {
                ins_pt,
                block_name,
                scale,
            }) => {
                println!(
                    "INSERT @({:.3},{:.3},{:.3}) scale=({:.3},{:.3},{:.3}) blk={}",
                    ins_pt.0, ins_pt.1, ins_pt.2, scale.0, scale.1, scale.2, block_name
                );
            }
            Some(Geometry::Polyline {
                flag,
                is_3d,
                vertices,
                ..
            }) => {
                let kind = if *is_3d { "3D" } else { "2D" };
                let first = vertices.first().map(|p| format!("({:.3},{:.3},{:.3})", p.0, p.1, p.2));
                println!(
                    "POLYLINE({}) flag=0x{:X} verts={} first={}",
                    kind,
                    flag,
                    vertices.len(),
                    first.unwrap_or_default()
                );
            }
            Some(Geometry::Ellipse {
                center,
                major_axis,
                axis_ratio,
                ..
            }) => {
                println!(
                    "ELLIPSE c=({:.3},{:.3},{:.3}) major=({:.3},{:.3},{:.3}) ratio={:.3}",
                    center.0, center.1, center.2, major_axis.0, major_axis.1, major_axis.2, axis_ratio
                );
            }
            Some(Geometry::Face3d { corners, .. }) => {
                println!(
                    "3DFACE ({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3})",
                    corners[0].0, corners[0].1, corners[0].2, corners[3].0, corners[3].1, corners[3].2
                );
            }
            Some(Geometry::Solid { corners, elevation }) => {
                println!(
                    "SOLID z={:.3} corners={} pts",
                    elevation, corners.len()
                );
            }
            None => println!("(no geometry)"),
        }
    }
    if n == 0 {
        println!("(none)");
    }

    // --- Extended attributes (EED + XData) --------------------------------
    println!("\n--- Extended attributes (EED on entities / XData in XRECORD) ---");
    let mut any = false;
    for obj in &dwg.objects {
        if obj.eed.is_empty() && obj.xdata.is_empty() {
            continue;
        }
        any = true;
        println!(
            "  [{}] #{:<4} handle=0x{:X} type={}",
            obj.type_name, obj.index, obj.handle, obj.type_code
        );
        for eed in &obj.eed {
            println!("    EED appid=0x{:X} items:", eed.appid_handle);
            for it in &eed.items {
                let desc = match (&it.text, &it.point, it.real, it.int16, it.int32, it.int64) {
                    (Some(t), _, _, _, _, _) => format!("text=\"{}\"", t),
                    (_, Some(p), _, _, _, _) => format!("point=({:.3},{:.3},{:.3})", p.0, p.1, p.2),
                    (_, _, Some(r), _, _, _) => format!("real={:.6}", r),
                    (_, _, _, Some(v), _, _) => format!("int16={}", v),
                    (_, _, _, _, Some(v), _) => format!("int32={}", v),
                    (_, _, _, _, _, Some(v)) => format!("int64={}", v),
                    _ => "(blob)".to_string(),
                };
                println!("      code {} -> {}", it.code, desc);
            }
        }
        for xd in &obj.xdata {
            println!("    XData group {} = {:?}", xd.code, xd.value);
        }
    }
    if !any {
        println!("(none found in this file)");
    }
}
