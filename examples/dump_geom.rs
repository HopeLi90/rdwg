//! Dump every entity's geometry at **full precision** for cross-checking
//! against a ground-truth DXF export.
//!
//! Output format (one line per geometric entity):
//!
//! ```text
//! [LINE] #45 layer=Tavolo 3 LINE (370.859,730.630,0.000) -> (450.040,619.219,0.000)
//! ```
//!
//! Run:
//! ```text
//! cargo run --example dump_geom -- /tmp/example_2007.dwg > /tmp/geom_ours.txt
//! ```

use dwgparse::{DwgFile, Geometry};

fn main() -> Result<(), String> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/tmp/example_2007.dwg".to_string());

    let dwg = DwgFile::parse(&path)?;
    println!("--- Entity geometry ---");
    for o in &dwg.objects {
        let layer = o
            .layer_handle
            .and_then(|h| dwg.layer_name(h))
            .unwrap_or("?");
        let tag = o.type_name.as_str();
        // Pad the type name to 10 chars so columns line up like the old dump.
        let pad = format!("{:<10}", tag);
        match &o.geometry {
            Some(Geometry::Line { start, end }) => println!(
                "  [{pad}] #{:>4} layer={:<12} {tag} ({:.10},{:.10},{:.10}) -> ({:.10},{:.10},{:.10})",
                o.index, layer, start.0, start.1, start.2, end.0, end.1, end.2
            ),
            Some(Geometry::Circle { center, radius }) => println!(
                "  [{pad}] #{:>4} layer={:<12} CIRCLE c=({:.10},{:.10},{:.10}) r={:.10}",
                o.index, layer, center.0, center.1, center.2, radius
            ),
            Some(Geometry::Arc { center, radius, start_angle, end_angle }) => println!(
                "  [{pad}] #{:>4} layer={:<12} ARC c=({:.10},{:.10},{:.10}) r={:.10} a=[{:.10},{:.10}]",
                o.index, layer, center.0, center.1, center.2, radius, start_angle, end_angle
            ),
            Some(Geometry::Point { x, y, z }) => println!(
                "  [{pad}] #{:>4} layer={:<12} POINT ({:.10},{:.10},{:.10})",
                o.index, layer, x, y, z
            ),
            Some(Geometry::Text { ins_pt, height, value }) => println!(
                "  [{pad}] #{:>4} layer={:<12} TEXT @({:.10},{:.10}) h={:.10} \"{}\"",
                o.index, layer, ins_pt.0, ins_pt.1, height, value
            ),
            Some(Geometry::MText { ins_pt, height, value }) => println!(
                "  [{pad}] #{:>4} layer={:<12} MTEXT @({:.10},{:.10},{:.10}) h={:.10} \"{}\"",
                o.index, layer, ins_pt.0, ins_pt.1, ins_pt.2, height, value
            ),
            Some(Geometry::Insert { ins_pt, block_name, scale }) => println!(
                "  [{pad}] #{:>4} layer={:<12} INSERT @({:.10},{:.10},{:.10}) scale=({:.10},{:.10},{:.10}) blk={}",
                o.index, layer, ins_pt.0, ins_pt.1, ins_pt.2, scale.0, scale.1, scale.2, block_name
            ),
            Some(Geometry::LwPolyline { flag, points, bulges: _ }) => {
                // Dump *every* vertex, not just the first: the cross-check
                // script compares point-for-point, and a partial dump would
                // silently hide an error in the DD delta decoding from the
                // second vertex onwards.
                let mut vs = String::new();
                for p in points {
                    vs.push_str(&format!("({:.10},{:.10})", p.0, p.1));
                }
                println!(
                    "  [{pad}] #{:>4} layer={:<12} LWPOLYLINE flag=0x{:X} verts={} v=[{}]",
                    o.index, layer, flag, points.len(), vs
                )
            }
            Some(Geometry::Polyline { flag, is_3d, vertices, .. }) => {
                let kind = if *is_3d { "3D" } else { "2D" };
                let mut vs = String::new();
                for v in vertices {
                    vs.push_str(&format!("({:.10},{:.10},{:.10})", v.0, v.1, v.2));
                }
                println!(
                    "  [{pad}] #{:>4} layer={:<12} POLYLINE({}) flag=0x{:X} verts={} v=[{}]",
                    o.index, layer, kind, flag, vertices.len(), vs
                )
            }
            Some(Geometry::Ellipse { center, major_axis, axis_ratio, start_angle, end_angle }) => {
                println!(
                    "  [{pad}] #{:>4} layer={:<12} ELLIPSE c=({:.10},{:.10},{:.10}) m=({:.10},{:.10},{:.10}) r={:.10} a=[{:.10},{:.10}]",
                    o.index, layer, center.0, center.1, center.2,
                    major_axis.0, major_axis.1, major_axis.2,
                    axis_ratio, start_angle, end_angle
                )
            }
            Some(Geometry::Face3d { corners, invisibility }) => {
                println!(
                    "  [{pad}] #{:>4} layer={:<12} 3DFACE inv=0x{:X} [({:.10},{:.10},{:.10});({:.10},{:.10},{:.10});({:.10},{:.10},{:.10});({:.10},{:.10},{:.10})]",
                    o.index, layer, invisibility,
                    corners[0].0, corners[0].1, corners[0].2,
                    corners[1].0, corners[1].1, corners[1].2,
                    corners[2].0, corners[2].1, corners[2].2,
                    corners[3].0, corners[3].1, corners[3].2
                )
            }
            Some(Geometry::Solid { corners, elevation }) => {
                println!(
                    "  [{pad}] #{:>4} layer={:<12} SOLID z={:.10} [({:.10},{:.10},{:.10});({:.10},{:.10},{:.10});({:.10},{:.10},{:.10});({:.10},{:.10},{:.10})]",
                    o.index, layer, elevation,
                    corners[0].0, corners[0].1, corners[0].2,
                    corners[1].0, corners[1].1, corners[1].2,
                    corners[2].0, corners[2].1, corners[2].2,
                    corners[3].0, corners[3].1, corners[3].2
                )
            }
            None => {}
        }
    }
    Ok(())
}
