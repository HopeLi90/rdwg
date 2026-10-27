//! API 演示：覆盖技术设计方案里给出的全部调用范式。
//!
//! 运行：
//! ```text
//! cargo run --example api_demo -- tests/data/sample_2000.dwg
//! ```
//!
//! 这个示例会被 `cargo build --examples` 编译，因此文档中的所有 API 用法
//! 都是经过类型检查的，不会出现"文档里能跑、实际编译不过"的情况。

use dwgparse::{DwgFile, Geometry, XDataValue};

fn main() -> Result<(), String> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "tests/data/sample_2000.dwg".to_string());

    // ---- 1. 打开 ----
    let dwg = DwgFile::parse(&path)?;
    println!("version={} objects={}", dwg.version, dwg.object_count);

    // ---- 2. 图层列表 ----
    println!("\n[图层]");
    for l in &dwg.layers {
        println!(
            "  0x{:X} {:<12} color={:<3} frozen={} off={} locked={} lw={}",
            l.handle, l.name, l.color_index, l.frozen, l.off, l.locked, l.lineweight
        );
    }

    // ---- 3. 按图层取图元几何 ----
    // 图层只存句柄，图元只存 layer_handle，两者通过 handle 关联。
    println!("\n[指定图层上的图元]");
    // "Tavolo 1" only exists in sample_2000.dwg; fall back to the first layer
    // so the demo is useful on any file.
    let want = if dwg.layer_by_name("Tavolo 1").is_some() {
        "Tavolo 1"
    } else {
        match dwg.layers.first() {
            Some(l) => l.name.as_str(),
            None => "",
        }
    };
    if let Some(layer) = dwg.layer_by_name(want) {
        for o in dwg.entities_on_layer(layer) {
            print!("  [{}] handle=0x{:X} ", o.type_name, o.handle);
            match &o.geometry {
                Some(Geometry::Circle { center, radius }) => println!(
                    "CIRCLE c=({:.4},{:.4}) r={:.4}",
                    center.0, center.1, radius
                ),
                Some(Geometry::Line { start, end }) => println!(
                    "LINE ({:.4},{:.4}) -> ({:.4},{:.4})",
                    start.0, start.1, end.0, end.1
                ),
                Some(Geometry::LwPolyline { points, .. }) => {
                    println!("LWPOLYLINE {} 顶点", points.len())
                }
                Some(Geometry::Text { ins_pt, value, .. }) => {
                    println!("TEXT @({:.4},{:.4}) \"{}\"", ins_pt.0, ins_pt.1, value)
                }
                Some(Geometry::MText { ins_pt, height, value, .. }) => {
                    println!(
                        "MTEXT @({:.4},{:.4}) h={:.4} \"{}\"",
                        ins_pt.0, ins_pt.1, height, value
                    )
                }
                Some(Geometry::Insert { ins_pt, block_name, scale }) => println!(
                    "INSERT @({:.4},{:.4}) blk={:?} scale=({:.2},{:.2},{:.2})",
                    ins_pt.0, ins_pt.1, block_name, scale.0, scale.1, scale.2
                ),
                Some(Geometry::Arc { center, radius, .. }) => {
                    println!("ARC c=({:.4},{:.4}) r={:.4}", center.0, center.1, radius)
                }
                Some(Geometry::Point { x, y, z }) => println!("POINT ({x:.4},{y:.4},{z:.4})"),
                Some(Geometry::Polyline { vertices, is_3d, .. }) => println!(
                    "POLYLINE({}) {} 顶点",
                    if *is_3d { "3D" } else { "2D/PFACE" },
                    vertices.len()
                ),
                Some(Geometry::Ellipse { center, major_axis, axis_ratio, .. }) => println!(
                    "ELLIPSE c=({:.4},{:.4}) m=({:.4},{:.4}) r={:.4}",
                    center.0, center.1, major_axis.0, major_axis.1, axis_ratio
                ),
                Some(Geometry::Face3d { corners, .. }) => println!(
                    "3DFACE 四角 ({:.4},{:.4})..({:.4},{:.4})",
                    corners[0].0, corners[0].1, corners[3].0, corners[3].1
                ),
                Some(Geometry::Solid { corners, elevation }) => println!(
                    "SOLID z={:.4} 四角 ({:.4},{:.4})..({:.4},{:.4})",
                    elevation, corners[0].0, corners[0].1, corners[3].0, corners[3].1
                ),
                None => println!("(无几何)"),
            }
        }
    }

    // ---- 4. 扩展属性 ----
    println!("\n[扩展属性]");
    println!("  带扩展属性的对象数 = {}", dwg.extended_attribute_count());

    // 4a. EED（实体级扩展数据，紧跟对象自身句柄之后）
    for o in &dwg.objects {
        for eed in &o.eed {
            println!(
                "  EED [{}] handle=0x{:X} appid=0x{:X}",
                o.type_name, o.handle, eed.appid_handle
            );
            for it in &eed.items {
                if let Some(t) = &it.text {
                    println!("      code {} -> \"{}\"", it.code, t);
                } else if let Some(p) = &it.point {
                    println!("      code {} -> ({:.3},{:.3},{:.3})", it.code, p.0, p.1, p.2);
                } else if let Some(v) = it.real {
                    println!("      code {} -> {}", it.code, v);
                } else if let Some(v) = it.int32 {
                    println!("      code {} -> {}", it.code, v);
                } else if let Some(v) = it.int16 {
                    println!("      code {} -> {}", it.code, v);
                }
            }
        }

        // 4b. XData（XRECORD 中的 resbuf 链）
        for x in &o.xdata {
            match &x.value {
                XDataValue::Str(s) => println!("  XData [{}] 组 {} = \"{}\"", o.type_name, x.code, s),
                XDataValue::Int16(v) => println!("  XData [{}] 组 {} = {}", o.type_name, x.code, v),
                XDataValue::Real(v) => println!("  XData [{}] 组 {} = {}", o.type_name, x.code, v),
                XDataValue::Handle(h) => {
                    println!("  XData [{}] 组 {} = handle 0x{:X}", o.type_name, x.code, h)
                }
                XDataValue::Point3(p) => {
                    println!("  XData [{}] 组 {} = ({},{},{})", o.type_name, x.code, p.0, p.1, p.2)
                }
                other => println!("  XData [{}] 组 {} = {:?}", o.type_name, x.code, other),
            }
        }
    }

    // ---- 5. 块 / INSERT 名字解析 ----
    // INSERT 二进制里只存 block_header 句柄，名字要二次解析。
    println!("\n[块]");
    for (handle, name) in dwg.objects.iter().filter_map(|o| o.block_name.as_ref().map(|n| (o.handle, n))) {
        println!("  BLOCK_HEADER 0x{handle:X} = {name}");
    }
    for o in dwg.entities().filter(|o| o.type_name == "INSERT") {
        if let (Some(bh), Some(Geometry::Insert { block_name, .. })) = (o.block_handle, &o.geometry) {
            println!("  INSERT 0x{:X} -> 块 {:?} (句柄 0x{:X})", o.handle, block_name, bh);
        }
    }

    // ---- 6. 按句柄查对象 ----
    if let Some(o) = dwg.object_by_handle(0x10) {
        println!("\n按句柄 0x10 查到: {} / {:?}", o.type_name, o.layer.as_ref().map(|l| &l.name));
    }

    Ok(())
}
