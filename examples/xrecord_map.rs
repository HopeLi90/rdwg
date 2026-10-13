//! `xrecord_map` — analyse the association between graphical entities and
//! their extension-dictionary XRECORD chains.
//!
//! Prints, for every entity with an `xdict_handle`, the XRECORDs reachable
//! from that dictionary (by owner link or reactor link), in chain order.
//!
//! `cargo run --release --example xrecord_map -- <file.dwg>`

use dwgparse::DwgFile;
use std::env;

fn main() {
    let path = env::args().nth(1).unwrap_or_else(|| "tests/data/地块解析_cad2018.dwg".into());
    let dwg = DwgFile::parse(&path).expect("failed to parse DWG");

    // Index all XRECORDs by handle.
    let mut xrecs: Vec<&dwgparse::ParsedObject> = Vec::new();
    for o in &dwg.objects {
        if o.type_code == 0x4f {
            xrecs.push(o);
        }
    }
    let find = |h: u64| dwg.objects.iter().find(|o| o.handle == h);

    println!("=== XRECORD inventory ===");
    for x in &xrecs {
        println!(
            "  0x{:X} owner={:?} xdict={:?} reactors={:?} n_xdata={}",
            x.handle,
            x.owner_handle.map(|v| format!("0x{v:X}")),
            x.xdict_handle.map(|v| format!("0x{v:X}")),
            x.reactors.iter().map(|v| format!("0x{v:X}")).collect::<Vec<_>>(),
            x.xdata.len()
        );
    }

    println!("\n=== entities with xdict_handle ===");
    for o in &dwg.objects {
        if let Some(xd) = o.xdict_handle {
            println!(
                "  entity 0x{:X} ({}) xdict=0x{:X} owner={:?} reactors={:?}",
                o.handle, o.type_name, xd,
                o.owner_handle.map(|v| format!("0x{v:X}")),
                o.reactors.iter().map(|v| format!("0x{v:X}")).collect::<Vec<_>>()
            );
            // what object is at the xdict handle?
            match find(xd) {
                Some(t) => println!(
                    "    xdict object: type={} ({:#X}) owner={:?} reactors={:?} n_xdata={}",
                    t.type_name, t.type_code,
                    t.owner_handle.map(|v| format!("0x{v:X}")),
                    t.reactors.iter().map(|v| format!("0x{v:X}")).collect::<Vec<_>>(),
                    t.xdata.len()
                ),
                None => println!("    xdict object: NOT PARSED (handle 0x{xd:X})"),
            }
        }
    }

    // XRECORDs whose owner is NOT an entity xdict — group them by owner.
    use std::collections::HashMap;
    let mut by_owner: HashMap<u64, Vec<u64>> = HashMap::new();
    for x in &xrecs {
        if let Some(own) = x.owner_handle {
            by_owner.entry(own).or_default().push(x.handle);
        }
    }
    println!("\n=== XRECORD chains grouped by owner ===");
    let mut owners: Vec<_> = by_owner.keys().copied().collect();
    owners.sort();
    for own in owners {
        let chain = &by_owner[&own];
        let otype = find(own).map(|o| o.type_name.clone()).unwrap_or("UNPARSED".into());
        println!(
            "  owner 0x{:X} ({}): {} xrecords -> {:?}",
            own, otype, chain.len(), chain
        );
    }
}
