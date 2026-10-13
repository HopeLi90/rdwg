//! Dump the R2004+ page container: file header fields, pages map, sections map
//! and the per-page sizes of one section.
//!
//! ```text
//! cargo run --example dbg_container -- tests/data/example_2007.dwg
//! ```
//!
//! This is the tool to reach for when a container-level parse fails: it shows
//! exactly which layer (header / pages map / sections map / data page) broke
//! and with which numbers.

use dwgparse::r2007::{
    load_sections, read_file_header, read_pages_map, read_sections_map, FIRST_PAGE_OFF,
};

fn main() {
    let path = std::env::args().nth(1).expect("usage: dbg_container <file.dwg>");
    let file = std::fs::read(&path).expect("read file");
    println!("file            : {path} ({} bytes)", file.len());
    println!(
        "version stamp   : {}",
        String::from_utf8_lossy(&file[0..6])
    );

    let h = match read_file_header(&file) {
        Ok(h) => h,
        Err(e) => {
            println!("file header     : FAILED: {e}");
            return;
        }
    };
    println!("--- file header (34 x int64) ---");
    let named: &[(usize, &str)] = &[
        (3, "pages_map_correction"),
        (7, "pages_map_offset"),
        (8, "pages_map_id"),
        (10, "pages_map_size_comp"),
        (11, "pages_map_size_uncomp"),
        (12, "pages_amount"),
        (20, "num_sections"),
        (22, "sections_map_size_comp"),
        (24, "sections_map_id"),
        (25, "sections_map_size_uncomp"),
        (27, "sections_map_correction"),
    ];
    for (i, name) in named {
        println!("  f[{i:<2}] {name:<24} = {}", h.f[*i]);
    }
    println!("  (first 6 raw: {:?})", &h.f[0..6]);

    // Pages map.
    let pm_off = FIRST_PAGE_OFF + h.pages_map_offset() as usize;
    println!("--- pages map @0x{pm_off:x} ---");
    let pages = match read_pages_map(
        &file,
        pm_off,
        h.pages_map_size_comp(),
        h.pages_map_size_uncomp(),
        h.pages_map_correction(),
    ) {
        Ok(p) => p,
        Err(e) => {
            println!("  FAILED: {e}");
            return;
        }
    };
    println!("  {:<5} {:<10} {:<10}", "id", "size", "offset");
    for p in &pages {
        println!("  {:<5} {:<10} 0x{:x}", p.id, p.size, p.offset);
    }

    // Sections map.
    let sm_page = match pages.iter().find(|p| p.id == h.sections_map_id()) {
        Some(p) => p,
        None => {
            println!("sections map page (id {}) not found", h.sections_map_id());
            return;
        }
    };
    println!("--- sections map (page id {} @0x{:x}) ---", h.sections_map_id(), sm_page.offset);
    let sections = match read_sections_map(
        &file,
        sm_page.offset as usize,
        h.sections_map_size_comp(),
        h.sections_map_size_uncomp(),
        h.sections_map_correction(),
    ) {
        Ok(s) => s,
        Err(e) => {
            println!("  FAILED: {e}");
            return;
        }
    };
    for s in &sections {
        println!(
            "  {:<20} data_size={:<9} pages={}",
            s.name, s.data_size, s.num_pages
        );
        for sp in &s.pages {
            println!(
                "      page id={:<5} off=0x{:<9x} uncomp={:<9} comp={}",
                sp.id, sp.offset, sp.uncomp_size, sp.comp_size
            );
        }
    }

    match load_sections(&file) {
        Ok(secs) => println!(
            "--- load_sections OK: objects={} handles={} classes={}",
            secs.objects.len(),
            secs.handles.len(),
            secs.classes.len()
        ),
        Err(e) => println!("--- load_sections FAILED: {e}"),
    }
}
