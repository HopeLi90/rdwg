// Debug dump of the R2007 `LAYER` bit streams.
//
// Run with no argument to print a summary table of every LAYER in the file
// (name / flag0 / raw CMC rgb / resolved index / handles), or with a handle in
// hex to get the full bit-by-bit walk of that one object.
//
// The bit order here mirrors `decode_layer` in `src/parser.rs`:
//   field stream : num_reactors(BL) is_xdic_missing(B)        <- common_object_handle_data
//                  name(T, string stream) is_xref_resolved(BS)
//                  flag0(BS) color(CMC)
//   handle stream: owner(4) reactors(4)*N xdict(3)
//                  xref(5) plotstyle(5) material(5) ltype(5)
use dwgparse::bitreader::BitReader;
use dwgparse::parser::decode_eed_debug;
use dwgparse::r2007;

/// Everything `decode_layer` needs, decoded by hand from one object's raw bits.
struct Walk {
    handle: u64,
    name: String,
    bitsize: usize,
    num_reactors: u32,
    is_xdic_missing: u8,
    is_xref_resolved: u16,
    flag0: u16,
    rgb: u32,
    cflag: u8,
    field_end: usize,
    owner: u64,
    xdict: u64,
    xref: u64,
    plotstyle: u64,
    material: u64,
    ltype: u64,
    hdl_end: usize,
    obj_bits: usize,
}

fn walk(obj_data: &[u8]) -> Walk {
    let mut rd = BitReader::at(obj_data, 0, 0);
    let _type_code = rd.read_bs();
    let bitsize = rd.read_rl() as usize;
    let h = rd.read_h();
    let handle = h.value;
    let _eed = decode_eed_debug(&mut rd, dwgparse::Dialect::R2007);

    // Field stream.
    let num_reactors = rd.read_bl();
    let is_xdic_missing = rd.read_b() as u8;
    let mut str_rd = dwgparse::parser::setup_string_stream_debug(obj_data, bitsize);
    let name = match &mut str_rd {
        Some(s) => s.read_tu(),
        None => String::new(),
    };
    let is_xref_resolved = rd.read_bs();
    let flag0 = rd.read_bs();
    let _legacy_index = rd.read_bs();
    let rgb = rd.read_bl();
    let cflag = rd.read_rc();
    let field_end = rd.byte * 8 + rd.bit as usize;

    // Handle stream.
    let mut hr = BitReader::at(obj_data, bitsize / 8, (bitsize % 8) as u32);
    let owner = hr.read_h().value;
    for _ in 0..num_reactors.min(1_000_000) {
        hr.read_h();
    }
    // `is_xdic_missing == 1` means there is no XDICTIONARY handle on the wire.
    let xdict = if is_xdic_missing == 0 {
        hr.read_h().value
    } else {
        0
    };
    let xref = hr.read_h().value;
    let plotstyle = hr.read_h().value;
    let material = hr.read_h().value;
    let ltype = hr.read_h().value;
    let hdl_end = hr.byte * 8 + hr.bit as usize;

    Walk {
        handle,
        name,
        bitsize,
        num_reactors,
        is_xdic_missing,
        is_xref_resolved,
        flag0,
        rgb,
        cflag,
        field_end,
        owner,
        xdict,
        xref,
        plotstyle,
        material,
        ltype,
        hdl_end,
        obj_bits: obj_data.len() * 8,
    }
}

fn main() {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "tests/data/example_2007.dwg".to_string());
    let target = std::env::args().nth(2).map(|s| u64::from_str_radix(&s, 16).expect("hex handle"));

    let file = std::fs::read(&path).expect("read");
    let secs = r2007::load_sections(&file).expect("load sections");
    let objs = &secs.objects;

    // Walk every object (same offset logic as the real parser), keeping the
    // ones whose raw type is LAYER (0x33).
    let offsets = dwgparse::parser::parse_handles_debug(
        &secs.handles,
        0,
        secs.handles.len(),
        objs.len(),
    )
    .expect("handles");
    let mut layers: Vec<Walk> = Vec::new();
    for &off in &offsets {
        let mut head = BitReader::at(objs, off, 0);
        let size = head.read_ms() as usize;
        let start = head.byte;
        if size == 0 || start + size > objs.len() {
            continue;
        }
        let obj_data = &objs[start..start + size];
        let mut probe = BitReader::at(obj_data, 0, 0);
        if probe.read_bs() == 0x33 {
            layers.push(walk(obj_data));
        }
    }

    match target {
        Some(h) => {
            let w = layers.iter().find(|w| w.handle == h).expect("handle not a LAYER");
            println!("handle 0x{:X}  name {:?}", w.handle, w.name);
            println!("  bitsize               = {}", w.bitsize);
            println!("  num_reactors(BL)      = {}", w.num_reactors);
            println!("  is_xdic_missing(B)    = {}", w.is_xdic_missing);
            println!("  is_xref_resolved(BS)  = {}", w.is_xref_resolved);
            println!(
                "  flag0(BS)             = {}  (frozen={} off={} locked={} plot={} linewt={})",
                w.flag0,
                w.flag0 & 1 != 0,
                w.flag0 & 2 != 0,
                w.flag0 & 8 != 0,
                w.flag0 & 16 != 0,
                (w.flag0 & 0x3E0) >> 5
            );
            println!("  CMC rgb               = 0x{:08X}  (method 0x{:02X}, flag {})",
                     w.rgb, w.rgb >> 24, w.cflag);
            println!("  CMC index             = {}", dwgparse::rgb_to_palette_index(w.rgb));
            println!("  field stream ends     = bit {}", w.field_end);
            println!("  owner(4)              = 0x{:X}", w.owner);
            println!("  xdict(3)              = 0x{:X}", w.xdict);
            println!("  xref(5)               = 0x{:X}", w.xref);
            println!("  plotstyle(5)          = 0x{:X}", w.plotstyle);
            println!("  material(5)           = 0x{:X}", w.material);
            println!("  ltype(5)              = 0x{:X}", w.ltype);
            println!("  handle stream ends    = bit {} (obj is {} bits)", w.hdl_end, w.obj_bits);
        }
        None => {
            println!(
                "{:<24} {:<6} {:>6} {:>6} {:>10} {:>6} {:>4} {:>6} {:>7} {:>7} {:>5}",
                "name", "hnd", "bitsz", "flag0", "rgb", "idx", "nxd", "owner", "xdict", "ltype",
                "fld"
            );
            for w in &layers {
                println!(
                    "{:<24} {:>#6x} {:>6} {:>6} {:>#10x} {:>6} {:>4} {:>#6x} {:>#7x} {:>#7x} {:>5}",
                    w.name,
                    w.handle,
                    w.bitsize,
                    w.flag0,
                    w.rgb,
                    dwgparse::cmc_color_index(w.rgb),
                    w.is_xdic_missing,
                    w.owner,
                    w.xdict,
                    w.ltype,
                    w.field_end,
                );
            }
        }
    }
}
