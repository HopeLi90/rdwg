// Raw dump of a single object in the R2007 object stream, by handle.
//
// `cargo run --example dbg_obj -- <file.dwg> <handle-hex>`
//
// Prints the object's raw bytes plus the header fields (`type`, `bitsize`,
// handle, EED) and where each of the three bit streams starts, which is what
// you need to hand-decode a field that is coming out wrong.
use dwgparse::bitreader::BitReader;
use dwgparse::parser::{decode_eed_debug, parse_handles_debug};
use dwgparse::r2007;

fn main() {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "tests/data/example_2007.dwg".to_string());
    let target = u64::from_str_radix(
        &std::env::args().nth(2).unwrap_or_else(|| "10".to_string()),
        16,
    )
    .expect("hex handle");

    let file = std::fs::read(&path).expect("read");
    let secs = r2007::load_sections(&file).expect("load sections");
    let objs = &secs.objects;
    let offsets =
        parse_handles_debug(&secs.handles, 0, secs.handles.len(), objs.len()).expect("handles");

    for &off in &offsets {
        let mut head = BitReader::at(objs, off, 0);
        let size = head.read_ms() as usize;
        let start = head.byte;
        if size == 0 || start + size > objs.len() {
            continue;
        }
        let obj_data = &objs[start..start + size];
        let mut rd = BitReader::at(obj_data, 0, 0);
        let type_code = rd.read_bs();
        let bitsize = rd.read_rl() as usize;
        let h = rd.read_h();
        if h.value != target {
            continue;
        }
        println!("handle 0x{:X}  type=0x{:X}  size={}", h.value, type_code, size);
        for row in obj_data.chunks(16) {
            print!("  ");
            for b in row {
                print!("{:02X} ", b);
            }
            println!();
        }
        let eed = decode_eed_debug(&mut rd, dwgparse::Dialect::R2007);
        println!("bitsize        = {}", bitsize);
        println!("EED items      = {}", eed.len());
        println!(
            "field stream   = bit {} (byte {} bit {})",
            rd.byte * 8 + rd.bit as usize,
            rd.byte,
            rd.bit
        );
        let str_rd = dwgparse::parser::setup_string_stream_debug(obj_data, bitsize);
        match str_rd {
            Some(s) => println!(
                "string stream  = bit {} (byte {} bit {})",
                s.byte * 8 + s.bit as usize,
                s.byte,
                s.bit
            ),
            None => println!("string stream  = none"),
        }
        println!("handle stream  = bit {}", bitsize);
        return;
    }
    eprintln!("handle 0x{:X} not found", target);
}
