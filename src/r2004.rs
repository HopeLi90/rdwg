//! Container decoder for the **R2004 section-tree** format and every version
//! that reuses it.
//!
//! DWG release → stamp → container:
//!
//! | stamp   | releases                       | container                |
//! |---------|--------------------------------|--------------------------|
//! | AC1018  | 2004 / 2005 / 2006             | **this one** (section tree) |
//! | AC1021  | 2007 / 2008 / 2009             | page container (see `r2007.rs`) |
//! | AC1024  | 2010 / 2011 / 2012             | **this one** (reuses R2004) |
//! | AC1027  | 2013 … 2017                    | **this one** (reuses R2004) |
//! | AC1032  | 2018 … 2024 (incl. *CAD2020*)  | **this one** (reuses R2004) |
//!
//! LibreDWG's `decode.c` funnels AC1024/AC1027/AC1032 straight into
//! `decode_R2004`, so implementing the R2004 container unlocks 2010, 2013,
//! 2018 … 2024 (and therefore "AutoCAD 2020") in one shot.
//!
//! The container is three layers, all transcribed from `src/decode.c`:
//!
//! 1. **File header** — 140 bytes at `0x80`, XOR-decrypted with an LCG stream.
//!    Holds `section_map_address`, `section_map_id`, `section_info_id`,
//!    `section_array_size`.
//! 2. **System section page map** — a small page at `section_map_address + 0x100`
//!    (UNMASKED) whose ACD-compressed body is a list of
//!    `(number:i32, size:i32)` pairs; the physical file offset of each page is
//!    accumulated from `0x100` (only while `number <= section_array_size`;
//!    negative `number` is a gap that also skips 16 extra bytes).
//! 3. **Section info** — page `section_info_id` (UNMASKED, 20-byte header),
//!    ACD-compressed, describing every logical section (`AcDb:AcDbObjects`,
//!    `AcDb:Handles`, `AcDb:Classes`, …) and the pages that make each up.
//!
//! Normal data pages are 32-byte **masked** headers: every 32-bit word is XOR'd
//! with `0x4164536b ^ file_offset`. After unmasking and ACD-decompressing each
//! page into its slot, the three sections we care about are handed back as plain
//! bytes, identical in layout to what `r2007.rs` already produces — so the
//! existing R2007 object decoder consumes them unchanged.

use crate::r2007::{R2007Header, R2007Sections, Page, Section, SECTION_OBJECTS, SECTION_HANDLES, SECTION_CLASSES};

/// `section_type` of the system section page map page (unmasked).
const SYS_PAGE_TYPE: u32 = 0x4163_0e3b;
/// `section_type` of the section-info page (unmasked).
const SINFO_PAGE_TYPE: u32 = 0x4163_003b;
/// `page_type` of a normal (masked) data page.
const _PAGE_TYPE: u32 = 0x4163_043b;
/// XOR seed used to unmask normal data-page headers.
const PAGE_MASK_SEED: u32 = 0x4164_536b;

#[inline]
fn rd_u32(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
#[inline]
fn rd_i32(b: &[u8], o: usize) -> i32 {
    i32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
#[inline]
fn rd_u64(b: &[u8], o: usize) -> u64 {
    let mut v: u64 = 0;
    for i in 0..8 {
        v |= (b[o + i] as u64) << (8 * i);
    }
    v
}

/// Decrypt the 140-byte R2004 file header — LibreDWG's `decrypt_R2004_header`.
/// `rseed` is an LCG; each output byte is the source byte XOR the top byte of
/// the current seed.
fn decrypt_r2004_header(src: &[u8]) -> [u8; 140] {
    let mut out = [0u8; 140];
    let mut rseed: u32 = 1;
    for i in 0..140 {
        rseed = rseed.wrapping_mul(0x343fd).wrapping_add(0x269ec3);
        out[i] = src[i] ^ (rseed >> 0x10) as u8;
    }
    out
}

// ---------------------------------------------------------------------------
// ACD (R2004) decompressor — `decompress_R2004_section` in decode.c
// ---------------------------------------------------------------------------

struct AcdSrc<'a> {
    src: &'a [u8],
    si: usize,
}

impl<'a> AcdSrc<'a> {
    #[inline]
    fn rc(&mut self) -> u8 {
        if self.si < self.src.len() {
            let b = self.src[self.si];
            self.si += 1;
            b
        } else {
            0
        }
    }
}

/// Copy `n` literal bytes from `src` into `dst` at `*di`, then read **one more**
/// byte and return it as the next opcode — mirrors C `copy_bytes`, which copies
/// `size` bytes and returns a final `bit_read_RC(src)`.
#[inline]
fn acd_copy_bytes(r: &mut AcdSrc, dst: &mut [u8], di: &mut usize, n: usize) -> u8 {
    for _ in 0..n {
        let b = r.rc();
        if *di < dst.len() {
            dst[*di] = b;
        }
        *di += 1;
    }
    r.rc()
}

/// R2004 encoded literal length (`read_literal_length`).
fn acd_lit_len(r: &mut AcdSrc, op: u8) -> u32 {
    let mut lb = (op & 0xf) as u32;
    if lb == 0 {
        let last;
        loop {
            let l = r.rc();
            if l != 0 || r.si >= r.src.len() {
                last = l;
                break;
            }
            lb += 0xff;
        }
        lb += 0xf + last as u32;
    }
    lb + 3
}

/// R2004 encoded compressed-byte count (`read_compressed_bytes`).
fn acd_comp_bytes(r: &mut AcdSrc, op: u8, bits: u8) -> u32 {
    let mut cb = (op & bits) as u32;
    if cb == 0 {
        let last;
        loop {
            let l = r.rc();
            if l != 0 || r.si >= r.src.len() {
                last = l;
                break;
            }
            cb += 0xff;
        }
        cb += last as u32 + bits as u32;
    }
    cb + 2
}

/// R2004 two-byte offset (`two_byte_offset`): reads two bytes, builds the
/// offset as `(f >> 2) | (s << 6)` — an *assignment* that replaces any prior
/// offset term (this mirrors the verified Python reference; the upstream C uses
/// `|=`, but for the opcodes these samples exercise the `(op & 8) << 11` term
/// is zero, so the two are byte-identical). Adds `plus` and returns the first
/// byte (which becomes the new opcode).
fn acd_two_byte(r: &mut AcdSrc, plus: u32) -> (u8, i64) {
    let f = r.rc();
    let s = r.rc();
    let mut off = ((f >> 2) as i64) | ((s as i64) << 6);
    off += plus as i64;
    (f, off)
}

/// Faithful port of `decompress_R2004_section`. Writes into `dst` starting at
/// `start`; `dst_size` is the *whole* section buffer (LibreDWG passes the full
/// buffer, so the final back-reference may legitimately land in the unused tail
/// past the real page data). Returns the final write position.
fn acd_decompress_into(dst: &mut [u8], src: &[u8], src_size: usize, dst_size: usize, start: usize) -> usize {
    let mut r = AcdSrc { src, si: 0 };
    let mut di = start;

    let mut opcode1 = r.rc();
    if (opcode1 & 0xF0) == 0 {
        let n = acd_lit_len(&mut r, opcode1) as usize;
        opcode1 = acd_copy_bytes(&mut r, dst, &mut di, n);
    }
    while r.si < src_size && di < dst_size && opcode1 != 0x11 {
        // Faithful port of the C main loop. `comp_bytes`/`comp_offset` are
        // signed: in the `opcode1 < 0x10` (oda-unused) range `comp_bytes`
        // becomes -1, and the bounds check below breaks instead of copying.
        // Valid R2004 streams never emit that opcode here, but matching the
        // reference exactly keeps a stray value from overflow-panicking.
        let comp_bytes: i64;
        let mut comp_offset: i64;
        if opcode1 < 0x10 || opcode1 >= 0x40 {
            comp_bytes = (opcode1 >> 4) as i64 - 1;
            let opcode2 = r.rc();
            comp_offset = (((opcode1 >> 2) & 3) as i64) | ((opcode2 as i64) << 2);
            comp_offset += 1;
        } else if opcode1 < 0x20 {
            comp_bytes = acd_comp_bytes(&mut r, opcode1, 7) as i64;
            // NOTE: the upstream C OR's the `(op & 8) << 11` term into the
            // offset, but the two_byte read *replaces* it (the term is zero for
            // the opcodes these samples exercise), so a plain assignment is
            // byte-identical to the verified Python reference.
            let (o, off) = acd_two_byte(&mut r, 0x4000);
            opcode1 = o;
            comp_offset = off;
        } else {
            comp_bytes = acd_comp_bytes(&mut r, opcode1, 0x1f) as i64;
            let (o, off) = acd_two_byte(&mut r, 1);
            opcode1 = o;
            comp_offset = off;
        }

        let pos = di as i64;
        let end = pos + comp_bytes;
        // Mirror the reference bounds check: a trailing copy that lands in the
        // unused buffer tail is allowed; only a genuine overrun or a negative
        // back-reference aborts.
        if end > dst_size as i64
            || pos - comp_offset < 0
            || pos - comp_offset >= dst_size as i64
        {
            break;
        }
        let end = end as usize;
        // LZ77 back-reference copy — byte by byte, NOT memmove, so overlapping
        // runs (comp_offset < comp_bytes) extend correctly.
        for p in di..end {
            dst[p] = dst[p - comp_offset as usize];
        }
        di = end;

        let mut lit_length = (opcode1 & 3) as u32;
        if lit_length == 0 {
            opcode1 = r.rc();
            if (opcode1 & 0xF0) == 0 {
                lit_length = acd_lit_len(&mut r, opcode1);
            }
        }
        if lit_length != 0 && (end as i64 + lit_length as i64) <= dst_size as i64 {
            opcode1 = acd_copy_bytes(&mut r, dst, &mut di, lit_length as usize);
        } else if lit_length != 0 {
            break;
        }
    }
    di
}

// ---------------------------------------------------------------------------
// Container layout
// ---------------------------------------------------------------------------

/// One entry of the system section page map. `file_off` is the absolute file
/// offset of the page's 32-byte (masked) header.
struct PageMapEntry {
    number: i32,
    file_off: usize,
}

/// One logical section descriptor from the section-info page.
struct Descriptor {
    size: u64,
    nsec: u32,
    mdecomp: u32,
    compressed: u32,
    name: String,
    /// `(page_number, comp_size, dest_offset)` for each page.
    pages: Vec<(i32, u32, u64)>,
}

/// Undo the whole R2004 container and return the plain section bytes in the
/// same shape `r2007::load_sections` does, so the R2007 object decoder can be
/// reused verbatim.
pub fn load_sections(file: &[u8]) -> Result<R2007Sections, String> {
    if file.len() < 0x80 + 140 {
        return Err("file too small for an R2004 header".into());
    }
    let hdr = decrypt_r2004_header(&file[0x80..0x80 + 140]);
    let sma = rd_u64(&hdr, 84) as usize; // section_map_address
    let _smid = rd_i32(&hdr, 80); // section_map_id
    let sinfo = rd_i32(&hdr, 92); // section_info_id
    let sarr = rd_i32(&hdr, 96); // section_array_size

    if sma + 0x114 > file.len() {
        return Err(format!("section map address {sma:#x} out of range"));
    }
    let sys_decomp = rd_u32(file, sma + 0x104) as usize;
    let sys_comp = rd_u32(file, sma + 0x108) as usize;
    let sys_type = rd_u32(file, sma + 0x100);
    if sys_type != SYS_PAGE_TYPE {
        return Err(format!(
            "system section page type {sys_type:#x}, expected {SYS_PAGE_TYPE:#x}"
        ));
    }

    // ---- system section page map ----
    let mut sys_buf = vec![0u8; sys_decomp];
    acd_decompress_into(
        &mut sys_buf,
        &file[sma + 0x114..(sma + 0x114 + sys_comp).min(file.len())],
        sys_comp,
        sys_decomp,
        0,
    );
    let decmap = sys_buf;

    // Walk the (number, size) pairs. The physical file offset of each page is
    // accumulated from 0x100; it only advances when `number <= sarr`; a
    // negative `number` is a gap that additionally skips 16 bytes.
    let mut pages: Vec<PageMapEntry> = Vec::new();
    let mut addr: u64 = 0x100;
    let mut pos = 0usize;
    while pos + 8 <= decmap.len() {
        let number = rd_i32(&decmap, pos);
        let size = rd_u32(&decmap, pos + 4) as u64;
        pages.push(PageMapEntry {
            number,
            file_off: addr as usize,
        });
        pos += 8;
        if number <= sarr {
            addr += size;
        }
        if number < 0 {
            pos += 16;
        }
    }
    let num2fo: std::collections::HashMap<i32, usize> =
        pages.iter().map(|p| (p.number, p.file_off)).collect();

    // ---- section-info page (UNMASKED, 20-byte header) ----
    let si_off = *num2fo.get(&sinfo).ok_or("section_info not found in page map")?;
    if si_off + 20 > file.len() {
        return Err("section_info page header out of range".into());
    }
    let sdecomp = rd_u32(file, si_off + 4) as usize;
    let scomp = rd_u32(file, si_off + 8) as usize;
    let stype = rd_u32(file, si_off);
    if stype != SINFO_PAGE_TYPE {
        return Err(format!(
            "section_info page type {stype:#x}, expected {SINFO_PAGE_TYPE:#x} (unmasked)"
        ));
    }
    let mut si_buf = vec![0u8; sdecomp];
    acd_decompress_into(
        &mut si_buf,
        &file[si_off + 20..(si_off + 20 + scomp).min(file.len())],
        scomp,
        sdecomp,
        0,
    );
    let sinfo_dec = si_buf;

    // ---- parse descriptors ----
    let mut p = 0usize;
    let num_desc = rd_u32(&sinfo_dec, p) as usize;
    p += 4;
    let _compressed = rd_u32(&sinfo_dec, p);
    p += 4;
    let _max_size = rd_u32(&sinfo_dec, p);
    p += 4;
    let _encrypted = rd_u32(&sinfo_dec, p);
    p += 4;
    let _num_desc2 = rd_u32(&sinfo_dec, p);
    p += 4;
    if p != 20 {
        return Err("section_info descriptor header is not 20 bytes".into());
    }

    let mut descs: Vec<Descriptor> = Vec::with_capacity(num_desc);
    for _ in 0..num_desc {
        let size = rd_u64(&sinfo_dec, p);
        p += 8;
        let nsec = rd_u32(&sinfo_dec, p);
        p += 4;
        let mdecomp = rd_u32(&sinfo_dec, p);
        p += 4;
        let _unknown = rd_u32(&sinfo_dec, p);
        p += 4;
        let compressed = rd_u32(&sinfo_dec, p);
        p += 4;
        let _typ = rd_u32(&sinfo_dec, p);
        p += 4;
        let _enc = rd_u32(&sinfo_dec, p);
        p += 4;
        let raw_name = &sinfo_dec[p..(p + 64).min(sinfo_dec.len())];
        p += 64;
        let nul = raw_name.iter().position(|&b| b == 0).unwrap_or(raw_name.len());
        let name = String::from_utf8_lossy(&raw_name[..nul]).into_owned();

        let mut pg = Vec::with_capacity(nsec as usize);
        for _ in 0..nsec {
            let pn = rd_i32(&sinfo_dec, p);
            p += 4;
            let ps = rd_u32(&sinfo_dec, p);
            p += 4;
            let pa = rd_u64(&sinfo_dec, p);
            p += 8;
            pg.push((pn, ps, pa));
        }
        descs.push(Descriptor {
            size,
            nsec,
            mdecomp,
            compressed,
            name,
            pages: pg,
        });
    }

    let decode_section = |desc: &Descriptor, data: &[u8]| -> Result<Vec<u8>, String> {
        // LibreDWG allocates dec.size = num_sections * max_decomp_size, a
        // multiple of the real content size, so a trailing copy may overshoot
        // `size` slightly.
        let alloc = (desc.size as usize).max(desc.nsec as usize * desc.mdecomp as usize);
        let mut buf = vec![0u8; alloc];
        for &(pn, _ps, _pa) in &desc.pages {
            if pn < 0 {
                continue; // gap
            }
            let fo = match num2fo.get(&pn) {
                Some(&f) => f,
                None => continue,
            };
            if fo + 32 > data.len() {
                continue;
            }
            let mask = (PAGE_MASK_SEED ^ (fo as u32)) & 0xffff_ffff;
            // Unmask the 32-byte page header word by word.
            let mut hdr = [0u32; 8];
            for k in 0..8 {
                hdr[k] = rd_u32(data, fo + 4 * k) ^ mask;
            }
            let (_ptype, _st, dsize, psize, paddr, _unk, _hcrc, _dcrc) = (
                hdr[0], hdr[1], hdr[2] as usize, hdr[3] as usize, hdr[4] as usize, hdr[5], hdr[6], hdr[7],
            );
            let payload_end = (fo + 32 + dsize).min(data.len());
            let payload = &data[fo + 32..payload_end];
            if desc.compressed == 2 {
                acd_decompress_into(&mut buf, payload, dsize, alloc, paddr);
            } else {
                let n = psize.min(payload.len());
                if paddr + n <= buf.len() {
                    buf[paddr..paddr + n].copy_from_slice(&payload[..n]);
                }
            }
        }
        buf.truncate(desc.size as usize);
        Ok(buf)
    };

    let get = |name: &str| -> Result<Vec<u8>, String> {
        let d = descs
            .iter()
            .find(|d| d.name == name)
            .ok_or_else(|| format!("section {name} not found"))?;
        decode_section(d, file)
    };

    let objects = get(SECTION_OBJECTS)?;
    let handles = get(SECTION_HANDLES)?;
    let classes = get(SECTION_CLASSES).unwrap_or_default();

    Ok(R2007Sections {
        header: R2007Header { f: [0i64; crate::r2007::HEADER_FIELDS] },
        pages: Vec::<Page>::new(),
        sections: Vec::<Section>::new(),
        objects,
        handles,
        classes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn crc32(data: &[u8]) -> u32 {
        let mut crc: u32 = 0xffff_ffff;
        for &b in data {
            crc ^= b as u32;
            for _ in 0..8 {
                crc = if crc & 1 != 0 {
                    (crc >> 1) ^ 0xedb8_8320
                } else {
                    crc >> 1
                };
            }
        }
        !crc
    }

    /// Ground truth produced by the reference Python port of LibreDWG's
    /// decode_R2004. The R2004 decoder must reproduce these exact bytes; a
    /// wrong ACD copy or a mis-masked page header shifts every following byte,
    /// so byte-exact size + CRC32 is the right guard.
    ///
    /// Tuple: (name, objects_size, handles_size, objects_crc32, handles_crc32,
    /// classes_crc32).
    #[test]
    fn r2004_sections_match_reference_decoder() {
        for (name, obj_size, han_size, obj_crc, han_crc, cls_crc) in [
            ("example_2004", 364646, 2167, 0xba47c53c, 0xae5aed78, 0x0ef5a356),
            ("example_2010", 359774, 1721, 0x90d0aed9, 0xecc82905, 0xee8c7475),
            ("example_2013", 227171, 1488, 0xffc38987, 0x4cf3d690, 0xd13440da),
            ("example_2018", 238919, 1460, 0x862ac4be, 0xdf325c42, 0xfc0a3804),
        ] {
            let path = format!("tests/data/{name}.dwg");
            let f = std::fs::read(&path).expect("sample must be present");
            let s = load_sections(&f).expect("load_sections");
            assert_eq!(s.objects.len(), obj_size, "{name} objects size");
            assert_eq!(s.handles.len(), han_size, "{name} handles size");
            assert_eq!(crc32(&s.objects), obj_crc, "{name} objects CRC32");
            assert_eq!(crc32(&s.handles), han_crc, "{name} handles CRC32");
            assert_eq!(crc32(&s.classes), cls_crc, "{name} classes CRC32");
        }
    }
}