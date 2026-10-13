//! Container decoder for **R2007 / AC1021** (AutoCAD 2007–2009, i.e. "CAD2008").
//!
//! From R2004 on, DWG stopped being a flat sequence of sections: the file
//! becomes a *container* of pages, with
//!
//! * a **file header** at `0x80` that is Reed–Solomon (255,239) interleaved and
//!   LZ77 compressed,
//! * a **pages map** (which physical page lives where in the file),
//! * a **sections map** (which pages belong to which logical section:
//!   `AcDb:AcDbObjects`, `AcDb:Handles`, `AcDb:Classes`, …),
//! * and per-page payloads that are again RS-interleaved and possibly LZ77
//!   compressed.
//!
//! This module undoes that container and hands back the *plain* section bytes.
//! After that, object decoding is (almost) the same as for R2000 — see the
//! version notes in `parser.rs`.
//!
//! Transcribed from LibreDWG `src/decode_r2007.c` and `src/reedsolomon.c`.
//! The Reed–Solomon *error correction* itself is **not** needed: LibreDWG only
//! de-interleaves the blocks (`decode_rs`) and ignores the 16 parity bytes. We
//! do the same, so no GF(256) arithmetic is required.

/// Number of `int64` fields in the R2007 file header (0x110 bytes).
pub const HEADER_FIELDS: usize = 34;
/// File offset of the (RS encoded, compressed) file header.
pub const FILE_HEADER_OFF: usize = 0x80;
/// Bytes of the file header block at `FILE_HEADER_OFF`.
pub const FILE_HEADER_SIZE: usize = 0x3d8;
/// First data page starts right after the file header block plus its trailer.
pub const FIRST_PAGE_OFF: usize = FILE_HEADER_OFF + FILE_HEADER_SIZE + 0x28; // 0x480

/// Section name of the object stream (note the doubled `AcDb` — that is really
/// what AutoCAD writes).
pub const SECTION_OBJECTS: &str = "AcDb:AcDbObjects";
pub const SECTION_HANDLES: &str = "AcDb:Handles";
pub const SECTION_CLASSES: &str = "AcDb:Classes";

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

fn rll(b: &[u8], off: usize) -> i64 {
    let mut v: u64 = 0;
    for i in (0..8).rev() {
        v = (v << 8) | (b.get(off + i).copied().unwrap_or(0) as u64);
    }
    v as i64
}

// ---------------------------------------------------------------------------
// Reed-Solomon de-interleaving
// ---------------------------------------------------------------------------

/// Undo the (255,239) interleaving: `block_count` blocks of `data_size` payload
/// bytes are stored round-robin (`A B C A B C …`) and are reassembled into
/// `block_count * data_size` contiguous bytes (`A A A … B B B …`).
///
/// The 16 parity bytes per block are simply skipped — LibreDWG does the same
/// (`decode_rs`), so a *detected* error is not corrected here either.
pub fn decode_rs(src: &[u8], block_count: usize, data_size: usize) -> Option<Vec<u8>> {
    if block_count == 0 || data_size == 0 || block_count * data_size > src.len() {
        return None;
    }
    let mut dst = vec![0u8; block_count * data_size];
    let mut k = 0usize;
    for i in 0..block_count {
        let mut pos = i;
        for _ in 0..data_size {
            dst[k] = src[pos];
            k += 1;
            pos += block_count;
        }
    }
    Some(dst)
}

/// Size (in bytes) of the RS coded form of `size_comp` payload bytes, as stored
/// in a data page. Used to detect "uncompressed but still RS coded" pages.
fn page_size_if_rs_coded(size_comp: i64) -> i64 {
    if size_comp <= 0 {
        return 0;
    }
    let pesize = (size_comp + 7) & !7;
    let block_count = (pesize + 0xFB - 1) / 0xFB;
    (block_count * 0xFF + 31) & !31
}

// ---------------------------------------------------------------------------
// LZ77 (AC1021 variant)
// ---------------------------------------------------------------------------

/// Copy `n` bytes from `src[off..]` to `dst[di..]`, honouring the odd
/// byte-order rules of the format (2/3 byte groups are reversed, a 16 byte
/// group is written high-half first).
#[inline]
fn put(dst: &mut [u8], di: usize, src: &[u8], off: usize, n: usize) {
    match n {
        1 => dst[di] = src[off],
        2 => {
            dst[di] = src[off + 1];
            dst[di + 1] = src[off];
        }
        3 => {
            dst[di] = src[off + 2];
            dst[di + 1] = src[off + 1];
            dst[di + 2] = src[off];
        }
        4 | 8 => dst[di..di + n].copy_from_slice(&src[off..off + n]),
        16 => {
            dst[di..di + 8].copy_from_slice(&src[off + 8..off + 16]);
            dst[di + 8..di + 16].copy_from_slice(&src[off..off + 8]);
        }
        _ => {}
    }
}

/// Literal chunk copy for lengths 0..31 (spec 5.0, page 30).
const TAIL: [&[(u32, u32)]; 32] = [
    &[],                                                    // 0
    &[(0, 1)],                                              // 1
    &[(0, 2)],                                              // 2
    &[(0, 3)],                                              // 3
    &[(0, 4)],                                              // 4
    &[(4, 1), (0, 4)],                                      // 5
    &[(5, 1), (1, 4), (0, 1)],                              // 6
    &[(5, 2), (1, 4), (0, 1)],                              // 7
    &[(0, 8)],                                              // 8
    &[(8, 1), (0, 8)],                                      // 9
    &[(9, 1), (1, 8), (0, 1)],                              // 10
    &[(9, 2), (1, 8), (0, 1)],                              // 11
    &[(8, 4), (0, 8)],                                      // 12
    &[(12, 1), (8, 4), (0, 8)],                             // 13
    &[(13, 1), (9, 4), (1, 8), (0, 1)],                     // 14
    &[(13, 2), (9, 4), (1, 8), (0, 1)],                     // 15
    &[(0, 16)],                                             // 16
    &[(9, 8), (8, 1), (0, 8)],                              // 17
    &[(17, 1), (1, 16), (0, 1)],                            // 18
    &[(16, 3), (0, 16)],                                    // 19
    &[(16, 4), (0, 16)],                                    // 20
    &[(20, 1), (16, 4), (0, 16)],                           // 21
    &[(20, 2), (16, 4), (0, 16)],                           // 22
    &[(20, 3), (16, 4), (0, 16)],                           // 23
    &[(16, 8), (0, 16)],                                    // 24
    &[(17, 8), (16, 1), (0, 16)],                           // 25
    &[(25, 1), (17, 8), (16, 1), (0, 16)],                  // 26
    &[(25, 2), (17, 8), (16, 1), (0, 16)],                  // 27
    &[(24, 4), (16, 8), (0, 16)],                           // 28
    &[(28, 1), (24, 4), (16, 8), (0, 16)],                  // 29
    &[(28, 2), (24, 4), (16, 8), (0, 16)],                  // 30
    // PITFALL: 31 does *not* follow the 28/29/30 pattern — the C fallback is
    // `copy_1(30); copy_4(26); copy_8(18); copy_16(2); copy_2(0)`, i.e. every
    // offset is 2 lower than for length 30 and a trailing 2-byte swap appears.
    // Getting this row wrong corrupts any literal run of exactly 31 bytes and
    // silently desynchronises the whole LZ77 stream from that point on.
    &[(30, 1), (26, 4), (18, 8), (2, 16), (0, 2)],           // 31
];

fn copy_compressed(dst: &mut [u8], di: usize, src: &[u8], si: usize, length: usize) {
    let mut di = di;
    let mut si = si;
    let mut len = length;
    while len >= 32 {
        put(dst, di, src, si + 16, 16);
        put(dst, di + 16, src, si, 16);
        si += 32;
        di += 32;
        len -= 32;
    }
    if len < 32 {
        for &(off, n) in TAIL[len] {
            put(dst, di, src, si + off as usize, n as usize);
            di += n as usize;
        }
    }
}

/// Overlapping (LZ77 back-reference) copy: `length` bytes from `offset` bytes
/// back in the *output*.
fn copy_back(dst: &mut [u8], di: usize, length: usize, offset: usize) {
    for i in 0..length {
        dst[di + i] = dst[di + i - offset];
    }
}

/// Read a literal length, extending it when the escape value `0x17` is hit.
fn read_literal_length(
    src: &[u8],
    si: &mut usize,
    src_end: usize,
    opcode: u8,
) -> Option<u32> {
    let mut length = opcode as u32 + 8;
    if length == 0x17 {
        if *si >= src_end {
            return None;
        }
        let mut n = src[*si] as u32;
        *si += 1;
        length += n;
        if n == 0xff {
            loop {
                if src_end - *si < 2 {
                    return None;
                }
                n = src[*si] as u32 | ((src[*si + 1] as u32) << 8);
                *si += 2;
                length += n;
                if n != 0xFFFF {
                    break;
                }
            }
        }
    }
    Some(length)
}

/// Decode one match instruction; returns `(offset, length)`.
fn read_instructions(
    src: &[u8],
    si: &mut usize,
    src_end: usize,
    opcode: &mut u8,
) -> Option<(u32, u32)> {
    let (offset, length);
    match *opcode >> 4 {
        0 => {
            length = (*opcode as u32 & 0xf) + 0x13;
            if src_end - *si < 2 {
                return None;
            }
            offset = src[*si] as u32;
            *si += 1;
            *opcode = src[*si];
            *si += 1;
            let length = ((*opcode as u32 >> 3) & 0x10) + length;
            let offset = ((*opcode as u32 & 0x78) << 5) + 1 + offset;
            return Some((offset, length));
        }
        1 => {
            length = (*opcode as u32 & 0xf) + 3;
            if src_end - *si < 2 {
                return None;
            }
            offset = src[*si] as u32;
            *si += 1;
            *opcode = src[*si];
            *si += 1;
            return Some((((*opcode as u32 & 0xf8) << 5) + 1 + offset, length));
        }
        2 => {
            if src_end - *si < 2 {
                return None;
            }
            offset = src[*si] as u32;
            *si += 1;
            let mut offset = ((src[*si] as u32) << 8 & 0xff00) | offset;
            *si += 1;
            let mut length = *opcode as u32 & 7;
            if (*opcode & 8) == 0 {
                if src_end - *si < 1 {
                    return None;
                }
                *opcode = src[*si];
                *si += 1;
                length = (*opcode as u32 & 0xf8) + length;
            } else {
                offset += 1;
                if src_end - *si < 1 {
                    return None;
                }
                length = ((src[*si] as u32) << 3) + length;
                *si += 1;
                if src_end - *si < 1 {
                    return None;
                }
                *opcode = src[*si];
                *si += 1;
                length = ((*opcode as u32 & 0xf8) << 8) + length + 0x100;
            }
            return Some((offset, length));
        }
        _ => {
            length = *opcode as u32 >> 4;
            offset = *opcode as u32 & 15;
            if src_end - *si < 1 {
                return None;
            }
            *opcode = src[*si];
            *si += 1;
            return Some((((*opcode as u32 & 0xf8) << 1) + offset + 1, length));
        }
    }
}

/// LZ77 decompress the AC1021 dialect. `src[..src_size]` → `dst_size` bytes.
pub fn decompress_r2007(src: &[u8], src_size: usize, dst_size: usize) -> Result<Vec<u8>, String> {
    if dst_size == 0 || src_size < 2 {
        return Err("invalid decompression arguments".into());
    }
    let src_size = src_size.min(src.len());
    let mut dst = vec![0u8; dst_size];
    let src_end = src_size;
    if src_end > src.len() {
        return Err("source shorter than src_size".into());
    }
    let mut si: usize = 0;
    let mut di: usize = 0;
    let mut length: u32 = 0;
    let mut offset: u32;

    let mut opcode = src[si];
    si += 1;
    if (opcode & 0xf0) == 0x20 {
        // opcode, then 2 skipped bytes, then a length byte (C: src += 2; *src++)
        si += 2;
        if si >= src_end {
            return Err("truncated compressed stream".into());
        }
        length = (src[si] & 0x07) as u32;
        si += 1;
        if length == 0 {
            return Err("decompression error: zero length".into());
        }
    }

    while si < src_end {
        if length == 0 {
            length = read_literal_length(src, &mut si, src_end, opcode)
                .ok_or("truncated literal length")?;
        }
        let l = length as usize;
        if di + l > dst_size || si + l > src_end {
            return Err("decompression error: length overflow".into());
        }
        copy_compressed(&mut dst, di, src, si, l);
        di += l;
        si += l;
        if si >= src_end {
            break;
        }
        opcode = src[si];
        si += 1;
        let (o, l2) = read_instructions(src, &mut si, src_end, &mut opcode)
            .ok_or("truncated match instruction")?;
        offset = o;
        length = l2;
        loop {
            let l = length as usize;
            if di + l > dst_size {
                return Err("decompression error: dst overflow".into());
            }
            if offset as usize > di {
                return Err("decompression error: offset underflow".into());
            }
            if offset > 0 {
                copy_back(&mut dst, di, l, offset as usize);
            }
            di += l;
            length = (opcode as u32) & 7;
            if length != 0 || si >= src_end {
                break;
            }
            opcode = src[si];
            si += 1;
            if (opcode >> 4) == 0 {
                break;
            }
            if (opcode >> 4) == 0x0f {
                opcode &= 0xf;
            }
            let (o, l2) = read_instructions(src, &mut si, src_end, &mut opcode)
                .ok_or("truncated match instruction")?;
            offset = o;
            length = l2;
        }
    }
    Ok(dst)
}

// ---------------------------------------------------------------------------
// File header / pages map / sections map
// ---------------------------------------------------------------------------

/// The R2007 file header (34 × `int64`, little-endian).
#[derive(Debug, Clone)]
pub struct R2007Header {
    pub f: [i64; HEADER_FIELDS],
}

impl R2007Header {
    pub fn pages_map_offset(&self) -> i64 {
        self.f[7]
    }
    pub fn pages_map_id(&self) -> i64 {
        self.f[8]
    }
    pub fn pages_map_size_comp(&self) -> i64 {
        self.f[10]
    }
    pub fn pages_map_size_uncomp(&self) -> i64 {
        self.f[11]
    }
    pub fn pages_amount(&self) -> i64 {
        self.f[12]
    }
    pub fn pages_map_correction(&self) -> i64 {
        self.f[3]
    }
    pub fn num_sections(&self) -> i64 {
        self.f[20]
    }
    pub fn sections_map_size_comp(&self) -> i64 {
        self.f[22]
    }
    pub fn sections_map_id(&self) -> i64 {
        self.f[24]
    }
    pub fn sections_map_size_uncomp(&self) -> i64 {
        self.f[25]
    }
    pub fn sections_map_correction(&self) -> i64 {
        self.f[27]
    }
}

/// Read and decompress the file header block at `0x80`.
pub fn read_file_header(file: &[u8]) -> Result<R2007Header, String> {
    if file.len() < FILE_HEADER_OFF + FILE_HEADER_SIZE {
        return Err("file too small for an R2007 header".into());
    }
    let block = &file[FILE_HEADER_OFF..FILE_HEADER_OFF + FILE_HEADER_SIZE];
    let pedata = decode_rs(block, 3, 239).ok_or("RS decode of the file header failed")?;
    let compr_len = i32::from_le_bytes([
        pedata[24], pedata[25], pedata[26], pedata[27],
    ]);
    let pedata_size: usize = 3 * 239;
    let mut raw: Vec<u8> = if compr_len > 0 {
        let src_len = (compr_len as usize).min(pedata_size - 32);
        decompress_r2007(&pedata[32..], src_len, HEADER_FIELDS * 8)?
    } else {
        pedata[32..32 + HEADER_FIELDS * 8].to_vec()
    };
    let mut f = [0i64; HEADER_FIELDS];
    for i in 0..HEADER_FIELDS {
        f[i] = rll(&raw, i * 8);
    }
    raw.clear();
    Ok(R2007Header { f })
}

/// One entry of the pages map.
#[derive(Debug, Clone)]
pub struct Page {
    pub id: i64,
    pub size: u64,
    pub offset: u64,
}

/// One page of a section.
#[derive(Debug, Clone)]
pub struct SectionPage {
    pub offset: u64,
    pub size: u64,
    pub id: i64,
    pub uncomp_size: u64,
    pub comp_size: u64,
}

/// One logical section (`AcDb:AcDbObjects`, …).
#[derive(Debug, Clone)]
pub struct Section {
    pub name: String,
    pub data_size: u64,
    pub num_pages: usize,
    pub pages: Vec<SectionPage>,
}

/// Read a *system* page (RS interleaved + LZ77) at `off`.
fn read_system_page(
    file: &[u8],
    off: usize,
    size_comp: i64,
    size_uncomp: i64,
    repeat_count: i64,
) -> Result<Vec<u8>, String> {
    if repeat_count < 0 || size_comp <= 0 || size_uncomp <= 0 {
        return Err("invalid system page parameters".into());
    }
    if size_comp as u64 >= file.len() as u64 || size_uncomp as u64 >= file.len() as u64 {
        return Err("system page size out of range".into());
    }
    let pesize = ((size_comp + 7) & !7) * repeat_count;
    let block_count = (pesize + 238) / 239;
    if block_count <= 0 || block_count > 0x10000 {
        return Err("invalid RS block count".into());
    }
    let page_size = (block_count * 255 + 7) & !7;
    if off + page_size as usize > file.len() {
        return Err("system page runs past the end of file".into());
    }
    let rsdata = &file[off..off + page_size as usize];
    let pedata = decode_rs(rsdata, block_count as usize, 239).ok_or("RS decode failed")?;
    let pedata_size = block_count as usize * 239;
    if size_comp < size_uncomp {
        let src_len = (size_comp as usize).min(pedata_size);
        decompress_r2007(&pedata, src_len, size_uncomp as usize)
    } else {
        let n = (size_uncomp as usize).min(pedata_size);
        Ok(pedata[..n].to_vec())
    }
}

/// Read the pages map: id/size of every physical page plus where it starts in
/// the file (pages are packed back to back starting at `FIRST_PAGE_OFF`).
pub fn read_pages_map(
    file: &[u8],
    off: usize,
    size_comp: i64,
    size_uncomp: i64,
    correction: i64,
) -> Result<Vec<Page>, String> {
    let sdat = read_system_page(file, off, size_comp, size_uncomp, correction)?;
    let mut pages = Vec::new();
    let mut offset: u64 = FIRST_PAGE_OFF as u64;
    let mut pos = 0usize;
    while pos + 16 <= sdat.len() && pos < size_uncomp as usize {
        let size = rll(&sdat, pos) as u64;
        pos += 8;
        let id = rll(&sdat, pos);
        pos += 8;
        pages.push(Page { id, size, offset });
        offset += size;
    }
    Ok(pages)
}

/// Read the sections map and return every section with its page list.
pub fn read_sections_map(
    file: &[u8],
    off: usize,
    size_comp: i64,
    size_uncomp: i64,
    correction: i64,
) -> Result<Vec<Section>, String> {
    let page = read_system_page(file, off, size_comp, size_uncomp, correction)?;
    let mut sections: Vec<Section> = Vec::new();
    let mut pos = 0usize;
    while pos < page.len() && pos < size_uncomp as usize {
        if pos + 64 > page.len() {
            break;
        }
        let data_size = rll(&page, pos) as u64;
        pos += 8;
        let _max_size = rll(&page, pos);
        pos += 8;
        let _encrypted = rll(&page, pos);
        pos += 8;
        let _hashcode = rll(&page, pos);
        pos += 8;
        let name_length = rll(&page, pos) as usize;
        pos += 8;
        let _unknown = rll(&page, pos);
        pos += 8;
        let _encoded = rll(&page, pos);
        pos += 8;
        let num_pages = rll(&page, pos);
        pos += 8;

        // Section name: UCS-2LE, `name_length` bytes.
        let mut sz = name_length;
        if sz % 2 != 0 {
            sz += 1;
        }
        if sz > 48 || pos + sz > page.len() {
            break;
        }
        let mut units = Vec::new();
        let mut i = 0;
        while i + 1 < sz {
            units.push((page[pos + i] as u16) | ((page[pos + i + 1] as u16) << 8));
            i += 2;
        }
        // Section names are NUL terminated (UCS-2 NUL), like every other
        // fixed-size string in the format.
        let name = String::from_utf16_lossy(&units);
        let name = name.trim_end_matches('\0').to_string();
        pos += sz;

        let mut spages = Vec::new();
        if num_pages > 0 && num_pages <= 0xf0000 {
            for _ in 0..num_pages {
                if pos + 56 > page.len() {
                    break;
                }
                let o = rll(&page, pos) as u64;
                pos += 8;
                let s = rll(&page, pos) as u64;
                pos += 8;
                let id = rll(&page, pos);
                pos += 8;
                let uncomp_size = rll(&page, pos) as u64;
                pos += 8;
                let comp_size = rll(&page, pos) as u64;
                pos += 8;
                let _checksum = rll(&page, pos);
                pos += 8;
                let _crc = rll(&page, pos);
                pos += 8;
                spages.push(SectionPage {
                    offset: o,
                    size: s,
                    id,
                    uncomp_size,
                    comp_size,
                });
            }
        }
        sections.push(Section {
            name,
            data_size,
            num_pages: spages.len(),
            pages: spages,
        });
    }
    Ok(sections)
}

/// Read (and decompress) one data page into `decomp[dest_off..]`.
fn read_data_page(
    file: &[u8],
    page_off: usize,
    page_size: u64,
    comp_size: u64,
    uncomp_size: u64,
    decomp: &mut [u8],
    dest_off: usize,
) -> Result<(), String> {
    let pesize = (comp_size as i64 + 7) & !7;
    let block_count = ((pesize + 0xFB - 1) / 0xFB) as usize;
    if block_count == 0 {
        return Err("empty data page".into());
    }
    let pedata_size = block_count * 0xFB;
    if page_off + page_size as usize > file.len() {
        return Err("data page runs past the end of file".into());
    }
    let rsdata = &file[page_off..page_off + page_size as usize];
    let pedata = decode_rs(rsdata, block_count, 0xFB).ok_or("RS decode of data page failed")?;
    let end = (dest_off + uncomp_size as usize).min(decomp.len());
    if dest_off > end || end - dest_off != uncomp_size as usize {
        return Err("data page does not fit into the section buffer".into());
    }
    let out = &mut decomp[dest_off..end];
    if (comp_size as i64) < uncomp_size as i64 {
        let src_len = (comp_size as usize).min(pedata_size);
        let d = decompress_r2007(&pedata, src_len, uncomp_size as usize)?;
        out.copy_from_slice(&d);
    } else {
        let n = (uncomp_size as usize).min(pedata.len());
        out[..n].copy_from_slice(&pedata[..n]);
    }
    Ok(())
}

/// Assemble the plain bytes of one logical section.
pub fn read_data_section(
    file: &[u8],
    section: &Section,
    pages: &[Page],
) -> Result<Vec<u8>, String> {
    if section.data_size > 0x2f00_0000 {
        return Err("section too large".into());
    }
    let mut decomp = vec![0u8; section.data_size as usize];
    for sp in &section.pages {
        let page = pages
            .iter()
            .find(|p| p.id == sp.id)
            .ok_or_else(|| format!("page {} not found in the pages map", sp.id))?;
        if sp.offset as usize > decomp.len() || sp.uncomp_size > section.data_size {
            return Err("section page out of bounds".into());
        }
        let dest = sp.offset as usize;
        if dest + sp.uncomp_size as usize > decomp.len() {
            return Err("section page overruns the section buffer".into());
        }
        let needs_rs = sp.comp_size != sp.uncomp_size
            || page.size as i64 == page_size_if_rs_coded(sp.comp_size as i64);
        if needs_rs {
            if page.offset as usize >= file.len() {
                return Err("page offset out of range".into());
            }
            read_data_page(
                file,
                page.offset as usize,
                page.size,
                sp.comp_size,
                sp.uncomp_size,
                &mut decomp,
                dest,
            )?;
        } else {
            let src = page.offset as usize;
            let n = sp.uncomp_size as usize;
            if src + n > file.len() {
                return Err("uncompressed page runs past the end of file".into());
            }
            decomp[dest..dest + n].copy_from_slice(&file[src..src + n]);
        }
    }
    Ok(decomp)
}

/// The three sections the parser actually needs.
#[derive(Debug)]
pub struct R2007Sections {
    pub header: R2007Header,
    pub pages: Vec<Page>,
    pub sections: Vec<Section>,
    /// `AcDb:AcDbObjects` — the object stream.
    pub objects: Vec<u8>,
    /// `AcDb:Handles` — the object map (identical layout to R2000).
    pub handles: Vec<u8>,
    /// `AcDb:Classes` — class definitions for custom (type >= 500) objects.
    pub classes: Vec<u8>,
}

/// Undo the whole R2007 container and return the plain section bytes.
pub fn load_sections(file: &[u8]) -> Result<R2007Sections, String> {
    let header = read_file_header(file)?;
    let pm_off = FIRST_PAGE_OFF + header.pages_map_offset() as usize;
    let pages = read_pages_map(
        file,
        pm_off,
        header.pages_map_size_comp(),
        header.pages_map_size_uncomp(),
        header.pages_map_correction(),
    )?;
    let sm_page = pages
        .iter()
        .find(|p| p.id == header.sections_map_id())
        .ok_or("sections map page not found")?;
    let sections = read_sections_map(
        file,
        sm_page.offset as usize,
        header.sections_map_size_comp(),
        header.sections_map_size_uncomp(),
        header.sections_map_correction(),
    )?;

    let get = |name: &str| -> Result<Vec<u8>, String> {
        let s = sections
            .iter()
            .find(|s| s.name == name)
            .ok_or_else(|| format!("section {name} not found"))?;
        read_data_section(file, s, &pages)
    };
    let objects = get(SECTION_OBJECTS)?;
    let handles = get(SECTION_HANDLES)?;
    let classes = get(SECTION_CLASSES).unwrap_or_default();
    Ok(R2007Sections {
        header,
        pages,
        sections,
        objects,
        handles,
        classes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/data/example_2007.dwg"
    );

    fn sample() -> Vec<u8> {
        std::fs::read(SAMPLE).expect("example_2007.dwg must be present")
    }

    #[test]
    fn decode_rs_deinterleaves_round_robin_blocks() {
        // 3 blocks x 4 bytes, stored A0 B0 C0 A1 B1 C1 ...
        let src = [1u8, 2, 3, 10, 11, 12, 20, 21, 22, 30, 31, 32];
        let out = decode_rs(&src, 3, 4).unwrap();
        assert_eq!(out, vec![1, 10, 20, 30, 2, 11, 21, 31, 3, 12, 22, 32]);
        assert!(decode_rs(&src, 4, 4).is_none()); // would read past src
    }

    #[test]
    fn file_header_is_decoded() {
        // Ground truth produced by LibreDWG's decode_r2007.c (see
        // docs/技术设计方案.md, chapter 10).
        let f = sample();
        let h = read_file_header(&f).expect("file header");
        assert_eq!(h.f[0], 112, "header_size");
        assert_eq!(h.f[1], 456000, "file_size");
        assert_eq!(h.pages_amount(), 22);
        assert_eq!(h.num_sections(), 13);
        assert_eq!(h.pages_map_offset(), 0);
        assert_eq!(h.pages_map_id(), 25);
        assert_eq!(h.sections_map_id(), 21);
    }

    #[test]
    fn pages_and_sections_maps_are_decoded() {
        let f = sample();
        let s = load_sections(&f).expect("sections");
        assert_eq!(s.pages.len(), 22, "pages map size");
        assert_eq!(s.pages[0].id, 25);
        assert_eq!(s.pages[0].size, 1024);
        assert_eq!(s.pages[0].offset, 0x480);
        assert_eq!(s.sections.len(), 13, "sections map size");
        let names: Vec<&str> = s.sections.iter().map(|x| x.name.as_str()).collect();
        assert!(names.contains(&SECTION_OBJECTS), "{names:?}");
        assert!(names.contains(&SECTION_HANDLES));
        assert!(names.contains(&SECTION_CLASSES));
    }

    #[test]
    fn object_and_handle_sections_match_the_reference_decoder() {
        // Ground truth: LibreDWG's own decoder produces exactly these bytes
        // (verified against a patched `dwgread` that dumps the decompressed
        // sections). Sizes *and* CRC32 must match — a wrong `TAIL[31]` row in
        // `copy_compressed` still yields the right sizes and the right leading
        // bytes while corrupting everything from ~byte 669 onwards, so the
        // first-16-bytes check alone does not catch it.
        let f = sample();
        let s = load_sections(&f).expect("sections");
        assert_eq!(s.objects.len(), 417109, "objects section size");
        assert_eq!(
            &s.objects[..16],
            &[
                0xca, 0x0d, 0x00, 0x00, 0x39, 0x00, 0x4c, 0x12, 0x80, 0x00, 0x00, 0x00, 0x40, 0x6a,
                0x87, 0x90
            ]
        );
        assert_eq!(s.handles.len(), 1739, "handles section size");
        assert_eq!(
            &s.handles[..16],
            &[
                0x06, 0xc5, 0x01, 0x04, 0x01, 0x88, 0xc6, 0x05, 0x01, 0xe7, 0x00, 0x02, 0x11, 0x01,
                0x17, 0x01
            ]
        );
        assert_eq!(s.classes.len(), 4538);
        assert_eq!(crc32(&s.objects), 0x5f28e517, "objects CRC32 mismatch");
        assert_eq!(crc32(&s.handles), 0x47b33106, "handles CRC32 mismatch");
    }

    /// Minimal CRC32 (IEEE 802.3, reflected) so the container tests can assert
    /// byte-exactness over a 400 KiB section without embedding it in the repo.
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
}

