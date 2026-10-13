//! Decoders for DWG *extended attributes* — the primary requirement.
//!
//! * **EED** (Extended Entity Data): stored on entities, decoded immediately
//!   after the entity's own handle. Format verified against
//!   `dwg_decode_eed` / `dwg_decode_eed_data` in LibreDWG's `decode.c`.
//! * **XData**: stored inside `XRECORD` (type 0x4f) objects as a resbuf chain
//!   of DXF group codes. The code→value-type table (`ValueType`) and the
//!   per-type payload widths are transcribed from `dwg_resbuf_value_type`
//!   (`src/dwg.c`) and `dwg_decode_xdata` (`src/decode.c`).

use crate::bitreader::BitReader;
use crate::parser::Dialect;
use crate::{Eed, EedItem, XDataItem, XDataValue};

/// Read a big-endian 8-byte integer (used by EED code 5: `RLL_BE`).
fn read_rl_be(r: &mut BitReader) -> u64 {
    let mut v: u64 = 0;
    for _ in 0..8 {
        v = (v << 8) | (r.read_rc() as u64);
    }
    v
}

/// Read a big-endian 2-byte integer (`RS_BE`). Used by EED code 0 codepage.
fn read_rs_be(r: &mut BitReader) -> u16 {
    let hi = r.read_rc() as u16;
    let lo = r.read_rc() as u16;
    (hi << 8) | lo
}

/// Decode a single EED data item given its internal `code`.
///
/// Mirrors `dwg_decode_eed_data`. Returns `None` for an unknown code (the
/// caller stops the inner item loop, as LibreDWG does).
fn decode_eed_item(r: &mut BitReader, code: u8, end: usize, dialect: Dialect) -> Option<EedItem> {
    let avail = end.saturating_sub(r.byte);
    let mut item = EedItem {
        code,
        raw: Vec::new(),
        text: None,
        point: None,
        real: None,
        int16: None,
        int32: None,
        int64: None,
    };
    match code {
        // 0: string  (R2000 path: RC length + RS_BE codepage + length bytes)
        0 => {
            if dialect == Dialect::R2007 {
                // SINCE(R_2007a): the string is UCS-2LE (TU) — a 2-byte `RS`
                // length (in code units) followed by `length * 2` bytes of
                // UTF-16LE. There is *no* codepage field. Reading it as the
                // R2000 single-byte format (RC len + RS_BE codepage + n bytes)
                // garbles every `type=507` save-history record ("AcDbSavedBy…")
                // because each UTF-16 unit's 0x00 high byte lands on a boundary.
                if avail < 2 {
                    return None;
                }
                let len = r.read_rs() as usize;
                if avail.saturating_sub(2) < len.saturating_mul(2) {
                    return None;
                }
                let mut units = Vec::with_capacity(len);
                for _ in 0..len {
                    units.push(r.read_rs());
                }
                item.text = Some(String::from_utf16_lossy(&units));
            } else {
                // R2000/R2004: RC length + RS_BE codepage + single-byte string.
                if avail < 3 {
                    return None;
                }
                let len = r.read_rc() as usize;
                let codepage = read_rs_be(r);
                let len = len.min(end.saturating_sub(r.byte));
                let bytes = r.read_tf(len);
                item.raw = bytes.clone();
                item.int16 = Some(codepage as i16);
                item.text = Some(crate::codepage_string(&bytes));
            }
        }
        // 1: appid index (RS)
        1 => {
            if avail < 2 {
                return None;
            }
            item.int16 = Some(r.read_rs() as i16);
        }
        // 2: close/open flag (RC)
        2 => {
            if avail < 1 {
                return None;
            }
            let close = r.read_rc();
            item.int16 = Some(close as i16);
        }
        // 3: layer handle (RLL, 8 bytes) — R2000 is LATER_VERSIONS
        3 => {
            if avail < 8 {
                return None;
            }
            item.int64 = Some(r.read_rl() as i64);
        }
        // 4: binary chunk (RC length + length bytes)
        4 => {
            if avail < 1 {
                return None;
            }
            let len = r.read_rc() as usize;
            let len = len.min(end.saturating_sub(r.byte));
            item.raw = r.read_tf(len);
        }
        // 5: entity handle (RLL_BE, 8 bytes)
        5 => {
            if avail < 8 {
                return None;
            }
            item.int64 = Some(read_rl_be(r) as i64);
        }
        // 10..=15: 3D point (3 x RD)
        10..=15 => {
            if avail < 24 {
                return None;
            }
            let x = r.read_rd();
            let y = r.read_rd();
            let z = r.read_rd();
            item.point = Some((x, y, z));
        }
        // 40..=42: real (RD)
        40..=42 => {
            if avail < 8 {
                return None;
            }
            item.real = Some(r.read_rd());
        }
        // 70: short (RS)
        70 => {
            if avail < 2 {
                return None;
            }
            item.int16 = Some(r.read_rs() as i16);
        }
        // 71: long (RL)
        71 => {
            if avail < 4 {
                return None;
            }
            item.int32 = Some(r.read_rl() as i32);
        }
        _ => return None,
    }
    Some(item)
}

/// Decode the full EED sequence for one object.
///
/// The on-disk layout (R13b1+) is a chain of blocks:
/// `BS size ; H appid ; <size bytes of data items>`. A `size` of 0 ends the
/// chain. Each data item is `RC code` followed by a code-dependent payload.
pub fn decode_eed(r: &mut BitReader, dialect: Dialect) -> Vec<Eed> {
    let mut out = Vec::new();
    loop {
        let outer_before = r.byte;
        let size = r.read_bs() as usize;
        if size == 0 {
            break;
        }
        let appid = r.read_h().value;
        let start = r.byte;
        // `size` is attacker-controlled. Clamp it to what is actually left in
        // the buffer, because reads past the end return 0 *without advancing
        // the cursor* — an unclamped `end` turns the loop below into an
        // unbounded `items.push` that ends in an OOM kill rather than an error.
        let end = start + size.min(r.remaining_bytes());
        let mut items = Vec::new();
        // The data region is `size` bytes starting after the appid handle.
        while r.byte < end {
            let before = r.byte;
            let code = r.read_rc();
            match decode_eed_item(r, code, end, dialect) {
                Some(it) => items.push(it),
                None => break,
            }
            // Every path that returns `Some` consumes at least one byte, so a
            // zero-advance iteration can only mean we hit the end of the data.
            // Bail out instead of looping forever.
            if r.byte <= before {
                break;
            }
        }
        // Mirror LibreDWG: force the cursor back to the end of this EED block so
        // a short/over-long item cannot poison the following field data.
        r.byte = end;
        out.push(Eed { appid_handle: appid, items });
        // Same reasoning one level up: the outer chain must always move forward.
        if r.byte <= outer_before {
            break;
        }
    }
    out
}

/// Payload kind of a resbuf entry — `DWG_VT_*` in LibreDWG's `dwg.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueType {
    String,
    Real,
    Bool,
    Int8,
    Int16,
    Int32,
    Int64,
    Point3,
    Binary,
    Handle,
    ObjectId,
    Invalid,
}

/// Transcription of LibreDWG's `dwg_resbuf_value_type` (`src/dwg.c`).
///
/// This maps a DXF group code to the resbuf payload type. Note how broad the
/// accepted range is: an `XRECORD`'s XData legitimately uses *ordinary* DXF
/// codes (1, 10, 40, 70, 102, 280, 340 …), unlike entity-level XData which is
/// restricted to 1000..1071.
pub fn resbuf_value_type(gc: i16) -> ValueType {
    use ValueType as V;
    if gc >= 300 {
        if gc >= 440 {
            if gc >= 1000 {
                // 1000-1071
                if gc == 1004 {
                    return V::Binary;
                }
                if gc <= 1009 {
                    return V::String;
                }
                if gc <= 1039 {
                    return V::Point3;
                }
                if gc <= 1042 {
                    return V::Real;
                }
                if gc <= 1069 {
                    return V::Point3;
                }
                if gc <= 1070 {
                    return V::Int16;
                }
                if gc == 1071 {
                    return V::Int32;
                }
            } else {
                // 440-999
                if gc <= 459 {
                    return V::Int32;
                }
                if gc <= 469 {
                    return V::Real;
                }
                if gc <= 479 {
                    return V::String;
                }
                if gc <= 998 {
                    return V::Invalid;
                }
                if gc == 999 {
                    return V::String;
                }
            }
        } else if gc >= 390 {
            // 390-439
            if gc <= 399 {
                return V::Handle;
            }
            if gc <= 409 {
                return V::Int16;
            }
            if gc <= 419 {
                return V::String;
            }
            if gc <= 429 {
                return V::Int32;
            }
            if gc <= 439 {
                return V::String;
            }
        } else {
            // 300-389
            if gc <= 309 {
                return V::String;
            }
            if gc <= 319 {
                return V::Binary;
            }
            if gc <= 329 {
                return V::Handle;
            }
            if gc <= 369 {
                return V::ObjectId;
            }
            if gc <= 389 {
                return V::Int16;
            }
        }
    } else if gc >= 105 {
        if gc >= 210 {
            // 210-299
            if gc <= 269 {
                return V::Point3;
            }
            if gc <= 279 {
                return V::Int16;
            }
            if gc <= 289 {
                return V::Int8;
            }
            if gc <= 299 {
                return V::Bool;
            }
        } else {
            // 105-209
            if gc == 105 {
                return V::Handle;
            }
            if gc <= 109 {
                return V::Invalid;
            }
            if gc <= 139 {
                return V::Point3;
            }
            if gc <= 149 {
                return V::Real;
            }
            if gc <= 169 {
                return V::Int64;
            }
            if gc <= 179 {
                return V::Int16;
            }
            if gc <= 209 {
                return V::Invalid;
            }
        }
    } else if gc >= 38 {
        // 38-102
        if gc <= 59 {
            return V::Real;
        }
        if gc <= 79 {
            return V::Int16;
        }
        if gc <= 99 {
            return V::Int32;
        }
        if gc <= 102 {
            return V::String;
        }
    } else {
        // 0-37
        if gc < 0 {
            return V::Handle;
        }
        if gc <= 4 {
            return V::String;
        }
        if gc == 5 {
            return V::Handle;
        }
        if gc <= 9 {
            return V::String;
        }
        if gc <= 37 {
            return V::Point3;
        }
    }
    V::Invalid
}

/// Decode a `XRECORD` XData resbuf chain of `xdata_size` bytes.
///
/// Each entry is `RS type` followed by a type-dependent payload. Strings are
/// `RS length + RC codepage + length bytes` on R2000/R2004 (`Dialect::R2000`),
/// but **UCS-2LE** (`RS length` code units, then `length` × `RS`) from R2007 on
/// (`Dialect::R2007`) — LibreDWG's `dwg_decode_xdata` gates the two with
/// `PRE (R_2007a)` / `LATER_VERSIONS`. Reading the R2000 single-byte form on a
/// 2007+ drawing garbles every string (each UTF-16 unit's 0x00 high byte lands
/// on a boundary) and desyncs every field that follows it. Mirrors
/// `dwg_decode_xdata`.
pub fn decode_xdata(r: &mut BitReader, xdata_size: usize, dialect: Dialect) -> Vec<XDataItem> {
    let start = r.byte;
    let end = start + xdata_size;
    let mut out = Vec::new();
    // Loop condition mirrors LibreDWG: while (dat->byte < end_address).
    while r.byte < end {
        let before = r.byte;
        if end.saturating_sub(r.byte) < 2 {
            break; // not even a group code left
        }
        let ty = r.read_rs() as i16;
        if r.byte == before {
            break; // no progress
        }
        let vtype = resbuf_value_type(ty);
        let item = match vtype {
            ValueType::Invalid => break,
            ValueType::String => {
                if dialect == Dialect::R2007 {
                    // SINCE(R_2007a): the string is UCS-2LE (TU) — a 2-byte `RS`
                    // length (in code units) followed by `length * 2` bytes of
                    // UTF-16LE, no codepage field. See the matching note in
                    // `decode_eed_item`.
                    if end.saturating_sub(r.byte) < 2 {
                        break;
                    }
                    let len = r.read_rs() as usize;
                    if end.saturating_sub(r.byte) < len.saturating_mul(2) {
                        break;
                    }
                    let mut units = Vec::with_capacity(len);
                    for _ in 0..len {
                        units.push(r.read_rs());
                    }
                    XDataItem {
                        code: ty,
                        value: XDataValue::Str(String::from_utf16_lossy(&units)),
                    }
                } else {
                    // R2000/R2004: `RS length + RC codepage + length bytes`.
                    if end.saturating_sub(r.byte) < 3 {
                        break;
                    }
                    let len = r.read_rs() as usize;
                    let _codepage = r.read_rc();
                    let len = len.min(end.saturating_sub(r.byte));
                    let bytes = r.read_tf(len);
                    XDataItem {
                        code: ty,
                        value: XDataValue::Str(crate::codepage_string(&bytes)),
                    }
                }
            }
            ValueType::Real => {
                if end.saturating_sub(r.byte) < 8 {
                    break;
                }
                XDataItem { code: ty, value: XDataValue::Real(r.read_rd()) }
            }
            ValueType::Bool | ValueType::Int8 => {
                if end.saturating_sub(r.byte) < 1 {
                    break;
                }
                XDataItem { code: ty, value: XDataValue::Int8(r.read_rc() as i8) }
            }
            ValueType::Int16 => {
                if end.saturating_sub(r.byte) < 2 {
                    break;
                }
                XDataItem { code: ty, value: XDataValue::Int16(r.read_rs() as i16) }
            }
            ValueType::Int32 => {
                if end.saturating_sub(r.byte) < 4 {
                    break;
                }
                XDataItem { code: ty, value: XDataValue::Int32(r.read_rl() as i32) }
            }
            ValueType::Int64 => {
                if end.saturating_sub(r.byte) < 8 {
                    break;
                }
                let mut v: u64 = 0;
                for i in 0..8 {
                    v |= (r.read_rc() as u64) << (i * 8);
                }
                XDataItem { code: ty, value: XDataValue::Int64(v as i64) }
            }
            ValueType::Point3 => {
                if end.saturating_sub(r.byte) < 24 {
                    break;
                }
                let x = r.read_rd();
                let y = r.read_rd();
                let z = r.read_rd();
                XDataItem { code: ty, value: XDataValue::Point3((x, y, z)) }
            }
            ValueType::Binary => {
                if end.saturating_sub(r.byte) < 1 {
                    break;
                }
                let len = r.read_rc() as usize;
                let len = len.min(end.saturating_sub(r.byte));
                let bytes = r.read_tf(len);
                XDataItem { code: ty, value: XDataValue::Binary(bytes) }
            }
            ValueType::Handle | ValueType::ObjectId => {
                if end.saturating_sub(r.byte) < 8 {
                    break;
                }
                let mut v: u64 = 0;
                for i in 0..8 {
                    v |= (r.read_rc() as u64) << (i * 8);
                }
                XDataItem { code: ty, value: XDataValue::Handle(v) }
            }
        };
        out.push(item);
    }
    // Force the cursor to the declared end of the region: LibreDWG leaves it
    // wherever the last item landed and warns on mismatch, but we must never
    // leave the caller inside the XData block.
    //
    // **`r.bit` must be preserved.** Every XData payload is a whole-byte read
    // (`RC`/`RS`/`RD`/`TF`), and `bit_read_RC` is *bit-aligned*: it consumes
    // 8 bits and leaves `bit` exactly where it found it (see `bits.c`). So the
    // bit offset on entry is the bit offset on exit, and the window above is a
    // pure byte window measured from `start` — exactly LibreDWG's
    // `while (dat->byte < end_address)`.
    //
    // Zeroing `bit` here silently dropped up to 7 bits and shifted every field
    // that follows. On R2004+ `XRECORD`s the XDATA starts at bit 1, so the
    // trailing `BS cloning` (DXF 280) came out as garbage (-24448 instead of 1)
    // on every release from AC1018 on; AC1015 was unaffected only because its
    // XDATA happens to start byte-aligned.
    r.byte = end.max(r.byte);
    out
}

/// Build an XData resbuf item list from hand-crafted bytes — used by the unit
/// tests to exercise individual group codes in isolation.
pub fn decode_xdata_bytes(bytes: &[u8]) -> Vec<XDataItem> {
    let mut r = BitReader::new(bytes);
    // The hand-crafted unit-test streams are always R2000/TV form.
    decode_xdata(&mut r, bytes.len(), Dialect::R2000)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::XDataValue;

    /// Append the low `n` bits of `value`, MSB-first — the on-disk bit order.
    fn push(bits: &mut Vec<u8>, value: u64, n: usize) {
        for i in (0..n).rev() {
            bits.push(((value >> i) & 1) as u8);
        }
    }

    /// Pack a bit vector into bytes, bit 0 == 0x80 (matches `bits.c`).
    fn pack(bits: &[u8]) -> Vec<u8> {
        let mut out = vec![0u8; (bits.len() + 7) / 8];
        for (i, &bit) in bits.iter().enumerate() {
            if bit != 0 {
                out[i / 8] |= 0x80 >> (i % 8);
            }
        }
        out
    }

    // --- `resbuf_value_type` classification -------------------------------
    #[test]
    fn classifies_common_group_codes() {
        // Sub-string codes (0..=4)
        assert_eq!(resbuf_value_type(1), ValueType::String);
        // 38..=102 string range (control strings like 102 "{…}")
        assert_eq!(resbuf_value_type(102), ValueType::String);
        // reals
        assert_eq!(resbuf_value_type(40), ValueType::Real);
        assert_eq!(resbuf_value_type(50), ValueType::Real);
        // 16-bit shorts (70)
        assert_eq!(resbuf_value_type(70), ValueType::Int16);
        // 8-bit ints (280..=289)
        assert_eq!(resbuf_value_type(280), ValueType::Int8);
        // 32-bit longs (80..=99, 90)
        assert_eq!(resbuf_value_type(90), ValueType::Int32);
        // handles / object ids (330..=369, 340)
        assert_eq!(resbuf_value_type(330), ValueType::ObjectId);
        assert_eq!(resbuf_value_type(340), ValueType::ObjectId);
        assert_eq!(resbuf_value_type(5), ValueType::Handle);
        // extended-data string / real / int
        assert_eq!(resbuf_value_type(1000), ValueType::String);
        assert_eq!(resbuf_value_type(1003), ValueType::String);
        assert_eq!(resbuf_value_type(1040), ValueType::Real);
        assert_eq!(resbuf_value_type(1071), ValueType::Int32);
        assert_eq!(resbuf_value_type(310), ValueType::Binary);
        assert_eq!(resbuf_value_type(999), ValueType::String);
        // a code that must be rejected (180..=209 band)
        assert_eq!(resbuf_value_type(206), ValueType::Invalid);
    }

    // --- `decode_xdata` on hand-crafted byte streams ----------------------
    #[test]
    fn decodes_string_int16_real_handle_chain() {
        // A single resbuf chain:
        //   group 1  "Hi"          (string)
        //   group 70 5             (16-bit int)
        //   group 40 2.5           (real)
        //   group 340 handle 106   (object id / handle)
        let mut b = Vec::new();
        // group 1 — String: RS group, RS length, RC codepage, bytes
        b.extend_from_slice(&[0x01, 0x00]); // group 1
        b.extend_from_slice(&[0x02, 0x00]); // length 2
        b.push(0x00); // codepage
        b.extend_from_slice(b"Hi");
        // group 70 — Int16
        b.extend_from_slice(&[0x46, 0x00]); // group 70
        b.extend_from_slice(&[0x05, 0x00]); // 5
        // group 40 — Real
        b.extend_from_slice(&[0x28, 0x00]); // group 40
        b.extend_from_slice(&2.5f64.to_le_bytes());
        // group 340 — ObjectId / Handle (8 bytes little-endian)
        b.extend_from_slice(&[0x54, 0x01]); // group 340 = 0x0154
        b.extend_from_slice(&106u64.to_le_bytes());

        let items = decode_xdata_bytes(&b);
        assert_eq!(items.len(), 4, "expected 4 resbuf entries");

        assert_eq!(items[0].code, 1);
        assert_eq!(items[0].value, XDataValue::Str("Hi".into()));

        assert_eq!(items[1].code, 70);
        assert_eq!(items[1].value, XDataValue::Int16(5));

        assert_eq!(items[2].code, 40);
        match items[2].value {
            XDataValue::Real(v) => assert!((v - 2.5).abs() < 1e-9, "real = {v}"),
            ref other => panic!("expected Real, got {other:?}"),
        }

        assert_eq!(items[3].code, 340);
        assert_eq!(items[3].value, XDataValue::Handle(106));
    }

    #[test]
    fn stops_on_invalid_group_code() {
        // group 110 sits in the 105..=209 "Invalid" band, so the chain must
        // terminate before reaching the trailing group-1 entry.
        let mut b = Vec::new();
        b.extend_from_slice(&[0x6E, 0x00]); // group 110 (invalid)
        b.extend_from_slice(&[0x01, 0x00]); // group 1 — never decoded
        b.push(0x00);
        b.extend_from_slice(b"Hi");
        let items = decode_xdata_bytes(&b);
        assert!(items.is_empty(), "chain must stop on invalid code");
    }

    // --- `decode_eed` on a hand-crafted EED block -------------------------
    #[test]
    fn decodes_eed_block() {
        // EED is bit-packed, so construct the stream with bit helpers that
        // mirror the real on-disk layout:
        //   BS size ; H appid ; <size bytes of RC-coded data items>
        let mut bits = Vec::new();
        // size = 3, encoded as BS code 1 (RC payload)
        push(&mut bits, 1, 2);
        push(&mut bits, 3, 8);
        // appid handle: code byte 0x00 -> size 0, code 0, value 0
        push(&mut bits, 0x00, 8);
        // one data item: code 70 (RC) then RS value 7 (little-endian)
        push(&mut bits, 0x46, 8);
        push(&mut bits, 7, 8);
        push(&mut bits, 0, 8);
        let data = pack(&bits);

        let mut r = crate::bitreader::BitReader::new(&data);
        let eed = decode_eed(&mut r, crate::Dialect::R2000);
        assert_eq!(eed.len(), 1, "one EED block expected");
        let block = &eed[0];
        assert_eq!(block.appid_handle, 0);
        assert_eq!(block.items.len(), 1);
        assert_eq!(block.items[0].code, 70);
        assert_eq!(block.items[0].int16, Some(7));
    }

    // --- the bit offset must survive the block ----------------------------
    /// The bug that turned `cloning` (DXF 280) into garbage on every R2004+
    /// `XRECORD`.
    ///
    /// `bit_read_RC` is *bit-aligned*: it consumes 8 bits and leaves
    /// `dat->bit` exactly where it found it, so a block that starts at bit 1
    /// also *ends* at bit 1 and LibreDWG's `while (dat->byte < end_address)`
    /// is a pure byte window measured from the start byte. Zeroing `bit` when
    /// the block closed dropped up to 7 bits and shifted the following
    /// `BS cloning` by one — on `example_2007.dwg` that reported `-24448`
    /// instead of the true `cloning: 1`.
    ///
    /// Payload below is the real one from handle `0x26B`
    /// (`dwgread -v4`: `xdata[0] type 270 / 1`, `xdata[1] type 271 / 1`).
    #[test]
    fn leaves_the_bit_offset_untouched_at_the_end_of_the_block() {
        // 270 == 0x010E -> LE 0x0E,0x01 ; 271 == 0x010F -> LE 0x0F,0x01
        let payload: [u8; 8] = [0x0E, 0x01, 0x01, 0x00, 0x0F, 0x01, 0x01, 0x00];

        let mut bits = Vec::new();
        push(&mut bits, 0, 1); // pad so the block starts at bit 1
        for b in payload {
            push(&mut bits, b as u64, 8);
        }
        push(&mut bits, 0b1010_1010, 8); // marker, right after the block
        let data = pack(&bits);

        let mut r = BitReader::at(&data, 0, 1);
        let items = decode_xdata(&mut r, 8, Dialect::R2000);

        assert_eq!(items.len(), 2, "two resbuf pairs: {items:?}");
        assert_eq!(items[0].code, 270, "group code 270");
        assert_eq!(items[0].value, XDataValue::Int16(1), "xdata[0] == 1");
        assert_eq!(items[1].code, 271, "group code 271");
        assert_eq!(items[1].value, XDataValue::Int16(1), "xdata[1] == 1");

        // 1 pad bit + 8 payload bytes == bit 65. A cursor reset to a byte
        // boundary lands on 64 and reads the marker shifted by one
        // (0b0101_0101 instead of 0b1010_1010).
        assert_eq!(r.bitpos(), 65, "cursor must end at bit 65, not 64");
        assert_eq!(
            r.read_rc(),
            0b1010_1010,
            "the field after the block must not be shifted"
        );
    }

    /// …and the byte-aligned case must behave identically, which is why
    /// AC1015 (`sample_2000.dwg`) never showed the bug.
    #[test]
    fn byte_aligned_block_ends_on_the_byte_boundary() {
        let payload: [u8; 8] = [0x0E, 0x01, 0x01, 0x00, 0x0F, 0x01, 0x01, 0x00];
        let mut bits = Vec::new();
        for b in payload {
            push(&mut bits, b as u64, 8);
        }
        push(&mut bits, 0b1010_1010, 8);
        let data = pack(&bits);

        let mut r = BitReader::new(&data);
        let items = decode_xdata(&mut r, 8, Dialect::R2000);
        assert_eq!(items.len(), 2);
        assert_eq!(r.bitpos(), 64, "byte-aligned block ends at bit 64");
        assert_eq!(r.read_rc(), 0b1010_1010);
    }

    /// R2007+ XData strings are UCS-2LE (TU), not the R2000 `RS len + RC
    /// codepage + bytes` (TV) form. The text "CDM" in UTF-16LE is the byte
    /// sequence `43 00 44 00 4D 00`; read as TV it would come out as the single
    /// byte `0x43` ('C') with a `0x00` codepage.
    #[test]
    fn r2007_strings_are_utf16le() {
        // group 1 (String), then TU: RS length 3, then "CDM" as 3 × RS.
        let mut b = Vec::new();
        b.extend_from_slice(&[0x01, 0x00]); // group 1
        b.extend_from_slice(&[0x03, 0x00]); // length 3 code units
        b.extend_from_slice(&[0x43, 0x00]); // 'C'
        b.extend_from_slice(&[0x44, 0x00]); // 'D'
        b.extend_from_slice(&[0x4D, 0x00]); // 'M'
        let mut r = BitReader::new(&b);
        let items = decode_xdata(&mut r, b.len(), Dialect::R2007);
        assert_eq!(items.len(), 1, "one string: {items:?}");
        assert_eq!(items[0].code, 1);
        assert_eq!(items[0].value, XDataValue::Str("CDM".to_string()));

        // …and the same bytes decoded as TV must NOT produce "CDM".
        let mut r2 = BitReader::new(&b);
        let tv = decode_xdata(&mut r2, b.len(), Dialect::R2000);
        assert_eq!(tv.len(), 1);
        assert_ne!(tv[0].value, XDataValue::Str("CDM".to_string()));
    }
}
