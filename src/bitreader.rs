//! Bit-level reader for the DWG (AC1015 / R2000) format.
//!
//! All read primitives follow the exact bit/byte ordering used by LibreDWG's
//! `bits.c`. Key conventions (verified against `bits.c`):
//!
//! * Bit 0 of a byte is the **most significant** bit (0x80 >> bit).
//! * The reader is bit-alignment aware: every multi-bit primitive may start at
//!   a non-byte boundary.
//! * Little-endian byte order for multi-byte integers (`RD`, `RS`, `RL`).
//! * `BD`/`DD` use the "compacted double" scheme (2-bit code prefix).

#[derive(Debug, Clone)]
pub struct BitReader<'a> {
    pub data: &'a [u8],
    pub byte: usize,
    pub bit: u32,
}

impl<'a> BitReader<'a> {
    pub fn new(data: &'a [u8]) -> BitReader<'a> {
        BitReader { data, byte: 0, bit: 0 }
    }

    /// Reader starting at an absolute bit/bit-offset within `data`.
    pub fn at(data: &'a [u8], byte: usize, bit: u32) -> BitReader<'a> {
        BitReader { data, byte, bit }
    }

    pub fn remaining_bits(&self) -> i64 {
        (self.data.len() as i64) * 8 - (self.byte as i64 * 8 + self.bit as i64)
    }

    pub fn at_end(&self) -> bool {
        self.byte >= self.data.len()
    }

    /// Bytes still available from the current byte cursor (bit offset ignored).
    ///
    /// Needed by the container-style decoders: reads past the end of `data`
    /// return 0 **without advancing the cursor**, so any `while byte < end`
    /// loop whose `end` came from the file must clamp it to this value *and*
    /// assert forward progress, or a corrupt length spins until OOM.
    pub fn remaining_bytes(&self) -> usize {
        self.data.len().saturating_sub(self.byte)
    }

    /// Current absolute bit position.
    pub fn bitpos(&self) -> usize {
        self.byte * 8 + self.bit as usize
    }

    pub fn set_bitpos(&mut self, pos: usize) {
        self.byte = pos / 8;
        self.bit = (pos % 8) as u32;
    }

    // --- raw bit primitives -------------------------------------------------

    /// Read a single bit (MSB-first).
    pub fn read_b(&mut self) -> u32 {
        if self.byte >= self.data.len() {
            return 0;
        }
        let v = ((self.data[self.byte] >> (7 - self.bit)) & 1) as u32;
        self.bit += 1;
        if self.bit == 8 {
            self.bit = 0;
            self.byte += 1;
        }
        v
    }

    /// Read 2 bits.
    pub fn read_bb(&mut self) -> u32 {
        (self.read_b() << 1) | self.read_b()
    }

    /// Bit-aligned 1-byte read (matches `bit_read_RC`).
    pub fn read_rc(&mut self) -> u8 {
        if self.byte >= self.data.len() {
            return 0;
        }
        if self.bit == 0 {
            let v = self.data[self.byte];
            self.byte += 1;
            v
        } else {
            let b1 = self.data[self.byte];
            let high = (b1 << self.bit) & 0xFF;
            self.byte += 1;
            let low = if self.byte < self.data.len() {
                (self.data[self.byte] >> (8 - self.bit)) as u8
            } else {
                0u8
            };
            (high | low) as u8
        }
    }

    /// Little-endian 2-byte integer (`bit_read_RS`).
    pub fn read_rs(&mut self) -> u16 {
        let lo = self.read_rc() as u16;
        let hi = self.read_rc() as u16;
        lo | (hi << 8)
    }

    /// Little-endian 4-byte integer (`bit_read_RL`).
    pub fn read_rl(&mut self) -> u32 {
        let mut v: u32 = 0;
        for i in 0..4 {
            v |= (self.read_rc() as u32) << (i * 8);
        }
        v
    }

    /// Little-endian 8-byte IEEE-754 double (`bit_read_RD`).
    pub fn read_rd(&mut self) -> f64 {
        let mut buf = [0u8; 8];
        for b in buf.iter_mut() {
            *b = self.read_rc();
        }
        f64::from_le_bytes(buf)
    }

    // --- modular / compacted primitives --------------------------------------

    /// `bit_read_BS`: 2-bit code -> {0: RS, 1: RC, 2: 0, 3: 256}.
    pub fn read_bs(&mut self) -> u16 {
        match self.read_bb() {
            0 => self.read_rs(),
            1 => self.read_rc() as u16,
            2 => 0,
            _ => 256,
        }
    }

    /// `bit_read_BL`: 2-bit code -> {0: RL, 1: RC, 2: 0, 3: 256}.
    ///
    /// **This is modular, not a plain 4-byte little-endian `RL`.** A zero
    /// value compacts to just the 2-bit `10` prefix and consumes **no**
    /// payload byte (see `bit_write_BL`, which only emits `BB(2)` for 0 and
    /// `BB(1) + RC` for 1..=255). Reading it as a raw `RL` shifts every later
    /// field: it is what made R2007 `LAYER` decoding run off by three bits —
    /// `flag0`, the CMC `rgb` and the whole handle stream all landed wrong.
    ///
    /// Note that `BS` and `BL` are bit-identical for every value in `0..=255`
    /// (code 2 / code 1 + RC), so confusing the two is harmless there; they
    /// only diverge for `> 255` (`RS` vs `RL`) and for the `256` sentinel.
    pub fn read_bl(&mut self) -> u32 {
        match self.read_bb() {
            0 => self.read_rl(),
            1 => self.read_rc() as u32,
            2 => 0,
            _ => 256,
        }
    }

    /// `bit_read_BD`: BB code {0: RD, 1: 1.0, 2: 0.0}.
    pub fn read_bd(&mut self) -> f64 {
        match self.read_bb() {
            0 => self.read_rd(),
            1 => 1.0,
            _ => 0.0,
        }
    }

    /// `bit_read_BT`: B {1: 0.0, 0: BD}.
    pub fn read_bt(&mut self) -> f64 {
        if self.read_b() != 0 {
            0.0
        } else {
            self.read_bd()
        }
    }

    /// `bit_read_BE`: B {1: (0,0,1), 0: 3x BD}.
    pub fn read_be(&mut self) -> (f64, f64, f64) {
        if self.read_b() != 0 {
            (0.0, 0.0, 1.0)
        } else {
            let x = self.read_bd();
            let y = self.read_bd();
            let z = self.read_bd();
            (x, y, z)
        }
    }

    /// `bit_read_BD1`: same compacted-double scheme as `BD` (ltype_scale
    /// default 1.0 is produced by BD code 1).
    pub fn read_bd1(&mut self) -> f64 {
        self.read_bd()
    }

    /// `bit_read_BD0` (R2000): like `BD` but a `0` code yields the default
    /// `0.0` *without* consuming an `RD`. Used for entity rotation, etc.
    pub fn read_bd0(&mut self) -> f64 {
        match self.read_bb() {
            0 => 0.0,
            1 => 1.0,
            2 => 0.0,
            3 => self.read_rd(),
            _ => 0.0,
        }
    }

    /// `bit_read_DD(def)`: BB code {0: def, 3: full RD, 1: 4-byte override,
    /// 2: 6-byte override}.
    ///
    /// **The override bytes are *not* a straight sequential fill.** LibreDWG's
    /// `bit_write_DD` emits the low bytes in a permuted order — `[4, 5, 0, 1,
    /// 2, 3]` for code 2 and `[0, 1, 2, 3]` for code 1 — and `bit_read_DD`
    /// must scatter them back into the same slots of `def`'s IEEE-754 bytes,
    /// keeping the *high* bytes (the exponent + top mantissa bits, which the
    /// encoder only drops when they already match `def`). A naive
    /// `db[0..6] = stream` swaps the byte order and silently corrupts every
    /// value whose low 6 bytes differ from `def` — i.e. exactly the `end` of a
    /// `LINE` (`end.x`/`end.y` are DD-relative to `start`, and for a long line
    /// the low 6 bytes diverge), which is why those came out tens-to-hundreds
    /// of units off while `start` (a full `RD`) stayed exact.
    pub fn read_dd(&mut self, default: f64) -> f64 {
        match self.read_bb() {
            0 => default,
            3 => self.read_rd(),
            1 => {
                // code 1: keep high 4 bytes of `def`; overwrite bytes 0..4 in
                // order (encoder writes them sequentially).
                let mut db = default.to_le_bytes();
                for i in 0..4 {
                    db[i] = self.read_rc();
                }
                f64::from_le_bytes(db)
            }
            _ => {
                // code 2: keep high 2 bytes (6,7) of `def`; overwrite the low
                // 6 bytes, which the encoder emitted as [4,5,0,1,2,3].
                let mut db = default.to_le_bytes();
                let b4 = self.read_rc();
                let b5 = self.read_rc();
                let b0 = self.read_rc();
                let b1 = self.read_rc();
                let b2 = self.read_rc();
                let b3 = self.read_rc();
                db[0] = b0;
                db[1] = b1;
                db[2] = b2;
                db[3] = b3;
                db[4] = b4;
                db[5] = b5;
                f64::from_le_bytes(db)
            }
        }
    }

    /// `bit_read_BOT`: R2010+ object type (`SINCE R_2010b`, §2.12).
    ///
    /// A 2-bit code selects the encoding: `0` → `RC` (0…255), `1` → `RC +
    /// 0x1f0` (496…751, the range the ODA/Autodesk type codes live in for
    /// later releases), `2`/`3` → `RS` (a full 16-bit little-endian type).
    ///
    /// Before R2010 the type is a plain `BS`; the two diverge because the type
    /// range grew past 255 and a plain `BS` would have blown the 1-byte `RC`
    /// path. Reading an R2010 object with `BS` (or vice-versa) shifts every
    /// field that follows the type — which is exactly why 2010/2013/2018 came
    /// out as 0 objects until this path was version-gated.
    pub fn read_bot(&mut self) -> u16 {
        match self.read_bb() {
            0 => self.read_rc() as u16,
            1 => (self.read_rc() as u16).wrapping_add(0x1f0),
            _ => self.read_rs(),
        }
    }

    /// `bit_read_MS`: at most two 15-bit words (`R13b1+`).
    pub fn read_ms(&mut self) -> u32 {
        let mut result: u32 = 0;
        for i in [1u32, 0u32] {
            let w = self.read_rs() as u32;
            let shift = if i == 1 { 0 } else { 15 };
            if w & 0x8000 == 0 {
                result |= (w & 0x7fff) << shift;
                return result;
            } else {
                result |= (w & 0x7fff) << shift;
            }
        }
        result
    }

    /// `bit_read_MC`: signed modular char (<=5 bytes).
    /// `bit_read_MC`: signed modular char (<=5 bytes).
    ///
    /// Matches LibreDWG's `bit_read_MC` *including its overflow guard*: when
    /// the accumulated value exceeds `INT32_MAX` the C code jumps to `err_mc`
    /// and returns **0** (the bytes are still consumed). Without that guard a
    /// single malformed entry makes `last_offset` in the object map drift, and
    /// every object after it gets a bogus address — in `example_2007.dwg` this
    /// silently dropped ~60% of the objects.
    pub fn read_mc(&mut self) -> i64 {
        let mut result: u64 = 0;
        let mut negative = false;
        for i in (0..5).rev() {
            let j = (4 - i) * 7;
            let b = self.read_rc();
            if b & 0x80 == 0 {
                let mut b2 = b & 0x7f;
                if b2 & 0x40 != 0 {
                    negative = true;
                    b2 &= 0xbf;
                }
                result |= (b2 as u64) << j;
                if result > i32::MAX as u64 {
                    return 0; // err_mc
                }
                return if negative {
                    -(result as i64)
                } else {
                    result as i64
                };
            } else {
                result |= (b as u64 & 0x7f) << j;
            }
        }
        // Ran out of bytes without a terminator: C logs and returns 0.
        0
    }

    /// `bit_read_UMC`: unsigned modular char (<=8 bytes).
    ///
    /// Like `read_mc`, an entry with no terminator within 8 bytes is an error
    /// and yields 0 in LibreDWG.
    pub fn read_umc(&mut self) -> u64 {
        let mut result: u64 = 0;
        for i in (0..8).rev() {
            let j = (7 - i) * 7;
            let b = self.read_rc();
            if b & 0x80 == 0 {
                result |= (b as u64 & 0x7f) << j;
                return result;
            } else {
                result |= (b as u64 & 0x7f) << j;
            }
        }
        0 // error: unterminated modular char
    }

    // --- strings / handles ---------------------------------------------------

    /// `bit_read_TV`: BS length + bytes. (R13b1+)
    ///
    /// The length is clamped to the remaining buffer so a misaligned read
    /// (stray length field) can never spin for billions of iterations.
    pub fn read_tv(&mut self) -> String {
        let mut len = self.read_bs() as usize;
        let remaining = self.data.len() - self.byte;
        if len > remaining {
            len = remaining;
        }
        let mut v = Vec::with_capacity(len);
        for _ in 0..len {
            v.push(self.read_rc());
        }
        // Trim a single trailing NUL if present (common for R2000 strings).
        while v.last() == Some(&0) {
            v.pop();
        }
        crate::codepage_string(&v)
    }

    /// `bit_read_TU`: BS length + length*2 bytes (UCS-2LE). Used for strings
    /// in R2007+; provided for completeness.
    pub fn read_tu(&mut self) -> String {
        let len = self.read_bs() as usize;
        let mut units = Vec::with_capacity(len);
        for _ in 0..len {
            let lo = self.read_rc() as u16;
            let hi = self.read_rc() as u16;
            units.push(lo | (hi << 8));
        }
        String::from_utf16_lossy(&units)
    }

    /// `bit_read_TF`: exactly `n` bit-aligned bytes (clamped to the buffer).
    pub fn read_tf(&mut self, n: usize) -> Vec<u8> {
        let n = n.min(self.data.len() - self.byte);
        let mut v = Vec::with_capacity(n);
        for _ in 0..n {
            v.push(self.read_rc());
        }
        v
    }

    /// `bit_read_H`: read a handle reference.
    ///
    /// `code = RC`; for R13b1+, `size = code & 0xf`, `code = (code >> 4) & 0xf`.
    /// Then `size` bytes are accumulated LSB-first into the value.
    pub fn read_h(&mut self) -> crate::DwgHandle {
        let code_byte = self.read_rc();
        let size = (code_byte & 0x0f) as usize;
        let code = ((code_byte >> 4) & 0x0f) as u8;
        let mut value: u64 = 0;
        for _ in 0..size {
            value = (value << 8) | (self.read_rc() as u64);
        }
        crate::DwgHandle { code, value }
    }

    /// `bit_read_CMC` for R2000: a single `BS` color index (no truecolor).
    pub fn read_cmc(&mut self) -> crate::Color {
        crate::Color {
            index: self.read_bs() as i16,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::BitReader;

    // --- bit-stream construction helpers ----------------------------------
    //
    // DWG is a *bit* format, so writing test vectors as bytes is error prone.
    // These helpers let every assert express its stream in bits and let the
    // helper do the MSB-first packing.

    fn pack(bits: &[u8]) -> Vec<u8> {
        let mut out = vec![0u8; (bits.len() + 7) / 8];
        for (i, &b) in bits.iter().enumerate() {
            if b != 0 {
                out[i / 8] |= 0x80 >> (i % 8);
            }
        }
        out
    }

    /// Push `n` bits of `value`, MSB-first.
    fn push(bits: &mut Vec<u8>, value: u64, n: usize) {
        for i in (0..n).rev() {
            bits.push(((value >> i) & 1) as u8);
        }
    }

    /// Push an 8-byte little-endian double bit pattern.
    fn push_rd(bits: &mut Vec<u8>, d: f64) {
        for b in d.to_le_bytes() {
            push(bits, b as u64, 8);
        }
    }

    /// Push `nbytes` of `value` in **little-endian** byte order — each byte is
    /// still emitted MSB-first (this is what `read_rs`/`read_rl`/`read_ms`
    /// expect on the wire).
    fn push_le(bits: &mut Vec<u8>, value: u64, nbytes: usize) {
        for i in 0..nbytes {
            push(bits, (value >> (8 * i)) & 0xFF, 8);
        }
    }

    /// Owns a literal byte array so a `BitReader` can borrow it.
    fn own(bytes: &[u8]) -> Vec<u8> {
        bytes.to_vec()
    }

    // --- bit primitives ----------------------------------------------------

    #[test]
    fn bit_order_is_msb_first() {
        let data = own(&[0xA0, 0xFF]);
        let mut rd = BitReader::new(&data); // 1010_0000 1111_1111
        for expected in [1u32, 0, 1, 0] {
            assert_eq!(rd.read_b(), expected);
        }
        assert_eq!(rd.bitpos(), 4);
        // byte0 = 0xA0 = 1010_0000, so bits 4..7 are all zero
        assert_eq!(rd.read_bb(), 0b00);
        assert_eq!(rd.read_bb(), 0b00);
        assert_eq!(rd.bitpos(), 8);
        // now into byte1 = 0xFF (all ones)
        assert_eq!(rd.read_bb(), 0b11);
        assert_eq!(rd.bitpos(), 10);
    }

    #[test]
    fn set_bitpos_is_absolute() {
        let data = own(&[0x12, 0x34, 0x56, 0x78]);
        let mut rd = BitReader::new(&data);
        rd.set_bitpos(3);
        assert_eq!((rd.byte, rd.bit), (0, 3));
        rd.set_bitpos(19);
        assert_eq!((rd.byte, rd.bit), (2, 3));
        assert_eq!(rd.bitpos(), 19);
    }

    #[test]
    fn rc_stitches_across_byte_boundaries() {
        // Start 3 bits into 0xB3: bits 3..7 = 0,0,1,1 wait ->
        // 0xB3 = 1011_0011 -> bits are 1,0,1,1,0,0,1,1
        let data = own(&[0xB3, 0xF5]);
        let mut rd = BitReader::new(&data);
        rd.set_bitpos(3);
        // bits 3..7 of byte0 = 1,0,0,1,1 ; bits 0..2 of byte1 (0xF5=1111_0101) = 1,1,1
        let expected = 0b10011_111u8;
        assert_eq!(rd.read_rc(), expected);
        assert_eq!(rd.bitpos(), 11);
    }

    #[test]
    fn rs_and_rl_are_little_endian() {
        let data = own(&[0x34, 0x12]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_rs(), 0x1234);
        let data = own(&[0x78, 0x56, 0x34, 0x12]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_rl(), 0x1234_5678);
    }

    #[test]
    fn rd_reads_ieee754_double() {
        let data = own(&[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_rd(), 1.5);
        assert!(rd.at_end());
    }

    // --- modular / compacted primitives ------------------------------------

    /// BS uses a 2-bit code: 0 -> RS, 1 -> RC, 2 -> 0, 3 -> 256.
    #[test]
    fn bs_all_four_codes() {
        // code 2 -> 0 (2 bits only)
        let data = pack(&[1, 0]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bs(), 0);
        assert_eq!(rd.bitpos(), 2);

        // code 3 -> 256
        let data = pack(&[1, 1]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bs(), 256);
        assert_eq!(rd.bitpos(), 2);

        // code 1 -> RC payload
        let mut bits = vec![0u8, 1u8];
        push(&mut bits, 0x2A, 8);
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bs(), 0x2A);
        assert_eq!(rd.bitpos(), 10);

        // code 0 -> RS payload, stored little-endian (low byte 0x34 first)
        let mut bits = vec![0u8, 0u8];
        push_le(&mut bits, 0x1234, 2);
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bs(), 0x1234);
        assert_eq!(rd.bitpos(), 18);
    }

    #[test]
    fn bs_code_zero_respects_little_endian_byte_order() {
        // RS payload of 0x1234 must appear as bytes [0x34, 0x12] (low first).
        let data = own(&[0x34, 0x12]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_rs(), 0x1234);

        // and the same value reached through BS code 0: a 2-bit code prefix of
        // `00` followed by the bit-aligned RS payload.
        let mut bits = Vec::new();
        push(&mut bits, 0, 2); // BS code 0
        push_le(&mut bits, 0x1234, 2); // RS payload, bit-aligned
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bs(), 0x1234);
    }

    #[test]
    fn rl_is_a_four_byte_little_endian_word() {
        let data = own(&[0x04, 0x00, 0x00, 0x00]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_rl(), 4);
        assert_eq!(rd.bitpos(), 32);
    }

    #[test]
    fn bl_is_modular_not_a_raw_rl() {
        // code 2 -> 0, and *no* payload byte: only 2 bits are consumed.
        let data = pack(&[1, 0]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bl(), 0);
        assert_eq!(rd.bitpos(), 2);

        // code 1 -> RC payload: 2 + 8 bits.
        let mut bits = Vec::new();
        push(&mut bits, 1, 2);
        push(&mut bits, 0xE0, 8);
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bl(), 0xE0);
        assert_eq!(rd.bitpos(), 10);

        // code 0 -> RL payload, little-endian: 2 + 32 bits. This is exactly
        // the shape a R2007 CMC `rgb` takes (0xC3000007 = ACI index 7).
        let mut bits = Vec::new();
        push(&mut bits, 0, 2);
        push_le(&mut bits, 0xC3000007, 4);
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bl(), 0xC3000007);
        assert_eq!(rd.bitpos(), 34);
    }

    #[test]
    fn bd_compacted_double_all_codes() {
        // code 1 -> 1.0 without any payload
        let data = pack(&[0, 1]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bd(), 1.0);
        assert_eq!(rd.bitpos(), 2);

        // code 2/3 -> 0.0
        let data = pack(&[1, 0]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bd(), 0.0);

        // code 0 -> full 8-byte RD of 2.0
        let mut bits = vec![0u8, 0u8];
        push_rd(&mut bits, 2.0);
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bd(), 2.0);
        assert_eq!(rd.bitpos(), 2 + 64);
    }

    #[test]
    fn dd_uses_two_bit_code_with_default() {
        // code 0 -> keep the default untouched
        let data = pack(&[0, 0]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_dd(7.5), 7.5);

        // code 1 -> overwrite the low 4 bytes of the default (encoder writes
        // them sequentially as [0,1,2,3]).
        let mut bits = vec![0u8, 1u8];
        let mut d = 7.5f64.to_le_bytes();
        d[0] = 0x11;
        d[1] = 0x22;
        d[2] = 0x33;
        d[3] = 0x44;
        for i in 0..4 {
            push(&mut bits, d[i] as u64, 8);
        }
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        let got = rd.read_dd(7.5);
        let expect = f64::from_le_bytes(d);
        assert_eq!(got.to_bits(), expect.to_bits());

        // code 2 -> overwrite the low 6 bytes, but the encoder emits them in the
        // permuted order [4,5,0,1,2,3] and the decoder must scatter them back.
        // Build a default D and a value V that keeps D's high 2 bytes (so mode 2
        // is the one the encoder would actually pick) and differs in the low 6,
        // then emit the stream the way `bit_write_DD` does.
        let d_bytes = 100.0f64.to_le_bytes();
        let mut v_bytes = d_bytes;
        v_bytes[0] = 0x11;
        v_bytes[1] = 0x22;
        v_bytes[2] = 0x33;
        v_bytes[3] = 0x44;
        v_bytes[4] = 0x55;
        v_bytes[5] = 0x66;
        let v = f64::from_le_bytes(v_bytes);
        let mut bits = vec![1u8, 0u8]; // BB code 2 == bits "10"
        for &b in &[v_bytes[4], v_bytes[5], v_bytes[0], v_bytes[1], v_bytes[2], v_bytes[3]] {
            push(&mut bits, b as u64, 8);
        }
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_dd(100.0).to_bits(), v.to_bits());

        // code 3 -> a full replacement double
        let mut bits = vec![1u8, 1u8];
        push_rd(&mut bits, -3.25);
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_dd(7.5), -3.25);
    }

    #[test]
    fn bt_and_be_short_circuit_defaults() {
        let data = pack(&[1]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_bt(), 0.0);
        assert_eq!(rd.bitpos(), 1);

        let data = pack(&[1]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_be(), (0.0, 0.0, 1.0));
        assert_eq!(rd.bitpos(), 1);
    }

    #[test]
    fn ms_reads_up_to_two_fifteen_bit_words() {
        // one word, continuation bit clear -> value 5 (LE bytes [0x05, 0x00])
        let mut bits = Vec::new();
        push_le(&mut bits, 5, 2);
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_ms(), 5);

        // two words: first word has the continuation bit (0x8000) set.
        // word1 = 0x8001 -> bytes [0x01, 0x80]; word2 = 2 -> [0x02, 0x00].
        let mut bits = Vec::new();
        push_le(&mut bits, 0x8001, 2);
        push_le(&mut bits, 2, 2);
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_ms(), 1 | (2 << 15));
    }

    /// `read_mc` mirrors `bit_read_MC`: little-endian 7-bit groups (max 5),
    /// sign taken from bit 6 of the terminating byte.
    #[test]
    fn mc_signed_modular_char() {
        let data = own(&[0x01]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_mc(), 1);

        // terminating-only byte with bit 6 set => negative of the masked value
        let data = own(&[0x7F]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_mc(), -0x3F);

        // two groups: 0x81 continuation, then 0x02 -> 1 | (2 << 7)
        let data = own(&[0x81, 0x02]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_mc(), 1 + (2 << 7));

        // negative over two groups
        let data = own(&[0x81, 0x42]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_mc(), -(1 + (2 << 7)));
    }

    #[test]
    fn umc_unsigned_modular_char() {
        let data = own(&[0x7F]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_umc(), 0x7F);
        let data = own(&[0x81, 0x02]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_umc(), 1 + (2u64 << 7));
    }

    // --- strings and handles -----------------------------------------------

    #[test]
    fn tf_reads_exact_bytes_and_clamps_at_end() {
        let data = own(&[0xAA, 0xBB, 0xCC]);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_tf(2), vec![0xAA, 0xBB]);
        assert_eq!(rd.read_tf(99), vec![0xCC]);
    }

    #[test]
    fn read_h_splits_code_size_and_value() {
        // code nibble high, size nibble low, value bytes follow (big endian)
        let data = own(&[0x41, 0x2A]);
        let mut rd = BitReader::new(&data);
        let h = rd.read_h();
        assert_eq!(h.code, 4);
        assert_eq!(h.value, 0x2A);
        assert_eq!(rd.bitpos(), 16);

        // two value bytes: 0x12 then 0x34 -> accumulated as 0x1234
        let data = own(&[0x02, 0x12, 0x34]);
        let mut rd = BitReader::new(&data);
        let h = rd.read_h();
        assert_eq!(h.code, 0);
        assert_eq!(h.value, 0x1234);

        // size 0 -> no value bytes, value stays 0
        let data = own(&[0x50]);
        let mut rd = BitReader::new(&data);
        let h = rd.read_h();
        assert_eq!(h.code, 5);
        assert_eq!(h.value, 0);
    }

    #[test]
    fn tv_is_length_prefixed_and_trims_trailing_nul() {
        let mut bits = vec![0u8, 1u8]; // BS code 1 -> RC length
        push(&mut bits, 3, 8); // length 3
        push(&mut bits, b'h' as u64, 8);
        push(&mut bits, b'i' as u64, 8);
        push(&mut bits, 0, 8); // trailing NUL is trimmed
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_tv(), "hi");
    }

    #[test]
    fn cmc_reads_a_single_bs_index() {
        let data = pack(&[1, 0]);
        let mut rd = BitReader::new(&data); // BS code 2 -> 0
        assert_eq!(rd.read_cmc().index, 0);

        let mut bits = vec![0u8, 1u8];
        push(&mut bits, 7, 8);
        let data = pack(&bits);
        let mut rd = BitReader::new(&data);
        assert_eq!(rd.read_cmc().index, 7);
    }
}
