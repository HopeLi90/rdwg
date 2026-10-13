//! Robustness: a malformed file must produce an `Err`, never a panic.
//!
//! A parser that reads attacker-supplied (or merely truncated / corrupted)
//! binary has a hard requirement distinct from "decodes valid files correctly":
//! **every input must terminate with a `Result`, never with unwinding, an
//! out-of-bounds abort, or an OOM**. This suite is that contract.
//!
//! Three strategies, cheapest first:
//!
//! 1. **Degenerate inputs** — empty, tiny, all-zero, random garbage.
//! 2. **Truncation sweep** — every prefix length at and around the structural
//!    boundaries (file header at `0x80`, first page at `0x480`).
//! 3. **Bit-flip fuzz** — a deterministic LCG (no dependency, reproducible)
//!    flips 1–4 bits per round; plus a systematic single-byte sweep over the
//!    first KiB of the smaller fixture.
//!
//! Everything is driven through `catch_unwind` so that a panic is reported as
//! a *test failure naming the offending input*, not as noise.

use std::panic::{catch_unwind, AssertUnwindSafe};

use dwgparse::DwgFile;

const SAMPLE_2000: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/sample_2000.dwg");
const SAMPLE_2007: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/example_2007.dwg");

/// Deterministic PRNG (a 64-bit LCG) so a failure is always reproducible —
/// no `rand` dependency, and the same seed gives the same mutations forever.
struct Lcg(u64);

impl Lcg {
    fn next_u64(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0
    }
    /// Uniform-ish in `0..n`.
    fn below(&mut self, n: usize) -> usize {
        (self.next_u64() >> 16) as usize % n
    }
}

/// Run `parse` on `bytes` written to a scratch file, and assert it does not
/// panic. Returns the parse result so callers can assert on `Err` vs `Ok`.
fn parse_safely(bytes: &[u8]) -> Result<DwgFile, String> {
    let mut path = std::env::temp_dir();
    path.push(format!("dwgparse_robust_{}.dwg", std::process::id()));
    std::fs::write(&path, bytes).expect("write scratch file");

    let outcome = catch_unwind(AssertUnwindSafe(|| DwgFile::parse(path.to_str().unwrap())));

    let _ = std::fs::remove_file(&path);

    match outcome {
        Ok(result) => result,
        Err(panic) => {
            let msg = panic
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_else(|| "<non-string panic payload>".to_string());
            panic!(
                "PANIC on a {} byte input: {msg}\n\
                 (a malformed file must return Err, not unwind)",
                bytes.len()
            );
        }
    }
}

#[test]
fn degenerate_inputs_are_rejected_without_panicking() {
    let cases: Vec<(&str, Vec<u8>)> = vec![
        ("empty", Vec::new()),
        ("one byte", vec![0u8]),
        ("29 bytes (just under the gate)", vec![b'A'; 29]),
        ("30 zero bytes", vec![0u8; 30]),
        ("30 bytes of 0xff", vec![0xffu8; 30]),
        ("AC1015 header only", b"AC1015".to_vec()),
        ("AC1021 header only", b"AC1021".to_vec()),
        ("AC1021 + 1 KiB of zeros", {
            let mut v = b"AC1021".to_vec();
            v.extend_from_slice(&[0u8; 1024]);
            v
        }),
        ("AC1021 + 1 KiB of 0xff", {
            let mut v = b"AC1021".to_vec();
            v.extend_from_slice(&[0xffu8; 1024]);
            v
        }),
        ("AC1015 + 4 KiB of 0x5a", {
            let mut v = b"AC1015".to_vec();
            v.extend_from_slice(&[0x5au8; 4096]);
            v
        }),
    ];

    for (name, bytes) in cases {
        match parse_safely(&bytes) {
            Ok(_) => panic!("{name}: unexpectedly parsed successfully"),
            Err(e) => assert!(!e.is_empty(), "{name}: error message must not be empty"),
        }
    }
}

#[test]
fn truncated_files_are_rejected_without_panicking() {
    for (label, path) in [("R2000", SAMPLE_2000), ("R2007", SAMPLE_2007)] {
        let full = std::fs::read(path).expect("fixture");

        let mut lengths: Vec<usize> = vec![0, 1, 6, 29, 30, 31, 64, 0x7f, 0x80, 0x81];
        // Structural boundaries of the R2007 container, plus a coarse sweep.
        lengths.extend([0x100, 0x3d8, 0x400, 0x47f, 0x480, 0x481, 0x1000]);
        let mut step = full.len() / 24;
        while step > 0 && lengths.len() < 60 {
            lengths.push(step);
            step = step.saturating_sub(full.len() / 24);
        }
        lengths.push(full.len() - 1);
        lengths.sort_unstable();
        lengths.dedup();

        for n in lengths {
            let n = n.min(full.len());
            let _ = parse_safely(&full[..n]);
            // Either outcome is acceptable — the requirement is "no panic".
        }
        let _ = label;
    }
}

/// Flip 1–4 bits in `base` and parse, over `rounds` deterministic rounds.
///
/// Returns `(rejected, accepted)`. Panics if *any* round unwinds.
fn fuzz(base: &[u8], seed: u64, rounds: usize, tag: &str) -> (usize, usize) {
    let mut rng = Lcg(seed);
    let mut errored = 0usize;
    let mut accepted = 0usize;

    for round in 0..rounds {
        let mut bytes = base.to_vec();
        let flips = 1 + rng.below(4);
        // stderr is unbuffered: if a round gets OOM-killed we still see which.
        eprintln!("{tag} fuzz round {round}/{rounds}");
        for _ in 0..flips {
            // Start at 6 so most rounds keep the version stamp intact and
            // actually exercise the container; occasionally hit the stamp too.
            let off = if rng.below(8) == 0 {
                rng.below(6)
            } else {
                6 + rng.below(bytes.len() - 6)
            };
            let bit = rng.below(8);
            bytes[off] ^= 1 << bit;
        }
        let result = std::panic::catch_unwind(AssertUnwindSafe(|| {
            let mut path = std::env::temp_dir();
            path.push(format!("dwgparse_fuzz_{}_{}.dwg", tag, std::process::id()));
            std::fs::write(&path, &bytes).expect("write");
            let r = DwgFile::parse(path.to_str().unwrap());
            let _ = std::fs::remove_file(&path);
            r
        }));
        match result {
            Ok(Ok(_)) => accepted += 1,
            Ok(Err(_)) => errored += 1,
            Err(_) => panic!("PANIC in {tag} fuzz round {round} — malformed input must return Err"),
        }
    }

    // Sanity: the fuzz must actually be stressing the parser, not accidentally
    // producing files that are all trivially rejected by the version gate.
    assert!(
        errored > 0,
        "{tag}: no fuzz round produced an error — the mutations are not reaching the decoder"
    );
    (errored, accepted)
}

#[test]
fn bit_flips_never_panic() {
    // Keep the test output readable: a caught panic would otherwise print a
    // backtrace for every single round.
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));

    // Both dialects: they are two independent decoders (flat sections vs page
    // container), so a hazard in one says nothing about the other.
    let r2000 = std::fs::read(SAMPLE_2000).expect("fixture");
    let r2007 = std::fs::read(SAMPLE_2007).expect("fixture");

    let (e0, a0) = fuzz(&r2000, 0x5eed_0001, 800, "r2000");
    let (e1, a1) = fuzz(&r2007, 0x5eed_1234, 1200, "r2007");

    std::panic::set_hook(prev);

    println!("bit-flip fuzz: AC1015 800 rounds -> {e0} rejected / {a0} parsed");
    println!("bit-flip fuzz: AC1021 1200 rounds -> {e1} rejected / {a1} parsed");
}

#[test]
fn single_byte_corruption_sweep_over_the_header_region() {
    // Systematic (not random): every byte of the first KiB is forced to 0x00 and
    // 0xff. Cheap on the small fixture and it covers the header/定位表 region
    // exhaustively, which is where a bad bound check hurts most.
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));

    let base = std::fs::read(SAMPLE_2000).expect("fixture");
    let limit = base.len().min(1024);
    let mut panics = Vec::new();

    for off in 0..limit {
        for value in [0x00u8, 0xffu8] {
            if base[off] == value {
                continue; // no mutation, skip
            }
            let mut bytes = base.clone();
            bytes[off] = value;
            let result = catch_unwind(AssertUnwindSafe(|| {
                let mut path = std::env::temp_dir();
                path.push(format!("dwgparse_sweep_{}.dwg", std::process::id()));
                std::fs::write(&path, &bytes).expect("write");
                let r = DwgFile::parse(path.to_str().unwrap());
                let _ = std::fs::remove_file(&path);
                r
            }));
            if result.is_err() {
                panics.push((off, value));
            }
        }
    }

    std::panic::set_hook(prev);

    assert!(
        panics.is_empty(),
        "{} single-byte mutations panicked (offset, value): {:?}",
        panics.len(),
        &panics[..panics.len().min(10)]
    );
}
