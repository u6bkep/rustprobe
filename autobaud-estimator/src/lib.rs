//! Pure baud-rate estimator for the PIO edge-timing autobaud engine.
//!
//! This is a faithful `no_std` port of the estimation core of debugprobe's
//! `autobaud.c` (`estimate_baud_rate` and its helpers), split out from all
//! hardware so it can be unit-tested on the host. Raw edge-timer samples go in
//! ([`BaudEstimator::push`]); a published `(baud, validity)` comes out when the
//! estimate becomes confident and has changed.
//!
//! # What a sample is
//!
//! The PIO program (`autobaud.pio`) waits for a falling edge on the UART RX
//! line, then counts loop iterations until the rising edge. Each loop iteration
//! is two PIO clocks, and the counter runs *down* from `0xFFFF_FFFF`, so the
//! raw FIFO word `raw` encodes a low-pulse width of
//! `(0xFFFF_FFFF - raw) * 2` PIO clocks — see [`raw_to_cycles`]. On an idle-high
//! UART line, the shortest low pulse is a single `0` bit, so the minimum
//! low-pulse width across many samples is one bit period, and
//! `baud = clock / bit_period`.
//!
//! # Algorithm (unchanged from the C)
//!
//! Each converted sample is binned in a fixed-size open-addressing hash table
//! keyed by exact cycle count. A cycle count seen in at least
//! [`MIN_FREQUENCY`] of all samples is treated as signal (not line noise). The
//! smallest such value is taken as the one-bit period; samples within +10% of
//! it are averaged to smooth sub-cycle jitter, and `baud = clock / average`.
//! A validity score combines *completeness* (how many samples we've seen,
//! `1 - e^(-N/40)`) and *consistency* (the fraction of "one-bit" samples that
//! are implausibly short relative to the longest pulse). A new baud is
//! published only when validity exceeds `0.6` and the value moved more than
//! 0.5% from the last published baud.
//!
//! # Deliberately-preserved C quirks
//!
//! * `raw == 0xFFFF_FFFF` (or the wrap at `raw == 0x7FFF_FFFF`) maps to a cycle
//!   count of `0`, which the hash table uses as its empty-slot sentinel, so
//!   such samples are silently dropped. These correspond to degenerate
//!   zero/near-zero-width pulses that carry no baud information anyway.
//! * `bit_time_sum` is a 32-bit accumulator, as in the C. It cannot overflow
//!   within a realistic autobaud session (thousands of samples of ~1e3 cycles),
//!   and keeping it 32-bit matches the C's rounding exactly.

#![cfg_attr(not(test), no_std)]

/// Minimum fraction of all samples a cycle count must reach to count as signal
/// rather than noise (`MIN_FREQUENCY` in the C).
pub const MIN_FREQUENCY: f32 = 0.05;

/// Size of the open-addressing frequency table (`HASH_TBL_SIZE` in the C).
const HASH_TBL_SIZE: usize = 500;

/// Validity threshold above which a new baud estimate is published.
const VALIDITY_THRESHOLD: f32 = 0.6;

/// Convert a raw edge-timer FIFO word to a low-pulse width in PIO clocks.
///
/// The PIO counter runs down from `0xFFFF_FFFF`, decrementing once per two-clock
/// loop iteration, so `(0xFFFF_FFFF - raw) * 2` is the elapsed clocks. The
/// multiply is intentionally wrapping, matching the C's `uint32_t` arithmetic.
#[inline]
pub fn raw_to_cycles(raw: u32) -> u32 {
    (u32::MAX - raw).wrapping_mul(2)
}

/// One open-addressing hash-table slot. `key == 0` marks an empty slot, so a
/// cycle count of exactly zero is never stored (a preserved C quirk).
#[derive(Clone, Copy, Default)]
struct Entry {
    key: u32,
    count: u32,
}

/// A published baud estimate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimate {
    /// Detected baud rate, rounded to the nearest integer.
    pub baud: u32,
    /// Confidence in `[0.0, 1.0]`; only values above `0.6` are ever emitted.
    pub validity: f32,
}

/// Streaming baud-rate estimator. Feed raw edge-timer samples with
/// [`push`](Self::push); state accumulates until an estimate is confident.
pub struct BaudEstimator {
    /// PIO clock frequency the samples were timed against, in Hz.
    clock_hz: u32,
    table: [Entry; HASH_TBL_SIZE],
    /// Last published baud, as an `f32` (the comparison basis, `0.0` initially).
    baud: f32,
    /// Shortest bit duration seen so far, in PIO cycles.
    min_cycles: u32,
    /// Longest pulse seen so far, in PIO cycles.
    max_cycles: u32,
    /// Total samples processed.
    total: u32,
    /// Running sum of one-bit-period samples (32-bit, as in the C).
    bit_time_sum: u32,
    /// Count of one-bit-period samples.
    bit_time_count: u32,
    /// One-bit samples that are implausibly short (< 1/9 of the longest pulse).
    outlier_count: u32,
}

impl BaudEstimator {
    /// Create an estimator for samples timed against a `clock_hz` PIO clock.
    /// The C hardcodes 125 MHz; here it is explicit so RP2040 and RP2350 (whose
    /// autobaud PIO also runs at 125 MHz via its clock divider) share one core.
    pub fn new(clock_hz: u32) -> Self {
        Self {
            clock_hz,
            table: [Entry::default(); HASH_TBL_SIZE],
            baud: 0.0,
            min_cycles: u32::MAX,
            max_cycles: 0,
            total: 0,
            bit_time_sum: 0,
            bit_time_count: 0,
            outlier_count: 0,
        }
    }

    /// Clear all accumulated state, as if freshly constructed. Called between
    /// autobaud sessions (the C frees and recreates the table instead).
    pub fn reset(&mut self) {
        let clock_hz = self.clock_hz;
        *self = Self::new(clock_hz);
    }

    /// Integer hash of `x` into `[0, HASH_TBL_SIZE)` (the C's `hash`).
    fn hash(x: u32) -> usize {
        let mut x = x;
        x = ((x >> 16) ^ x).wrapping_mul(0x45d9_f3b);
        x = ((x >> 16) ^ x).wrapping_mul(0x45d9_f3b);
        x = (x >> 16) ^ x;
        (x as usize) % HASH_TBL_SIZE
    }

    /// Increment the occurrence count for `key` (the C's `insert`).
    fn insert(&mut self, key: u32) {
        let mut idx = Self::hash(key);
        while self.table[idx].key != 0 {
            if self.table[idx].key == key {
                self.table[idx].count += 1;
                return;
            }
            idx = (idx + 1) % HASH_TBL_SIZE;
        }
        self.table[idx] = Entry { key, count: 1 };
    }

    /// Occurrence count of `key`, or 0 if absent (the C's `get_count`).
    fn get_count(&self, key: u32) -> u32 {
        let mut idx = Self::hash(key);
        while self.table[idx].key != 0 {
            if self.table[idx].key == key {
                return self.table[idx].count;
            }
            idx = (idx + 1) % HASH_TBL_SIZE;
        }
        0
    }

    /// Whether `new_baud` differs from the last published `baud` by more than
    /// 0.5% (the C's `baud_changed`).
    fn baud_changed(&self, new_baud: f32) -> bool {
        let hi = (self.baud * 1.005) as u32;
        let lo = (self.baud * 0.995) as u32;
        let new = new_baud as u32;
        new > hi || new < lo
    }

    /// Feed one raw edge-timer sample. Returns `Some(Estimate)` when this sample
    /// causes a new, confident baud rate to be published.
    ///
    /// Mirrors the body of the C `estimate_baud_rate` per-sample loop exactly.
    pub fn push(&mut self, raw: u32) -> Option<Estimate> {
        let cycles = raw_to_cycles(raw);
        self.insert(cycles);
        self.total += 1;

        if cycles > self.max_cycles {
            self.max_cycles = cycles;
        }

        let freq = self.get_count(cycles) as f32 / self.total as f32;
        // Ignore values too rare to be part of the signal.
        if freq < MIN_FREQUENCY {
            return None;
        }
        // A new minimum resets the one-bit-period accumulators.
        if cycles < self.min_cycles {
            self.min_cycles = cycles;
            self.bit_time_sum = 0;
            self.bit_time_count = 0;
            self.outlier_count = 0;
            return None;
        }
        // Within +10% of the minimum: treat as a one-bit period.
        if ((cycles - self.min_cycles) as f32) < (self.min_cycles as f32 * 0.1) {
            self.bit_time_sum += cycles;
            self.bit_time_count += 1;
            // A one-bit period below 1/9 of the longest pulse is implausible.
            if cycles < self.max_cycles / 9 {
                self.outlier_count += 1;
            }
            let avg_bit_time = self.bit_time_sum as f32 / self.bit_time_count as f32;
            let new_baud = self.clock_hz as f32 / avg_bit_time;
            if self.baud_changed(new_baud) {
                let completeness = 1.0 - libm::expf(-(self.total as f32) / 40.0);
                let noise_ratio = self.outlier_count as f32 / self.bit_time_count as f32;
                let consistency = 1.0 - fminf(noise_ratio * 2.0, 1.0);
                let validity = completeness * consistency;
                if validity > VALIDITY_THRESHOLD {
                    self.baud = new_baud;
                    return Some(Estimate {
                        baud: libm::roundf(new_baud) as u32,
                        validity,
                    });
                }
            }
        }
        None
    }

    /// Feed a batch of samples, returning the last estimate published within it
    /// (later publishes supersede earlier ones, matching the C, which overwrites
    /// its single-slot queue).
    pub fn push_batch(&mut self, raws: &[u32]) -> Option<Estimate> {
        let mut last = None;
        for &raw in raws {
            if let Some(est) = self.push(raw) {
                last = Some(est);
            }
        }
        last
    }
}

/// `fminf` without pulling in the platform libm float-method surface.
#[inline]
fn fminf(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// PIO clock the firmware runs autobaud at (matches `autobaud.c`).
    const CLOCK_HZ: u32 = 125_000_000;

    /// PIO cycles per bit at `baud` on our `CLOCK_HZ` clock.
    fn bit_cycles(baud: u32) -> f32 {
        CLOCK_HZ as f32 / baud as f32
    }

    /// Invert [`raw_to_cycles`]: build the raw FIFO word the PIO would push for
    /// a low pulse of `cycles` PIO clocks (`cycles` is halved because the PIO
    /// counts in two-clock steps, exactly as the hardware does).
    fn cycles_to_raw(cycles: u32) -> u32 {
        u32::MAX - (cycles / 2)
    }

    /// Turn a byte stream into the low-pulse-width samples (in PIO cycles) that
    /// the edge-timer PIO would produce for an 8N1 frame at `baud`.
    ///
    /// Each frame is `start(0) d0..d7 stop(1)` with LSB-first data and an
    /// idle-high line, so a low pulse is a maximal run of `0` bits in the
    /// concatenated `[0, d0..d7, 1]` sequence. `jitter` adds a deterministic
    /// ±jitter-cycle wobble per sample to emulate sub-cycle sampling error.
    fn frames_to_samples(baud: u32, bytes: &[u8], jitter: i32) -> Vec<u32> {
        let bit = bit_cycles(baud);
        let mut samples = Vec::new();
        let mut wobble: i32 = 0;
        for &b in bytes {
            // Frame bits, oldest first: start bit, 8 data bits LSB-first, stop bit.
            let mut bits = [true; 10];
            bits[0] = false; // start bit is low
            for i in 0..8 {
                bits[1 + i] = (b >> i) & 1 != 0;
            }
            bits[9] = true; // stop bit is high
            // Emit a sample for each maximal run of low bits.
            let mut run = 0u32;
            for &high in &bits {
                if !high {
                    run += 1;
                } else if run > 0 {
                    // Cycle up/down through -jitter..=jitter for a stable but
                    // non-trivial per-sample perturbation.
                    let j = if jitter == 0 { 0 } else { (wobble % (2 * jitter + 1)) - jitter };
                    wobble = wobble.wrapping_add(1);
                    let width = (run as f32 * bit).round() as i32 + j;
                    samples.push(cycles_to_raw(width.max(2) as u32));
                    run = 0;
                }
            }
        }
        samples
    }

    /// A stream of 0xFF bytes: every frame is a single low start bit bounded by
    /// stop/idle highs, so every sample is a clean one-bit low pulse.
    #[test]
    fn clean_single_bit_stream_115200() {
        let mut est = BaudEstimator::new(CLOCK_HZ);
        let bytes = [0xFFu8; 200];
        let samples = frames_to_samples(115_200, &bytes, 0);
        let est_result = est.push_batch(&samples).expect("should publish");
        let err = (est_result.baud as f32 - 115_200.0).abs() / 115_200.0;
        assert!(err < 0.005, "baud {} off by {:.4}", est_result.baud, err);
        assert!(est_result.validity > 0.6);
    }

    /// Realistic mixed data with sub-cycle jitter, across several bauds.
    #[test]
    fn mixed_data_within_half_percent() {
        // "Hello, world!" repeated — a spread of 1..several-bit low runs.
        let msg = b"Hello, world! The quick brown fox jumps over 0123456789.\r\n";
        let mut bytes = Vec::new();
        for _ in 0..40 {
            bytes.extend_from_slice(msg);
        }
        for &baud in &[9600u32, 19200, 57600, 115_200, 230_400, 921_600] {
            let mut est = BaudEstimator::new(CLOCK_HZ);
            let samples = frames_to_samples(baud, &bytes, 1);
            let est_result = est
                .push_batch(&samples)
                .unwrap_or_else(|| panic!("no estimate at baud {baud}"));
            let err = (est_result.baud as f32 - baud as f32).abs() / baud as f32;
            assert!(
                err < 0.005,
                "baud {baud}: estimated {} off by {:.4}",
                est_result.baud,
                err
            );
        }
    }

    /// Pure line noise (random-ish short/long pulses, no consistent minimum)
    /// must not publish a confident baud.
    #[test]
    fn noise_does_not_publish() {
        let mut est = BaudEstimator::new(CLOCK_HZ);
        // Widths scattered with no value reaching 5% frequency.
        let mut published = None;
        for i in 0..300u32 {
            let width = 100 + (i.wrapping_mul(2_654_435_761) % 4000);
            if let Some(e) = est.push(cycles_to_raw(width)) {
                published = Some(e);
            }
        }
        assert!(published.is_none(), "noise produced {published:?}");
    }

    /// The empty-slot sentinel quirk: a zero-cycle sample is dropped, not
    /// counted (mirrors the C).
    #[test]
    fn zero_cycle_sample_is_dropped() {
        let mut est = BaudEstimator::new(CLOCK_HZ);
        // raw = u32::MAX -> cycles 0 -> never stored.
        assert_eq!(est.push(u32::MAX), None);
        assert_eq!(est.get_count(0), 0);
    }

    #[test]
    fn raw_cycle_roundtrip() {
        for &c in &[2u32, 100, 1085, 13020, 65536] {
            assert_eq!(raw_to_cycles(cycles_to_raw(c)), c & !1);
        }
    }
}
