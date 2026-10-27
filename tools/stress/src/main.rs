//! `mpstress` — concurrent-lane stress test and benchmark for rustprobe.
//!
//! One thread per lane (a lane is one CMSIS-DAP interface of the probe, i.e.
//! one configured SWD probe). Threads open their sessions, meet at a barrier,
//! then run their mode until the deadline. Every operation is non-halting and
//! leaves the target untouched, so it is safe against running firmware:
//!
//! * `bulk` — block-read the boot ROM and compare it with a golden copy read
//!   (twice) at startup. Measures throughput, catches data corruption.
//! * `word` — write a random word to core 0's DCRDR (a debug-only scratch
//!   register) and read it back. Measures command round trip.
//! * `churn` — open probe + set speed + attach + one ROM read + drop, in a
//!   loop, timing each phase. Exercises interface open/close.
//! * `uptime` — poll the target's free-running µs timer; a backwards step
//!   means the target rebooted.
//! * `hold` — open + attach once, then hold the session idle.
//!
//! Summary on stderr, one JSON object per lane on stdout. With `--trace`, each
//! lane keeps a ring of its recent probe-rs trace lines and dumps it to a file
//! on every error or slow op (see README).

use std::collections::BTreeMap;
use std::str::FromStr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Barrier, OnceLock};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{bail, Context, Result};
use clap::{Parser, ValueEnum};
use probe_rs::probe::{list::Lister, DebugProbeSelector};
use probe_rs::{Core, MemoryInterface, Permissions, Session};

const RUSTPROBE_VID: u16 = 0x2e8a;
const RUSTPROBE_PID: u16 = 0x000c;
/// Core debug register DCRDR: debug-only scratch, safe to write while running.
const DCRDR: u64 = 0xE000_EDF8;

/// Target chip: what to attach as, and where the test regions live.
#[derive(Clone, Copy, ValueEnum)]
enum Chip {
    Rp2350,
    Rp2040,
}

impl Chip {
    fn probe_rs_name(self) -> &'static str {
        match self {
            Chip::Rp2350 => "RP235x",
            Chip::Rp2040 => "RP2040",
        }
    }
    /// Boot ROM at 0x0 (immutable, identical across parts of one revision).
    fn rom_len(self) -> usize {
        match self {
            Chip::Rp2350 => 32 * 1024,
            Chip::Rp2040 => 16 * 1024,
        }
    }
    /// Base of the µs TIMER block (TIMERAWH at +0x24, TIMERAWL at +0x28).
    fn timer_base(self) -> u64 {
        match self {
            Chip::Rp2350 => 0x400b_0000, // TIMER0
            Chip::Rp2040 => 0x4005_4000,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, ValueEnum)]
enum Mode {
    Bulk,
    Word,
    Churn,
    Uptime,
    Hold,
}

impl FromStr for Mode {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        <Mode as ValueEnum>::from_str(s, true)
    }
}

impl Mode {
    fn name(self) -> &'static str {
        match self {
            Mode::Bulk => "bulk",
            Mode::Word => "word",
            Mode::Churn => "churn",
            Mode::Uptime => "uptime",
            Mode::Hold => "hold",
        }
    }
}

#[derive(Parser)]
#[command(about = "Concurrent-lane stress test / benchmark for rustprobe")]
struct Args {
    /// Lanes as `LANE[:MODE[:KHZ]]`, comma separated, e.g. `0,1:word,2:churn:100`.
    /// LANE is the probe index (= its CMSIS-DAP interface number).
    #[arg(long)]
    lanes: String,
    /// Mode for lanes that don't name one.
    #[arg(long, value_enum, default_value_t = Mode::Bulk)]
    mode: Mode,
    /// SWD clock (kHz) for lanes that don't name one.
    #[arg(long, default_value_t = 1000)]
    khz: u32,
    /// Run time after all lanes have attached.
    #[arg(long, default_value_t = 20)]
    secs: u64,
    /// Bytes per bulk read.
    #[arg(long, default_value_t = 4096)]
    size: usize,
    /// Target chip on every lane.
    #[arg(long, value_enum, default_value_t = Chip::Rp2350)]
    chip: Chip,
    /// USB serial of the probe (`rustprobe list`); needed only when several
    /// rustprobes are attached.
    #[arg(long)]
    serial: Option<String>,
    /// Report any single op (churn: whole cycle) slower than this.
    #[arg(long, default_value_t = 500)]
    slow_ms: u64,
    /// Free-form tag copied into the JSON.
    #[arg(long, default_value = "")]
    label: String,
    /// Progress line interval, s (0 = off).
    #[arg(long, default_value_t = 5)]
    progress: u64,
    /// Keep a per-lane ring of probe-rs trace lines matching this filter
    /// (e.g. `probe_rs::probe::cmsisdap=trace`) and dump it on every error
    /// and slow op.
    #[arg(long)]
    trace: Option<String>,
    #[arg(long, default_value = "stress-traces")]
    trace_dir: String,
}

struct LaneSpec {
    lane: u8,
    mode: Mode,
    khz: u32,
}

fn parse_lanes(args: &Args) -> Result<Vec<LaneSpec>> {
    args.lanes
        .split(',')
        .map(|s| {
            let mut it = s.trim().split(':');
            let lane = it.next().unwrap().parse().with_context(|| format!("lane number in {s:?}"))?;
            let mode = match it.next().filter(|m| !m.is_empty()) {
                Some(m) => m.parse().map_err(|e: String| anyhow::anyhow!(e))?,
                None => args.mode,
            };
            let khz = match it.next() {
                Some(k) => k.parse().with_context(|| format!("kHz in {s:?}"))?,
                None => args.khz,
            };
            Ok(LaneSpec { lane, mode, khz })
        })
        .collect()
}

/// The probe's USB serial: the one given, or the only rustprobe attached.
fn resolve_serial(given: Option<&str>) -> Result<String> {
    if let Some(s) = given {
        return Ok(s.to_string());
    }
    let mut serials: Vec<String> = Lister::new()
        .list_all()
        .into_iter()
        .filter(|p| p.vendor_id == RUSTPROBE_VID && p.product_id == RUSTPROBE_PID)
        .filter_map(|p| p.serial_number)
        .collect();
    serials.sort();
    serials.dedup();
    match serials.as_slice() {
        [one] => Ok(one.clone()),
        [] => bail!("no rustprobe ({RUSTPROBE_VID:04x}:{RUSTPROBE_PID:04x}) attached"),
        many => bail!("several rustprobes attached, pick one with --serial: {many:?}"),
    }
}

/// Per-thread ring of the most recent trace lines; dumped on error.
mod ring {
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::io::Write;

    const DEPTH: usize = 600;
    thread_local! {
        static RING: RefCell<VecDeque<Vec<u8>>> = RefCell::new(VecDeque::with_capacity(DEPTH));
    }

    /// `MakeWriter` target for tracing-subscriber: appends to this thread's ring.
    pub struct Writer;
    impl Write for Writer {
        fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
            push(b.to_vec());
            Ok(b.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    pub fn push(line: Vec<u8>) {
        RING.with(|r| {
            let mut r = r.borrow_mut();
            if r.len() >= DEPTH {
                r.pop_front();
            }
            r.push_back(line);
        });
    }

    pub fn dump(path: &str) {
        let result = RING.with(|r| -> std::io::Result<()> {
            let mut f = std::fs::File::create(path)?;
            r.borrow().iter().try_for_each(|l| f.write_all(l))
        });
        if let Err(e) = result {
            eprintln!("trace dump to {path} failed: {e}");
        }
    }
}

static TRACE_DIR: OnceLock<String> = OnceLock::new();

#[derive(Default)]
struct Live {
    bytes: AtomicU64,
    errs: AtomicU64,
}

#[derive(Default)]
struct LaneResult {
    lane: u8,
    mode: &'static str,
    khz: u32,
    setup_err: Option<String>,
    ops: u64,
    bytes: u64,
    errors: u64,
    mismatches: u64,
    reconnects: u64,
    reconnect_fail: u64,
    elapsed: f64,
    lat_us: Vec<u32>,
    err_kinds: BTreeMap<String, u64>,
    first_errs: Vec<(f64, String)>,
    /// (wall clock, description) of slow ops and detected target reboots.
    events: Vec<(f64, String)>,
}

struct Ctx {
    serial: String,
    chip: Chip,
    size: usize,
    secs: u64,
    slow_ms: u64,
}

impl Ctx {
    fn selector(&self, lane: u8) -> DebugProbeSelector {
        DebugProbeSelector {
            vendor_id: RUSTPROBE_VID,
            product_id: RUSTPROBE_PID,
            interface: Some(lane),
            serial_number: Some(self.serial.clone()),
        }
    }

    fn open(&self, lane: u8, khz: u32) -> Result<Session> {
        let mut probe = Lister::new().open(self.selector(lane)).context("open probe")?;
        probe.set_speed(khz).context("set speed")?;
        probe.attach(self.chip.probe_rs_name(), Permissions::default()).context("attach")
    }
}

/// Word-wide block read (probe-rs `read_8` may use byte transfers).
fn read_words(core: &mut Core, addr: u64, out: &mut [u8]) -> Result<()> {
    let mut words = vec![0u32; out.len() / 4];
    core.read_32(addr, &mut words)?;
    for (c, w) in out.chunks_exact_mut(4).zip(&words) {
        c.copy_from_slice(&w.to_le_bytes());
    }
    Ok(())
}

/// 64-bit µs since target boot from the raw timer halves (no latching, so the
/// target firmware's own TIMEHR/TIMELR reads are not disturbed).
fn target_uptime_us(core: &mut Core, timer_base: u64) -> Result<u64> {
    loop {
        let hi = core.read_word_32(timer_base + 0x24)?;
        let lo = core.read_word_32(timer_base + 0x28)?;
        if core.read_word_32(timer_base + 0x24)? == hi {
            return Ok((hi as u64) << 32 | lo as u64);
        }
    }
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0 as u32
    }
}

/// Error text with digit runs collapsed, so identical failures bucket together.
fn err_kind(e: &anyhow::Error) -> String {
    let s = format!("{e:#}");
    let mut out = String::new();
    let mut in_number = false;
    for c in s.chars() {
        if c.is_ascii_hexdigit() && (c.is_ascii_digit() || in_number) {
            if !in_number {
                out.push('#');
            }
            in_number = true;
        } else {
            in_number = false;
            out.push(c);
        }
    }
    out
}

/// Wall clock, s since the epoch — comparable across lanes, trace dumps and
/// usbmon captures.
fn wall() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64()
}

fn dump_trace(name: String) {
    if let Some(dir) = TRACE_DIR.get() {
        ring::dump(&format!("{dir}/{name}"));
    }
}

fn record_error(r: &mut LaneResult, live: &Live, op: &str, e: &anyhow::Error) {
    ring::push(format!("=== ERROR @{:.6} {op}: {e:#}\n", wall()).into_bytes());
    dump_trace(format!("error-L{}-{}.log", r.lane, r.errors));
    r.errors += 1;
    live.errs.fetch_add(1, Ordering::Relaxed);
    *r.err_kinds.entry(format!("{op}: {}", err_kind(e))).or_default() += 1;
    if r.first_errs.len() < 20 {
        r.first_errs.push((wall(), format!("{op}: {e:#}")));
    }
}

fn record_event(r: &mut LaneResult, what: String) {
    ring::push(format!("=== EVENT @{:.6} {what}\n", wall()).into_bytes());
    r.events.push((wall(), what));
    dump_trace(format!("event-L{}-{}.log", r.lane, r.events.len()));
}

/// One churn cycle with each phase timed, so slow outliers can be located.
fn churn_cycle(ctx: &Ctx, lane: u8, khz: u32) -> Result<(u64, Option<String>)> {
    let mut ms = [0u128; 5];
    let t = Instant::now();
    let mut probe = Lister::new().open(ctx.selector(lane)).context("open probe")?;
    ms[0] = t.elapsed().as_millis();
    let t = Instant::now();
    probe.set_speed(khz).context("set speed")?;
    ms[1] = t.elapsed().as_millis();
    let t = Instant::now();
    let mut session = probe.attach(ctx.chip.probe_rs_name(), Permissions::default()).context("attach")?;
    ms[2] = t.elapsed().as_millis();
    let t = Instant::now();
    session.core(0)?.read_word_32(0)?;
    ms[3] = t.elapsed().as_millis();
    let t = Instant::now();
    drop(session);
    ms[4] = t.elapsed().as_millis();
    let slow = (ms.iter().sum::<u128>() > ctx.slow_ms as u128).then(|| {
        format!("slow churn cycle: open={} speed={} attach={} read={} detach={} ms", ms[0], ms[1], ms[2], ms[3], ms[4])
    });
    Ok((4, slow))
}

fn lane_thread(ctx: Arc<Ctx>, spec: LaneSpec, live: Arc<Live>, start: Arc<Barrier>) -> LaneResult {
    let LaneSpec { lane, mode, khz } = spec;
    let mut r = LaneResult { lane, mode: mode.name(), khz, ..Default::default() };
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15 ^ ((lane as u64 + 1) << 32));
    let rom_len = ctx.chip.rom_len();

    // Setup before the barrier: the session, plus the golden ROM for bulk
    // (read twice; the two reads must agree).
    let mut session: Option<Session> = None;
    let mut golden = vec![0u8; rom_len];
    if mode != Mode::Churn {
        let setup = (|| -> Result<Session> {
            let mut s = ctx.open(lane, khz)?;
            if mode == Mode::Bulk {
                let mut again = vec![0u8; rom_len];
                let mut core = s.core(0)?;
                read_words(&mut core, 0, &mut golden)?;
                read_words(&mut core, 0, &mut again)?;
                if golden != again {
                    bail!("golden ROM reads disagree");
                }
            }
            Ok(s)
        })();
        match setup {
            Ok(s) => session = Some(s),
            Err(e) => r.setup_err = Some(format!("{e:#}")),
        }
    }
    start.wait();
    if r.setup_err.is_some() {
        return r;
    }

    let t0 = Instant::now();
    let deadline = t0 + Duration::from_secs(ctx.secs);
    let mut consecutive_errs = 0u32;
    let mut last_uptime: Option<u64> = None;
    let mut buf = vec![0u8; ctx.size.min(rom_len)];
    let mut off = (lane as usize * 4096) % rom_len;

    while Instant::now() < deadline {
        if mode == Mode::Hold {
            thread::sleep(Duration::from_millis(100));
            continue;
        }
        if mode != Mode::Churn && session.is_none() {
            match ctx.open(lane, khz) {
                Ok(s) => {
                    session = Some(s);
                    r.reconnects += 1;
                }
                Err(e) => {
                    r.reconnect_fail += 1;
                    record_error(&mut r, &live, "reopen", &e);
                    thread::sleep(Duration::from_millis(200));
                    continue;
                }
            }
        }

        let t = Instant::now();
        let res: Result<(u64, Option<String>)> = (|| match mode {
            Mode::Bulk => {
                let n = buf.len().min(rom_len - off);
                read_words(&mut session.as_mut().unwrap().core(0)?, off as u64, &mut buf[..n])?;
                let want = &golden[off..off + n];
                if let Some(first) = buf[..n].iter().zip(want).position(|(a, b)| a != b) {
                    let diff = buf[..n].iter().zip(want).filter(|(a, b)| a != b).count();
                    bail!("MISMATCH {diff} bytes, first at rom+{:#x}", off + first);
                }
                off = (off + n) % rom_len;
                Ok((n as u64, None))
            }
            Mode::Word => {
                let v = rng.next();
                let mut core = session.as_mut().unwrap().core(0)?;
                core.write_word_32(DCRDR, v)?;
                let got = core.read_word_32(DCRDR)?;
                if got != v {
                    bail!("MISMATCH DCRDR wrote {v:#010x} read {got:#010x}");
                }
                Ok((8, None))
            }
            Mode::Churn => churn_cycle(&ctx, lane, khz),
            Mode::Uptime => {
                thread::sleep(Duration::from_millis(100));
                let mut core = session.as_mut().unwrap().core(0)?;
                let now = target_uptime_us(&mut core, ctx.chip.timer_base())?;
                let reboot = last_uptime
                    .filter(|&prev| now < prev)
                    .map(|prev| format!("target REBOOT: uptime {prev} us -> {now} us"));
                last_uptime = Some(now);
                Ok((8, reboot))
            }
            Mode::Hold => unreachable!(),
        })();

        let elapsed = t.elapsed();
        if mode != Mode::Churn && elapsed.as_millis() > ctx.slow_ms as u128 {
            let outcome = if res.is_ok() { "ok" } else { "error" };
            record_event(&mut r, format!("slow {} op: {} ms ({outcome})", mode.name(), elapsed.as_millis()));
        }
        match res {
            Ok((n, event)) => {
                if let Some(event) = event {
                    record_event(&mut r, event);
                }
                r.lat_us.push(elapsed.as_micros() as u32);
                r.ops += 1;
                r.bytes += n;
                live.bytes.fetch_add(n, Ordering::Relaxed);
                consecutive_errs = 0;
            }
            Err(e) => {
                if e.to_string().starts_with("MISMATCH") {
                    r.mismatches += 1;
                }
                record_error(&mut r, &live, mode.name(), &e);
                consecutive_errs += 1;
                // Two strikes on a held session: drop it and re-open.
                if consecutive_errs >= 2 && mode != Mode::Churn {
                    session = None;
                    consecutive_errs = 0;
                }
                if mode == Mode::Churn {
                    thread::sleep(Duration::from_millis(20));
                }
            }
        }
    }
    r.elapsed = t0.elapsed().as_secs_f64();
    r
}

fn percentile(sorted: &[u32], p: f64) -> u32 {
    if sorted.is_empty() {
        return 0;
    }
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

fn main() -> Result<()> {
    let args = Args::parse();
    let specs = parse_lanes(&args)?;
    if let Some(filter) = &args.trace {
        std::fs::create_dir_all(&args.trace_dir).with_context(|| format!("creating {}", args.trace_dir))?;
        tracing_subscriber::fmt()
            .with_env_filter(tracing_subscriber::EnvFilter::new(filter))
            .with_writer(|| ring::Writer)
            .with_ansi(false)
            .with_thread_names(true)
            .init();
        TRACE_DIR.set(args.trace_dir.clone()).unwrap();
    }
    let ctx = Arc::new(Ctx {
        serial: resolve_serial(args.serial.as_deref())?,
        chip: args.chip,
        size: args.size,
        secs: args.secs,
        slow_ms: args.slow_ms,
    });

    let start = Arc::new(Barrier::new(specs.len() + 1));
    let lives: Vec<Arc<Live>> = specs.iter().map(|_| Arc::new(Live::default())).collect();
    let lane_ids: Vec<u8> = specs.iter().map(|s| s.lane).collect();
    let handles: Vec<_> = specs
        .into_iter()
        .zip(&lives)
        .map(|(spec, live)| {
            let (ctx, live, start) = (ctx.clone(), live.clone(), start.clone());
            thread::Builder::new()
                .name(format!("L{}", spec.lane))
                .spawn(move || lane_thread(ctx, spec, live, start))
                .unwrap()
        })
        .collect();
    start.wait();
    let t0 = Instant::now();

    if args.progress > 0 {
        let lives = lives.clone();
        let lane_ids = lane_ids.clone();
        let (secs, every) = (args.secs, args.progress);
        thread::spawn(move || {
            let mut prev = vec![0u64; lives.len()];
            let mut t = 0;
            while t + every <= secs {
                thread::sleep(Duration::from_secs(every));
                t += every;
                let mut line = format!("[{t:>4}s]");
                for ((lane, l), p) in lane_ids.iter().zip(&lives).zip(&mut prev) {
                    let b = l.bytes.load(Ordering::Relaxed);
                    let kbs = (b - *p) as f64 / every as f64 / 1024.0;
                    line += &format!(" L{lane}:{kbs:.1}KB/s e{}", l.errs.load(Ordering::Relaxed));
                    *p = b;
                }
                eprintln!("{line}");
            }
        });
    }

    let mut results: Vec<LaneResult> = handles.into_iter().map(|h| h.join().unwrap()).collect();
    let wall_s = t0.elapsed().as_secs_f64();
    eprintln!("== lanes={} khz={} size={} secs={} label={}", args.lanes, args.khz, args.size, args.secs, args.label);
    let mut aggregate = 0.0;
    for r in &mut results {
        r.lat_us.sort_unstable();
        let kbs = r.bytes as f64 / r.elapsed.max(1e-9) / 1024.0;
        aggregate += kbs;
        let (p50, p99, max) = (percentile(&r.lat_us, 0.5), percentile(&r.lat_us, 0.99), percentile(&r.lat_us, 1.0));
        eprintln!(
            "L{} {:<6} {:>5}k ops={:<7} {:>7.1} KB/s  op/s={:<7.1} lat p50={p50}us p99={p99}us max={max}us  err={} mism={} reconn={}/{}fail{}",
            r.lane,
            r.mode,
            r.khz,
            r.ops,
            kbs,
            r.ops as f64 / r.elapsed.max(1e-9),
            r.errors,
            r.mismatches,
            r.reconnects,
            r.reconnect_fail,
            r.setup_err.as_ref().map(|e| format!("  SETUP: {e}")).unwrap_or_default()
        );
        for (t, what) in &r.events {
            eprintln!("     @{t:.3} {what}");
        }
        for (kind, n) in &r.err_kinds {
            eprintln!("     {n:>5} × {kind}");
        }
        let json = serde_json::json!({
            "label": args.label, "lanes": args.lanes, "size": args.size,
            "lane": r.lane, "mode": r.mode, "khz": r.khz, "setup_err": r.setup_err,
            "ops": r.ops, "bytes": r.bytes, "elapsed": r.elapsed, "kbs": kbs,
            "lat_p50_us": p50, "lat_p99_us": p99, "lat_max_us": max,
            "errors": r.errors, "mismatches": r.mismatches,
            "reconnects": r.reconnects, "reconnect_fail": r.reconnect_fail,
            "err_kinds": r.err_kinds, "first_errs": r.first_errs, "events": r.events,
        });
        println!("{json}");
    }
    eprintln!("aggregate {aggregate:.1} KB/s over {wall_s:.1}s");
    Ok(())
}
