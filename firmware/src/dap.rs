//! `dap_async::Driver` over the PIO SWD engine.
//!
//! The transfer sequence is the reference CMSIS-DAP `SWD_Transfer` as the C
//! debugprobe's `sw_dp_pio.c` runs it on the PIO command FIFO: request
//! packet, turnaround + ACK, then the data phase, with the configured
//! turnaround length, WAIT/FAULT data phase and idle cycles. FIFO entries
//! that do not depend on a result are queued together, so a read costs one
//! wait for the ACK and one for the data instead of a round trip per entry.
//!
//! # Hot path
//!
//! Transfers are hand-written futures ([`TransferFuture`], and
//! [`ReadBlockFuture`] / [`WriteBlockFuture`] for `DAP_TransferBlock`), not
//! `async fn`s: the hot path is a synchronous loop over a small plan of FIFO
//! ops with no generator state, and a whole block runs in one poll unless a
//! FIFO op blocks. Measured on the RP2350 (issue #5), the `async fn` form of
//! the same sequence cost ~5 µs per transfer in future construction and
//! state spills, more than the wire time of a word at 20 MHz. FIFO words
//! move through [`FifoRegs`] directly; a `dyn` call per word was a third of
//! the driver's CPU time.
//!
//! # Waiting
//!
//! A blocked FIFO op spins for a budget that depends on the SWCLK, then
//! yields cooperatively (`wake_by_ref` + `Pending`): the executor runs the
//! other lanes and polls this one again on its next round. There is no
//! interrupt in the path. Measured, an interrupt-driven wake cost more than
//! the waits it was meant to cover at 4 MHz, and a cooperative round trip
//! through the executor is ~2 µs. Above [`FAST_KHZ`] every wait is shorter
//! than that, so the lane spins through them (as the blocking engine did);
//! below it the lane yields and other lanes' CPU work overlaps its wire
//! time. A lane running alone never yields: the waits would only come
//! back to it.

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use dap_async::swd::{ack, packet, req};
use dap_async::{Driver, Pins, Transfer, TransferParams};
use embassy_futures::yield_now;
use embassy_time::{Duration, Instant, Timer};
use portable_atomic::{AtomicU32, Ordering};

use crate::instances::DynEngine;
use crate::swd::{CmdAddrs, FifoRegs};

/// Lanes currently inside `process_command`; maintained by `main.rs`.
pub static ACTIVE_LANES: AtomicU32 = AtomicU32::new(0);

/// Polls a future from a RAM-resident function. The future's generated
/// `poll` (and everything it inlines) ends up in that function, so a whole
/// `process_command` runs from SRAM instead of the XIP cache. Measured with
/// the stall-diag XIP counters: ~85 flash-cache misses per `DAP_Transfer`
/// command from flash, costing more than the command's own work (issue #6).
pub struct InRam<F>(pub F);

impl<F: Future> Future for InRam<F> {
    type Output = F::Output;

    #[unsafe(link_section = ".data.swd_hot")]
    #[inline(never)]
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<F::Output> {
        // Safety: structural pinning of the only field.
        let inner = unsafe { self.map_unchecked_mut(|s| &mut s.0) };
        inner.poll(cx)
    }
}

/// At or above this SWCLK a wait never exceeds a cooperative yield's cost:
/// spin through everything.
const FAST_KHZ: u32 = 6000;
/// FIFO polls before yielding when spinning through waits (a 32-bit word
/// at 6 MHz is ~5.3 µs).
const SPIN_FAST: u32 = 160;
/// FIFO polls before yielding at lower clocks: one re-check, then yield.
const SPIN_SLOW: u32 = 2;
/// With other lanes active, a lane that spins through its waits still
/// yields once every this many transfers, so a long command (a 512-byte
/// block is ~126 words, ~340 µs at 20 MHz) bounds the others' wait to a
/// few words instead of a whole command.
const YIELD_EVERY: u32 = 8;

/// The FIFO operations of one stage of a transfer: words to push, with a
/// bit set in `pulls` for each slot that is a pull instead. Long enough for
/// the biggest stage (a write with idle cycles: 8 pushes).
#[derive(Clone, Copy)]
struct Plan {
    ops: [u32; 10],
    pulls: u16,
    len: u8,
}

impl Plan {
    const fn new() -> Self {
        Self { ops: [0; 10], pulls: 0, len: 0 }
    }
    #[inline(always)]
    fn push(&mut self, v: u32) {
        self.ops[self.len as usize] = v;
        self.len += 1;
    }
    #[inline(always)]
    fn pull(&mut self) {
        self.pulls |= 1 << self.len;
        self.len += 1;
    }
    #[inline(always)]
    fn is_pull(&self, i: usize) -> bool {
        self.pulls & (1 << i) != 0
    }
}

/// Progress through a [`Plan`]: next op, and the words pulled so far.
#[derive(Clone, Copy)]
struct Cursor {
    idx: u8,
    n_out: u8,
    out: [u32; 2],
}

impl Cursor {
    const fn new() -> Self {
        Self { idx: 0, n_out: 0, out: [0; 2] }
    }
}

/// One probe's driver: the type-erased PIO engine plus the async FIFO
/// protocol over it.
pub struct PioSwd {
    bus: DynEngine,
    /// The engine's FIFO registers, for direct access in the hot loop.
    fifo: FifoRegs,
    /// The engine's command routine addresses.
    addrs: CmdAddrs,
    /// FIFO polls before yielding, for the current SWCLK.
    spin: u32,
    /// Transfers completed since this lane last yielded.
    run: u32,
}

impl PioSwd {
    pub fn new(bus: DynEngine) -> Self {
        let fifo = bus.fifo();
        let addrs = bus.addrs();
        let mut this = Self { bus, fifo, addrs, spin: 0, run: 0 };
        this.update_spin();
        this
    }

    fn update_spin(&mut self) {
        self.spin = if self.bus.swclk_khz() >= FAST_KHZ { SPIN_FAST } else { SPIN_SLOW };
    }

    #[inline(always)]
    fn write_cmd(&self, bits: u32) -> u32 {
        CmdAddrs::cmd(bits, true, self.addrs.write)
    }
    #[inline(always)]
    fn read_cmd(&self, bits: u32) -> u32 {
        CmdAddrs::cmd(bits, false, self.addrs.read)
    }
    #[inline(always)]
    fn hiz_cmd(&self, bits: u32) -> u32 {
        CmdAddrs::cmd(bits, false, self.addrs.turnaround)
    }
    #[inline(always)]
    fn dir_cmd(&self, out: bool) -> u32 {
        CmdAddrs::cmd(1, out, self.addrs.skip)
    }

    /// Run `plan` from `cur` as far as the FIFOs allow within the spin
    /// budget. Returns true when the plan is complete; false when the op at
    /// `cur.idx` is blocked.
    #[inline(always)]
    fn step(&mut self, plan: &Plan, cur: &mut Cursor) -> bool {
        let spin = if ACTIVE_LANES.load(Ordering::Relaxed) <= 1 { SPIN_FAST } else { self.spin };
        while (cur.idx as usize) < plan.len as usize {
            let i = cur.idx as usize;
            if plan.is_pull(i) {
                let mut got = self.fifo.try_pull();
                let mut n = spin;
                while got.is_none() && n > 0 {
                    got = self.fifo.try_pull();
                    n -= 1;
                }
                match got {
                    Some(v) => {
                        cur.out[cur.n_out as usize] = v;
                        cur.n_out += 1;
                    }
                    None => return false,
                }
            } else {
                let v = plan.ops[i];
                let mut ok = self.fifo.try_push(v);
                let mut n = spin;
                while !ok && n > 0 {
                    ok = self.fifo.try_push(v);
                    n -= 1;
                }
                if !ok {
                    return false;
                }
            }
            cur.idx += 1;
        }
        true
    }

    /// Run a whole plan, yielding while blocked. For the cold paths
    /// (sequences, direction changes).
    async fn exec(&mut self, plan: &Plan) -> [u32; 2] {
        let mut cur = Cursor::new();
        while !self.step(plan, &mut cur) {
            crate::stall_diag::fifo_yield();
            yield_now().await;
        }
        cur.out
    }

    /// Clock out `bits` (1..=256) bits, LSB first, driving SWDIO.
    async fn write_bits(&mut self, bits: u32, data: u32) {
        let mut plan = Plan::new();
        plan.push(self.write_cmd(bits));
        plan.push(data);
        self.exec(&plan).await;
    }

    /// Clock in `bits` (1..=32) bits, LSB first, SWDIO released; the result
    /// is right-aligned.
    async fn read_bits(&mut self, bits: u32) -> u32 {
        let mut plan = Plan::new();
        plan.push(self.read_cmd(bits));
        plan.pull();
        self.exec(&plan).await[0] >> (32 - bits)
    }

    /// Switch the SWDIO direction and wait until the SM has applied it
    /// (TX FIFO drained, SM stalled on `pull`).
    async fn set_dir(&mut self, out: bool) {
        let mut plan = Plan::new();
        plan.push(self.dir_cmd(out));
        self.exec(&plan).await;
        // `tx_stalled` is read-then-clear: the first call drops a stale flag.
        self.bus.tx_stalled();
        while !self.bus.tx_stalled() {
            yield_now().await;
        }
    }

    /// Stage 1 of a transfer: request packet, then turnaround + ACK.
    #[inline(always)]
    fn plan_request(&self, request: u8, trn: u32) -> Plan {
        let mut plan = Plan::new();
        plan.push(self.write_cmd(8));
        plan.push(packet(request) as u32);
        plan.push(self.read_cmd(trn + 3));
        plan.pull();
        plan
    }

    /// Stage 2 of a transfer, given the ACK: the data phase per the
    /// reference `SWD_Transfer`.
    #[inline(always)]
    fn plan_data(&self, ack: u8, rnw: bool, data: u32, p: &TransferParams) -> Plan {
        let trn = p.turnaround as u32;
        let mut plan = Plan::new();
        let hiz = |plan: &mut Plan| {
            if trn > 0 {
                plan.push(self.hiz_cmd(trn));
                plan.push(0);
            }
        };
        match ack {
            ack::OK => {
                if rnw {
                    // Data, parity, turnaround back to host-driven, idle.
                    plan.push(self.read_cmd(32));
                    plan.push(self.read_cmd(1));
                    hiz(&mut plan);
                    if p.idle_cycles > 0 {
                        plan.push(self.write_cmd(p.idle_cycles as u32));
                        plan.push(0);
                    }
                    plan.pull();
                    plan.pull();
                } else {
                    hiz(&mut plan);
                    plan.push(self.write_cmd(32));
                    plan.push(data);
                    plan.push(self.write_cmd(1));
                    plan.push(data.count_ones() & 1);
                    if p.idle_cycles > 0 {
                        plan.push(self.write_cmd(p.idle_cycles as u32));
                        plan.push(0);
                    }
                }
            }
            ack::WAIT | ack::FAULT => {
                if p.data_phase && rnw {
                    plan.push(self.read_cmd(33));
                    plan.pull();
                }
                hiz(&mut plan);
                if p.data_phase && !rnw {
                    plan.push(self.write_cmd(32));
                    plan.push(0);
                    plan.push(self.write_cmd(1));
                    plan.push(0);
                }
            }
            _ => {
                // Protocol error: back off a full data phase on the line.
                plan.push(self.read_cmd(trn + 32 + 1));
                plan.pull();
            }
        }
        plan
    }
}

/// Where a transfer is.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Stage {
    Start,
    Request,
    Data,
}

/// One SWD transfer's progress; resumable after a yield, reusable after
/// completion.
struct TransferState {
    stage: Stage,
    ack: u8,
    plan: Plan,
    cur: Cursor,
}

impl TransferState {
    const fn new() -> Self {
        Self { stage: Stage::Start, ack: 0, plan: Plan::new(), cur: Cursor::new() }
    }

    /// Drive the transfer of `request` (`data` for writes). `Pending` means
    /// a FIFO op is blocked and the task has been re-queued. On `Ready` the
    /// state is reset for the next transfer.
    #[inline(always)]
    fn poll(&mut self, drv: &mut PioSwd, request: u8, data: u32, p: &TransferParams, cx: &mut Context<'_>) -> Poll<Transfer> {
        let rnw = request & req::RNW != 0;
        loop {
            match self.stage {
                Stage::Start => {
                    if drv.run >= YIELD_EVERY {
                        drv.run = 0;
                        if ACTIVE_LANES.load(Ordering::Relaxed) > 1 {
                            crate::stall_diag::fifo_yield();
                            cx.waker().wake_by_ref();
                            return Poll::Pending;
                        }
                    }
                    self.plan = drv.plan_request(request, p.turnaround as u32);
                    self.cur = Cursor::new();
                    self.stage = Stage::Request;
                }
                Stage::Request => {
                    if !drv.step(&self.plan, &mut self.cur) {
                        crate::stall_diag::fifo_yield();
                        cx.waker().wake_by_ref();
                        return Poll::Pending;
                    }
                    let trn = p.turnaround as u32;
                    self.ack = ((self.cur.out[0] >> (32 - (trn + 3))) >> trn) as u8 & 7;
                    self.plan = drv.plan_data(self.ack, rnw, data, p);
                    self.cur = Cursor::new();
                    self.stage = Stage::Data;
                }
                Stage::Data => {
                    if !drv.step(&self.plan, &mut self.cur) {
                        crate::stall_diag::fifo_yield();
                        cx.waker().wake_by_ref();
                        return Poll::Pending;
                    }
                    let t = if self.ack == ack::OK && rnw {
                        let data = self.cur.out[0];
                        let parity = self.cur.out[1] >> 31;
                        let status = if (data.count_ones() ^ parity) & 1 != 0 { ack::ERROR } else { ack::OK };
                        Transfer { ack: status, data }
                    } else {
                        Transfer { ack: self.ack, data: 0 }
                    };
                    self.stage = Stage::Start;
                    drv.run += 1;
                    return Poll::Ready(t);
                }
            }
        }
    }
}

/// A transfer plus the `DAP_TransferConfigure` WAIT retry loop, as a
/// resumable state: `retry` counts attempts left for the current transfer.
struct RetryState {
    xfer: TransferState,
    retry: u16,
}

impl RetryState {
    const fn new(wait_retries: u16) -> Self {
        Self { xfer: TransferState::new(), retry: wait_retries }
    }

    /// Like [`TransferState::poll`], retrying on WAIT up to `wait_retries`
    /// times; the retry budget is reset on `Ready`.
    #[inline(always)]
    fn poll(
        &mut self,
        drv: &mut PioSwd,
        request: u8,
        data: u32,
        p: &TransferParams,
        wait_retries: u16,
        cx: &mut Context<'_>,
    ) -> Poll<Transfer> {
        loop {
            let t = core::task::ready!(self.xfer.poll(drv, request, data, p, cx));
            if t.ack == ack::WAIT && self.retry > 0 {
                self.retry -= 1;
                continue;
            }
            self.retry = wait_retries;
            return Poll::Ready(t);
        }
    }
}

/// One SWD transfer in flight (`Driver::transfer`).
pub struct TransferFuture<'a> {
    drv: &'a mut PioSwd,
    request: u8,
    data: u32,
    params: TransferParams,
    st: TransferState,
}

impl Future for TransferFuture<'_> {
    type Output = Transfer;

    #[unsafe(link_section = ".data.swd_hot")]
    #[inline(never)]
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Transfer> {
        let this = self.get_mut();
        this.st.poll(this.drv, this.request, this.data, &this.params, cx)
    }
}

/// One SWD transfer with WAIT retries in flight (`Driver::transfer_retry`).
pub struct RetryFuture<'a> {
    drv: &'a mut PioSwd,
    request: u8,
    data: u32,
    params: TransferParams,
    wait_retries: u16,
    st: RetryState,
}

impl Future for RetryFuture<'_> {
    type Output = Transfer;

    #[unsafe(link_section = ".data.swd_hot")]
    #[inline(never)]
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Transfer> {
        let this = self.get_mut();
        this.st.poll(this.drv, this.request, this.data, &this.params, this.wait_retries, cx)
    }
}

/// A `DAP_TransferBlock` read in flight (`Driver::read_block`): the posting
/// read for an AP, then one transfer per word with the last one through
/// `RDBUFF`, each with WAIT retries. Words land in `out` little-endian.
pub struct ReadBlockFuture<'a> {
    drv: &'a mut PioSwd,
    request: u8,
    params: TransferParams,
    wait_retries: u16,
    out: &'a mut [u8],
    /// Next word index; `None` while the AP read is being posted.
    next: Option<u16>,
    count: u16,
    st: RetryState,
}

impl Future for ReadBlockFuture<'_> {
    type Output = (u16, u8);

    #[unsafe(link_section = ".data.swd_hot")]
    #[inline(never)]
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<(u16, u8)> {
        let this = self.get_mut();
        let ap = this.request & req::APNDP != 0;
        loop {
            let (request, word) = match this.next {
                None => (this.request, None),
                Some(i) => (if ap && i + 1 == this.count { req::RDBUFF_READ } else { this.request }, Some(i)),
            };
            let t = core::task::ready!(this.st.poll(this.drv, request, 0, &this.params, this.wait_retries, cx));
            let done = word.unwrap_or(0);
            if t.ack != ack::OK {
                return Poll::Ready((done, t.ack));
            }
            match word {
                None => this.next = Some(0),
                Some(i) => {
                    let at = i as usize * 4;
                    this.out[at..at + 4].copy_from_slice(&t.data.to_le_bytes());
                    if i + 1 == this.count {
                        return Poll::Ready((this.count, ack::OK));
                    }
                    this.next = Some(i + 1);
                }
            }
        }
    }
}

/// A `DAP_TransferBlock` write in flight (`Driver::write_block`): one
/// transfer per word, then the `RDBUFF` read that completes the last one.
pub struct WriteBlockFuture<'a> {
    drv: &'a mut PioSwd,
    request: u8,
    params: TransferParams,
    wait_retries: u16,
    data: &'a [u8],
    /// Next word index; `count` means the trailing `RDBUFF` check.
    next: u16,
    count: u16,
    st: RetryState,
}

impl Future for WriteBlockFuture<'_> {
    type Output = (u16, u8);

    #[unsafe(link_section = ".data.swd_hot")]
    #[inline(never)]
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<(u16, u8)> {
        let this = self.get_mut();
        loop {
            let i = this.next;
            if i == this.count {
                let t = core::task::ready!(this.st.poll(this.drv, req::RDBUFF_READ, 0, &this.params, this.wait_retries, cx));
                return Poll::Ready((this.count, t.ack));
            }
            let at = i as usize * 4;
            let d = &this.data[at..at + 4];
            let word = u32::from_le_bytes([d[0], d[1], d[2], d[3]]);
            let t = core::task::ready!(this.st.poll(this.drv, this.request, word, &this.params, this.wait_retries, cx));
            if t.ack != ack::OK {
                return Poll::Ready((i, t.ack));
            }
            this.next = i + 1;
        }
    }
}

impl Driver for PioSwd {
    fn set_clock(&mut self, hz: u32) -> bool {
        if hz == 0 {
            return false;
        }
        self.bus.set_swclk_freq_khz((hz / 1000).max(1));
        self.update_spin();
        true
    }

    async fn connect(&mut self) {
        self.set_dir(true).await;
    }

    async fn disconnect(&mut self) {
        self.set_dir(false).await;
        self.bus.set_nreset(false);
    }

    fn transfer(&mut self, request: u8, data: u32, params: &TransferParams) -> impl Future<Output = Transfer> {
        TransferFuture { drv: self, request, data, params: *params, st: TransferState::new() }
    }

    fn transfer_retry(&mut self, request: u8, data: u32, params: &TransferParams, wait_retries: u16) -> impl Future<Output = Transfer> {
        RetryFuture { drv: self, request, data, params: *params, wait_retries, st: RetryState::new(wait_retries) }
    }

    // One `async fn` layer per block command (not per transfer) around the
    // hand-written futures; that matches the trait's signature cheaply.
    async fn read_block(&mut self, request: u8, out: &mut [u8], params: &TransferParams, wait_retries: u16) -> (u16, u8) {
        let count = (out.len() / 4) as u16;
        if count == 0 {
            return (0, 0);
        }
        ReadBlockFuture {
            drv: self,
            request,
            params: *params,
            wait_retries,
            out,
            next: if request & req::APNDP != 0 { None } else { Some(0) },
            count,
            st: RetryState::new(wait_retries),
        }
        .await
    }

    async fn write_block(&mut self, request: u8, data: &[u8], params: &TransferParams, wait_retries: u16) -> (u16, u8) {
        let count = (data.len() / 4) as u16;
        if count == 0 {
            return (0, 0);
        }
        WriteBlockFuture {
            drv: self,
            request,
            params: *params,
            wait_retries,
            data,
            next: 0,
            count,
            st: RetryState::new(wait_retries),
        }
        .await
    }

    async fn write_sequence(&mut self, mut bits: usize, data: &[u8]) {
        for byte in data {
            if bits == 0 {
                break;
            }
            let chunk = bits.min(8);
            self.write_bits(chunk as u32, *byte as u32).await;
            bits -= chunk;
        }
    }

    async fn read_sequence(&mut self, mut bits: usize, data: &mut [u8]) {
        for byte in data {
            if bits == 0 {
                break;
            }
            let chunk = bits.min(8);
            *byte = self.read_bits(chunk as u32).await as u8;
            bits -= chunk;
        }
    }

    async fn swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        // SWCLK/SWDIO are owned by the PIO state machine and are not
        // individually drivable; like the C firmware, only nRESET is
        // supported here (open-drain: high = released).
        if mask.contains(Pins::NRESET) {
            let high = output.contains(Pins::NRESET);
            self.bus.set_nreset(!high);
            if wait_us > 0 {
                let deadline = Instant::now() + Duration::from_micros(wait_us as u64);
                while self.bus.nreset_level() != high && Instant::now() < deadline {
                    Timer::after_micros(10).await;
                }
            }
        }
        let mut ret = Pins(0);
        if self.bus.nreset_level() {
            ret.0 |= Pins::NRESET.0;
        }
        ret
    }

    fn timestamp(&self) -> u32 {
        Instant::now().as_micros() as u32
    }
}
