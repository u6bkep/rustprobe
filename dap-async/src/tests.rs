//! Host tests: a scripted mock driver drives the command processor through
//! the transfer semantics the reference implementation defines.

use std::collections::VecDeque;
use std::future::Future;
use std::pin::pin;
use std::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use crate::swd::{ack, req, Driver, Pins, Transfer, TransferParams};
use crate::Dap;

/// What the mock expects the processor to do, in order.
#[derive(Debug, PartialEq, Eq, Clone)]
enum Call {
    /// `transfer(request, data)` answered with `(ack, data)`.
    Xfer { request: u8, data: u32, reply: (u8, u32) },
    WriteSeq { bits: usize, data: Vec<u8> },
    /// `read_sequence(bits)` fills `data`.
    ReadSeq { bits: usize, data: Vec<u8> },
    Connect,
    Disconnect,
    Clock(u32),
    Pins { output: u8, mask: u8, wait_us: u32, reply: u8 },
}

#[derive(Default)]
struct Mock {
    script: VecDeque<Call>,
    /// `params` seen by the last `transfer`.
    last_params: Option<TransferParams>,
}

impl Mock {
    fn next(&mut self) -> Call {
        self.script.pop_front().expect("unexpected driver call")
    }
}

impl Driver for Mock {
    fn set_clock(&mut self, hz: u32) -> bool {
        assert_eq!(self.next(), Call::Clock(hz));
        hz != 0
    }
    async fn connect(&mut self) {
        assert_eq!(self.next(), Call::Connect);
    }
    async fn disconnect(&mut self) {
        assert_eq!(self.next(), Call::Disconnect);
    }
    async fn transfer(&mut self, request: u8, data: u32, params: &TransferParams) -> Transfer {
        self.last_params = Some(*params);
        match self.next() {
            Call::Xfer { request: r, data: d, reply } => {
                assert_eq!((request, data), (r, d), "transfer request/data");
                Transfer { ack: reply.0, data: reply.1 }
            }
            other => panic!("expected {other:?}, got transfer({request:#x}, {data:#x})"),
        }
    }
    async fn write_sequence(&mut self, bits: usize, data: &[u8]) {
        assert_eq!(self.next(), Call::WriteSeq { bits, data: data.to_vec() });
    }
    async fn read_sequence(&mut self, bits: usize, data: &mut [u8]) {
        match self.next() {
            Call::ReadSeq { bits: b, data: d } => {
                assert_eq!(bits, b);
                assert!(data.len() >= d.len());
                data[..d.len()].copy_from_slice(&d);
            }
            other => panic!("expected {other:?}, got read_sequence({bits})"),
        }
    }
    async fn swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        match self.next() {
            Call::Pins { output: o, mask: m, wait_us: w, reply } => {
                assert_eq!((output.0, mask.0, wait_us), (o, m, w));
                Pins(reply)
            }
            other => panic!("expected {other:?}, got swj_pins"),
        }
    }
    fn timestamp(&self) -> u32 {
        0x1234_5678
    }
}

struct NoDelay;
impl embedded_hal_async::delay::DelayNs for NoDelay {
    async fn delay_ns(&mut self, _ns: u32) {}
}

/// The mock never pends, so a single poll completes every future.
fn block_on<F: Future>(fut: F) -> F::Output {
    fn noop(_: *const ()) {}
    fn clone(p: *const ()) -> RawWaker {
        RawWaker::new(p, &VTABLE)
    }
    static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
    let waker = unsafe { Waker::from_raw(RawWaker::new(std::ptr::null(), &VTABLE)) };
    let mut cx = Context::from_waker(&waker);
    match pin!(fut).poll(&mut cx) {
        Poll::Ready(v) => v,
        Poll::Pending => panic!("mock future pended"),
    }
}

type TestDap = Dap<Mock, NoDelay>;

fn dap(script: Vec<Call>) -> TestDap {
    Dap::new(Mock { script: script.into(), last_params: None }, NoDelay, "2.1.0", 64)
}

/// Run one command, assert the script was consumed, return the reply.
fn run(dap: &mut TestDap, request: &[u8]) -> Vec<u8> {
    let mut buf = [0u8; 64];
    let n = block_on(dap.process_command(request, &mut buf));
    assert!(dap.driver().script.is_empty(), "unconsumed script: {:?}", dap.driver().script);
    buf[..n].to_vec()
}

fn xfer(request: u8, data: u32, reply: (u8, u32)) -> Call {
    Call::Xfer { request, data, reply }
}

const OK: u8 = ack::OK;
const AP_READ: u8 = req::APNDP | req::RNW | req::A2 | req::A3; // AP 0xC (DRW)
const DP_READ: u8 = req::RNW | req::A2; // DP 0x4 (CTRL/STAT)
const AP_WRITE: u8 = req::APNDP | req::A2 | req::A3;
const RDBUFF: u8 = req::RDBUFF_READ;

fn word(v: u32) -> [u8; 4] {
    v.to_le_bytes()
}

// --- DAP_Info / housekeeping -------------------------------------------------

#[test]
fn info_packet_size_and_capabilities() {
    let mut d = dap(vec![]);
    assert_eq!(run(&mut d, &[0x00, 0xFF]), [0x00, 2, 64, 0]);
    assert_eq!(run(&mut d, &[0x00, 0xF0]), [0x00, 1, 0x01]);
    assert_eq!(run(&mut d, &[0x00, 0xFE]), [0x00, 1, 1]);
    assert_eq!(run(&mut d, &[0x00, 0x04]), [0x00, 5, b'2', b'.', b'1', b'.', b'0']);
    assert_eq!(run(&mut d, &[0x00, 0x03]), [0x00, 0]);
    assert_eq!(run(&mut d, &[0x00, 0xFD]), [0x00, 4, 0, 0, 0, 0]);
}

#[test]
fn unknown_jtag_and_truncated_commands() {
    let mut d = dap(vec![]);
    assert_eq!(run(&mut d, &[0x42]), [0xFF]);
    assert_eq!(run(&mut d, &[0x14, 1, 0, 0]), [0x14, 0xFF]);
    assert_eq!(run(&mut d, &[0x15, 0]), [0x15, 0xFF]);
    assert_eq!(run(&mut d, &[0x16, 0]), [0x16, 0xFF]);
    // Truncated DAP_Transfer: the transfers before the cut still run (the
    // request is parsed as it executes, as in the reference); the reply is
    // an error, not a panic.
    let mut d = dap(vec![xfer(AP_READ, 0, (OK, 0))]);
    assert_eq!(run(&mut d, &[0x05, 0, 3, AP_READ]), [0x05, 0xFF]);
    // Empty request: nothing to say.
    let mut buf = [0u8; 64];
    assert_eq!(block_on(d.process_command(&[], &mut buf)), 0);
    // TransferAbort: no reply.
    assert_eq!(block_on(d.process_command(&[0x07], &mut buf)), 0);
}

#[test]
fn connect_disconnect_clock_pins() {
    let mut d = dap(vec![Call::Connect, Call::Disconnect, Call::Clock(4_000_000), Call::Clock(0)]);
    assert_eq!(run_keep(&mut d, &[0x02, 0]), [0x02, 1]);
    assert_eq!(run_keep(&mut d, &[0x02, 2]), [0x02, 0]); // JTAG: failed, no driver call
    assert_eq!(run_keep(&mut d, &[0x03]), [0x03, 0]);
    assert_eq!(run_keep(&mut d, &[0x11, 0, 9, 61, 0]), [0x11, 0]);
    assert_eq!(run(&mut d, &[0x11, 0, 0, 0, 0]), [0x11, 0xFF]);

    let mut d = dap(vec![Call::Pins { output: 0x80, mask: 0x80, wait_us: 3_000_000, reply: 0x80 }]);
    // Wait is capped at 3 s.
    assert_eq!(run(&mut d, &[0x10, 0x80, 0x80, 0xFF, 0xFF, 0xFF, 0xFF]), [0x10, 0x80]);
    assert_eq!(run(&mut d, &[0x0A]), [0x0A, 0, 0]);
    assert_eq!(run(&mut d, &[0x09, 10, 0]), [0x09, 0]);
}

/// Like `run` but allows the script to carry over to the next command.
fn run_keep(dap: &mut TestDap, request: &[u8]) -> Vec<u8> {
    let mut buf = [0u8; 64];
    let n = block_on(dap.process_command(request, &mut buf));
    buf[..n].to_vec()
}

#[test]
fn write_abort_ignores_ack_and_host_status_rejects_unknown_type() {
    let mut d = dap(vec![xfer(req::ABORT_WRITE, 0x1E, (ack::NO_ACK, 0))]);
    assert_eq!(run(&mut d, &[0x08, 0, 0x1E, 0, 0, 0]), [0x08, 0]);
    assert_eq!(run(&mut d, &[0x01, 0, 1]), [0x01, 0]);
    assert_eq!(run(&mut d, &[0x01, 2, 1]), [0x01, 0xFF]);
}

#[test]
fn configure_reaches_the_driver() {
    let mut d = dap(vec![xfer(req::ABORT_WRITE, 0x1E, (OK, 0))]);
    // SWD_Configure: turnaround 3, data phase on. TransferConfigure: idle 4.
    assert_eq!(run_keep(&mut d, &[0x13, 0b110]), [0x13, 0]);
    assert_eq!(run_keep(&mut d, &[0x04, 4, 100, 0, 0, 0]), [0x04, 0]);
    assert_eq!(run(&mut d, &[0x08, 0, 0x1E, 0, 0, 0]), [0x08, 0]);
    assert_eq!(
        d.driver().last_params,
        Some(TransferParams { turnaround: 3, data_phase: true, idle_cycles: 4 })
    );
}

// --- DAP_SWJ_Sequence / DAP_SWD_Sequence (ported from dap-rs) ---------------

#[test]
fn swj_sequence() {
    let mut d = dap(vec![Call::WriteSeq { bits: 256, data: vec![0xAA; 32] }]);
    let mut r = vec![0x12, 0];
    r.extend([0xAA; 32]);
    assert_eq!(run(&mut d, &r), [0x12, 0]);
    let mut d = dap(vec![Call::WriteSeq { bits: 51, data: vec![0xFF; 7] }]);
    let mut r = vec![0x12, 51];
    r.extend([0xFF; 7]);
    assert_eq!(run(&mut d, &r), [0x12, 0]);
    // Too short a payload: error, no call.
    let mut d = dap(vec![]);
    assert_eq!(run(&mut d, &[0x12, 51, 0xFF]), [0x12, 0xFF]);
}

#[test]
fn swd_sequence_output() {
    let seq = [0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0];
    let mut d = dap(vec![Call::WriteSeq { bits: 52, data: seq.to_vec() }]);
    let mut r = vec![0x1D, 1, 52];
    r.extend(seq);
    assert_eq!(run(&mut d, &r), [0x1D, 0]);

    // 0 means 64 bits.
    let seq = [0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00];
    let mut d = dap(vec![Call::WriteSeq { bits: 64, data: seq.to_vec() }]);
    let mut r = vec![0x1D, 1, 0];
    r.extend(seq);
    assert_eq!(run(&mut d, &r), [0x1D, 0]);
}

#[test]
fn swd_sequence_input() {
    let seq = [0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0];
    let mut d = dap(vec![Call::ReadSeq { bits: 52, data: seq.to_vec() }]);
    let mut want = vec![0x1D, 0];
    want.extend(seq);
    assert_eq!(run(&mut d, &[0x1D, 1, 0x80 | 52]), want);

    let seq = [0xFFu8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00];
    let mut d = dap(vec![Call::ReadSeq { bits: 64, data: seq.to_vec() }]);
    let mut want = vec![0x1D, 0];
    want.extend(seq);
    assert_eq!(run(&mut d, &[0x1D, 1, 0x80]), want);
}

#[test]
fn swd_sequence_target_select() {
    // write 8 bits, read 5 bits, write 33 bits
    let mut d = dap(vec![
        Call::WriteSeq { bits: 8, data: vec![0b10111101] },
        Call::ReadSeq { bits: 5, data: vec![0x1F] },
        Call::WriteSeq { bits: 33, data: vec![0x56, 0x83, 0xAB, 0x32, 0x01] },
    ]);
    let r = [0x1D, 3, 8, 0b10111101, 0x80 | 5, 33, 0x56, 0x83, 0xAB, 0x32, 0x01];
    assert_eq!(run(&mut d, &r), [0x1D, 0, 0x1F]);
}

// --- DAP_Transfer ------------------------------------------------------------

#[test]
fn transfer_posts_ap_reads() {
    // AP read, AP read, DP read: the first AP read is posted, the second
    // returns its data and posts itself, RDBUFF collects it before the DP
    // read, and the DP read is direct.
    let mut d = dap(vec![
        xfer(AP_READ, 0, (OK, 0xDEAD)),
        xfer(AP_READ, 0, (OK, 0x1111)),
        xfer(RDBUFF, 0, (OK, 0x2222)),
        xfer(DP_READ, 0, (OK, 0x3333)),
    ]);
    let r = [0x05, 0, 3, AP_READ, AP_READ, DP_READ];
    let mut want = vec![0x05, 3, OK];
    want.extend(word(0x1111));
    want.extend(word(0x2222));
    want.extend(word(0x3333));
    assert_eq!(run(&mut d, &r), want);
}

#[test]
fn transfer_single_ap_read_collects_via_rdbuff() {
    let mut d = dap(vec![xfer(AP_READ, 0, (OK, 0)), xfer(RDBUFF, 0, (OK, 0xCAFE))]);
    let mut want = vec![0x05, 1, OK];
    want.extend(word(0xCAFE));
    assert_eq!(run(&mut d, &[0x05, 0, 1, AP_READ]), want);
}

#[test]
fn transfer_write_then_check() {
    // Two writes, then RDBUFF so the last one completes. Count 2, OK.
    let mut d = dap(vec![
        xfer(AP_WRITE, 0x11, (OK, 0)),
        xfer(AP_WRITE, 0x22, (OK, 0)),
        xfer(RDBUFF, 0, (OK, 0)),
    ]);
    let mut r = vec![0x05, 0, 2, AP_WRITE];
    r.extend(word(0x11));
    r.push(AP_WRITE);
    r.extend(word(0x22));
    assert_eq!(run(&mut d, &r), [0x05, 2, OK]);
}

#[test]
fn transfer_write_after_posted_read_collects_first() {
    let mut d = dap(vec![
        xfer(AP_READ, 0, (OK, 0)),
        xfer(RDBUFF, 0, (OK, 0xAB)),
        xfer(AP_WRITE, 0x5, (OK, 0)),
        xfer(RDBUFF, 0, (OK, 0)),
    ]);
    let mut r = vec![0x05, 0, 2, AP_READ, AP_WRITE];
    r.extend(word(0x5));
    let mut want = vec![0x05, 2, OK];
    want.extend(word(0xAB));
    assert_eq!(run(&mut d, &r), want);
}

#[test]
fn transfer_fault_excludes_failing_transfer() {
    let mut d = dap(vec![xfer(DP_READ, 0, (OK, 1)), xfer(DP_READ, 0, (ack::FAULT, 0))]);
    let mut want = vec![0x05, 1, ack::FAULT];
    want.extend(word(1));
    assert_eq!(run(&mut d, &[0x05, 0, 3, DP_READ, DP_READ, DP_READ]), want);
}

#[test]
fn transfer_wait_retries_then_gives_up() {
    let mut d = dap(vec![
        xfer(DP_READ, 0, (ack::WAIT, 0)),
        xfer(DP_READ, 0, (ack::WAIT, 0)),
        xfer(DP_READ, 0, (ack::WAIT, 0)),
        xfer(DP_READ, 0, (OK, 7)),
    ]);
    // wait_retry = 3: up to four attempts.
    assert_eq!(run_keep(&mut d, &[0x04, 0, 3, 0, 0, 0]), [0x04, 0]);
    let mut want = vec![0x05, 1, OK];
    want.extend(word(7));
    assert_eq!(run(&mut d, &[0x05, 0, 1, DP_READ]), want);

    let mut d = dap(vec![xfer(DP_READ, 0, (ack::WAIT, 0)); 4]);
    assert_eq!(run_keep(&mut d, &[0x04, 0, 3, 0, 0, 0]), [0x04, 0]);
    assert_eq!(run(&mut d, &[0x05, 0, 1, DP_READ]), [0x05, 0, ack::WAIT]);
}

#[test]
fn transfer_match_mask_and_value() {
    // Mask write (no wire traffic), then a DP read with value match: two
    // mismatches then a hit within match_retry = 2.
    let mut d = dap(vec![
        xfer(DP_READ, 0, (OK, 0x0F00)),
        xfer(DP_READ, 0, (OK, 0x0F01)),
        xfer(DP_READ, 0, (OK, 0x0F0F)),
    ]);
    assert_eq!(run_keep(&mut d, &[0x04, 0, 0, 0, 2, 0]), [0x04, 0]);
    let mut r = vec![0x05, 0, 2, req::MATCH_MASK];
    r.extend(word(0x00FF));
    r.push(DP_READ | req::MATCH_VALUE);
    r.extend(word(0x000F));
    assert_eq!(run(&mut d, &r), [0x05, 2, OK]);

    // Never matches: status OK | MISMATCH, count excludes it.
    let mut d = dap(vec![xfer(DP_READ, 0, (OK, 0)); 3]);
    assert_eq!(run_keep(&mut d, &[0x04, 0, 0, 0, 2, 0]), [0x04, 0]);
    let mut r = vec![0x05, 0, 2, req::MATCH_MASK];
    r.extend(word(0xFF));
    r.push(DP_READ | req::MATCH_VALUE);
    r.extend(word(0x0F));
    assert_eq!(run(&mut d, &r), [0x05, 1, OK | ack::MISMATCH]);
}

#[test]
fn transfer_ap_match_value_posts_first() {
    // Mask write, then an AP read with value match: the AP read is posted
    // (data discarded) and read again for the comparison. No RDBUFF
    // afterwards: the match path leaves nothing posted.
    let mut d = dap(vec![xfer(AP_READ, 0, (OK, 0)), xfer(AP_READ, 0, (OK, 0x42))]);
    let mut r = vec![0x05, 0, 2, req::MATCH_MASK];
    r.extend(word(0xFF));
    r.push(AP_READ | req::MATCH_VALUE);
    r.extend(word(0x42));
    assert_eq!(run(&mut d, &r), [0x05, 2, OK]);
}

// --- DAP_TransferBlock -------------------------------------------------------

#[test]
fn block_ap_read_pipelines() {
    let mut d = dap(vec![
        xfer(AP_READ, 0, (OK, 0)),
        xfer(AP_READ, 0, (OK, 1)),
        xfer(AP_READ, 0, (OK, 2)),
        xfer(RDBUFF, 0, (OK, 3)),
    ]);
    let mut want = vec![0x06, 3, 0, OK];
    for v in 1..=3 {
        want.extend(word(v));
    }
    assert_eq!(run(&mut d, &[0x06, 0, 3, 0, AP_READ]), want);
}

#[test]
fn block_dp_read_is_direct() {
    let mut d = dap(vec![xfer(DP_READ, 0, (OK, 5)), xfer(DP_READ, 0, (OK, 6))]);
    let mut want = vec![0x06, 2, 0, OK];
    want.extend(word(5));
    want.extend(word(6));
    assert_eq!(run(&mut d, &[0x06, 0, 2, 0, DP_READ]), want);
}

#[test]
fn block_read_error_counts_only_data_returned() {
    // The dap-rs off-by-one (issue #4 section 4): a NoAck mid-block must
    // report as many transfers as there are data words.
    let mut d = dap(vec![
        xfer(AP_READ, 0, (OK, 0)),
        xfer(AP_READ, 0, (OK, 1)),
        xfer(AP_READ, 0, (ack::NO_ACK, 0)),
    ]);
    let mut want = vec![0x06, 1, 0, ack::NO_ACK];
    want.extend(word(1));
    assert_eq!(run(&mut d, &[0x06, 0, 3, 0, AP_READ]), want);

    // Failure on the posting read: count 0, no data.
    let mut d = dap(vec![xfer(AP_READ, 0, (ack::FAULT, 0))]);
    assert_eq!(run(&mut d, &[0x06, 0, 3, 0, AP_READ]), [0x06, 0, 0, ack::FAULT]);
}

#[test]
fn block_write_then_check() {
    let mut d = dap(vec![
        xfer(AP_WRITE, 0xA, (OK, 0)),
        xfer(AP_WRITE, 0xB, (OK, 0)),
        xfer(RDBUFF, 0, (OK, 0)),
    ]);
    let mut r = vec![0x06, 0, 2, 0, AP_WRITE];
    r.extend(word(0xA));
    r.extend(word(0xB));
    assert_eq!(run(&mut d, &r), [0x06, 2, 0, OK]);

    // A FAULT on the second write: count 1, no check read.
    let mut d = dap(vec![xfer(AP_WRITE, 0xA, (OK, 0)), xfer(AP_WRITE, 0xB, (ack::FAULT, 0))]);
    assert_eq!(run(&mut d, &r), [0x06, 1, 0, ack::FAULT]);
}

#[test]
fn block_zero_transfers() {
    let mut d = dap(vec![]);
    assert_eq!(run(&mut d, &[0x06, 0, 0, 0]), [0x06, 0, 0, 0]);
}

#[test]
fn packet_helper() {
    // DP read of DPIDR (A2=A3=0, RnW=1): 1, 0, 1, 0, 0, parity 1, stop 0, park 1
    assert_eq!(crate::swd::packet(req::RNW), 0b1010_0101);
    // AP write to 0xC: APnDP=1, RnW=0, A2=1, A3=1 → parity 1
    assert_eq!(crate::swd::packet(AP_WRITE), 0b1011_1011);
    assert_eq!(crate::swd::packet(req::RDBUFF_READ), 0b1011_1101);
}
