//! PIO edge-timer autobaud capture, driving the [`autobaud_estimator`] core.
//!
//! Ports debugprobe's `autobaud.pio` + `autobaud.c`. When the host selects the
//! magic baud rate (`MAGIC_BAUD` = 9728) on a CDC port, the corresponding
//! [`uart_bridge`](crate::uart_bridge) asks this engine to *measure* the
//! target's baud instead of setting a literal line rate; the detected baud is
//! sent back and applied with `set_baudrate`.
//!
//! # Design vs. the C firmware
//!
//! * **Estimator split out.** The statistical core lives in the host-testable
//!   `autobaud-estimator` crate; this module is only the hardware capture.
//! * **Capture path.** The C runs a free-running pair of chained DMA channels
//!   into a 1024-word ring, and periodically diffs the ring write pointer. The
//!   embassy DMA API models a single finite transfer with a completion waker
//!   (`StateMachineRx::dma_pull`), so we instead pull fixed [`CHUNK`]-word
//!   blocks in a loop, feeding each block to the estimator and re-arming. Edges
//!   dropped in the gap between blocks are harmless — the estimator is
//!   statistical — and this keeps us off per-word async FIFO reads (which can't
//!   keep up with ~1 µs edge spacing at high baud). One DMA channel, not two.
//! * **One long-lived SM.** The C claims and frees a PIO state machine per
//!   session. Because dropping an embassy PIO handle resets funcsel on the
//!   whole block's pins (see `instances::block_engines`), we instead claim one
//!   SM at boot (the SM `probe_config`'s validation always reserves when a UART
//!   exists) and keep it for the firmware's life, only enabling capture during
//!   a session. The snooped RX pin is set per session, since either UART may
//!   trigger autobaud.
//! * **Pin snooping.** The RX GPIO is already owned by `BufferedUart`. We do
//!   *not* `make_pio_pin` it (that would steal its funcsel from the UART);
//!   instead we point the SM's IN and JMP-pin mappings at the raw pin number via
//!   the config's exec/pin fields. On both RP2040 (§2.19.2) and RP2350, a pad's
//!   input is delivered to every peripheral's input regardless of `FUNCSEL` —
//!   `FUNCSEL` only selects which peripheral drives the output/OE — so PIO reads
//!   the live UART RX line while the UART keeps receiving. We only force the
//!   pad's input enable on (the UART already sets it, but PIO input requires it).

use embassy_futures::select::{select, Either};
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::Channel as DmaChannel;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Common, Config, Instance, StateMachine};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use fixed::traits::ToFixed;
use probe_config::MAX_UARTS;

use autobaud_estimator::BaudEstimator;

/// PIO clock the edge timer runs at, in Hz. The estimator converts cycle counts
/// to baud against this, so it must match the SM's effective clock. Fixed at
/// 125 MHz (as in the C) via the clock divider, independent of the system clock.
const PIO_CLOCK_HZ: u32 = 125_000_000;

/// Words pulled per DMA transfer before the block is handed to the estimator.
/// Small enough to stay responsive, large enough that per-block overhead is
/// negligible against ~1 µs edge spacing.
const CHUNK: usize = 128;

/// Host → engine command (which UART's RX to snoop, or stop).
#[derive(Clone, Copy)]
enum Command {
    Start { uart: u8, rx_pin: u8 },
    Stop,
}

/// Engine → bridge published estimate, tagged with the UART it belongs to.
#[derive(Clone, Copy)]
pub struct BaudResult {
    /// Index of the UART bridge this estimate is for.
    pub uart: u8,
    /// Detected baud rate.
    pub baud: u32,
    /// Confidence in `[0.6, 1.0]`.
    pub validity: f32,
}

static CONTROL: Signal<CriticalSectionRawMutex, Command> = Signal::new();
static RESULT: Watch<CriticalSectionRawMutex, BaudResult, MAX_UARTS> = Watch::new();

/// A per-bridge receiver of autobaud results.
pub type ResultReceiver =
    embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, BaudResult, MAX_UARTS>;

/// Ask the engine to begin measuring `uart`'s target baud, snooping `rx_pin`.
pub fn start(uart: u8, rx_pin: u8) {
    CONTROL.signal(Command::Start { uart, rx_pin });
}

/// Ask the engine to stop the current session.
pub fn stop() {
    CONTROL.signal(Command::Stop);
}

/// Obtain this bridge's result receiver (one per UART; `None` if exhausted).
pub fn result_receiver() -> Option<ResultReceiver> {
    RESULT.receiver()
}

/// Force a pad's input buffer on so PIO can read it. `BufferedUart` already
/// enables this on its RX pin; we assert it defensively because PIO input is
/// undefined without it, regardless of the pin's funcsel.
fn pad_input_enable(pin: u8) {
    embassy_rp::pac::PADS_BANK0
        .gpio(pin as usize)
        .modify(|w| w.set_ie(true));
}

/// The edge-timer capture bound to one PIO state machine. Holds the SM and a
/// prepared [`Config`] whose IN/JMP pin is retargeted per session.
pub struct AutobaudCapture<'d, P: Instance, const SM: usize> {
    sm: StateMachine<'d, P, SM>,
    cfg: Config<'d, P>,
}

impl<'d, P: Instance, const SM: usize> AutobaudCapture<'d, P, SM> {
    /// Load the edge-timer program into `common` and prepare `sm` (left
    /// disabled). The program is receive-only, so no output pins are claimed.
    pub fn new(common: &mut Common<'d, P>, mut sm: StateMachine<'d, P, SM>) -> Self {
        // Port of autobaud.pio: time the low pulse between a falling and the
        // next rising edge. X counts down from 0xFFFFFFFF, two clocks per step.
        let prg = pio_asm!(
            ".wrap_target",
            "falling_edge:",
            "    wait 0 pin 0",          // wait for the line to go low
            "    set x, 0",
            "    mov x, ~x",             // x = 0xFFFFFFFF
            "count_cycles:",
            "    jmp pin rising_edge",   // line high again -> done
            "    jmp x-- count_cycles",  // else keep counting
            "rising_edge:",
            "    mov isr, x",
            "    push noblock",          // push elapsed count, never stall
            "    jmp falling_edge",
            ".wrap",
        );
        let loaded = common.load_program(&prg.program);

        let mut cfg = Config::default();
        cfg.use_program(&loaded, &[]); // receive-only: no side-set pins
        // Run the SM at PIO_CLOCK_HZ regardless of the system clock, matching
        // the C's `div = clk_sys / 125MHz`.
        let div = (clk_sys_freq() as f32 / PIO_CLOCK_HZ as f32).max(1.0);
        cfg.clock_divider = div.to_fixed();
        // Default shift config is fine: the program pushes explicitly, no
        // autopush, and shift direction is irrelevant to `mov isr, x`.
        sm.set_config(&cfg);

        Self { sm, cfg }
    }

    /// Point the SM's IN/JMP mapping at `rx_pin` (without touching its funcsel)
    /// and start capturing.
    fn arm(&mut self, rx_pin: u8) {
        let mut exec = self.cfg.get_exec();
        exec.jmp_pin = rx_pin;
        // SAFETY: we only set the jmp pin index; the rest of the exec config is
        // whatever `use_program` established.
        unsafe { self.cfg.set_exec(exec) };

        let mut pins = self.cfg.get_pins();
        pins.in_base = rx_pin;
        // SAFETY: only the IN base pin is changed; `in_count` stays 0, which
        // means "no masking" on RP2350 and is unused on RP2040.
        unsafe { self.cfg.set_pins(pins) };

        pad_input_enable(rx_pin);

        self.sm.set_enable(false);
        // `set_config` also writes the block-wide GPIOBASE window on RP2350.
        // With no output/side-set pins and `in_count == 0`, our pin ranges are
        // empty, so it selects window 0 — matching the probes in a shared block
        // as long as every pin is < 32 (true on RP2040 and RP2350A; the >32-pin
        // RP2350B window binning is the separate `TODO(rp2350b)` in validate).
        self.sm.set_config(&self.cfg);
        self.sm.clear_fifos();
        self.sm.restart();
        self.sm.set_enable(true);
    }

    /// Stop capturing (the SM stays claimed for the firmware's life).
    fn disarm(&mut self) {
        self.sm.set_enable(false);
    }
}

/// Capture loop for one concrete `(PIO block, SM)`. Idles until [`start`],
/// pulls DMA blocks into the estimator, and publishes results until [`stop`].
async fn run<P: Instance, const SM: usize>(
    mut cap: AutobaudCapture<'static, P, SM>,
    mut dma: DmaChannel<'static>,
) -> ! {
    let mut estimator = BaudEstimator::new(PIO_CLOCK_HZ);
    let sender = RESULT.sender();
    let mut buf = [0u32; CHUNK];
    let mut active: Option<u8> = None; // UART id, when a session is running

    loop {
        match active {
            None => match CONTROL.wait().await {
                Command::Start { uart, rx_pin } => {
                    estimator.reset();
                    cap.arm(rx_pin);
                    active = Some(uart);
                    defmt::info!("autobaud: start on uart {} (RX GP{})", uart, rx_pin);
                }
                Command::Stop => {}
            },
            Some(uart) => {
                // Await either a full DMA block or a control change. Dropping the
                // transfer future (on a control change) aborts the DMA channel.
                let xfer = cap.sm.rx().dma_pull(&mut dma, &mut buf, false);
                match select(xfer, CONTROL.wait()).await {
                    Either::First(()) => {
                        if let Some(est) = estimator.push_batch(&buf) {
                            defmt::info!(
                                "autobaud: uart {} -> {} baud (validity {})",
                                uart,
                                est.baud,
                                est.validity
                            );
                            sender.send(BaudResult { uart, baud: est.baud, validity: est.validity });
                        }
                    }
                    Either::Second(Command::Start { uart: next, rx_pin }) => {
                        estimator.reset();
                        cap.arm(rx_pin);
                        active = Some(next);
                        defmt::info!("autobaud: switch to uart {} (RX GP{})", next, rx_pin);
                    }
                    Either::Second(Command::Stop) => {
                        cap.disarm();
                        active = None;
                        defmt::info!("autobaud: stop");
                    }
                }
            }
        }
    }
}

/// One of the concrete `(block, SM)` slots autobaud can occupy. Probes fill
/// slots from PIO0/SM0 upward, so the reserved SM is either the last SM (index
/// 3) of a partially-filled block or SM0 of a fresh later block — never the
/// intermediate indices. See [`instances::build_engines`](crate::instances).
pub enum AutobaudSm {
    /// Fresh block (PIO0), SM0 — 0 probes.
    P0S0(AutobaudCapture<'static, PIO0, 0>),
    /// Shared block (PIO0), SM3 — 1..=3 probes.
    P0S3(AutobaudCapture<'static, PIO0, 3>),
    /// Fresh block (PIO1), SM0 — 4 probes.
    P1S0(AutobaudCapture<'static, PIO1, 0>),
    /// Shared block (PIO1), SM3 — 5..=7 probes.
    P1S3(AutobaudCapture<'static, PIO1, 3>),
    /// Fresh block (PIO2), SM0 — 8 probes (RP2350 only).
    #[cfg(feature = "rp2350")]
    P2S0(AutobaudCapture<'static, embassy_rp::peripherals::PIO2, 0>),
    /// Shared block (PIO2), SM3 — 9..=11 probes (RP2350 only).
    #[cfg(feature = "rp2350")]
    P2S3(AutobaudCapture<'static, embassy_rp::peripherals::PIO2, 3>),
}

/// The autobaud capture task. Dispatches the type-erased SM slot to the generic
/// [`run`] loop. Runs on core 0.
#[embassy_executor::task]
pub async fn autobaud_task(sm: AutobaudSm, dma: DmaChannel<'static>) -> ! {
    match sm {
        AutobaudSm::P0S0(c) => run(c, dma).await,
        AutobaudSm::P0S3(c) => run(c, dma).await,
        AutobaudSm::P1S0(c) => run(c, dma).await,
        AutobaudSm::P1S3(c) => run(c, dma).await,
        #[cfg(feature = "rp2350")]
        AutobaudSm::P2S0(c) => run(c, dma).await,
        #[cfg(feature = "rp2350")]
        AutobaudSm::P2S3(c) => run(c, dma).await,
    }
}
