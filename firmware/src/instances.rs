//! Runtime instantiation of probe engines from a validated `Topology`.
//!
//! Two pieces of impedance matching between "config is data" and embassy's
//! compile-time-typed hardware model:
//!
//! * [`PinTable`] — GPIOs are distinct types; this holds them all as
//!   `Option`s and hands them out by number.
//! * [`build_engines`] — PIO state machines are const-generic types; each
//!   (block, sm) slot gets a statically allocated `SwdEngine` and is returned
//!   type-erased as `&'static mut dyn SwdBus`.
//!
//! Probe N maps to PIO block N/4, state machine N%4, as in the C firmware.

use embassy_rp::gpio::AnyPin;
use embassy_rp::pio::{Common, Instance, Pin as PioPinHandle, Pio};
use embassy_rp::uart::{self, BufferedUart};
use embassy_rp::Peri;
use heapless::Vec;
use probe_config::{ProbeConfig, Topology, MAX_PROBES};
use static_cell::StaticCell;

use crate::autobaud::{AutobaudCapture, AutobaudSm};
use crate::swd::{SwdBus, SwdEngine, SwdProgram};
use crate::Irqs;

/// A type-erased probe engine.
pub type DynEngine = &'static mut dyn SwdBus;

macro_rules! pin_table {
    ($( $field:ident : $pin:ident => $num:literal, )*) => {
        /// All user-assignable GPIOs, claimable by number.
        pub struct PinTable {
            $( $field: Option<Peri<'static, embassy_rp::peripherals::$pin>>, )*
        }

        impl PinTable {
            /// Build the table from the peripheral singletons.
            pub fn new($( $field: Peri<'static, embassy_rp::peripherals::$pin>, )*) -> Self {
                Self { $( $field: Some($field), )* }
            }

            /// Claim a pin for PIO use on block `P`. Panics if the pin was
            /// already claimed or doesn't exist (validation prevents both).
            pub fn claim_pio<P: Instance>(
                &mut self,
                common: &mut Common<'static, P>,
                n: u8,
            ) -> PioPinHandle<'static, P> {
                match n {
                    $( $num => common.make_pio_pin(self.$field.take().expect("pin already claimed")), )*
                    _ => panic!("no such pin"),
                }
            }

            /// Claim a pin for plain GPIO use.
            pub fn claim_gpio(&mut self, n: u8) -> Peri<'static, AnyPin> {
                match n {
                    $( $num => self.$field.take().expect("pin already claimed").into(), )*
                    _ => panic!("no such pin"),
                }
            }
        }
    };
}

pin_table! {
    p0: PIN_0 => 0, p1: PIN_1 => 1, p2: PIN_2 => 2, p3: PIN_3 => 3,
    p4: PIN_4 => 4, p5: PIN_5 => 5, p6: PIN_6 => 6, p7: PIN_7 => 7,
    p8: PIN_8 => 8, p9: PIN_9 => 9, p10: PIN_10 => 10, p11: PIN_11 => 11,
    p12: PIN_12 => 12, p13: PIN_13 => 13, p14: PIN_14 => 14, p15: PIN_15 => 15,
    p16: PIN_16 => 16, p17: PIN_17 => 17, p18: PIN_18 => 18, p19: PIN_19 => 19,
    p20: PIN_20 => 20, p21: PIN_21 => 21, p22: PIN_22 => 22, p23: PIN_23 => 23,
    p24: PIN_24 => 24, p25: PIN_25 => 25, p26: PIN_26 => 26, p27: PIN_27 => 27,
    p28: PIN_28 => 28, p29: PIN_29 => 29,
}

/// Generate `PinTable::claim_uart{0,1}`, the UART analogue of `claim_pio`.
///
/// `BufferedUart::new` needs pin *values whose types* implement
/// `TxPin<UARTx>`/`RxPin<UARTx>`, so — as with [`PinTable::claim_pio`] — we
/// match the runtime pin numbers against the compile-time-legal pins and build
/// the driver inside the matching arm. TX and RX mux independently, so the arms
/// enumerate the `(tx, rx)` cross product for the instance (given flat, since
/// both pin lists are captured at the same macro depth and can't be nested).
///
/// Only the pin set common to RP2040 and RP2350 is handled; a validated
/// topology (`UartConfig::instance`) never presents anything else, so the `_`
/// arm is unreachable in practice. See `probe_config::UartConfig::instance` for
/// the RP2350-only alternate pins not yet supported here.
macro_rules! uart_claimer {
    (
        $fn:ident, $inst:ident,
        [ $( ($tx:literal, $txf:ident, $rx:literal, $rxf:ident) ),* $(,)? ] $(,)?
    ) => {
        impl PinTable {
            /// Claim a hardware UART's TX/RX pins and build its `BufferedUart`.
            /// Panics on an already-claimed pin or an illegal pin pair
            /// (validation prevents both).
            pub fn $fn(
                &mut self,
                uart: Peri<'static, embassy_rp::peripherals::$inst>,
                config: uart::Config,
                tx_pin: u8,
                rx_pin: u8,
                tx_buf: &'static mut [u8],
                rx_buf: &'static mut [u8],
            ) -> BufferedUart {
                match (tx_pin, rx_pin) {
                    $(
                        ($tx, $rx) => {
                            let tx = self.$txf.take().expect("uart tx pin already claimed");
                            let rx = self.$rxf.take().expect("uart rx pin already claimed");
                            BufferedUart::new(uart, tx, rx, crate::Irqs, tx_buf, rx_buf, config)
                        }
                    )*
                    _ => panic!("illegal uart pin pair (validation should prevent this)"),
                }
            }
        }
    };
}

uart_claimer!(
    claim_uart0, UART0,
    [
        (0, p0, 1, p1),
        (0, p0, 13, p13),
        (0, p0, 17, p17),
        (0, p0, 29, p29),
        (12, p12, 1, p1),
        (12, p12, 13, p13),
        (12, p12, 17, p17),
        (12, p12, 29, p29),
        (16, p16, 1, p1),
        (16, p16, 13, p13),
        (16, p16, 17, p17),
        (16, p16, 29, p29),
        (28, p28, 1, p1),
        (28, p28, 13, p13),
        (28, p28, 17, p17),
        (28, p28, 29, p29),
    ],
);
uart_claimer!(
    claim_uart1, UART1,
    [
        (4, p4, 5, p5),
        (4, p4, 9, p9),
        (4, p4, 21, p21),
        (4, p4, 25, p25),
        (8, p8, 5, p5),
        (8, p8, 9, p9),
        (8, p8, 21, p21),
        (8, p8, 25, p25),
        (20, p20, 5, p5),
        (20, p20, 9, p9),
        (20, p20, 21, p21),
        (20, p20, 25, p25),
        (24, p24, 5, p5),
        (24, p24, 9, p9),
        (24, p24, 21, p21),
        (24, p24, 25, p25),
    ],
);

/// PIO peripherals handed to `build_engines`.
pub struct PioBlocks {
    pub pio0: Peri<'static, embassy_rp::peripherals::PIO0>,
    pub pio1: Peri<'static, embassy_rp::peripherals::PIO1>,
    #[cfg(feature = "rp2350")]
    pub pio2: Peri<'static, embassy_rp::peripherals::PIO2>,
}

/// Create one engine per (block, sm) slot used by `probes`, in probe order.
///
/// When `reserve_sm3` is set, SM3 is handed to autobaud instead of being
/// forgotten: the autobaud program is loaded into this block's `Common` and the
/// prepared capture is returned. `reserve_sm3` is only set for a block that has
/// fewer than four probes (so SM3 carries no probe), and only when a UART is
/// configured — see [`build_engines`].
fn block_engines<P: Instance + Send>(
    pio: Pio<'static, P>,
    probes: &[ProbeConfig],
    pins: &mut PinTable,
    engines: &mut Vec<DynEngine, MAX_PROBES>,
    cells: (
        &'static StaticCell<SwdEngine<'static, P, 0>>,
        &'static StaticCell<SwdEngine<'static, P, 1>>,
        &'static StaticCell<SwdEngine<'static, P, 2>>,
        &'static StaticCell<SwdEngine<'static, P, 3>>,
    ),
    reserve_sm3: bool,
) -> Option<AutobaudCapture<'static, P, 3>> {
    let Pio { mut common, sm0, sm1, sm2, sm3, .. } = pio;
    let program = SwdProgram::load(&mut common);

    // Dropping PIO handles decrements the block's user refcount, and when it
    // hits 1 embassy resets the funcsel of EVERY pin the block ever claimed
    // (`on_pio_drop`) — disconnecting live probes. Engines keep their own SM
    // alive in a StaticCell; unused SMs and the Common must never be dropped.
    macro_rules! slot {
        ($k:literal, $sm:expr, $cell:expr) => {
            if let Some(cfg) = probes.get($k) {
                let swclk = pins.claim_pio(&mut common, cfg.swclk);
                let swdio = pins.claim_pio(&mut common, cfg.swdio);
                let nreset = cfg.reset.map(|n| pins.claim_gpio(n));
                let engine = SwdEngine::new(&program, $sm, swclk, swdio, nreset);
                engines.push($cell.init(engine)).ok().expect("MAX_PROBES");
            } else {
                core::mem::forget($sm);
            }
        };
    }

    slot!(0, sm0, cells.0);
    slot!(1, sm1, cells.1);
    slot!(2, sm2, cells.2);

    let autobaud = if reserve_sm3 {
        // Autobaud shares this block: load its program into the same Common and
        // keep SM3. `common` is forgotten below, so the loaded program (and the
        // block's funcsel state) survive for the firmware's life.
        Some(AutobaudCapture::new(&mut common, sm3))
    } else {
        slot!(3, sm3, cells.3);
        None
    };

    core::mem::forget(common);
    autobaud
}

/// Claim a probe-free PIO block solely for autobaud (SM0). The block's other
/// SMs and its `Common` are forgotten; the block claims no pins (autobaud reads
/// the raw RX input), so nothing is disconnected by forgetting them.
fn fresh_autobaud_block<P: Instance + Send>(
    pio: Pio<'static, P>,
) -> AutobaudCapture<'static, P, 0> {
    let Pio { mut common, sm0, sm1, sm2, sm3, .. } = pio;
    let cap = AutobaudCapture::new(&mut common, sm0);
    core::mem::forget(sm1);
    core::mem::forget(sm2);
    core::mem::forget(sm3);
    core::mem::forget(common);
    cap
}

macro_rules! block_cells {
    ($p:ty) => {{
        static C0: StaticCell<SwdEngine<'static, $p, 0>> = StaticCell::new();
        static C1: StaticCell<SwdEngine<'static, $p, 1>> = StaticCell::new();
        static C2: StaticCell<SwdEngine<'static, $p, 2>> = StaticCell::new();
        static C3: StaticCell<SwdEngine<'static, $p, 3>> = StaticCell::new();
        (&C0, &C1, &C2, &C3)
    }};
}

/// Instantiate the SWD engines for a validated topology, plus the autobaud
/// capture SM when at least one UART is configured.
///
/// Probes fill slots from PIO0/SM0 upward (probe N → block N/4, SM N%4). The
/// SM `probe_config`'s validation reserves for autobaud is the next free slot,
/// which is therefore either SM3 of the last partially-filled block (when the
/// probe count isn't a multiple of four) or SM0 of the next, otherwise-unused
/// block. Only those two SM indices are ever used, keeping [`AutobaudSm`] small.
pub fn build_engines(
    topo: &Topology,
    blocks: PioBlocks,
    pins: &mut PinTable,
) -> (Vec<DynEngine, MAX_PROBES>, Option<AutobaudSm>) {
    let mut engines: Vec<DynEngine, MAX_PROBES> = Vec::new();
    let probes = topo.probes.as_slice();
    let n = probes.len();
    let has_uart = !topo.uarts.is_empty();

    // Which block the reserved autobaud SM lives in, and whether it shares a
    // partially-filled block (SM3) or claims a fresh one (SM0).
    let ab_block = has_uart.then_some(n / 4);
    let shares = |block: usize| ab_block == Some(block) && n % 4 != 0;
    let fresh = |block: usize| ab_block == Some(block) && n % 4 == 0;

    let mut autobaud: Option<AutobaudSm> = None;

    // Block 0 (PIO0).
    if !probes.is_empty() {
        let cap = block_engines(
            Pio::new(blocks.pio0, Irqs),
            &probes[..n.min(4)],
            pins,
            &mut engines,
            block_cells!(embassy_rp::peripherals::PIO0),
            shares(0),
        );
        autobaud = cap.map(AutobaudSm::P0S3);
    } else if fresh(0) {
        autobaud = Some(AutobaudSm::P0S0(fresh_autobaud_block(Pio::new(blocks.pio0, Irqs))));
    }

    // Block 1 (PIO1).
    if n > 4 {
        let cap = block_engines(
            Pio::new(blocks.pio1, Irqs),
            &probes[4..n.min(8)],
            pins,
            &mut engines,
            block_cells!(embassy_rp::peripherals::PIO1),
            shares(1),
        );
        if let Some(c) = cap {
            autobaud = Some(AutobaudSm::P1S3(c));
        }
    } else if fresh(1) {
        autobaud = Some(AutobaudSm::P1S0(fresh_autobaud_block(Pio::new(blocks.pio1, Irqs))));
    }

    // Block 2 (PIO2, RP2350 only).
    #[cfg(feature = "rp2350")]
    if n > 8 {
        let cap = block_engines(
            Pio::new(blocks.pio2, crate::IrqsPio2),
            &probes[8..n.min(12)],
            pins,
            &mut engines,
            block_cells!(embassy_rp::peripherals::PIO2),
            shares(2),
        );
        if let Some(c) = cap {
            autobaud = Some(AutobaudSm::P2S3(c));
        }
    } else if fresh(2) {
        autobaud = Some(AutobaudSm::P2S0(fresh_autobaud_block(Pio::new(blocks.pio2, crate::IrqsPio2))));
    }

    (engines, autobaud)
}
