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
use embassy_rp::Peri;
use heapless::Vec;
use probe_config::{ProbeConfig, Topology, MAX_PROBES};
use static_cell::StaticCell;

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

/// PIO peripherals handed to `build_engines`.
pub struct PioBlocks {
    pub pio0: Peri<'static, embassy_rp::peripherals::PIO0>,
    pub pio1: Peri<'static, embassy_rp::peripherals::PIO1>,
    #[cfg(feature = "rp2350")]
    pub pio2: Peri<'static, embassy_rp::peripherals::PIO2>,
}

/// Create one engine per (block, sm) slot used by `topo`, in probe order.
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
) {
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
    slot!(3, sm3, cells.3);

    core::mem::forget(common);
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

/// Instantiate the SWD engines for a validated topology.
pub fn build_engines(
    topo: &Topology,
    blocks: PioBlocks,
    pins: &mut PinTable,
) -> Vec<DynEngine, MAX_PROBES> {
    let mut engines: Vec<DynEngine, MAX_PROBES> = Vec::new();
    let probes = topo.probes.as_slice();

    if !probes.is_empty() {
        block_engines(
            Pio::new(blocks.pio0, Irqs),
            &probes[..probes.len().min(4)],
            pins,
            &mut engines,
            block_cells!(embassy_rp::peripherals::PIO0),
        );
    }
    if probes.len() > 4 {
        block_engines(
            Pio::new(blocks.pio1, Irqs),
            &probes[4..probes.len().min(8)],
            pins,
            &mut engines,
            block_cells!(embassy_rp::peripherals::PIO1),
        );
    }
    #[cfg(feature = "rp2350")]
    if probes.len() > 8 {
        block_engines(
            Pio::new(blocks.pio2, crate::IrqsPio2),
            &probes[8..probes.len().min(12)],
            pins,
            &mut engines,
            block_cells!(embassy_rp::peripherals::PIO2),
        );
    }

    engines
}
