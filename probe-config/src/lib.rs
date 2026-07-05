//! Shared configuration schema and validation for the rustprobe firmware.
//!
//! This crate is `no_std` and is compiled into both the firmware and host
//! tools (CLI, and any future frontend such as a touchscreen UI). All
//! validation lives here so every frontend rejects the same configs the
//! firmware would.
//!
//! The configuration model has two layers, both stored in flash on the probe:
//!
//! * [`BoardProfile`] — what the *hardware* allows: which GPIOs exist and
//!   which are reserved for other duties (onboard LED, W25Q pins on a Pico,
//!   display pins on a future touchscreen build, ...).
//! * [`Topology`] — what the *user* wants: N SWD probes and M UART bridges,
//!   with pin assignments. Validated against the profile and chip limits.

#![no_std]
#![deny(missing_docs)]

pub mod protocol;

use heapless::Vec;
use serde::{Deserialize, Serialize};

/// Hardware maximum number of SWD probe instances (RP2350: 3 PIO blocks × 4 SMs).
pub const MAX_PROBES: usize = 12;
/// Hardware maximum number of UART bridges (hardware UART peripherals).
pub const MAX_UARTS: usize = 2;

/// Pin assignment for one SWD probe instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProbeConfig {
    /// SWCLK GPIO number.
    pub swclk: u8,
    /// SWDIO GPIO number.
    pub swdio: u8,
    /// Optional nRESET GPIO number (open-drain emulated).
    pub reset: Option<u8>,
}

/// Pin assignment for one CDC-UART bridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct UartConfig {
    /// TX GPIO number (probe → target).
    pub tx: u8,
    /// RX GPIO number (target → probe).
    pub rx: u8,
    /// Initial baud rate.
    pub baud: u32,
}

/// The user-selected probe topology. This is the primary flash config block.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct Topology {
    /// SWD probe instances, in USB interface order.
    pub probes: Vec<ProbeConfig, MAX_PROBES>,
    /// UART bridges, in CDC interface order.
    pub uarts: Vec<UartConfig, MAX_UARTS>,
}

/// Which GPIOs a given board makes available. Secondary flash config block.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BoardProfile {
    /// Bitmask of GPIOs that exist and are wired out (bit N = GPIO N).
    pub available: u64,
    /// Bitmask of GPIOs reserved for non-probe duties (LEDs, display, ...).
    /// Reserved pins are never assignable even if `available`.
    pub reserved: u64,
}

impl BoardProfile {
    /// Profile for a bare Raspberry Pi Pico / Pico 2: GP0–GP22 + GP26–GP28
    /// wired to the headers; GP23–GP25 and GP29 are used by the board itself.
    pub const PICO: BoardProfile = BoardProfile {
        available: 0x1FFF_FFFF,
        reserved: 0x2380_0000,
    };

    /// True if `pin` may be assigned to a probe function.
    pub fn is_assignable(&self, pin: u8) -> bool {
        pin < 64 && (self.available & !self.reserved) & (1u64 << pin) != 0
    }
}

/// Per-chip hardware limits, selected by the firmware build (and reported to
/// host tools via the config protocol so they can pre-validate).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChipLimits {
    /// Number of GPIOs on the package.
    pub gpio_count: u8,
    /// Number of PIO blocks.
    pub pio_blocks: u8,
    /// State machines per PIO block.
    pub sms_per_block: u8,
    /// Usable USB endpoint numbers per direction (EP0 excluded).
    pub ep_numbers_per_dir: u8,
    /// Width of the pin window a single PIO block can address (RP2350B has
    /// more GPIOs than a PIO block can see at once; the block's GPIOBASE
    /// selects a window).
    pub pio_pin_window: u8,
}

/// RP2040 limits.
pub const RP2040: ChipLimits = ChipLimits {
    gpio_count: 30,
    pio_blocks: 2,
    sms_per_block: 4,
    ep_numbers_per_dir: 15,
    pio_pin_window: 32,
};

/// RP2350A (QFN-60, e.g. Pico 2) limits.
pub const RP2350A: ChipLimits = ChipLimits {
    gpio_count: 30,
    pio_blocks: 3,
    sms_per_block: 4,
    ep_numbers_per_dir: 15,
    pio_pin_window: 32,
};

/// RP2350B (QFN-80) limits. PIO blocks see a 32-pin window of the 48 GPIOs.
pub const RP2350B: ChipLimits = ChipLimits {
    gpio_count: 48,
    pio_blocks: 3,
    sms_per_block: 4,
    ep_numbers_per_dir: 15,
    pio_pin_window: 32,
};

/// Why a topology was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ValidationError {
    /// More probes than PIO state machines allow (accounting for autobaud).
    TooManyProbes,
    /// More UARTs than hardware UART peripherals.
    TooManyUarts,
    /// USB endpoint budget exceeded (IN: probes + 2×uarts, OUT: probes + uarts, each ≤ 15).
    EndpointBudget,
    /// A pin is out of range, unavailable, or reserved on this board.
    PinUnavailable(u8),
    /// The same pin is assigned twice.
    PinConflict(u8),
    /// Probes cannot be binned into PIO blocks such that each block's pins
    /// fit one GPIOBASE window (RP2350B).
    PioWindow,
}

impl Topology {
    /// Validate this topology against chip limits and a board profile.
    pub fn validate(&self, chip: &ChipLimits, profile: &BoardProfile) -> Result<(), ValidationError> {
        let n_probes = self.probes.len();
        let n_uarts = self.uarts.len();

        // PIO SM budget. Autobaud claims one SM (in any block) when at least
        // one UART is configured.
        let total_sms = (chip.pio_blocks as usize) * (chip.sms_per_block as usize);
        let autobaud_sms = if n_uarts > 0 { 1 } else { 0 };
        if n_probes + autobaud_sms > total_sms {
            return Err(ValidationError::TooManyProbes);
        }
        if n_uarts > MAX_UARTS {
            return Err(ValidationError::TooManyUarts);
        }

        // USB endpoint budget, per direction (IN and OUT allocate independently).
        let ep_in = n_probes + 2 * n_uarts; // bulk IN + CDC notification IN
        let ep_out = n_probes + n_uarts;
        if ep_in > chip.ep_numbers_per_dir as usize || ep_out > chip.ep_numbers_per_dir as usize {
            return Err(ValidationError::EndpointBudget);
        }

        // Pin availability and conflicts.
        let mut used: u64 = 0;
        let mut claim = |pin: u8| -> Result<(), ValidationError> {
            if pin >= chip.gpio_count || !profile.is_assignable(pin) {
                return Err(ValidationError::PinUnavailable(pin));
            }
            let bit = 1u64 << pin;
            if used & bit != 0 {
                return Err(ValidationError::PinConflict(pin));
            }
            used |= bit;
            Ok(())
        };
        for p in &self.probes {
            claim(p.swclk)?;
            claim(p.swdio)?;
            if let Some(r) = p.reset {
                claim(r)?;
            }
        }
        for u in &self.uarts {
            claim(u.tx)?;
            claim(u.rx)?;
        }

        // TODO(rp2350b): bin probes into PIO blocks such that each block's
        // SWCLK/SWDIO pins fit a single `pio_pin_window` GPIOBASE window, and
        // reject with `ValidationError::PioWindow` if impossible. On RP2040 /
        // RP2350A every pin fits the window, so this is a no-op there.

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn probe(swclk: u8, swdio: u8) -> ProbeConfig {
        ProbeConfig { swclk, swdio, reset: None }
    }

    #[test]
    fn stock_single_probe_valid() {
        let mut t = Topology::default();
        t.probes.push(ProbeConfig { swclk: 2, swdio: 3, reset: Some(1) }).unwrap();
        t.uarts.push(UartConfig { tx: 4, rx: 5, baud: 115200 }).unwrap();
        assert_eq!(t.validate(&RP2040, &BoardProfile::PICO), Ok(()));
    }

    #[test]
    fn pin_conflict_rejected() {
        let mut t = Topology::default();
        t.probes.push(probe(2, 3)).unwrap();
        t.probes.push(probe(3, 6)).unwrap();
        assert_eq!(t.validate(&RP2040, &BoardProfile::PICO), Err(ValidationError::PinConflict(3)));
    }

    #[test]
    fn reserved_pin_rejected() {
        let mut t = Topology::default();
        t.probes.push(probe(25, 3)).unwrap(); // GP25 = Pico onboard LED
        assert_eq!(
            t.validate(&RP2040, &BoardProfile::PICO),
            Err(ValidationError::PinUnavailable(25))
        );
    }

    #[test]
    fn sm_budget_counts_autobaud() {
        let mut t = Topology::default();
        // 8 probes consume all RP2040 SMs; adding a UART needs a 9th for autobaud.
        for i in 0..8 {
            t.probes.push(probe(2 * i, 2 * i + 1)).unwrap();
        }
        assert_eq!(t.validate(&RP2040, &BoardProfile::PICO), Ok(()));
        t.uarts.push(UartConfig { tx: 16, rx: 17, baud: 115200 }).unwrap();
        assert_eq!(t.validate(&RP2040, &BoardProfile::PICO), Err(ValidationError::TooManyProbes));
    }
}
