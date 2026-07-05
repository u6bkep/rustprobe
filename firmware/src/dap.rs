//! dap-rs trait implementations over the PIO SWD engine.
//!
//! The SWD transfer logic is a port of debugprobe's `sw_dp_pio.c`
//! (`SWD_Transfer` et al.), restructured into dap-rs's `Swd` trait shape.
//!
//! Differences from the C original, both deliberate:
//! * Turnaround is fixed at 1 cycle and the WAIT/FAULT data phase is off
//!   (dap-rs's `swd::Config` carries no fields for them yet; probe-rs uses
//!   the defaults).
//! * Eight trailing idle clocks are driven after every successful transfer.
//!   dap-rs parses `DAP_TransferConfigure`'s idle_cycles but never applies
//!   it, and without trailing clocks the last write of a sequence never
//!   commits. Matches rusty-probe-firmware behavior.

use dap_rs::dap::{DapLeds, HostStatus};
use dap_rs::swd::{APnDP, Ack, DPRegister, RnW};
use dap_rs::{jtag, swd, swj};
use embassy_rp::pio::Instance;
use embassy_time::{block_for, Duration, Instant};

use crate::swd::SwdEngine;

/// Fixed 1-cycle turnaround (see module docs).
const TRN: u32 = 1;

/// Owns one probe instance's hardware; morphed between mode states by dap-rs.
pub struct ProbeContext<'d, P: Instance, const SM: usize> {
    engine: SwdEngine<'d, P, SM>,
    swd_config: swd::Config,
    jtag_config: jtag::Config,
}

impl<'d, P: Instance, const SM: usize> ProbeContext<'d, P, SM> {
    pub fn new(engine: SwdEngine<'d, P, SM>) -> Self {
        Self {
            engine,
            swd_config: swd::Config::default(),
            // Empty scan chain: JTAG is reported unavailable.
            jtag_config: jtag::Config::new(&mut []),
        }
    }

    /// Shared SWJ clock handler (Hz). Returns false for 0.
    fn set_swj_clock(&mut self, max_frequency: u32) -> bool {
        if max_frequency == 0 {
            return false;
        }
        self.engine.set_swclk_freq_khz((max_frequency / 1000).max(1));
        true
    }
}

impl<'d, P: Instance, const SM: usize> swj::Dependencies<ProbeSwd<'d, P, SM>, ProbeJtag<'d, P, SM>>
    for ProbeContext<'d, P, SM>
{
    fn process_swj_pins(&mut self, output: swj::Pins, mask: swj::Pins, wait_us: u32) -> swj::Pins {
        // SWCLK/SWDIO are owned by the PIO state machine and are not
        // individually drivable; like the C firmware, only nRESET is
        // supported here (open-drain: high = released).
        if mask.contains(swj::Pins::NRESET) {
            self.engine.set_nreset(!output.contains(swj::Pins::NRESET));
            if wait_us > 0 {
                let deadline = Instant::now() + Duration::from_micros(wait_us as u64);
                while self.engine.nreset_level() != output.contains(swj::Pins::NRESET)
                    && Instant::now() < deadline
                {
                    block_for(Duration::from_micros(1));
                }
            }
        }
        let mut ret = swj::Pins::empty();
        ret.set(swj::Pins::NRESET, self.engine.nreset_level());
        ret
    }

    fn process_swj_sequence(&mut self, data: &[u8], mut bits: usize) {
        for byte in data {
            if bits == 0 {
                break;
            }
            let chunk = bits.min(8);
            self.engine.write_bits(chunk as u32, *byte as u32);
            bits -= chunk;
        }
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        self.set_swj_clock(max_frequency)
    }

    fn high_impedance_mode(&mut self) {
        self.engine.read_mode();
        self.engine.set_nreset(false);
    }

    fn swd_config(&mut self) -> &mut swd::Config {
        &mut self.swd_config
    }

    fn jtag_config(&mut self) -> &mut jtag::Config {
        &mut self.jtag_config
    }
}

/// SWD mode state.
pub struct ProbeSwd<'d, P: Instance, const SM: usize>(ProbeContext<'d, P, SM>);

impl<'d, P: Instance, const SM: usize> From<ProbeContext<'d, P, SM>> for ProbeSwd<'d, P, SM> {
    fn from(mut ctx: ProbeContext<'d, P, SM>) -> Self {
        ctx.engine.write_mode();
        Self(ctx)
    }
}

impl<'d, P: Instance, const SM: usize> From<ProbeSwd<'d, P, SM>> for ProbeContext<'d, P, SM> {
    fn from(swd: ProbeSwd<'d, P, SM>) -> Self {
        swd.0
    }
}

impl<'d, P: Instance, const SM: usize> ProbeSwd<'d, P, SM> {
    /// Read the 3-bit ACK (preceded by the read turnaround).
    fn read_ack(&mut self) -> Result<(), swd::Error> {
        let ack = (self.0.engine.read_bits(TRN + 3) >> TRN) as u8;
        Ack::try_ok(ack)
    }

    /// Recovery clocking after a non-OK ACK, per the C original.
    fn abort_data_phase(&mut self, err: &swd::Error) {
        match err {
            swd::Error::AckWait | swd::Error::AckFault => {
                // Turnaround back to host-driven, no data phase.
                self.0.engine.hiz_clocks(TRN);
            }
            _ => {
                // Protocol error: back off a full data phase on the line.
                self.0.engine.read_bits(TRN + 32);
                self.0.engine.read_bits(1);
            }
        }
    }

    /// Trailing idle clocks, driving SWDIO low (see module docs).
    fn trailing_idle(&mut self) {
        self.0.engine.write_bits(8, 0);
    }
}

impl<'d, P: Instance, const SM: usize> swd::Swd<ProbeContext<'d, P, SM>> for ProbeSwd<'d, P, SM> {
    fn available(_deps: &ProbeContext<'d, P, SM>) -> bool {
        true
    }

    fn timestamp(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    fn config(&mut self) -> &mut swd::Config {
        &mut self.0.swd_config
    }

    fn read_inner(&mut self, apndp: APnDP, a: DPRegister) -> swd::Result<u32> {
        let req = swd::make_request(apndp, RnW::R, a);
        self.0.engine.write_bits(8, req as u32);

        if let Err(e) = self.read_ack() {
            self.abort_data_phase(&e);
            return Err(e);
        }

        let val = self.0.engine.read_bits(32);
        let parity = self.0.engine.read_bits(1);
        // Turnaround back to host-driven, then park the line.
        self.0.engine.hiz_clocks(TRN);
        self.trailing_idle();

        if (val.count_ones() as u32 ^ parity) & 1 != 0 {
            return Err(swd::Error::BadParity);
        }
        Ok(val)
    }

    fn write_inner(&mut self, apndp: APnDP, a: DPRegister, data: u32) -> swd::Result<()> {
        let req = swd::make_request(apndp, RnW::W, a);
        self.0.engine.write_bits(8, req as u32);

        if let Err(e) = self.read_ack() {
            self.abort_data_phase(&e);
            return Err(e);
        }

        // Turnaround to host-driven, then data + parity.
        self.0.engine.hiz_clocks(TRN);
        self.0.engine.write_bits(32, data);
        self.0.engine.write_bits(1, data.count_ones() & 1);
        self.trailing_idle();
        Ok(())
    }

    fn write_sequence(&mut self, mut num_bits: usize, data: &[u8]) -> swd::Result<()> {
        for byte in data {
            if num_bits == 0 {
                break;
            }
            let chunk = num_bits.min(8);
            self.0.engine.write_bits(chunk as u32, *byte as u32);
            num_bits -= chunk;
        }
        Ok(())
    }

    fn read_sequence(&mut self, mut num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        for byte in data {
            if num_bits == 0 {
                break;
            }
            let chunk = num_bits.min(8);
            *byte = self.0.engine.read_bits(chunk as u32) as u8;
            num_bits -= chunk;
        }
        Ok(())
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_swj_clock(max_frequency)
    }
}

/// JTAG mode state — not implemented (reported unavailable), kept as the
/// expansion seam for a future JTAG PIO program.
pub struct ProbeJtag<'d, P: Instance, const SM: usize>(ProbeContext<'d, P, SM>);

impl<'d, P: Instance, const SM: usize> From<ProbeContext<'d, P, SM>> for ProbeJtag<'d, P, SM> {
    fn from(ctx: ProbeContext<'d, P, SM>) -> Self {
        Self(ctx)
    }
}

impl<'d, P: Instance, const SM: usize> From<ProbeJtag<'d, P, SM>> for ProbeContext<'d, P, SM> {
    fn from(jtag: ProbeJtag<'d, P, SM>) -> Self {
        jtag.0
    }
}

impl<'d, P: Instance, const SM: usize> jtag::Jtag<ProbeContext<'d, P, SM>> for ProbeJtag<'d, P, SM> {
    fn available(_deps: &ProbeContext<'d, P, SM>) -> bool {
        false
    }

    fn config(&mut self) -> &mut jtag::Config {
        &mut self.0.jtag_config
    }

    fn sequence(&mut self, _info: jtag::SequenceInfo, _tdi: &[u8], _rxbuf: &mut [u8]) {}

    fn tms_sequence(&mut self, _tms: &[bool]) {}

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_swj_clock(max_frequency)
    }
}

/// LED feedback — placeholder until board LEDs are wired up.
pub struct NoLeds;

impl DapLeds for NoLeds {
    fn react_to_host_status(&mut self, host_status: HostStatus) {
        defmt::debug!("DAP host status changed");
        let _ = host_status;
    }
}
