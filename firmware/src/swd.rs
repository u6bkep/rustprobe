//! PIO-based SWD bit engine.
//!
//! Port of debugprobe's `probe.pio` + `probe.c` (the `PROBE_IO_RAW` variant:
//! SWDIO direction is switched with `out pindirs` inside the PIO program, no
//! external buffer OEn pin).
//!
//! Every TX FIFO entry is either a command word or up to 32 bits of data.
//! Command word format:
//!
//! ```text
//! | 13:9 |  8  |  7:0  |
//! | Cmd  | Dir | Count |
//! ```
//!
//! `Count` is the number of bits transferred, minus 1. `Dir` is the SWDIO
//! output enable. `Cmd` is the absolute PIO address of the `write_cmd`,
//! `read_cmd` or `get_next_cmd` routine. One SWCLK period is 4 SM cycles.

use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::gpio::{AnyPin, Flex, Pull};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Common, Config, Direction as PioDirection, Instance, LoadedProgram, Pin as PioPinHandle,
    ShiftDirection, StateMachine,
};
use embassy_rp::Peri;
use fixed::traits::ToFixed;

/// Jump targets within the loaded PIO program (absolute addresses).
#[derive(Clone, Copy)]
struct CmdAddrs {
    write: u8,
    /// `get_next_cmd`: a no-op command that only applies `Dir` — used to
    /// switch SWDIO direction without clocking.
    skip: u8,
    /// Alias of `write` in the RAW-IO program; clocks with SWDIO released.
    turnaround: u8,
    read: u8,
}

/// The SWD PIO program, loaded once per PIO block and shared by its state
/// machines.
pub struct SwdProgram<'d, P: Instance> {
    loaded: LoadedProgram<'d, P>,
    addrs: CmdAddrs,
}

impl<'d, P: Instance> SwdProgram<'d, P> {
    /// Assemble and load the SWD program into a PIO block.
    pub fn load(common: &mut Common<'d, P>) -> Self {
        let prg = pio_asm!(
            ".side_set 1 opt",
            "public write_cmd:",
            "public turnaround_cmd:",       // alias of write_cmd in RAW-IO mode
            "    pull",
            "write_bitloop:",
            "    out pins, 1        [1] side 0x0", // data output on negedge...
            "    jmp x-- write_bitloop [1] side 0x1", // ...captured by target on posedge
            ".wrap_target",
            "public get_next_cmd:",
            "    pull                   side 0x0", // SWCLK idles low
            "    out x, 8",                        // bit count
            "    out pindirs, 1",                  // SWDIO direction
            "    out pc, 5",                       // jump to command routine
            "read_bitloop:",
            "    nop",
            "public read_cmd:",
            "    in pins, 1         [1] side 0x1", // captured by host on posedge
            "    jmp x-- read_bitloop   side 0x0",
            "    push",
            ".wrap",
        );
        let loaded = common.load_program(&prg.program);
        let origin = loaded.origin;
        let addrs = CmdAddrs {
            write: origin + prg.public_defines.write_cmd as u8,
            skip: origin + prg.public_defines.get_next_cmd as u8,
            turnaround: origin + prg.public_defines.turnaround_cmd as u8,
            read: origin + prg.public_defines.read_cmd as u8,
        };
        Self { loaded, addrs }
    }
}

/// Object-safe SWD wire operations — what the DAP layer needs from an
/// engine, independent of which PIO block / state machine backs it.
pub trait SwdBus: Send {
    /// Set the SWCLK frequency.
    fn set_swclk_freq_khz(&mut self, freq_khz: u32);
    /// Clock out `bit_count` (1..=32) bits, LSB first, driving SWDIO.
    fn write_bits(&mut self, bit_count: u32, data: u32);
    /// Clock in `bit_count` (1..=32) bits, LSB first, SWDIO released.
    fn read_bits(&mut self, bit_count: u32) -> u32;
    /// Run `bit_count` clocks with SWDIO released (turnaround / line idle).
    fn hiz_clocks(&mut self, bit_count: u32);
    /// Switch SWDIO to input (probe releases the line). Blocks until applied.
    fn read_mode(&mut self);
    /// Switch SWDIO to output (probe drives the line). Blocks until applied.
    fn write_mode(&mut self);
    /// Drive (true) or release (false) the nRESET line.
    fn set_nreset(&mut self, asserted: bool);
    /// Current level of the nRESET line (true = high / released).
    fn nreset_level(&mut self) -> bool;
}

impl<'d, P: Instance + Send, const SM: usize> SwdBus for SwdEngine<'d, P, SM> {
    fn set_swclk_freq_khz(&mut self, freq_khz: u32) {
        SwdEngine::set_swclk_freq_khz(self, freq_khz)
    }
    fn write_bits(&mut self, bit_count: u32, data: u32) {
        SwdEngine::write_bits(self, bit_count, data)
    }
    fn read_bits(&mut self, bit_count: u32) -> u32 {
        SwdEngine::read_bits(self, bit_count)
    }
    fn hiz_clocks(&mut self, bit_count: u32) {
        SwdEngine::hiz_clocks(self, bit_count)
    }
    fn read_mode(&mut self) {
        SwdEngine::read_mode(self)
    }
    fn write_mode(&mut self) {
        SwdEngine::write_mode(self)
    }
    fn set_nreset(&mut self, asserted: bool) {
        SwdEngine::set_nreset(self, asserted)
    }
    fn nreset_level(&mut self) -> bool {
        SwdEngine::nreset_level(self)
    }
}

/// One SWD probe instance: a PIO state machine driving SWCLK/SWDIO, plus an
/// optional open-drain-emulated nRESET pin.
pub struct SwdEngine<'d, P: Instance, const SM: usize> {
    sm: StateMachine<'d, P, SM>,
    addrs: CmdAddrs,
    nreset: Option<Flex<'d>>,
    /// Last SWCLK frequency requested, to skip redundant divider writes.
    cached_freq_khz: u32,
}

impl<'d, P: Instance, const SM: usize> SwdEngine<'d, P, SM> {
    /// Set up GPIOs and start the state machine idling in `get_next_cmd`.
    /// `program` must be loaded into the same PIO block as `sm`.
    pub fn new(
        program: &SwdProgram<'d, P>,
        mut sm: StateMachine<'d, P, SM>,
        mut swclk: PioPinHandle<'d, P>,
        mut swdio: PioPinHandle<'d, P>,
        nreset: Option<Peri<'d, AnyPin>>,
    ) -> Self {
        let addrs = program.addrs;

        // SWDIO idles high; pull-up so reads see a released line as 1.
        swdio.set_pull(Pull::Up);
        swclk.set_pull(Pull::None);

        let mut cfg = Config::default();
        cfg.use_program(&program.loaded, &[&swclk]); // side-set = SWCLK
        cfg.set_out_pins(&[&swdio]);
        cfg.set_set_pins(&[&swdio]);
        cfg.set_in_pins(&[&swdio]);
        // Shift right (SWD is LSB-first), no autopull/autopush.
        cfg.shift_out.direction = ShiftDirection::Right;
        cfg.shift_out.auto_fill = false;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.auto_fill = false;
        sm.set_config(&cfg);
        // Both pins driven by the SM, starting as outputs.
        sm.set_pin_dirs(PioDirection::Out, &[&swclk, &swdio]);

        let nreset = nreset.map(|pin| {
            let mut nreset = Flex::new(pin);
            // Emulate open drain: input + pull-up = released, output-low = asserted.
            nreset.set_pull(Pull::Up);
            nreset.set_as_input();
            nreset.set_low();
            nreset
        });

        let mut this = Self { sm, addrs, nreset, cached_freq_khz: 0 };
        this.set_swclk_freq_khz(1000);

        // Enter the command loop and start.
        unsafe {
            use embassy_rp::pio::program::{InstructionOperands, JmpCondition};
            this.sm.exec_instr(
                InstructionOperands::JMP {
                    condition: JmpCondition::Always,
                    address: this.addrs.skip,
                }
                .encode(),
            );
        }
        this.sm.set_enable(true);
        this
    }

    fn fmt_command(&self, bit_count: u32, out_en: bool, cmd_addr: u8) -> u32 {
        ((bit_count - 1) & 0xff) | ((out_en as u32) << 8) | ((cmd_addr as u32) << 9)
    }

    fn push_blocking(&mut self, v: u32) {
        while !self.sm.tx().try_push(v) {}
    }

    fn pull_blocking(&mut self) -> u32 {
        loop {
            if let Some(v) = self.sm.rx().try_pull() {
                return v;
            }
        }
    }

    /// Set the SWCLK frequency. One SWCLK period is 4 SM cycles.
    pub fn set_swclk_freq_khz(&mut self, freq_khz: u32) {
        if freq_khz == self.cached_freq_khz || freq_khz == 0 {
            return;
        }
        self.cached_freq_khz = freq_khz;
        let clk_sys_khz = clk_sys_freq() / 1000;
        let divider = (clk_sys_khz.div_ceil(freq_khz) + 3) / 4;
        let divider = divider.clamp(1, 65535);
        self.sm.set_clock_divider(divider.to_fixed());
        defmt::debug!("SWCLK {} kHz -> divider {}", freq_khz, divider);
    }

    /// Clock out `bit_count` (1..=32) bits, LSB first, driving SWDIO.
    pub fn write_bits(&mut self, bit_count: u32, data: u32) {
        let cmd = self.fmt_command(bit_count, true, self.addrs.write);
        self.push_blocking(cmd);
        self.push_blocking(data);
    }

    /// Clock in `bit_count` (1..=32) bits, LSB first, SWDIO released.
    pub fn read_bits(&mut self, bit_count: u32) -> u32 {
        let cmd = self.fmt_command(bit_count, false, self.addrs.read);
        self.push_blocking(cmd);
        let data = self.pull_blocking();
        if bit_count < 32 {
            data >> (32 - bit_count)
        } else {
            data
        }
    }

    /// Run `bit_count` clocks with SWDIO released (turnaround / line idle).
    pub fn hiz_clocks(&mut self, bit_count: u32) {
        if bit_count == 0 {
            return;
        }
        let cmd = self.fmt_command(bit_count, false, self.addrs.turnaround);
        self.push_blocking(cmd);
        self.push_blocking(0);
    }

    /// Busy-wait until the SM has drained the TX FIFO and stalled on `pull`,
    /// i.e. all queued commands have completed on the wire.
    fn wait_idle(&mut self) {
        // `stalled()` is read-then-clear: first call clears any stale flag.
        self.sm.tx().stalled();
        while !self.sm.tx().stalled() {}
    }

    /// Switch SWDIO to input (probe releases the line). Blocks until applied.
    pub fn read_mode(&mut self) {
        let cmd = self.fmt_command(1, false, self.addrs.skip);
        self.push_blocking(cmd);
        self.wait_idle();
    }

    /// Switch SWDIO to output (probe drives the line). Blocks until applied.
    pub fn write_mode(&mut self) {
        let cmd = self.fmt_command(1, true, self.addrs.skip);
        self.push_blocking(cmd);
        self.wait_idle();
    }

    /// Drive (true) or release (false) the nRESET line.
    pub fn set_nreset(&mut self, asserted: bool) {
        if let Some(nreset) = &mut self.nreset {
            if asserted {
                nreset.set_as_output();
            } else {
                nreset.set_as_input();
            }
        }
    }

    /// Current level of the nRESET line (true = high / released).
    pub fn nreset_level(&mut self) -> bool {
        match &mut self.nreset {
            Some(nreset) => nreset.is_high(),
            None => true,
        }
    }
}
