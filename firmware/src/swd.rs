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
//!
//! The engine's FIFOs are reached two ways: [`FifoRegs`] gives the async
//! driver in `dap.rs` raw register access for its hot loop, and the
//! object-safe [`SwdBus`] carries everything else (clock, direction, nRESET),
//! so one driver type serves every PIO block / state machine combination.

use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::gpio::{AnyPin, Flex, Pull};
use embassy_rp::pac;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Common, Config, Direction as PioDirection, Instance, LoadedProgram, Pin as PioPinHandle,
    ShiftDirection, StateMachine,
};
use embassy_rp::Peri;
use fixed::traits::ToFixed;

/// A PIO block's register block, for [`FifoRegs`]. embassy-rp keeps the
/// equivalent item sealed.
pub trait PioBlock: Instance {
    /// Register block.
    const PAC: pac::pio::Pio;
}

impl PioBlock for embassy_rp::peripherals::PIO0 {
    const PAC: pac::pio::Pio = pac::PIO0;
}

impl PioBlock for embassy_rp::peripherals::PIO1 {
    const PAC: pac::pio::Pio = pac::PIO1;
}

#[cfg(feature = "rp2350")]
impl PioBlock for embassy_rp::peripherals::PIO2 {
    const PAC: pac::pio::Pio = pac::PIO2;
}

/// Jump targets within the loaded PIO program (absolute addresses).
#[derive(Clone, Copy)]
pub struct CmdAddrs {
    pub write: u8,
    /// `get_next_cmd`: a no-op command that only applies `Dir` — used to
    /// switch SWDIO direction without clocking.
    pub skip: u8,
    /// Alias of `write` in the RAW-IO program; clocks with SWDIO released.
    pub turnaround: u8,
    pub read: u8,
}

impl CmdAddrs {
    /// Format a command word: `bits` (1..=256) transferred, SWDIO output
    /// enable, routine address.
    #[inline(always)]
    pub fn cmd(bits: u32, out_en: bool, addr: u8) -> u32 {
        ((bits - 1) & 0xff) | ((out_en as u32) << 8) | ((addr as u32) << 9)
    }
}

/// Raw access to one state machine's FIFOs, for the driver's hot loop:
/// a `dyn` call per FIFO word costs more than the word itself.
///
/// The pointers come from the PAC and stay valid for the firmware's life;
/// the engine that handed them out owns the state machine exclusively, so
/// whoever holds the engine may use them.
#[derive(Clone, Copy)]
pub struct FifoRegs {
    fstat: *const u32,
    txf: *mut u32,
    rxf: *const u32,
    txfull: u32,
    rxempty: u32,
}

unsafe impl Send for FifoRegs {}

impl FifoRegs {
    /// Push one TX FIFO entry if there is room.
    #[inline(always)]
    pub fn try_push(&self, v: u32) -> bool {
        unsafe {
            if self.fstat.read_volatile() & self.txfull != 0 {
                return false;
            }
            self.txf.write_volatile(v);
        }
        true
    }

    /// Take one RX FIFO entry if there is one.
    #[inline(always)]
    pub fn try_pull(&self) -> Option<u32> {
        unsafe {
            if self.fstat.read_volatile() & self.rxempty != 0 {
                return None;
            }
            Some(self.rxf.read_volatile())
        }
    }
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

/// Object-safe, non-blocking view of one engine — what the async driver in
/// `dap.rs` needs, independent of which PIO block / state machine backs it.
///
/// The hot path moves FIFO words through [`FifoRegs`] directly.
pub trait SwdBus: Send {
    /// Set the SWCLK frequency.
    fn set_swclk_freq_khz(&mut self, freq_khz: u32);
    /// The SWCLK frequency last set.
    fn swclk_khz(&self) -> u32;
    /// Raw FIFO registers of this engine's state machine.
    fn fifo(&self) -> FifoRegs;
    /// Command routine addresses of this engine's program.
    fn addrs(&self) -> CmdAddrs;
    /// Read-and-clear: the SM has stalled on `pull` (TX FIFO drained) since
    /// the last call.
    fn tx_stalled(&mut self) -> bool;
    /// Drive (true) or release (false) the nRESET line.
    fn set_nreset(&mut self, asserted: bool);
    /// Current level of the nRESET line (true = high / released).
    fn nreset_level(&mut self) -> bool;
}

/// One SWD probe instance: a PIO state machine driving SWCLK/SWDIO, plus an
/// optional open-drain-emulated nRESET pin.
pub struct SwdEngine<'d, P: PioBlock, const SM: usize> {
    sm: StateMachine<'d, P, SM>,
    addrs: CmdAddrs,
    nreset: Option<Flex<'d>>,
    /// Last SWCLK frequency requested, to skip redundant divider writes.
    cached_freq_khz: u32,
}

impl<'d, P: PioBlock, const SM: usize> SwdEngine<'d, P, SM> {
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
}

impl<'d, P: PioBlock + Send, const SM: usize> SwdBus for SwdEngine<'d, P, SM> {
    fn set_swclk_freq_khz(&mut self, freq_khz: u32) {
        SwdEngine::set_swclk_freq_khz(self, freq_khz)
    }
    fn swclk_khz(&self) -> u32 {
        self.cached_freq_khz
    }
    fn fifo(&self) -> FifoRegs {
        // FSTAT: RXEMPTY at bits 8..11, TXFULL at bits 16..19, one per SM.
        FifoRegs {
            fstat: P::PAC.fstat().as_ptr() as *const u32,
            txf: P::PAC.txf(SM).as_ptr(),
            rxf: P::PAC.rxf(SM).as_ptr() as *const u32,
            txfull: 1 << (16 + SM),
            rxempty: 1 << (8 + SM),
        }
    }
    fn addrs(&self) -> CmdAddrs {
        self.addrs
    }
    fn tx_stalled(&mut self) -> bool {
        self.sm.tx().stalled()
    }
    fn set_nreset(&mut self, asserted: bool) {
        if let Some(nreset) = &mut self.nreset {
            if asserted {
                nreset.set_as_output();
            } else {
                nreset.set_as_input();
            }
        }
    }
    fn nreset_level(&mut self) -> bool {
        match &mut self.nreset {
            Some(nreset) => nreset.is_high(),
            None => true,
        }
    }
}
