//! The hardware driver interface and SWD wire-level definitions.

/// Request-byte flags of `DAP_Transfer` / `DAP_TransferBlock`. The low
/// nibble is also the SWD request packet payload (APnDP, RnW, A2, A3).
#[allow(missing_docs)]
pub mod req {
    pub const APNDP: u8 = 1 << 0;
    pub const RNW: u8 = 1 << 1;
    pub const A2: u8 = 1 << 2;
    pub const A3: u8 = 1 << 3;
    pub const MATCH_VALUE: u8 = 1 << 4;
    pub const MATCH_MASK: u8 = 1 << 5;
    pub const TIMESTAMP: u8 = 1 << 7;
    /// Read of DP `RDBUFF` (address 0xC): collects a posted AP read.
    pub const RDBUFF_READ: u8 = RNW | A2 | A3;
    /// Write of DP `ABORT` (address 0x0).
    pub const ABORT_WRITE: u8 = 0;
}

/// Transfer status values (the `Transfer Response` byte's low bits).
#[allow(missing_docs)]
pub mod ack {
    pub const OK: u8 = 0b001;
    pub const WAIT: u8 = 0b010;
    pub const FAULT: u8 = 0b100;
    pub const NO_ACK: u8 = 0b111;
    /// Bit 3: protocol error (bad read parity).
    pub const ERROR: u8 = 1 << 3;
    /// Bit 4: value mismatch (`DAP_Transfer` only).
    pub const MISMATCH: u8 = 1 << 4;
}

/// The 8-bit SWD request packet for a request nibble: start, APnDP, RnW,
/// A2, A3, parity, stop, park. LSB first on the wire.
pub fn packet(request: u8) -> u8 {
    let nibble = request & 0x0F;
    let parity = (nibble.count_ones() & 1) as u8;
    1 | (nibble << 1) | (parity << 5) | (1 << 7)
}

/// Pins of `DAP_SWJ_Pins`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Pins(pub u8);

#[allow(missing_docs)]
impl Pins {
    pub const SWCLK: Pins = Pins(1 << 0);
    pub const SWDIO: Pins = Pins(1 << 1);
    pub const TDI: Pins = Pins(1 << 2);
    pub const TDO: Pins = Pins(1 << 3);
    pub const NTRST: Pins = Pins(1 << 5);
    pub const NRESET: Pins = Pins(1 << 7);

    /// True when every bit of `other` is set.
    pub fn contains(self, other: Pins) -> bool {
        self.0 & other.0 == other.0
    }
}

/// Wire parameters of one SWD transfer, from `DAP_SWD_Configure` and
/// `DAP_TransferConfigure`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransferParams {
    /// Turnaround clocks, 1..=4.
    pub turnaround: u8,
    /// Clock a dummy data phase through on WAIT/FAULT.
    pub data_phase: bool,
    /// Idle clocks (SWDIO low) after a successful transfer.
    pub idle_cycles: u8,
}

impl Default for TransferParams {
    fn default() -> Self {
        Self { turnaround: 1, data_phase: false, idle_cycles: 0 }
    }
}

/// Outcome of one SWD transfer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Transfer {
    /// [`ack`] value; `ack::ERROR` for a parity error.
    pub ack: u8,
    /// Data read (valid when `ack == ack::OK` and the request was a read).
    pub data: u32,
}

/// Host status from `DAP_HostStatus`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostStatus {
    /// Debugger connected to the target (or not).
    Connected(bool),
    /// Target running (or not).
    Running(bool),
}

/// The probe hardware behind one DAP interface.
pub trait Driver {
    /// Set the SWCLK frequency. Returns false for an unsupported value.
    fn set_clock(&mut self, hz: u32) -> bool;

    /// `DAP_Connect`: take the SWD port (drive SWDIO).
    async fn connect(&mut self);

    /// `DAP_Disconnect` / interface down: release SWDIO and nRESET.
    async fn disconnect(&mut self);

    /// One SWD transfer: request packet, turnaround, ACK, and the data
    /// phase per the reference `SWD_Transfer`, using `params`. `request` is
    /// the low nibble (APnDP, RnW, A2, A3); `data` is the write value.
    async fn transfer(&mut self, request: u8, data: u32, params: &TransferParams) -> Transfer;

    /// Clock out `bits` bits from `data`, LSB first, driving SWDIO.
    async fn write_sequence(&mut self, bits: usize, data: &[u8]);

    /// Clock in `bits` bits into `data`, LSB first, SWDIO released.
    async fn read_sequence(&mut self, bits: usize, data: &mut [u8]);

    /// `DAP_SWJ_Pins`: drive the pins in `mask` to `output`, wait up to
    /// `wait_us` for them to settle, return the pin state.
    async fn swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins;

    /// One transfer with the `DAP_TransferConfigure` WAIT retry loop: up
    /// to `wait_retries` further attempts while the target answers WAIT.
    /// The default loops over [`Driver::transfer`]; a driver can fold the
    /// loop into its own future.
    async fn transfer_retry(&mut self, request: u8, data: u32, params: &TransferParams, wait_retries: u16) -> Transfer {
        let mut retry = wait_retries;
        loop {
            let t = self.transfer(request & 0x0F, data, params).await;
            if t.ack != ack::WAIT || retry == 0 {
                return t;
            }
            retry -= 1;
        }
    }

    /// `DAP_TransferBlock` read body: `out.len() / 4` transfers of
    /// `request` back to back, each with up to `wait_retries` WAIT retries,
    /// storing little-endian words into `out`. The first AP read is posted
    /// and the last is collected through `RDBUFF`, as in the reference
    /// `DAP_SWD_TransferBlock`. Returns the number of words stored and the
    /// status of the last transfer (0 for an empty block).
    ///
    /// The default runs [`Driver::transfer`] per word; a driver can run the
    /// whole block in its own hot loop.
    async fn read_block(&mut self, request: u8, out: &mut [u8], params: &TransferParams, wait_retries: u16) -> (u16, u8) {
        let count = (out.len() / 4) as u16;
        if count == 0 {
            return (0, 0);
        }
        let ap = request & req::APNDP != 0;
        if ap {
            let t = self.transfer_retry(request, 0, params, wait_retries).await;
            if t.ack != ack::OK {
                return (0, t.ack);
            }
        }
        for i in 0..count {
            let r = if ap && i + 1 == count { req::RDBUFF_READ } else { request };
            let t = self.transfer_retry(r, 0, params, wait_retries).await;
            if t.ack != ack::OK {
                return (i, t.ack);
            }
            let at = i as usize * 4;
            out[at..at + 4].copy_from_slice(&t.data.to_le_bytes());
        }
        (count, ack::OK)
    }

    /// `DAP_TransferBlock` write body: one transfer of `request` per
    /// little-endian word of `data`, then an `RDBUFF` read so the last write
    /// completes; its status is the block's status. Returns the number of
    /// words written and that status.
    async fn write_block(&mut self, request: u8, data: &[u8], params: &TransferParams, wait_retries: u16) -> (u16, u8) {
        let count = (data.len() / 4) as u16;
        if count == 0 {
            return (0, 0);
        }
        for i in 0..count {
            let at = i as usize * 4;
            let word = u32::from_le_bytes([data[at], data[at + 1], data[at + 2], data[at + 3]]);
            let t = self.transfer_retry(request, word, params, wait_retries).await;
            if t.ack != ack::OK {
                return (i, t.ack);
            }
        }
        let t = self.transfer_retry(req::RDBUFF_READ, 0, params, wait_retries).await;
        (count, t.ack)
    }

    /// Test-domain timer value (microseconds), if any.
    fn timestamp(&self) -> u32 {
        0
    }

    /// `DAP_HostStatus` notification (LEDs).
    fn host_status(&mut self, _status: HostStatus) {}
}

