//! The command processor.

use embedded_hal_async::delay::DelayNs;

use crate::command::{cmd, info, CAPABILITIES, DAP_ERROR, DAP_OK};
use crate::io::{Request, Response};
use crate::swd::{ack, req, Driver, HostStatus, Pins, Transfer, TransferParams};

/// One DAP interface: the command processor plus its hardware driver.
pub struct Dap<D, T> {
    driver: D,
    delay: T,
    version: &'static str,
    packet_size: u16,
    params: TransferParams,
    /// `DAP_TransferConfigure` WAIT retries per transfer.
    wait_retries: u16,
    /// `DAP_TransferConfigure` value-match retries.
    match_retries: u16,
    /// Mask set by a `DAP_Transfer` match-mask write.
    match_mask: u32,
}

impl<D: Driver, T: DelayNs> Dap<D, T> {
    /// `version` is the `DAP_Info` firmware version string; `packet_size`
    /// the maximum request/response size the transport carries.
    pub fn new(driver: D, delay: T, version: &'static str, packet_size: u16) -> Self {
        Self {
            driver,
            delay,
            version,
            packet_size,
            params: TransferParams::default(),
            // Reference `DAP_Data` defaults.
            wait_retries: 100,
            match_retries: 0,
            match_mask: 0,
        }
    }

    /// The hardware driver.
    pub fn driver(&mut self) -> &mut D {
        &mut self.driver
    }

    /// The interface went down: release the target.
    pub async fn suspend(&mut self) {
        self.driver.disconnect().await;
    }

    /// Process one request packet, writing the reply into `response`.
    /// Returns the reply length; 0 means no reply (`DAP_TransferAbort`).
    pub async fn process_command(&mut self, request: &[u8], response: &mut [u8]) -> usize {
        let Some((&command, payload)) = request.split_first() else {
            return 0;
        };
        if response.len() < 2 {
            return 0;
        }
        let mut req = Request::new(payload);
        let mut resp = Response::new(response, command);
        let ok = match command {
            cmd::INFO => self.info(&mut req, &mut resp),
            cmd::HOST_STATUS => self.host_status(&mut req, &mut resp),
            cmd::CONNECT => self.connect(&mut req, &mut resp).await,
            cmd::DISCONNECT => self.disconnect(&mut resp).await,
            cmd::TRANSFER_CONFIGURE => self.transfer_configure(&mut req, &mut resp),
            cmd::TRANSFER => self.transfer(&mut req, &mut resp).await,
            cmd::TRANSFER_BLOCK => self.transfer_block(&mut req, &mut resp).await,
            cmd::TRANSFER_ABORT => return 0,
            cmd::WRITE_ABORT => self.write_abort(&mut req, &mut resp).await,
            cmd::DELAY => self.delay(&mut req, &mut resp).await,
            cmd::RESET_TARGET => resp.u8(DAP_OK).and_then(|_| resp.u8(0)),
            cmd::SWJ_PINS => self.swj_pins(&mut req, &mut resp).await,
            cmd::SWJ_CLOCK => self.swj_clock(&mut req, &mut resp),
            cmd::SWJ_SEQUENCE => self.swj_sequence(&mut req, &mut resp).await,
            cmd::SWD_CONFIGURE => self.swd_configure(&mut req, &mut resp),
            cmd::SWD_SEQUENCE => self.swd_sequence(&mut req, &mut resp).await,
            // JTAG is not available: DAP_ERROR, so hosts skip JTAG probing.
            cmd::JTAG_SEQUENCE | cmd::JTAG_CONFIGURE | cmd::JTAG_IDCODE => None,
            // No SWO. Mirrors what dap-rs's `NoSwo` answered.
            cmd::SWO_TRANSPORT => self.swo_transport(&mut req, &mut resp),
            cmd::SWO_MODE | cmd::SWO_CONTROL => None,
            cmd::SWO_BAUDRATE => resp.u32(0),
            cmd::SWO_STATUS => resp.u8(0).and_then(|_| resp.u32(0)),
            cmd::SWO_EXTENDED_STATUS => resp.slice(&[0; 13]),
            cmd::SWO_DATA => resp.u8(0).and_then(|_| resp.u16(0)),
            _ => {
                resp.set_u8(0, cmd::INVALID);
                return 1;
            }
        };
        match ok {
            Some(()) => resp.len(),
            None => {
                resp.set_u8(1, DAP_ERROR);
                2
            }
        }
    }

    fn info(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        match req.u8()? {
            info::FW_VERSION => {
                resp.u8(self.version.len() as u8)?;
                resp.slice(self.version.as_bytes())
            }
            info::CAPABILITIES => {
                resp.u8(1)?;
                resp.u8(CAPABILITIES)
            }
            info::SWO_BUFFER_SIZE => {
                resp.u8(4)?;
                resp.u32(0)
            }
            info::MAX_PACKET_COUNT => {
                resp.u8(1)?;
                resp.u8(1)
            }
            info::MAX_PACKET_SIZE => {
                resp.u8(2)?;
                resp.u16(self.packet_size)
            }
            // Vendor/product/serial/target strings come from the USB
            // descriptors (zero-length answer); the test-domain timer and
            // unknown ids likewise answer with no value.
            _ => resp.u8(0),
        }
    }

    fn host_status(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        let kind = req.u8()?;
        let value = req.u8()? != 0;
        match kind {
            0 => self.driver.host_status(HostStatus::Connected(value)),
            1 => self.driver.host_status(HostStatus::Running(value)),
            _ => return None,
        }
        resp.u8(DAP_OK)
    }

    async fn connect(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        match req.u8()? {
            // Default or SWD.
            0 | 1 => {
                self.driver.connect().await;
                resp.u8(1)
            }
            // JTAG or unknown: failed.
            _ => resp.u8(0),
        }
    }

    async fn disconnect(&mut self, resp: &mut Response<'_>) -> Option<()> {
        self.driver.disconnect().await;
        resp.u8(DAP_OK)
    }

    fn transfer_configure(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        self.params.idle_cycles = req.u8()?;
        self.wait_retries = req.u16()?;
        self.match_retries = req.u16()?;
        resp.u8(DAP_OK)
    }

    async fn write_abort(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        let _index = req.u8()?;
        let word = req.u32()?;
        // The reference ignores the ACK: the host uses ABORT to recover from
        // sticky errors, when the DP may well not answer OK.
        self.driver.transfer(req::ABORT_WRITE, word, &self.params).await;
        resp.u8(DAP_OK)
    }

    async fn delay(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        let us = req.u16()?;
        self.delay.delay_us(us as u32).await;
        resp.u8(DAP_OK)
    }

    async fn swj_pins(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        let output = Pins(req.u8()?);
        let mask = Pins(req.u8()?);
        // The spec caps the wait at 3 s.
        let wait_us = req.u32()?.min(3_000_000);
        let pins = self.driver.swj_pins(output, mask, wait_us).await;
        resp.u8(pins.0)
    }

    fn swj_clock(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        let hz = req.u32()?;
        resp.u8(if self.driver.set_clock(hz) { DAP_OK } else { DAP_ERROR })
    }

    async fn swj_sequence(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        let bits = match req.u8()? {
            0 => 256,
            n => n as usize,
        };
        let data = req.bytes(bits.div_ceil(8))?;
        self.driver.write_sequence(bits, data).await;
        resp.u8(DAP_OK)
    }

    fn swd_configure(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        let config = req.u8()?;
        self.params.turnaround = (config & 0b11) + 1;
        self.params.data_phase = config & 0b100 != 0;
        resp.u8(DAP_OK)
    }

    async fn swd_sequence(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        let count = req.u8()?;
        resp.u8(DAP_OK)?;
        for _ in 0..count {
            let info = req.u8()?;
            let bits = match info & 0x3F {
                0 => 64,
                n => n as usize,
            };
            let nbytes = bits.div_ceil(8);
            if info & 0x80 == 0 {
                let data = req.bytes(nbytes)?;
                self.driver.write_sequence(bits, data).await;
            } else {
                let buf = resp.reserve_mut(nbytes)?;
                self.driver.read_sequence(bits, buf).await;
            }
        }
        Some(())
    }

    fn swo_transport(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        match req.u8()? {
            0..=2 => resp.u8(DAP_OK),
            _ => None,
        }
    }

    /// One transfer with the configured WAIT retries.
    #[inline(always)]
    async fn xfer(&mut self, request: u8, data: u32) -> Transfer {
        self.driver.transfer_retry(request, data, &self.params, self.wait_retries).await
    }

    /// `DAP_Transfer`, after the reference `DAP_SWD_Transfer`: AP reads are
    /// posted (each AP read returns the previous one's data; `RDBUFF`
    /// collects the last), a trailing write is followed by an `RDBUFF`
    /// read so it completes, and the transfer count excludes the transfer
    /// that failed.
    async fn transfer(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        let _index = req.u8()?;
        let mut remaining = req.u8()?;
        let head = resp.reserve(2)?;

        let mut count: u8 = 0;
        let mut status: u8 = 0;
        let mut post_read = false;
        let mut check_write = false;

        while remaining > 0 {
            remaining -= 1;
            let rv = req.u8()?;
            if rv & req::RNW != 0 {
                if post_read {
                    // Collect the previously posted AP read: a plain AP
                    // read posts the next one at the same time, anything
                    // else goes through RDBUFF.
                    let t = if rv & (req::APNDP | req::MATCH_VALUE) == req::APNDP {
                        self.xfer(rv, 0).await
                    } else {
                        post_read = false;
                        self.xfer(req::RDBUFF_READ, 0).await
                    };
                    status = t.ack;
                    if status != ack::OK {
                        break;
                    }
                    resp.u32(t.data)?;
                    if post_read && rv & req::TIMESTAMP != 0 {
                        resp.u32(self.driver.timestamp())?;
                    }
                }
                if rv & req::MATCH_VALUE != 0 {
                    let target = req.u32()?;
                    let mut match_retry = self.match_retries;
                    if rv & req::APNDP != 0 {
                        // Post the AP read.
                        status = self.xfer(rv, 0).await.ack;
                        if status != ack::OK {
                            break;
                        }
                    }
                    let mut t;
                    loop {
                        t = self.xfer(rv, 0).await;
                        if t.ack != ack::OK || (t.data & self.match_mask) == target || match_retry == 0 {
                            break;
                        }
                        match_retry -= 1;
                    }
                    status = t.ack;
                    if status == ack::OK && (t.data & self.match_mask) != target {
                        status |= ack::MISMATCH;
                    }
                    if status != ack::OK {
                        break;
                    }
                } else if rv & req::APNDP != 0 {
                    if !post_read {
                        // Post the AP read; its data comes with the next one.
                        status = self.xfer(rv, 0).await.ack;
                        if status != ack::OK {
                            break;
                        }
                        if rv & req::TIMESTAMP != 0 {
                            resp.u32(self.driver.timestamp())?;
                        }
                        post_read = true;
                    }
                } else {
                    let t = self.xfer(rv, 0).await;
                    status = t.ack;
                    if status != ack::OK {
                        break;
                    }
                    if rv & req::TIMESTAMP != 0 {
                        resp.u32(self.driver.timestamp())?;
                    }
                    resp.u32(t.data)?;
                }
                check_write = false;
            } else {
                if post_read {
                    let t = self.xfer(req::RDBUFF_READ, 0).await;
                    status = t.ack;
                    if status != ack::OK {
                        break;
                    }
                    resp.u32(t.data)?;
                    post_read = false;
                }
                let data = req.u32()?;
                if rv & req::MATCH_MASK != 0 {
                    self.match_mask = data;
                    status = ack::OK;
                } else {
                    status = self.xfer(rv, data).await.ack;
                    if status != ack::OK {
                        break;
                    }
                    if rv & req::TIMESTAMP != 0 {
                        resp.u32(self.driver.timestamp())?;
                    }
                    check_write = true;
                }
            }
            count += 1;
        }

        if status == ack::OK {
            if post_read {
                let t = self.xfer(req::RDBUFF_READ, 0).await;
                status = t.ack;
                if status == ack::OK {
                    resp.u32(t.data)?;
                }
            } else if check_write {
                status = self.xfer(req::RDBUFF_READ, 0).await.ack;
            }
        }

        resp.set_u8(head, count);
        resp.set_u8(head + 1, status);
        Some(())
    }

    /// `DAP_TransferBlock`, after the reference `DAP_SWD_TransferBlock`.
    /// The count is the number of transfers that succeeded, which is also
    /// the number of data words in a read reply. The transfers themselves
    /// run in [`Driver::read_block`] / [`Driver::write_block`].
    async fn transfer_block(&mut self, req: &mut Request<'_>, resp: &mut Response<'_>) -> Option<()> {
        let _index = req.u8()?;
        let total = req.u16()?;
        let head = resp.reserve(3)?;

        let (count, status) = if total > 0 {
            let rv = req.u8()?;
            let nbytes = total as usize * 4;
            if rv & req::RNW != 0 {
                let at = resp.len();
                let out = resp.reserve_mut(nbytes)?;
                let (count, status) = self.driver.read_block(rv, out, &self.params, self.wait_retries).await;
                resp.truncate(at + count as usize * 4);
                (count, status)
            } else {
                let data = req.bytes(nbytes)?;
                self.driver.write_block(rv, data, &self.params, self.wait_retries).await
            }
        } else {
            (0, 0)
        };

        resp.set_u16(head, count);
        resp.set_u8(head + 2, status);
        Some(())
    }
}
