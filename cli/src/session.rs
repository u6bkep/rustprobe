//! Config-protocol framing over a [`Probe`] transport.
//!
//! Each method issues one or more request/response packets, echoing the
//! command byte and checking the status byte (see `probe_config::protocol`).

use anyhow::{bail, Context, Result};
use probe_config::protocol::*;
use probe_config::Topology;

use crate::transport::Probe;

/// Max topology bytes per SET chunk (64-byte packet minus cmd + offset).
/// Mirrors the firmware's GET chunk size so a full topology fits either way.
const SET_CHUNK: usize = 61;

/// A config session bound to one opened probe interface.
pub struct Session {
    probe: Probe,
}

impl Session {
    /// Wrap an opened probe.
    pub fn new(probe: Probe) -> Self {
        Self { probe }
    }

    /// Read [`FirmwareInfo`] (`CMD_INFO`).
    pub fn info(&self) -> Result<FirmwareInfo> {
        let resp = self.probe.transceive(&[CMD_INFO])?;
        check(&resp, CMD_INFO)?;
        postcard::from_bytes(&resp[2..]).context("decode FirmwareInfo")
    }

    /// Read the active topology (chunked `CMD_GET_TOPOLOGY`).
    pub fn get_topology(&self) -> Result<Topology> {
        let mut buf: Vec<u8> = Vec::new();
        loop {
            let resp = self
                .probe
                .transceive(&[CMD_GET_TOPOLOGY, buf.len() as u8])?;
            check(&resp, CMD_GET_TOPOLOGY)?;
            if resp.len() < 3 {
                bail!("short GET_TOPOLOGY response ({} bytes)", resp.len());
            }
            let total_len = resp[2] as usize;
            let chunk = &resp[3..];
            if buf.len() >= total_len {
                break;
            }
            if chunk.is_empty() {
                bail!("probe returned an empty chunk before the topology was complete");
            }
            buf.extend_from_slice(chunk);
            if buf.len() >= total_len {
                buf.truncate(total_len);
                break;
            }
        }
        postcard::from_bytes(&buf).context("decode Topology")
    }

    /// Stage a topology (chunked `CMD_SET_TOPOLOGY`) and `CMD_COMMIT` it.
    /// The firmware re-validates before writing flash.
    pub fn set_topology(&self, topo: &Topology) -> Result<()> {
        let encoded = postcard::to_stdvec(topo).context("encode Topology")?;
        if encoded.len() > TOPOLOGY_BUF_LEN {
            bail!(
                "encoded topology is {} bytes, firmware staging buffer holds {}",
                encoded.len(),
                TOPOLOGY_BUF_LEN
            );
        }
        // Offset 0 resets the staging buffer, so send it first even when empty.
        let mut offset = 0;
        loop {
            let end = (offset + SET_CHUNK).min(encoded.len());
            let mut req = Vec::with_capacity(2 + (end - offset));
            req.push(CMD_SET_TOPOLOGY);
            req.push(offset as u8);
            req.extend_from_slice(&encoded[offset..end]);
            let resp = self.probe.transceive(&req)?;
            check(&resp, CMD_SET_TOPOLOGY)?;
            offset = end;
            if offset >= encoded.len() {
                break;
            }
        }
        let resp = self.probe.transceive(&[CMD_COMMIT, encoded.len() as u8])?;
        check(&resp, CMD_COMMIT)?;
        Ok(())
    }

    /// Reboot the probe (`CMD_REBOOT`); it re-enumerates afterward.
    pub fn reboot(&self) -> Result<()> {
        let resp = self.probe.transceive(&[CMD_REBOOT])?;
        check(&resp, CMD_REBOOT)?;
        Ok(())
    }
}

/// Human-readable meaning of a protocol status byte.
fn status_str(status: u8) -> &'static str {
    match status {
        STATUS_OK => "ok",
        STATUS_ERR_DECODE => "payload failed to decode",
        STATUS_ERR_INVALID => "topology failed validation",
        STATUS_ERR_FLASH => "flash write failed",
        STATUS_ERR_BAD_REQUEST => "unknown command or malformed arguments",
        _ => "unknown status",
    }
}

/// Verify a response echoes `cmd` and carries `STATUS_OK`.
fn check(resp: &[u8], cmd: u8) -> Result<()> {
    if resp.len() < 2 {
        bail!("short response ({} bytes)", resp.len());
    }
    if resp[0] != cmd {
        bail!(
            "response command 0x{:02X} does not echo request 0x{:02X}",
            resp[0],
            cmd
        );
    }
    if resp[1] != STATUS_OK {
        bail!(
            "probe reported error: {} (status 0x{:02X})",
            status_str(resp[1]),
            resp[1]
        );
    }
    Ok(())
}
