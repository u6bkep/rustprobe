//! Rustprobe configuration protocol, carried as CMSIS-DAP vendor commands
//! (command bytes 0x80..=0x9F) over any of the probe's DAP interfaces.
//!
//! Payloads are postcard-encoded types from this crate, so firmware and host
//! tools share one wire format. `Topology` blobs can exceed a single 64-byte
//! DAP packet, so GET/SET carry `(offset, chunk)` pairs.
//!
//! Frame formats:
//!
//! * Request:  `[cmd, args...]`
//! * Response: `[cmd, status, payload...]`

/// First vendor command byte used by rustprobe.
pub const VENDOR_BASE: u8 = 0x80;
/// Last vendor command byte reserved for rustprobe.
pub const VENDOR_END: u8 = 0x9F;

/// `[CMD_INFO]` → `[cmd, status, postcard(FirmwareInfo)]`
pub const CMD_INFO: u8 = 0x80;
/// `[CMD_GET_TOPOLOGY, offset]` → `[cmd, status, total_len, chunk...]`
/// Returns the *active* (booted) topology, postcard-encoded.
pub const CMD_GET_TOPOLOGY: u8 = 0x81;
/// `[CMD_SET_TOPOLOGY, offset, chunk...]` → `[cmd, status]`
/// Stages topology bytes in RAM; offset 0 resets the staging buffer.
pub const CMD_SET_TOPOLOGY: u8 = 0x82;
/// `[CMD_COMMIT, total_len]` → `[cmd, status]`
/// Decodes + validates the staged bytes and writes them to flash.
pub const CMD_COMMIT: u8 = 0x83;
/// `[CMD_REBOOT]` → `[cmd, status]`, then the probe resets and re-enumerates.
pub const CMD_REBOOT: u8 = 0x84;
/// `[CMD_BOOTSEL]` → `[cmd, status]`, then the probe reboots into the
/// BOOTSEL bootloader (for picotool flashing without the button).
pub const CMD_BOOTSEL: u8 = 0x85;

/// Command handled successfully.
pub const STATUS_OK: u8 = 0x00;
/// Payload failed to decode.
pub const STATUS_ERR_DECODE: u8 = 0x01;
/// Topology failed validation (see `FirmwareInfo` for limits).
pub const STATUS_ERR_INVALID: u8 = 0x02;
/// Flash write failed.
pub const STATUS_ERR_FLASH: u8 = 0x03;
/// Unknown command or malformed arguments.
pub const STATUS_ERR_BAD_REQUEST: u8 = 0x04;

/// Maximum size of a postcard-encoded `Topology` the protocol supports.
pub const TOPOLOGY_BUF_LEN: usize = 128;

use serde::{Deserialize, Serialize};

use crate::ChipLimits;

/// Which chip the firmware was built for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Chip {
    /// RP2040
    Rp2040,
    /// RP2350 (A or B package)
    Rp2350,
}

/// Response payload of [`CMD_INFO`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct FirmwareInfo {
    /// Protocol version, bumped on breaking changes to this module.
    pub protocol_version: u8,
    /// Firmware crate version (major, minor, patch).
    pub firmware_version: (u8, u8, u8),
    /// Chip the firmware runs on.
    pub chip: Chip,
    /// The chip's resource limits, for host-side validation.
    pub limits: ChipLimits,
    /// Number of active SWD probes in the booted topology.
    pub active_probes: u8,
    /// Number of active UART bridges in the booted topology.
    pub active_uarts: u8,
    /// True if the stored config was missing/invalid and the firmware fell
    /// back to the default topology.
    pub config_fault: bool,
}

/// Current protocol version.
pub const PROTOCOL_VERSION: u8 = 1;
