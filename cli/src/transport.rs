//! Thin USB transport for the rustprobe config protocol.
//!
//! Config commands ride on the vendor-bulk CMSIS-DAP v2 interfaces: one
//! request packet OUT (≤64 bytes), one response packet IN (≤64 bytes). This
//! module handles discovery and the single-packet transceive; the protocol
//! framing lives in [`crate::session`].

use anyhow::{bail, Context, Result};
use futures_lite::future::block_on;
use nusb::transfer::{Direction, EndpointType, RequestBuffer};
use nusb::{DeviceInfo, Interface, InterfaceInfo};

/// Rustprobe USB vendor id (shared with the Raspberry Pi debugprobe).
pub const VID: u16 = 0x2E8A;
/// Rustprobe USB product id.
pub const PID: u16 = 0x000C;

/// Bulk packet size (full-speed vendor bulk MPS).
const PACKET_SIZE: usize = 64;

/// Substring identifying a CMSIS-DAP interface in its interface string.
const DAP_MARKER: &str = "CMSIS-DAP";

/// True if `iface` is one of the probe's CMSIS-DAP interfaces.
pub fn is_dap_interface(iface: &InterfaceInfo) -> bool {
    iface
        .interface_string()
        .is_some_and(|s| s.contains(DAP_MARKER))
}

/// Enumerate attached rustprobe devices (matched by VID:PID).
pub fn list_devices() -> Result<Vec<DeviceInfo>> {
    Ok(nusb::list_devices()
        .context("enumerate USB devices")?
        .filter(|d| d.vendor_id() == VID && d.product_id() == PID)
        .collect())
}

/// Select a single device, optionally filtered by serial number.
pub fn find_device(serial: Option<&str>) -> Result<DeviceInfo> {
    let mut devices = list_devices()?;
    match serial {
        Some(s) => devices
            .into_iter()
            .find(|d| d.serial_number() == Some(s))
            .with_context(|| format!("no rustprobe with serial {s:?}")),
        None => match devices.len() {
            0 => bail!("no rustprobe devices found (VID:PID {VID:04X}:{PID:04X})"),
            1 => Ok(devices.remove(0)),
            n => bail!("{n} rustprobe devices attached; select one with --serial"),
        },
    }
}

/// A summary of a discovered device for `list`.
pub struct DeviceSummary {
    /// Device serial number ("<uid>:0"), if reported.
    pub serial: Option<String>,
    /// USB product string.
    pub product: Option<String>,
    /// Interface numbers whose string marks them as CMSIS-DAP.
    pub dap_interfaces: Vec<u8>,
}

/// Summarize a device without opening it (Linux exposes interface strings
/// through sysfs, so no control transfer is needed).
pub fn summarize(info: &DeviceInfo) -> DeviceSummary {
    DeviceSummary {
        serial: info.serial_number().map(str::to_owned),
        product: info.product_string().map(str::to_owned),
        dap_interfaces: info
            .interfaces()
            .filter(|i| is_dap_interface(i))
            .map(|i| i.interface_number())
            .collect(),
    }
}

/// An opened probe interface, ready to exchange config packets.
pub struct Probe {
    interface: Interface,
    ep_out: u8,
    ep_in: u8,
}

impl Probe {
    /// Open `info` and claim a DAP interface for config commands.
    ///
    /// Config commands are served on any probe interface, so the
    /// lowest-numbered CMSIS-DAP interface is used.
    pub fn open(info: &DeviceInfo) -> Result<Probe> {
        let iface_num = info
            .interfaces()
            .filter(|i| is_dap_interface(i))
            .map(|i| i.interface_number())
            .min()
            .context("device exposes no CMSIS-DAP interface")?;
        let device = info.open().context("open USB device")?;
        let interface = device
            .claim_interface(iface_num)
            .with_context(|| format!("claim interface {iface_num}"))?;
        let (ep_out, ep_in) =
            bulk_endpoints(&interface).context("locate bulk endpoints on DAP interface")?;
        Ok(Probe { interface, ep_out, ep_in })
    }

    /// Send one request packet and return the single response packet.
    pub fn transceive(&self, request: &[u8]) -> Result<Vec<u8>> {
        block_on(self.interface.bulk_out(self.ep_out, request.to_vec()))
            .into_result()
            .context("bulk OUT")?;
        let response = block_on(
            self.interface
                .bulk_in(self.ep_in, RequestBuffer::new(PACKET_SIZE)),
        )
        .into_result()
        .context("bulk IN")?;
        Ok(response)
    }
}

/// Find the (bulk OUT, bulk IN) endpoint addresses on the interface's first
/// alternate setting.
fn bulk_endpoints(interface: &Interface) -> Result<(u8, u8)> {
    let alt = interface
        .descriptors()
        .next()
        .context("interface has no alternate setting")?;
    let mut ep_out = None;
    let mut ep_in = None;
    for ep in alt.endpoints() {
        if ep.transfer_type() != EndpointType::Bulk {
            continue;
        }
        match ep.direction() {
            Direction::Out => ep_out.get_or_insert(ep.address()),
            Direction::In => ep_in.get_or_insert(ep.address()),
        };
    }
    match (ep_out, ep_in) {
        (Some(out), Some(inn)) => Ok((out, inn)),
        _ => bail!("interface lacks a bulk IN/OUT endpoint pair"),
    }
}
