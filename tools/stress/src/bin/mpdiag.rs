//! `mpdiag` — read the per-lane reply-stall counters from a rustprobe running
//! the `stall-diag` firmware (DAP_Info id 0xA0, answered on core 0).
//!
//! Stock firmware answers id 0xA0 with an empty value; that is reported, not
//! treated as an error. Lanes held open by another process are skipped.

use std::time::Duration;

use anyhow::{anyhow, bail, Context, Result};
use clap::Parser;
use nusb::descriptors::TransferType;
use nusb::transfer::{Buffer, Bulk, Direction, In, Out};
use nusb::MaybeFuture;

const DIAG_INFO_ID: u8 = 0xA0;
/// Zeroes the lane's counters, then returns them.
const DIAG_RESET_ID: u8 = 0xA1;
/// Field order of the 0xA0 reply (u32 LE each), as defined by the firmware.
const FIELDS: [&str; 13] = [
    "cmds", "slow_worker", "max_worker_us", "unread", "max_unread_ms", "slow_arm", "max_arm_us", "busy_us", "max_busy_us",
    "fifo_yields", "worker_us", "inbound_us", "outbound_us",
];
const TIMEOUT: Duration = Duration::from_secs(1);

#[derive(Parser)]
#[command(about = "Read rustprobe stall-diag counters from every lane")]
struct Args {
    /// USB serial of the probe; needed only when several are attached.
    #[arg(long)]
    serial: Option<String>,
    /// Zero every lane's counters (and print the zeroed values).
    #[arg(long)]
    reset: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut probes: Vec<_> = nusb::list_devices()
        .wait()?
        .filter(|d| d.vendor_id() == 0x2e8a && d.product_id() == 0x000c)
        .filter(|d| args.serial.as_deref().is_none_or(|s| d.serial_number() == Some(s)))
        .collect();
    let info = match probes.len() {
        1 => probes.remove(0),
        0 => bail!("no matching rustprobe attached"),
        _ => bail!("several rustprobes attached, pick one with --serial"),
    };
    let dev = info.open().wait().context("open device")?;
    let lanes: Vec<u8> = info
        .interfaces()
        .filter(|i| i.class() == 0xff && i.interface_string().is_some_and(|s| s.contains("CMSIS-DAP")))
        .map(|i| i.interface_number())
        .collect();
    for lane in lanes {
        match read_lane(&dev, lane, args.reset) {
            Ok(line) => println!("lane {lane}: {line}"),
            Err(e) => println!("lane {lane}: {e:#}"),
        }
    }
    Ok(())
}

fn read_lane(dev: &nusb::Device, lane: u8, reset: bool) -> Result<String> {
    let iface = dev.claim_interface(lane).wait().context("claim (lane in use?)")?;
    let desc = iface.descriptor().ok_or_else(|| anyhow!("no interface descriptor"))?;
    let bulk_ep = |dir: Direction| {
        desc.endpoints()
            .find(|e| e.transfer_type() == TransferType::Bulk && e.direction() == dir)
            .map(|e| e.address())
            .ok_or_else(|| anyhow!("no bulk {dir:?} endpoint"))
    };
    let (out_addr, in_addr) = (bulk_ep(Direction::Out)?, bulk_ep(Direction::In)?);
    let mut out = iface.endpoint::<Bulk, Out>(out_addr)?;
    let mut inp = iface.endpoint::<Bulk, In>(in_addr)?;

    let mut req = Buffer::new(2);
    let id = if reset { DIAG_RESET_ID } else { DIAG_INFO_ID };
    req.extend_from_slice(&[0x00, id]); // DAP_Info, id
    out.submit(req);
    out.wait_next_complete(TIMEOUT).ok_or_else(|| anyhow!("write timeout"))?.status?;
    inp.submit(Buffer::new(64));
    let c = inp.wait_next_complete(TIMEOUT).ok_or_else(|| anyhow!("read timeout"))?;
    c.status?;
    let reply = &c.buffer[..c.actual_len];

    if reply.get(1) == Some(&0) {
        return Ok("no counters (firmware without stall-diag)".into());
    }
    // Older firmware reports a prefix of the field list.
    let n = reply[1] as usize / 4;
    if reply[1] % 4 != 0 || n > FIELDS.len() || reply.len() < 2 + 4 * n {
        bail!("unexpected reply {reply:02x?}");
    }
    let values: Vec<u32> = (0..n)
        .map(|i| u32::from_le_bytes(reply[2 + 4 * i..6 + 4 * i].try_into().unwrap()))
        .collect();
    let mut fields: Vec<String> = FIELDS.iter().zip(&values).map(|(name, v)| format!("{name}={v}")).collect();
    let cmds = values[0];
    if cmds > 0 {
        if let Some(busy_us) = values.get(7) {
            fields.push(format!("mean_busy_us={:.1}", *busy_us as f64 / cmds as f64));
        }
        if let Some(worker_us) = values.get(10) {
            fields.push(format!("mean_worker_us={:.1}", *worker_us as f64 / cmds as f64));
        }
        if let (Some(i), Some(o)) = (values.get(11), values.get(12)) {
            fields.push(format!("in/cmd={:.1} out/cmd={:.1}", *i as f64 / cmds as f64, *o as f64 / cmds as f64));
        }
    }
    Ok(fields.join(" "))
}
