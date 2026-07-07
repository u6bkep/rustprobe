//! `rustprobe` — host CLI for configuring rustprobe multi-SWD-probe firmware.
//!
//! Talks the config protocol (`probe_config::protocol`) over the probe's
//! vendor-bulk CMSIS-DAP interfaces. See `transport` for the USB layer and
//! `session` for the protocol framing.

mod session;
mod transport;

use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use probe_config::protocol::FirmwareInfo;
use probe_config::BoardProfile;

use probe_config_toml::board_toml::{self, TomlBoardProfile};
use probe_config_toml::topology_toml::TomlTopology;

use crate::session::Session;
use crate::transport::Probe;

#[derive(Parser)]
#[command(name = "rustprobe", version, about = "Configure rustprobe firmware over USB")]
struct Cli {
    /// Select a device by serial ("<16-hex-uid>:0") when several are attached.
    #[arg(long, global = true)]
    serial: Option<String>,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List attached rustprobe devices.
    List,
    /// Show firmware info (chip, versions, limits, active probes/uarts).
    Info,
    /// Read the active topology and print it as TOML.
    Get,
    /// Validate a TOML topology file and write it to the probe.
    Set {
        /// Path to a TOML topology file.
        file: PathBuf,
        /// Reboot the probe after committing so the new topology takes effect.
        #[arg(long)]
        reboot: bool,
    },
    /// Read the current board profile and print it as TOML.
    GetBoard,
    /// Validate a TOML board profile file and write it to the probe.
    SetBoard {
        /// Path to a TOML board profile file (see configs/boards/).
        file: PathBuf,
    },
    /// Reboot the probe.
    Reboot {
        /// Reboot into the BOOTSEL bootloader (for flashing with picotool).
        #[arg(long)]
        bootsel: bool,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let serial = cli.serial.as_deref();
    match cli.command {
        Command::List => cmd_list(),
        Command::Info => cmd_info(serial),
        Command::Get => cmd_get(serial),
        Command::Set { file, reboot } => cmd_set(serial, &file, reboot),
        Command::GetBoard => cmd_get_board(serial),
        Command::SetBoard { file } => cmd_set_board(serial, &file),
        Command::Reboot { bootsel } => cmd_reboot(serial, bootsel),
    }
}

/// Open a config session on the selected device.
fn open(serial: Option<&str>) -> Result<Session> {
    let info = transport::find_device(serial)?;
    Ok(Session::new(Probe::open(&info)?))
}

fn cmd_list() -> Result<()> {
    let devices = transport::list_devices()?;
    if devices.is_empty() {
        println!("No rustprobe devices found.");
        return Ok(());
    }
    for info in &devices {
        let s = transport::summarize(info);
        println!("{}", s.serial.as_deref().unwrap_or("<no serial>"));
        println!("  product:    {}", s.product.as_deref().unwrap_or("?"));
        println!("  interfaces: {} CMSIS-DAP", s.dap_interfaces.len());
    }
    Ok(())
}

fn cmd_info(serial: Option<&str>) -> Result<()> {
    let session = open(serial)?;
    let info = session.info()?;
    print_info(&info);
    if info.protocol_version >= 2 {
        let profile = session.get_profile()?;
        println!("board profile:");
        println!("  available:      {}", board_toml::format_pin_ranges(profile.available));
        println!("  reserved:       {}", board_toml::format_pin_ranges(profile.reserved));
    }
    Ok(())
}

fn cmd_get(serial: Option<&str>) -> Result<()> {
    let topo = open(serial)?.get_topology()?;
    let text = toml::to_string(&TomlTopology::from_topology(&topo))
        .context("serialize topology to TOML")?;
    if text.trim().is_empty() {
        println!("# empty topology (no probes or uarts)");
    } else {
        print!("{text}");
    }
    Ok(())
}

fn cmd_set(serial: Option<&str>, file: &PathBuf, reboot: bool) -> Result<()> {
    let text = std::fs::read_to_string(file)
        .with_context(|| format!("read {}", file.display()))?;
    let parsed: TomlTopology = toml::from_str(&text).context("parse TOML topology")?;
    let topo = parsed.into_topology()?;

    let session = open(serial)?;

    // Validate locally against the chip's reported limits and board profile
    // before staging.
    let info = session.info()?;
    let profile = device_profile(&session, &info)?;
    topo.validate(&info.limits, &profile)
        .map_err(|e| anyhow::anyhow!("topology rejected by local validation: {e:?}"))?;

    session.set_topology(&topo)?;
    println!(
        "topology written ({} probes, {} uarts)",
        topo.probes.len(),
        topo.uarts.len()
    );

    if reboot {
        session.reboot()?;
        println!("reboot requested; the probe will re-enumerate");
    } else {
        println!("run `rustprobe reboot` to apply (or re-run with --reboot)");
    }
    Ok(())
}

/// The board profile the device validates against: fetched over the protocol,
/// or the Pico default (with a warning) for pre-profile firmware.
fn device_profile(session: &Session, info: &FirmwareInfo) -> Result<BoardProfile> {
    if info.protocol_version >= 2 {
        session.get_profile()
    } else {
        eprintln!(
            "warning: firmware protocol version {} predates board profiles; assuming a bare Pico",
            info.protocol_version
        );
        Ok(BoardProfile::PICO)
    }
}

fn cmd_get_board(serial: Option<&str>) -> Result<()> {
    let session = open(serial)?;
    let info = session.info()?;
    let profile = device_profile(&session, &info)?;
    print!(
        "{}",
        toml::to_string(&TomlBoardProfile::from_profile(&profile))
            .context("serialize board profile to TOML")?
    );
    Ok(())
}

fn cmd_set_board(serial: Option<&str>, file: &PathBuf) -> Result<()> {
    let text = std::fs::read_to_string(file)
        .with_context(|| format!("read {}", file.display()))?;
    let parsed: TomlBoardProfile = toml::from_str(&text).context("parse TOML board profile")?;
    let profile = parsed.into_profile()?;

    let session = open(serial)?;
    let info = session.info()?;
    if info.protocol_version < 2 {
        anyhow::bail!(
            "firmware protocol version {} predates board profiles; reflash the firmware first",
            info.protocol_version
        );
    }
    profile
        .validate(&info.limits)
        .map_err(|e| anyhow::anyhow!("board profile rejected by local validation: {e:?}"))?;

    session.set_profile(&profile)?;
    println!(
        "board profile written (available {}, reserved {})",
        board_toml::format_pin_ranges(profile.available),
        board_toml::format_pin_ranges(profile.reserved),
    );

    // The firmware re-checks the stored topology against the profile at boot;
    // warn now if the currently-active topology would fail that check.
    let topo = session.get_topology()?;
    if let Err(e) = topo.validate(&info.limits, &profile) {
        eprintln!(
            "warning: the active topology violates the new profile ({e:?}); \
             the probe will fall back to its default topology on next boot \
             unless you `set` a compatible one"
        );
    }
    Ok(())
}

fn cmd_reboot(serial: Option<&str>, bootsel: bool) -> Result<()> {
    if bootsel {
        open(serial)?.reboot_bootsel()?;
        println!("BOOTSEL reboot requested; flash with picotool");
    } else {
        open(serial)?.reboot()?;
        println!("reboot requested; the probe will re-enumerate");
    }
    Ok(())
}

/// Pretty-print a [`FirmwareInfo`] response.
fn print_info(info: &FirmwareInfo) {
    let (major, minor, patch) = info.firmware_version;
    println!("chip:             {:?}", info.chip);
    println!("firmware version: {major}.{minor}.{patch}");
    println!("protocol version: {}", info.protocol_version);
    println!("active probes:    {}", info.active_probes);
    println!("active uarts:     {}", info.active_uarts);
    println!("config fault:     {}", info.config_fault);
    let l = &info.limits;
    println!("limits:");
    println!("  gpio count:     {}", l.gpio_count);
    println!("  pio blocks:     {}", l.pio_blocks);
    println!("  sms per block:  {}", l.sms_per_block);
    println!("  ep numbers/dir: {}", l.ep_numbers_per_dir);
    println!("  pio pin window: {}", l.pio_pin_window);
}
