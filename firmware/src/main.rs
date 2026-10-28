//! Rustprobe firmware: N SWD probes over CMSIS-DAP v2, topology read from
//! flash at boot.
//!
//! Core 0 runs USB — the device task, all DAP endpoint I/O, and the UART
//! bridges. Core 1 runs only the SWD execution workers, one per probe,
//! fed over per-probe request/response channels. Two invariants hang off
//! this split (see issue #3, the concurrent-open wedge):
//!
//! * Core 1 runs one async executor for every probe's worker. A worker
//!   yields when a PIO FIFO wait would be long (`dap.rs`), so lanes
//!   interleave at SWD-transfer granularity and a slow lane does not hold
//!   the others (issue #5); still, nothing latency-sensitive lives there.
//!   Commands that touch no probe hardware — the `DAP_Info` family and the
//!   rustprobe config vendor commands — are answered directly on core 0,
//!   so interface opens (probe-rs sends `DAP_Info` with a 50 ms timeout)
//!   never wait behind other probes' wire traffic.
//! * Every USB endpoint register is touched from core 0 only. The
//!   embassy-rp USB driver's buffer-control write protocol assumes all
//!   writers share one execution context; endpoint futures on core 1
//!   would race the control-path (SET_CONFIGURATION/SET_INTERFACE) writes.
//!
//! Config changes arrive as vendor commands (staged → commit → reboot);
//! see `probe_config::protocol`.

#![no_std]
#![no_main]

mod autobaud;
mod dap;
mod flash_config;
mod instances;
mod reset_iface;
mod stall_diag;
mod swd;
mod uart_bridge;
mod vendor;

use core::cell::UnsafeCell;

use defmt::{info, warn};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::{Executor, Spawner};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::USB;
use embassy_rp::uart;
use embassy_rp::usb::{Driver, Endpoint, In, InterruptHandler, Out};
use embassy_rp::watchdog::Watchdog;
use embassy_rp::{bind_interrupts, pio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb::class::cdc_acm;
use embassy_usb::driver::{Endpoint as _, EndpointIn, EndpointOut};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::types::StringIndex;
use embassy_usb::{Builder, Config, Handler, UsbDevice};
use heapless::Vec;
use portable_atomic::Ordering;
use probe_config::protocol::{Chip, FirmwareInfo, PROTOCOL_VERSION, VENDOR_BASE, VENDOR_END};
use probe_config::{BoardProfile, ChipLimits, ProbeConfig, Topology, MAX_PROBES, MAX_UARTS};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use crate::dap::PioSwd;
use crate::flash_config::{load_profile, load_topology, ProbeFlash, FLASH_SIZE};
use crate::instances::{build_engines, PinTable, PioBlocks};
use crate::uart_bridge::CdcClass;
use crate::vendor::{AfterResponse, ConfigService};

bind_interrupts!(pub struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    PIO1_IRQ_0 => pio::InterruptHandler<embassy_rp::peripherals::PIO1>;
    UART0_IRQ => embassy_rp::uart::BufferedInterruptHandler<embassy_rp::peripherals::UART0>;
    UART1_IRQ => embassy_rp::uart::BufferedInterruptHandler<embassy_rp::peripherals::UART1>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>;
});

#[cfg(feature = "rp2350")]
bind_interrupts!(pub struct IrqsPio2 {
    PIO2_IRQ_0 => pio::InterruptHandler<embassy_rp::peripherals::PIO2>;
});

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 3] = [
    embassy_rp::binary_info::rp_program_name!(c"rustprobe"),
    embassy_rp::binary_info::rp_program_description!(c"Multi-probe CMSIS-DAP firmware"),
    embassy_rp::binary_info::rp_cargo_version!(),
];

#[cfg(feature = "rp2040")]
const CHIP: Chip = Chip::Rp2040;
#[cfg(feature = "rp2040")]
const LIMITS: ChipLimits = probe_config::RP2040;
#[cfg(not(feature = "rp2040"))]
const CHIP: Chip = Chip::Rp2350;
#[cfg(not(feature = "rp2040"))]
const LIMITS: ChipLimits = probe_config::RP2350A;

/// Fallback when no valid topology is stored: the stock debugprobe-on-pico
/// probe (SWCLK=GP2, SWDIO=GP3, nRESET=GP1) plus its CDC-UART bridge
/// (TX=GP4, RX=GP5 on UART1 @ 115200), matching the C firmware's stock pico
/// config.
fn default_topology() -> Topology {
    let mut t = Topology::default();
    t.probes
        .push(ProbeConfig { swclk: 2, swdio: 3, reset: Some(1) })
        .unwrap();
    t.uarts
        .push(probe_config::UartConfig { tx: 4, rx: 5, baud: 115200 })
        .unwrap();
    t
}

/// Full-speed bulk endpoint packet size.
const USB_MPS: u16 = 64;

/// CMSIS-DAP logical packet size, as advertised by `DAP_Info`. Larger than
/// the USB packet: a request or reply spans several USB packets, and a
/// `DAP_TransferBlock` carries ~126 words per USB round trip instead of 14
/// (issue #6). Hosts size their requests from this value; vendor config
/// commands stay single-packet (`probe_config::protocol`).
const DAP_PACKET_SIZE: u16 = 512;

/// CMSIS-DAP firmware version reported by `DAP_Info`; passed to dap-async
/// and mirrored by the core-0 fast path (`dap_info_response`).
const DAP_FW_VERSION: &str = "2.1.0";

const DAP_INTERFACE_STRING: &str = "CMSIS-DAP v2 Interface";

/// Same WinUSB GUID as the C firmware, so existing host setups keep working.
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{CDB3B5AD-293B-4663-AA36-1AAE46463776}"];

type DapHandler = dap_async::Dap<PioSwd, embassy_time::Delay>;

/// Serves the CMSIS-DAP interface string (probe-rs discovers probes by it)
/// and the pico-sdk-style "Reset" interface string.
struct InterfaceStrings {
    dap_str: StringIndex,
    reset_str: StringIndex,
}

impl Handler for InterfaceStrings {
    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        if index == self.dap_str {
            Some(DAP_INTERFACE_STRING)
        } else if index == self.reset_str {
            Some("Reset")
        } else {
            None
        }
    }
}

/// Read the per-device unique id (flash uid on RP2040, OTP chipid on RP2350).
fn unique_id(flash: &mut embassy_rp::flash::Flash<'static, embassy_rp::peripherals::FLASH, embassy_rp::flash::Blocking, FLASH_SIZE>) -> u64 {
    #[cfg(feature = "rp2040")]
    {
        let mut uid = [0u8; 8];
        flash.blocking_unique_id(&mut uid).expect("flash unique id");
        u64::from_be_bytes(uid)
    }
    #[cfg(not(feature = "rp2040"))]
    {
        let _ = flash;
        embassy_rp::otp::get_chipid().expect("otp chipid")
    }
}

/// `"<uid hex>:<instance>"`, the serial scheme of the C multiprobe firmware.
/// Instance is decimal (up to two digits — MAX_PROBES is 12).
fn format_serial(buf: &mut [u8; 20], uid: u64, instance: usize) -> &str {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for i in 0..16 {
        buf[i] = HEX[((uid >> (60 - 4 * i)) & 0xf) as usize];
    }
    buf[16] = b':';
    let len = if instance >= 10 {
        buf[17] = b'0' + (instance / 10) as u8;
        buf[18] = b'0' + (instance % 10) as u8;
        19
    } else {
        buf[17] = b'0' + instance as u8;
        18
    };
    core::str::from_utf8(&buf[..len]).unwrap()
}

static CORE1_STACK: StaticCell<Stack<8192>> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("rustprobe starting");

    // --- Flash: unique id + stored topology -------------------------------
    let mut flash = embassy_rp::flash::Flash::new_blocking(p.FLASH);
    let uid = unique_id(&mut flash);
    let mut flash: ProbeFlash = BlockingAsync::new(flash);

    // Stored board profile, or the built-in Pico default. A stored profile
    // that doesn't fit this chip (e.g. flash carried over from another board)
    // is ignored rather than trusted.
    let profile = match load_profile(&mut flash).await {
        Some(p) if p.validate(&LIMITS).is_ok() => p,
        Some(_) => {
            warn!("stored board profile invalid for this chip, using Pico default");
            BoardProfile::PICO
        }
        None => BoardProfile::PICO,
    };
    let (topology, config_fault) = match load_topology(&mut flash).await {
        Some(t) if t.validate(&LIMITS, &profile).is_ok() => (t, false),
        Some(_) => {
            warn!("stored topology invalid, using default");
            (default_topology(), true)
        }
        None => {
            info!("no stored topology, using default");
            (default_topology(), false)
        }
    };
    info!(
        "topology: {} probes, {} uarts{}",
        topology.probes.len(),
        topology.uarts.len(),
        if config_fault { " (config fault)" } else { "" }
    );

    // --- Serial strings ----------------------------------------------------
    static SERIAL_BUFS: StaticCell<[[u8; 20]; MAX_PROBES]> = StaticCell::new();
    let serial_bufs = SERIAL_BUFS.init([[0; 20]; MAX_PROBES]);
    let mut serials: Vec<&'static str, MAX_PROBES> = Vec::new();
    for (i, buf) in serial_bufs.iter_mut().enumerate() {
        serials.push(format_serial(buf, uid, i)).unwrap();
    }

    // --- SWD engines --------------------------------------------------------
    let mut pins = PinTable::new(
        p.PIN_0, p.PIN_1, p.PIN_2, p.PIN_3, p.PIN_4, p.PIN_5, p.PIN_6, p.PIN_7, p.PIN_8, p.PIN_9,
        p.PIN_10, p.PIN_11, p.PIN_12, p.PIN_13, p.PIN_14, p.PIN_15, p.PIN_16, p.PIN_17, p.PIN_18,
        p.PIN_19, p.PIN_20, p.PIN_21, p.PIN_22, p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_26, p.PIN_27,
        p.PIN_28, p.PIN_29,
    );
    let blocks = PioBlocks {
        pio0: p.PIO0,
        pio1: p.PIO1,
        #[cfg(feature = "rp2350")]
        pio2: p.PIO2,
    };
    let (engines, autobaud_sm) = build_engines(&topology, blocks, &mut pins);

    // --- Config service -----------------------------------------------------
    // Built before the USB device: the reset interface's control handler
    // shares its watchdog.
    let info = FirmwareInfo {
        protocol_version: PROTOCOL_VERSION,
        firmware_version: fw_version(),
        chip: CHIP,
        limits: LIMITS,
        active_probes: topology.probes.len() as u8,
        active_uarts: topology.uarts.len() as u8,
        config_fault,
    };
    static SERVICE: StaticCell<ConfigService> = StaticCell::new();
    let service: &'static ConfigService = SERVICE.init(ConfigService::new(
        Watchdog::new(p.WATCHDOG),
        info,
        topology.clone(),
        LIMITS,
        profile,
    ));
    // Flash ops must run on core 0; the worker owns the flash from here on.
    spawner.spawn(flash_config::flash_worker(flash).unwrap());

    // --- USB device ---------------------------------------------------------
    let driver = Driver::new(p.USB, Irqs);

    let mut config = Config::new(0x2E8A, 0x000C); // Raspberry Pi debugprobe VID:PID
    config.manufacturer = Some("Raspberry Pi");
    config.product = Some("Rustprobe (CMSIS-DAP)");
    config.serial_number = Some(serials[0]);
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    // probe-rs deny-lists VID 2e8a PID 000c with bcdDevice < 2.20 (picoprobe
    // era debugprobe firmware); claim debugprobe 2.2.0 parity.
    config.device_release = 0x0220;

    static CONFIG_DESC: StaticCell<[u8; 1024]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    // ~170 B per WinUSB function subset × 12 probes, plus headers.
    static MSOS_DESC: StaticCell<[u8; 3072]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 1024]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 3072]),
        CONTROL_BUF.init([0; 64]),
    );

    builder.msos_descriptor(windows_version::WIN8_1, 2);
    let dap_str = builder.string();

    // One CMSIS-DAP v2 function per probe: a vendor interface with bulk OUT
    // then bulk IN (order mandated by the CMSIS-DAP spec).
    let mut endpoints: Vec<(Endpoint<'static, USB, Out>, Endpoint<'static, USB, In>), MAX_PROBES> =
        Vec::new();
    for _ in 0..engines.len() {
        let mut func = builder.function(0xFF, 0, 0);
        func.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
        func.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
            "DeviceInterfaceGUIDs",
            msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
        ));
        let mut iface = func.interface();
        let mut alt = iface.alt_setting(0xFF, 0, 0, Some(dap_str));
        let out_ep = alt.endpoint_bulk_out(None, USB_MPS);
        let in_ep = alt.endpoint_bulk_in(None, USB_MPS);
        drop(func);
        endpoints.push((out_ep, in_ep)).ok().unwrap();
    }

    // One CDC-ACM function per configured UART, added *after* the DAP
    // interfaces so DAP interface numbers stay 0..N-1. Each consumes a static
    // `State` (control-transfer scratch); the data copy uses static ring
    // buffers allocated below.
    static CDC_STATES: StaticCell<[cdc_acm::State<'static>; MAX_UARTS]> = StaticCell::new();
    let cdc_states = CDC_STATES.init([const { cdc_acm::State::new() }; MAX_UARTS]);
    let mut cdc_classes: Vec<CdcClass, MAX_UARTS> = Vec::new();
    for state in cdc_states.iter_mut().take(topology.uarts.len()) {
        cdc_classes
            .push(cdc_acm::CdcAcmClass::new(
                &mut builder,
                state,
                uart_bridge::CDC_PACKET_SIZE,
            ))
            .ok()
            .expect("MAX_UARTS");
    }

    // pico-sdk style reset interface (picotool force-reboot), added last so
    // the DAP interfaces keep numbers 0..probes and CDC numbering matches the
    // C firmware.
    let reset_str = builder.string();
    let reset_handler = reset_iface::ResetHandler::add(&mut builder, service, reset_str);
    static RESET_HANDLER: StaticCell<reset_iface::ResetHandler> = StaticCell::new();
    builder.handler(RESET_HANDLER.init(reset_handler));

    static STRINGS: StaticCell<InterfaceStrings> = StaticCell::new();
    builder.handler(STRINGS.init(InterfaceStrings { dap_str, reset_str }));

    let usb = builder.build();

    // --- SWD workers (core 1) and DAP endpoint I/O tasks (core 0) ----------
    // The worker owns the DAP command processor (and with it the PIO
    // engine); the I/O task owns the USB endpoints. They meet at
    // WORKER_REQ/WORKER_RESP.
    let mut workers: Vec<DapHandler, MAX_PROBES> = Vec::new();
    for engine in engines {
        let dap = dap_async::Dap::new(PioSwd::new(engine), embassy_time::Delay, DAP_FW_VERSION, DAP_PACKET_SIZE);
        workers.push(dap).ok().unwrap();
    }

    spawn_core1(p.CORE1, CORE1_STACK.init(Stack::new()), move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner1| {
            for (i, dap) in workers.into_iter().enumerate() {
                spawner1.spawn(swd_worker(dap, i).unwrap());
            }
        })
    });

    for (i, (out_ep, in_ep)) in endpoints.into_iter().enumerate() {
        spawner.spawn(dap_io_task(i, out_ep, in_ep, serials[i], service).unwrap());
    }

    // --- CDC-UART bridges, executed on core 0 (this executor) --------------
    // Each bridge pairs the CDC class built above (in UART order) with a
    // `BufferedUart` claimed on the instance the validated config selects.
    static UART_TX_BUFS: StaticCell<[[u8; uart_bridge::UART_BUF_SIZE]; MAX_UARTS]> = StaticCell::new();
    static UART_RX_BUFS: StaticCell<[[u8; uart_bridge::UART_BUF_SIZE]; MAX_UARTS]> = StaticCell::new();
    let tx_bufs = UART_TX_BUFS.init([[0; uart_bridge::UART_BUF_SIZE]; MAX_UARTS]);
    let rx_bufs = UART_RX_BUFS.init([[0; uart_bridge::UART_BUF_SIZE]; MAX_UARTS]);

    let mut uart0 = Some(p.UART0);
    let mut uart1 = Some(p.UART1);
    for (i, (((ucfg, cdc), tx_buf), rx_buf)) in topology
        .uarts
        .iter()
        .zip(cdc_classes)
        .zip(tx_bufs.iter_mut())
        .zip(rx_bufs.iter_mut())
        .enumerate()
    {
        let mut cfg = uart::Config::default();
        cfg.baudrate = ucfg.baud;
        let bridge_uart = match ucfg.instance().expect("validated topology") {
            0 => pins.claim_uart0(uart0.take().expect("UART0 free"), cfg, ucfg.tx, ucfg.rx, tx_buf, rx_buf),
            1 => pins.claim_uart1(uart1.take().expect("UART1 free"), cfg, ucfg.tx, ucfg.rx, tx_buf, rx_buf),
            _ => unreachable!("validated: UART instance is 0 or 1"),
        };
        spawner.spawn(uart_bridge::uart_bridge(bridge_uart, cdc, i as u8, ucfg.rx).unwrap());
    }

    // Autobaud capture task (core 0). Present only when a UART is configured;
    // `build_engines` reserves its state machine. One dedicated DMA channel
    // streams the PIO edge-timer FIFO into the estimator.
    if let Some(autobaud_sm) = autobaud_sm {
        let dma = embassy_rp::dma::Channel::new(p.DMA_CH0, Irqs);
        spawner.spawn(autobaud::autobaud_task(autobaud_sm, dma).unwrap());
    }

    spawner.spawn(usb_task(usb).unwrap());
}

/// Firmware version from Cargo, as (major, minor, patch).
fn fw_version() -> (u8, u8, u8) {
    fn part(s: &str) -> u8 {
        s.parse().unwrap_or(0)
    }
    (
        part(env!("CARGO_PKG_VERSION_MAJOR")),
        part(env!("CARGO_PKG_VERSION_MINOR")),
        part(env!("CARGO_PKG_VERSION_PATCH")),
    )
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

/// A lane's DAP request and reply buffers, shared by its core-0 I/O task
/// and its core-1 SWD worker without copying.
///
/// Ownership alternates by protocol: the I/O task fills `req` and sends
/// `WorkerRequest::Process`, then touches neither buffer until the
/// `Done` reply arrives; the worker touches them only between those two
/// channel operations. The channels' critical sections order the accesses
/// across cores. Moving the packets through the channels by value instead
/// cost ~45 µs per hop at 512 bytes (byte-wise moves, measured with the
/// stall-diag inbound/outbound counters, issue #6).
struct LaneBufs {
    req: UnsafeCell<[u8; DAP_PACKET_SIZE as usize]>,
    resp: UnsafeCell<[u8; DAP_PACKET_SIZE as usize]>,
}

// Safety: see the protocol above; no two tasks access a lane's buffers at
// the same time.
unsafe impl Sync for LaneBufs {}

impl LaneBufs {
    const fn new() -> Self {
        Self { req: UnsafeCell::new([0; DAP_PACKET_SIZE as usize]), resp: UnsafeCell::new([0; DAP_PACKET_SIZE as usize]) }
    }

    /// Safety: the caller holds the lane's turn (see the type docs).
    #[allow(clippy::mut_from_ref)]
    unsafe fn get(&self) -> (&mut [u8; DAP_PACKET_SIZE as usize], &mut [u8; DAP_PACKET_SIZE as usize]) {
        unsafe { (&mut *self.req.get(), &mut *self.resp.get()) }
    }
}

static LANE_BUFS: [LaneBufs; MAX_PROBES] = [const { LaneBufs::new() }; MAX_PROBES];

/// Request to a core-1 SWD worker.
enum WorkerRequest {
    /// Run the CMSIS-DAP command of `len` bytes in the lane's request
    /// buffer; the worker answers with `WorkerReply` (len 0 = no response,
    /// e.g. `DAP_TransferAbort`). `stamp`: hand-off time (stall-diag).
    Process { len: u16, stamp: u32 },
    /// The interface went down: `Dap::suspend` (park SWDIO, release nRESET).
    /// No response.
    Suspend,
}

/// Reply from a core-1 SWD worker: the response is in the lane's reply
/// buffer.
struct WorkerReply {
    len: u16,
    stamp: u32,
}

/// Per-probe channels between I/O task (core 0) and SWD worker (core 1).
/// Depth 1: CMSIS-DAP is strictly request/response per interface, and the
/// I/O task always collects the response before sending anything else, so
/// a `Suspend` behind an abandoned `Process` is the deepest it ever gets.
static WORKER_REQ: [Channel<CriticalSectionRawMutex, WorkerRequest, 1>; MAX_PROBES] =
    [const { Channel::new() }; MAX_PROBES];
static WORKER_RESP: [Channel<CriticalSectionRawMutex, WorkerReply, 1>; MAX_PROBES] =
    [const { Channel::new() }; MAX_PROBES];

/// SWD execution worker: the only code that touches this probe's PIO engine.
/// Runs on core 1 alongside the other probes' workers; `process_command`
/// yields whenever it waits on the wire.
#[embassy_executor::task(pool_size = MAX_PROBES)]
async fn swd_worker(mut dap: DapHandler, idx: usize) -> ! {
    loop {
        match WORKER_REQ[idx].receive().await {
            WorkerRequest::Suspend => dap.suspend().await,
            WorkerRequest::Process { len, stamp } => {
                // Safety: our turn with the lane's buffers (see `LaneBufs`).
                let (req, resp) = unsafe { LANE_BUFS[idx].get() };
                stall_diag::inbound(idx, stamp);
                let busy = stall_diag::worker_start();
                dap::ACTIVE_LANES.fetch_add(1, Ordering::Relaxed);
                let len = dap::InRam(dap.process_command(&req[..len as usize], resp)).await;
                dap::ACTIVE_LANES.fetch_sub(1, Ordering::Relaxed);
                stall_diag::worker_done(idx, busy);
                WORKER_RESP[idx].send(WorkerReply { len: len as u16, stamp: stall_diag::now_us() }).await;
            }
        }
    }
}

/// DAP endpoint I/O for one probe interface. Runs on core 0 so endpoint
/// registers are never touched from core 1 and so `DAP_Info` / config
/// vendor commands are answered at USB latency regardless of what the SWD
/// workers are doing (see module docs).
#[embassy_executor::task(pool_size = MAX_PROBES)]
async fn dap_io_task(
    idx: usize,
    mut out_ep: Endpoint<'static, USB, Out>,
    mut in_ep: Endpoint<'static, USB, In>,
    serial: &'static str,
    service: &'static ConfigService,
) -> ! {
    let mut diag = stall_diag::Lane::new(idx, in_ep.info().addr.index());
    loop {
        out_ep.wait_enabled().await;
        info!("DAP interface enabled ({})", serial);
        // Polled from RAM: the request/reply path, with the endpoint
        // futures it inlines, otherwise walks ~8 KB of flash per command
        // and competes with core 1 for the XIP cache (see `dap::InRam`).
        while dap::InRam(serve_request(idx, &mut out_ep, &mut in_ep, serial, service, &mut diag)).await.is_ok() {}
        info!("DAP interface disabled ({})", serial);
        WORKER_REQ[idx].send(WorkerRequest::Suspend).await;
    }
}

/// One request/reply exchange on a DAP interface. `Err` when the endpoint
/// went down (unconfigured/suspended).
async fn serve_request(
    idx: usize,
    out_ep: &mut Endpoint<'static, USB, Out>,
    in_ep: &mut Endpoint<'static, USB, In>,
    serial: &'static str,
    service: &'static ConfigService,
    diag: &mut stall_diag::Lane,
) -> Result<(), ()> {
    const DAP_INFO: u8 = 0x00;
    // Safety: our turn with the lane's buffers (see `LaneBufs`); the worker
    // holds them only between the `Process` send and the reply receive.
    let (request, response) = unsafe { LANE_BUFS[idx].get() };
    let n = match read_request(out_ep, request).await? {
        Some(n) => n,
        None => return Ok(()), // empty or oversized request: nothing to answer
    };
    let stamp = diag.request();

    let (len, after) = match request[0] {
        DAP_INFO => {
            let len = match diag.info(&request[..n], response) {
                Some(len) => len,
                None => dap_info_response(&request[..n], serial, response),
            };
            (len, AfterResponse::Nothing)
        }
        cmd if (VENDOR_BASE..=VENDOR_END).contains(&cmd) => service.handle(&request[..n], response).await,
        _ => {
            WORKER_REQ[idx].send(WorkerRequest::Process { len: n as u16, stamp: stall_diag::now_us() }).await;
            // Always collect the response, even if the endpoint dies
            // meanwhile, so the channels never go out of step.
            let reply = WORKER_RESP[idx].receive().await;
            stall_diag::outbound(idx, reply.stamp);
            diag.worker_answered(&stamp);
            (reply.len as usize, AfterResponse::Nothing)
        }
    };
    if len == 0 {
        return Ok(()); // e.g. DAP_TransferAbort: no response
    }
    // Standard-command hosts read `DAP_PACKET_SIZE` at a time, so a reply
    // that is an exact multiple of the USB packet needs a zero-length
    // packet to end it. Vendor hosts (the CLI, the web UI) read exactly one
    // USB packet and must not get one.
    let zlp = !(VENDOR_BASE..=VENDOR_END).contains(&request[0]);
    write_response(in_ep, &response[..len], zlp).await?;
    diag.reply_armed(&stamp);
    match after {
        AfterResponse::Nothing => {}
        AfterResponse::Reboot => service.reboot().await,
        AfterResponse::Bootsel => {
            // Give the host time to collect the response first.
            embassy_time::Timer::after_millis(100).await;
            embassy_rp::rom_data::reset_to_usb_boot(0, 0);
        }
    }
    Ok(())
}

/// Read one DAP request, reassembling it from USB packets: a short packet
/// ends it, and so does reaching the length the command itself implies
/// (`dap_async::request_len`), which is how a request that is an exact
/// multiple of the USB packet size ends. Vendor commands are single-packet
/// by protocol. Returns `Ok(None)` for an empty request or one that would
/// overflow the buffer (the rest of it is discarded packet by packet as it
/// arrives, as the next "request"), `Err` when the endpoint goes down.
async fn read_request(
    out_ep: &mut Endpoint<'static, USB, Out>,
    request: &mut [u8; DAP_PACKET_SIZE as usize],
) -> Result<Option<usize>, ()> {
    use dap_async::Framing;
    let mps = USB_MPS as usize;
    let mut len = 0;
    loop {
        if len + mps > request.len() {
            warn!("DAP request over {} bytes, dropped", request.len());
            return Ok(None);
        }
        let n = out_ep.read(&mut request[len..len + mps]).await.map_err(|_| ())?;
        len += n;
        if n < mps {
            break;
        }
        match dap_async::request_len(&request[..len]) {
            Framing::NeedMore => continue,
            Framing::Complete(_) | Framing::Unknown => break,
        }
    }
    Ok(if len == 0 { None } else { Some(len) })
}

/// Send one DAP reply as USB packets, ending an exact multiple of the USB
/// packet size with a zero-length packet when `zlp` is set.
async fn write_response(in_ep: &mut Endpoint<'static, USB, In>, data: &[u8], zlp: bool) -> Result<(), ()> {
    let mps = USB_MPS as usize;
    for chunk in data.chunks(mps) {
        in_ep.write(chunk).await.map_err(|_| ())?;
    }
    if zlp && data.len().is_multiple_of(mps) {
        in_ep.write(&[]).await.map_err(|_| ())?;
    }
    Ok(())
}

/// Answer a `DAP_Info` request on core 0, byte-identical to what dap-async
/// produces for this firmware's fixed configuration (SWD-only capabilities,
/// no SWO, no test-domain timer, 64-byte packets) — except for the serial
/// number, where dap-async returns an empty string and we return the
/// per-instance serial that lets hosts tell multiprobe instances apart.
/// Returns the response length.
fn dap_info_response(request: &[u8], serial: &str, response: &mut [u8]) -> usize {
    const DAP_INFO: u8 = 0x00;
    const ID_SERIAL: u8 = 0x03;
    const ID_FW_VERSION: u8 = 0x04;
    const ID_CAPABILITIES: u8 = 0xF0;
    const ID_SWO_BUFFER_SIZE: u8 = 0xFD;
    const ID_MAX_PACKET_COUNT: u8 = 0xFE;
    const ID_MAX_PACKET_SIZE: u8 = 0xFF;
    const CAPABILITIES: u8 = dap_async::command::CAPABILITIES;

    response[0] = DAP_INFO;
    match request.get(1) {
        Some(&ID_SERIAL) => {
            let bytes = serial.as_bytes();
            response[1] = bytes.len() as u8 + 1; // include NUL
            response[2..2 + bytes.len()].copy_from_slice(bytes);
            response[2 + bytes.len()] = 0;
            bytes.len() + 3
        }
        Some(&ID_FW_VERSION) => {
            let bytes = DAP_FW_VERSION.as_bytes();
            response[1] = bytes.len() as u8;
            response[2..2 + bytes.len()].copy_from_slice(bytes);
            bytes.len() + 2
        }
        Some(&ID_CAPABILITIES) => {
            response[1] = 1;
            response[2] = CAPABILITIES;
            3
        }
        Some(&ID_SWO_BUFFER_SIZE) => {
            // NoSwo: zero-byte buffer.
            response[1] = 4;
            response[2..6].copy_from_slice(&0u32.to_le_bytes());
            6
        }
        Some(&ID_MAX_PACKET_COUNT) => {
            // One packet at a time, matching dap-async.
            response[1] = 1;
            response[2] = 1;
            3
        }
        Some(&ID_MAX_PACKET_SIZE) => {
            response[1] = 2;
            response[2..4].copy_from_slice(&DAP_PACKET_SIZE.to_le_bytes());
            4
        }
        // Everything else (vendor/product/target strings, test-domain
        // timer, unknown ids, missing id byte): zero-length value.
        _ => {
            response[1] = 0;
            2
        }
    }
}
