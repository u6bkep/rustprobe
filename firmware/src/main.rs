//! Rustprobe firmware: N SWD probes over CMSIS-DAP v2, topology read from
//! flash at boot.
//!
//! Core 0 runs USB (and, later, the UART bridges); core 1 runs all DAP
//! tasks. Config changes arrive as vendor commands (staged → commit →
//! reboot); see `probe_config::protocol`.

#![no_std]
#![no_main]

mod dap;
mod flash_config;
mod instances;
mod swd;
mod uart_bridge;
mod vendor;

use defmt::{info, warn};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::{Executor, Spawner};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::USB;
use embassy_rp::uart;
use embassy_rp::usb::{Driver, Endpoint, In, InterruptHandler, Out};
use embassy_rp::watchdog::Watchdog;
use embassy_rp::{bind_interrupts, pio};
use embassy_usb::class::cdc_acm;
use embassy_usb::driver::{Endpoint as _, EndpointIn, EndpointOut};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::types::StringIndex;
use embassy_usb::{Builder, Config, Handler, UsbDevice};
use heapless::Vec;
use probe_config::protocol::{Chip, FirmwareInfo, PROTOCOL_VERSION, VENDOR_BASE, VENDOR_END};
use probe_config::{BoardProfile, ChipLimits, ProbeConfig, Topology, MAX_PROBES, MAX_UARTS};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use crate::dap::{NoLeds, ProbeContext, ProbeJtag, ProbeSwd};
use crate::flash_config::{load_topology, ProbeFlash, FLASH_SIZE};
use crate::instances::{build_engines, PinTable, PioBlocks};
use crate::uart_bridge::CdcClass;
use crate::vendor::{AfterResponse, ConfigService};

bind_interrupts!(pub struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    PIO1_IRQ_0 => pio::InterruptHandler<embassy_rp::peripherals::PIO1>;
    UART0_IRQ => embassy_rp::uart::BufferedInterruptHandler<embassy_rp::peripherals::UART0>;
    UART1_IRQ => embassy_rp::uart::BufferedInterruptHandler<embassy_rp::peripherals::UART1>;
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

/// CMSIS-DAP packet size (bulk full-speed MPS).
const DAP_PACKET_SIZE: u16 = 64;

const DAP_INTERFACE_STRING: &str = "CMSIS-DAP v2 Interface";

/// Same WinUSB GUID as the C firmware, so existing host setups keep working.
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{CDB3B5AD-293B-4663-AA36-1AAE46463776}"];

type DapHandler = dap_rs::dap::Dap<
    'static,
    ProbeContext,
    NoLeds,
    embassy_time::Delay,
    ProbeJtag,
    ProbeSwd,
    dap_rs::swo::NoSwo,
>;

/// Serves the CMSIS-DAP interface string (probe-rs discovers probes by it).
struct InterfaceStrings {
    dap_str: StringIndex,
}

impl Handler for InterfaceStrings {
    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        (index == self.dap_str).then_some(DAP_INTERFACE_STRING)
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

    let profile = BoardProfile::PICO;
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
    let engines = build_engines(&topology, blocks, &mut pins);

    // --- USB device ---------------------------------------------------------
    let driver = Driver::new(p.USB, Irqs);

    let mut config = Config::new(0x2E8A, 0x000C); // Raspberry Pi debugprobe VID:PID
    config.manufacturer = Some("Raspberry Pi");
    config.product = Some("Rustprobe (CMSIS-DAP)");
    config.serial_number = Some(serials[0]);
    config.max_power = 100;
    config.max_packet_size_0 = 64;

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
        let out_ep = alt.endpoint_bulk_out(None, DAP_PACKET_SIZE);
        let in_ep = alt.endpoint_bulk_in(None, DAP_PACKET_SIZE);
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

    static STRINGS: StaticCell<InterfaceStrings> = StaticCell::new();
    builder.handler(STRINGS.init(InterfaceStrings { dap_str }));

    let usb = builder.build();

    // --- Config service -----------------------------------------------------
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

    // --- DAP handlers, executed on core 1 ----------------------------------
    let mut daps: Vec<(DapHandler, Endpoint<'static, USB, Out>, Endpoint<'static, USB, In>, &'static str), MAX_PROBES> =
        Vec::new();
    for (i, (engine, (out_ep, in_ep))) in engines.into_iter().zip(endpoints).enumerate() {
        let dap: DapHandler = dap_rs::dap::Dap::new(
            ProbeContext::new(engine),
            NoLeds,
            embassy_time::Delay,
            dap_rs::swo::NoSwo,
            "2.1.0",
            DAP_PACKET_SIZE,
        );
        daps.push((dap, out_ep, in_ep, serials[i])).ok().unwrap();
    }

    spawn_core1(p.CORE1, CORE1_STACK.init(Stack::new()), move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner1| {
            for (dap, out_ep, in_ep, serial) in daps {
                spawner1.spawn(dap_task(dap, out_ep, in_ep, serial, service).unwrap());
            }
        })
    });

    // --- CDC-UART bridges, executed on core 0 (this executor) --------------
    // Each bridge pairs the CDC class built above (in UART order) with a
    // `BufferedUart` claimed on the instance the validated config selects.
    static UART_TX_BUFS: StaticCell<[[u8; uart_bridge::UART_BUF_SIZE]; MAX_UARTS]> = StaticCell::new();
    static UART_RX_BUFS: StaticCell<[[u8; uart_bridge::UART_BUF_SIZE]; MAX_UARTS]> = StaticCell::new();
    let tx_bufs = UART_TX_BUFS.init([[0; uart_bridge::UART_BUF_SIZE]; MAX_UARTS]);
    let rx_bufs = UART_RX_BUFS.init([[0; uart_bridge::UART_BUF_SIZE]; MAX_UARTS]);

    let mut uart0 = Some(p.UART0);
    let mut uart1 = Some(p.UART1);
    for (((ucfg, cdc), tx_buf), rx_buf) in topology
        .uarts
        .iter()
        .zip(cdc_classes)
        .zip(tx_bufs.iter_mut())
        .zip(rx_bufs.iter_mut())
    {
        let mut cfg = uart::Config::default();
        cfg.baudrate = ucfg.baud;
        let bridge_uart = match ucfg.instance().expect("validated topology") {
            0 => pins.claim_uart0(uart0.take().expect("UART0 free"), cfg, ucfg.tx, ucfg.rx, tx_buf, rx_buf),
            1 => pins.claim_uart1(uart1.take().expect("UART1 free"), cfg, ucfg.tx, ucfg.rx, tx_buf, rx_buf),
            _ => unreachable!("validated: UART instance is 0 or 1"),
        };
        spawner.spawn(uart_bridge::uart_bridge(bridge_uart, cdc).unwrap());
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

#[embassy_executor::task(pool_size = MAX_PROBES)]
async fn dap_task(
    mut dap: DapHandler,
    mut out_ep: Endpoint<'static, USB, Out>,
    mut in_ep: Endpoint<'static, USB, In>,
    serial: &'static str,
    service: &'static ConfigService,
) -> ! {
    let mut request = [0u8; DAP_PACKET_SIZE as usize];
    let mut response = [0u8; DAP_PACKET_SIZE as usize];
    loop {
        out_ep.wait_enabled().await;
        info!("DAP interface enabled ({})", serial);
        loop {
            let n = match out_ep.read(&mut request).await {
                Ok(n) => n,
                Err(_) => break, // endpoint disabled (unconfigured/suspended)
            };
            if n == 0 {
                continue;
            }

            let (len, after) = process_request(&mut dap, &request[..n], &mut response, serial, service).await;
            if len == 0 {
                continue; // e.g. DAP_TransferAbort: no response
            }
            if in_ep.write(&response[..len]).await.is_err() {
                break;
            }
            if let AfterResponse::Reboot = after {
                service.reboot().await;
            }
        }
        info!("DAP interface disabled ({})", serial);
        dap.suspend();
    }
}

/// Dispatch one CMSIS-DAP request, handling what dap-rs doesn't: the
/// serial-number info request and the rustprobe config vendor commands.
async fn process_request(
    dap: &mut DapHandler,
    request: &[u8],
    response: &mut [u8],
    serial: &'static str,
    service: &'static ConfigService,
) -> (usize, AfterResponse) {
    const DAP_INFO: u8 = 0x00;
    const DAP_INFO_SERIAL: u8 = 0x03;
    match *request {
        [DAP_INFO, DAP_INFO_SERIAL, ..] => {
            // dap-rs answers this with an empty string; the per-instance
            // serial is how hosts tell multiprobe instances apart.
            let bytes = serial.as_bytes();
            response[0] = DAP_INFO;
            response[1] = bytes.len() as u8 + 1; // include NUL
            response[2..2 + bytes.len()].copy_from_slice(bytes);
            response[2 + bytes.len()] = 0;
            (bytes.len() + 3, AfterResponse::Nothing)
        }
        [cmd, ..] if (VENDOR_BASE..=VENDOR_END).contains(&cmd) => {
            service.handle(request, response).await
        }
        _ => (dap.process_command(request, response), AfterResponse::Nothing),
    }
}
