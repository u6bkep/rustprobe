//! Rustprobe firmware — spike: one SWD probe, hardcoded topology.
//!
//! Presents a single CMSIS-DAP v2 (vendor bulk) interface over USB, driving
//! SWD via a PIO state machine. Multi-instance topology, flash config, and
//! CDC-UART bridges land on top of this skeleton.

#![no_std]
#![no_main]

mod dap;
mod swd;

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, USB};
use embassy_rp::pio::{self, Pio};
use embassy_rp::usb::{Driver, Endpoint, In, InterruptHandler, Out};
use embassy_usb::driver::{Endpoint as _, EndpointIn, EndpointOut};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::types::StringIndex;
use embassy_usb::{Builder, Config, Handler, UsbDevice};
use probe_config::ProbeConfig;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use crate::dap::{NoLeds, ProbeContext, ProbeJtag, ProbeSwd};
use crate::swd::SwdEngine;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 3] = [
    embassy_rp::binary_info::rp_program_name!(c"rustprobe"),
    embassy_rp::binary_info::rp_program_description!(c"Multi-probe CMSIS-DAP firmware"),
    embassy_rp::binary_info::rp_cargo_version!(),
];

/// Spike topology: probe 0 of the C multiprobe pico config.
const PROBE0: ProbeConfig = ProbeConfig { swclk: 2, swdio: 3, reset: Some(1) };

/// CMSIS-DAP packet size (bulk full-speed MPS).
const DAP_PACKET_SIZE: u16 = 64;

const DAP_INTERFACE_STRING: &str = "CMSIS-DAP v2 Interface";

/// Same WinUSB GUID as the C firmware, so existing host setups keep working.
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{CDB3B5AD-293B-4663-AA36-1AAE46463776}"];

type DapHandler = dap_rs::dap::Dap<
    'static,
    ProbeContext<'static, PIO0, 0>,
    NoLeds,
    embassy_time::Delay,
    ProbeJtag<'static, PIO0, 0>,
    ProbeSwd<'static, PIO0, 0>,
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
fn unique_id(p_flash: embassy_rp::Peri<'static, embassy_rp::peripherals::FLASH>) -> u64 {
    #[cfg(feature = "rp2040")]
    {
        let mut flash =
            embassy_rp::flash::Flash::<_, embassy_rp::flash::Blocking, { 2 * 1024 * 1024 }>::new_blocking(
                p_flash,
            );
        let mut uid = [0u8; 8];
        flash.blocking_unique_id(&mut uid).expect("flash unique id");
        u64::from_be_bytes(uid)
    }
    #[cfg(not(feature = "rp2040"))]
    {
        let _ = p_flash;
        embassy_rp::otp::get_chipid().expect("otp chipid")
    }
}

/// `"<uid hex>:<instance>"`, the serial scheme of the C multiprobe firmware.
fn format_serial(buf: &mut [u8; 19], uid: u64, instance: u8) -> &str {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for i in 0..16 {
        buf[i] = HEX[((uid >> (60 - 4 * i)) & 0xf) as usize];
    }
    buf[16] = b':';
    buf[17] = b'0' + instance;
    buf[18] = 0;
    core::str::from_utf8(&buf[..18]).unwrap()
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("rustprobe starting");

    // --- Unique serial ---------------------------------------------------
    static SERIAL_BUF: StaticCell<[u8; 19]> = StaticCell::new();
    let serial: &'static str = format_serial(SERIAL_BUF.init([0; 19]), unique_id(p.FLASH), 0);
    info!("serial: {}", serial);

    // --- SWD engine + DAP handler ----------------------------------------
    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let engine: SwdEngine<'static, PIO0, 0> = SwdEngine::new(
        &mut common,
        sm0,
        p.PIN_2, // PROBE0.swclk
        p.PIN_3, // PROBE0.swdio
        Some(p.PIN_1), // PROBE0.reset
    );
    let _ = PROBE0; // pins above must match; runtime pin selection comes with the config layer
    let ctx = ProbeContext::new(engine);
    let dap: DapHandler = dap_rs::dap::Dap::new(
        ctx,
        NoLeds,
        embassy_time::Delay,
        dap_rs::swo::NoSwo,
        "2.1.0",
        DAP_PACKET_SIZE,
    );

    // --- USB device -------------------------------------------------------
    let driver = Driver::new(p.USB, Irqs);

    let mut config = Config::new(0x2E8A, 0x000C); // Raspberry Pi debugprobe VID:PID
    config.manufacturer = Some("Raspberry Pi");
    config.product = Some("Rustprobe (CMSIS-DAP)");
    config.serial_number = Some(serial);
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    builder.msos_descriptor(windows_version::WIN8_1, 2);

    // CMSIS-DAP v2 function: one vendor interface, bulk OUT then bulk IN
    // (the order is mandated by the CMSIS-DAP spec).
    let mut func = builder.function(0xFF, 0, 0);
    func.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    func.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));
    let mut iface = func.interface();
    let dap_str = iface.string();
    let mut alt = iface.alt_setting(0xFF, 0, 0, Some(dap_str));
    let out_ep = alt.endpoint_bulk_out(None, DAP_PACKET_SIZE);
    let in_ep = alt.endpoint_bulk_in(None, DAP_PACKET_SIZE);
    drop(func);

    static STRINGS: StaticCell<InterfaceStrings> = StaticCell::new();
    builder.handler(STRINGS.init(InterfaceStrings { dap_str }));

    let usb = builder.build();

    spawner.spawn(usb_task(usb).unwrap());
    spawner.spawn(dap_task(dap, out_ep, in_ep, serial).unwrap());
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn dap_task(
    mut dap: DapHandler,
    mut out_ep: Endpoint<'static, USB, Out>,
    mut in_ep: Endpoint<'static, USB, In>,
    serial: &'static str,
) -> ! {
    let mut request = [0u8; DAP_PACKET_SIZE as usize];
    let mut response = [0u8; DAP_PACKET_SIZE as usize];
    loop {
        out_ep.wait_enabled().await;
        info!("DAP interface enabled");
        loop {
            let n = match out_ep.read(&mut request).await {
                Ok(n) => n,
                Err(_) => break, // endpoint disabled (unconfigured/suspended)
            };
            if n == 0 {
                continue;
            }

            let len = process_request(&mut dap, &request[..n], &mut response, serial);
            if len == 0 {
                continue; // e.g. DAP_TransferAbort: no response
            }
            if in_ep.write(&response[..len]).await.is_err() {
                break;
            }
        }
        info!("DAP interface disabled");
        dap.suspend();
    }
}

/// Dispatch one CMSIS-DAP request, handling what dap-rs doesn't:
/// the serial-number info request and (soon) rustprobe vendor commands.
fn process_request(
    dap: &mut DapHandler,
    request: &[u8],
    response: &mut [u8],
    serial: &'static str,
) -> usize {
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
            bytes.len() + 3
        }
        // TODO(config): intercept rustprobe vendor commands (0x80..=0x9F)
        // here before they reach dap-rs.
        _ => dap.process_command(request, response),
    }
}
