//! pico-sdk compatible USB reset interface, so picotool can reboot the
//! running firmware (`picotool reboot -f -u`, `picotool load -f`, ...).
//!
//! Ports pico_stdio_usb's `reset_interface.c`: a vendor interface (class
//! 0xFF, subclass 0x00, protocol 0x01, no endpoints) answering two
//! interface-directed vendor control requests:
//!
//! * `RESET_REQUEST_BOOTSEL` (0x01): reboot into the BOOTSEL bootloader.
//!   `wValue` bit 8 set means bits 9.. name an activity-LED gpio; the low
//!   7 bits are the bootrom interface-disable mask.
//! * `RESET_REQUEST_FLASH` (0x02): reboot back into the application.
//!
//! picotool discovers the interface by that class triple, which is what the
//! C multiprobe firmware lacked (BOOTSEL button required to reflash).

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::Duration;
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::Driver;
use embassy_usb::types::{InterfaceNumber, StringIndex};
use embassy_usb::{Builder, Handler};

use crate::vendor::ConfigService;

const RESET_REQUEST_BOOTSEL: u8 = 0x01;
const RESET_REQUEST_FLASH: u8 = 0x02;

/// TODO(remove-diag): ring of the last few control requests this handler was
/// offered, readable over the bulk vendor protocol (CMD_CTRL_DIAG) while we
/// debug why delegated control transfers stall on hardware.
pub static CTRL_DIAG: Mutex<CriticalSectionRawMutex, RefCell<heapless::Deque<[u8; 8], 6>>> =
    Mutex::new(RefCell::new(heapless::Deque::new()));

fn record(dir_in: bool, req: &Request) {
    let entry = [
        if dir_in { 0x1B } else { 0x0B }, // marker + direction
        req.request_type as u8,
        req.recipient as u8,
        req.request,
        req.value as u8,
        (req.value >> 8) as u8,
        req.index as u8,
        (req.index >> 8) as u8,
    ];
    CTRL_DIAG.lock(|d| {
        let mut d = d.borrow_mut();
        if d.is_full() {
            d.pop_front();
        }
        let _ = d.push_back(entry);
    });
}

/// Watchdog delay on a reboot-to-application request, so the host sees the
/// control transfer complete first (pico-sdk uses 100 ms too).
const RESET_TO_FLASH_DELAY: Duration = Duration::from_millis(100);

pub struct ResetHandler {
    itf: InterfaceNumber,
    service: &'static ConfigService,
}

impl ResetHandler {
    /// Append the reset interface to `builder`. Added after every other
    /// function so the DAP interfaces keep numbers `0..probes`.
    pub fn add<D: Driver<'static>>(
        builder: &mut Builder<'static, D>,
        service: &'static ConfigService,
        name: StringIndex,
    ) -> Self {
        let mut func = builder.function(0xFF, 0x00, 0x01);
        let mut iface = func.interface();
        let itf = iface.interface_number();
        let _alt = iface.alt_setting(0xFF, 0x00, 0x01, Some(name));
        drop(func);
        ResetHandler { itf, service }
    }
}

impl Handler for ResetHandler {
    // TODO(remove-diag): record IN requests too, purely for the diag ring.
    fn control_in<'a>(&'a mut self, req: Request, _buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        record(true, &req);
        None
    }

    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        record(false, &req);
        if req.request_type != RequestType::Vendor
            || req.recipient != Recipient::Interface
            || req.index != u16::from(u8::from(self.itf))
        {
            return None;
        }
        match req.request {
            RESET_REQUEST_BOOTSEL => {
                let gpio_mask = if req.value & 0x100 != 0 {
                    1u32.checked_shl(u32::from(req.value >> 9)).unwrap_or(0)
                } else {
                    0
                };
                // Does not return (the rp2350 wrapper returns only on a rom
                // failure, which leaves us rejecting the request).
                embassy_rp::rom_data::reset_to_usb_boot(gpio_mask, u32::from(req.value & 0x7f));
                Some(OutResponse::Rejected)
            }
            RESET_REQUEST_FLASH => {
                if self.service.arm_watchdog_reboot(RESET_TO_FLASH_DELAY) {
                    Some(OutResponse::Accepted)
                } else {
                    Some(OutResponse::Rejected)
                }
            }
            _ => Some(OutResponse::Rejected),
        }
    }
}
