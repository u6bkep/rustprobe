//! CDC-ACM ⇄ hardware-UART bridges, one per configured UART.
//!
//! Ports debugprobe's `cdc_uart.c`. Each bridge copies bytes in both directions
//! between a USB CDC-ACM serial port (host side) and an RP2040/RP2350 hardware
//! UART (target side), and applies host line-coding / control-line changes to
//! the UART. Bridges run on the core-0 (USB) executor, alongside `usb_task`.
//!
//! Differences from the C firmware, forced by the embassy CDC/UART APIs:
//!
//! * The C code polls on a baud-scaled tick (`cdc_thread`) and batches ≤16-byte
//!   chunks through the PL011 FIFO. We instead await both endpoints and the
//!   UART's interrupt-fed ring buffers (embassy `BufferedUart`, 256-byte RX/TX
//!   buffers) and copy up to a USB packet at a time — no polling interval.
//! * Live line-coding: `BufferedUart` changes baud at runtime (`set_baudrate`)
//!   but has no `set_format`, so data-bit/parity/stop-bit changes cannot be
//!   applied live without dropping and recreating the driver. We apply baud and
//!   compute the rest into a config that only takes effect at construction. See
//!   [`apply_line_coding`] and the control branch of [`uart_bridge`].
//! * Break: embassy's `CdcAcmClass` does not surface the CDC `SEND_BREAK`
//!   request (`tud_cdc_send_break_cb` in the C firmware), so host-driven breaks
//!   are dropped. See `TODO(break)`. `BufferedUart::send_break` is ready for
//!   when it can be wired up.
//! * DTR: as in the C firmware (`tud_cdc_line_state_cb`, which suspends the UART
//!   task when DTR is deasserted), a deasserted DTR pauses bridging; we resume
//!   when it reasserts.

use defmt::info;
use embassy_futures::select::{select3, Either3};
use embassy_rp::peripherals::USB;
use embassy_rp::uart::{self, BufferedUart};
use embassy_rp::usb::Driver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, LineCoding, ParityType, StopBits};
use embedded_io_async::{Read as _, Write as _};
use probe_config::MAX_UARTS;

/// Static RX/TX ring-buffer size for each `BufferedUart`, in bytes. The C
/// firmware works one 32-byte PL011 FIFO at a time; 256 bytes of interrupt
/// buffering here decouples the bridge task from FIFO timing.
pub const UART_BUF_SIZE: usize = 256;

/// CDC bulk packet size (full-speed max packet size), matching the DAP
/// interfaces.
pub const CDC_PACKET_SIZE: u16 = 64;

/// The autobaud trigger baud from the C firmware (`MAGIC_BAUD`, 0x2600). A host
/// selecting this rate asks the probe to *measure* the target's baud rather than
/// set a literal line rate.
const MAGIC_BAUD: u32 = 9728;

/// The concrete CDC-ACM class type the bridges consume.
pub type CdcClass = CdcAcmClass<'static, Driver<'static, USB>>;

/// Translate a host CDC line coding into an embassy UART config, starting from
/// `base` (which carries fields we don't derive from the host, e.g. the invert
/// flags). Mirrors `tud_cdc_line_coding_cb` in `cdc_uart.c`: 5–8 data bits (else
/// 8), odd/even/none parity (else none), and 1 or 2 stop bits, translating 1.5
/// stop bits to 2 as the PL011 can't do 1.5 (the C firmware makes the same
/// choice).
pub fn apply_line_coding(base: uart::Config, coding: &LineCoding) -> uart::Config {
    let mut c = base;
    c.baudrate = coding.data_rate();
    c.data_bits = match coding.data_bits() {
        5 => uart::DataBits::DataBits5,
        6 => uart::DataBits::DataBits6,
        7 => uart::DataBits::DataBits7,
        _ => uart::DataBits::DataBits8,
    };
    c.parity = match coding.parity_type() {
        ParityType::Odd => uart::Parity::ParityOdd,
        ParityType::Even => uart::Parity::ParityEven,
        _ => uart::Parity::ParityNone,
    };
    c.stop_bits = match coding.stop_bits() {
        StopBits::Two | StopBits::OnePointFive => uart::StopBits::STOP2,
        StopBits::One => uart::StopBits::STOP1,
    };
    c
}

/// Bridge one CDC-ACM port to one hardware UART until reboot.
///
/// Single-task, select-driven copy: only one endpoint borrows each half inside
/// the `select`, and the follow-up write happens after the losing futures are
/// dropped, so no half is aliased. Dropping a partially-progressed
/// `read`/`read_packet` future is safe — both read from interrupt-fed buffers.
#[embassy_executor::task(pool_size = MAX_UARTS)]
pub async fn uart_bridge(mut uart: BufferedUart, class: CdcClass) -> ! {
    let (mut usb_tx, mut usb_rx, control) = class.split_with_control();
    let mut usb_buf = [0u8; CDC_PACKET_SIZE as usize];
    let mut uart_buf = [0u8; CDC_PACKET_SIZE as usize];

    loop {
        usb_rx.wait_connection().await;
        info!("uart bridge: host connected");

        'connected: loop {
            // DTR deasserted ⇒ host has the port closed. Mirror `cdc_uart.c`,
            // which suspends its UART task: stop pumping data until the control
            // state changes (DTR reasserts, or the host disconnects).
            if !control.dtr() {
                control.control_changed().await;
                continue;
            }

            match select3(
                usb_rx.read_packet(&mut usb_buf),
                uart.read(&mut uart_buf),
                control.control_changed(),
            )
            .await
            {
                // Host → target.
                Either3::First(res) => match res {
                    Ok(n) => {
                        if uart.write_all(&usb_buf[..n]).await.is_err() {
                            break 'connected;
                        }
                    }
                    Err(_) => break 'connected, // endpoint disabled: host gone
                },
                // Target → host. A UART error (framing/overrun/parity/break)
                // just drops the affected bytes, as the C firmware does.
                Either3::Second(res) => {
                    if let Ok(n) = res {
                        if usb_tx.write_packet(&uart_buf[..n]).await.is_err() {
                            break 'connected;
                        }
                    }
                }
                // Line-coding / control-line change.
                Either3::Third(()) => {
                    let coding = control.line_coding();
                    if coding.data_rate() == MAGIC_BAUD {
                        // TODO(autobaud): a later task starts the PIO autobaud
                        // estimator here (see debugprobe `autobaud.c`) instead of
                        // applying 9728 as a literal line rate.
                    } else {
                        // embassy-rp `BufferedUart` exposes `set_baudrate` but no
                        // `set_format`, so only baud is applied live. The
                        // data-bit/parity/stop-bit fields of `cfg` would need a
                        // drop+recreate of the driver to take effect; they are
                        // honoured only at construction (documented limitation).
                        let cfg = apply_line_coding(uart::Config::default(), &coding);
                        uart.set_baudrate(cfg.baudrate);
                    }
                    // TODO(break): embassy's `CdcAcmClass` does not surface the
                    // CDC SEND_BREAK request, so `tud_cdc_send_break_cb`'s
                    // behaviour (timed/steady `uart_set_break`) is unavailable.
                }
            }
        }

        info!("uart bridge: host disconnected");
    }
}
