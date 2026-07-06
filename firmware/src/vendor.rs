//! Rustprobe config protocol handler (CMSIS-DAP vendor commands 0x80..=0x9F).
//!
//! Commands arrive on any probe's DAP interface; all of them share this
//! service. See `probe_config::protocol` for the wire format.

use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use probe_config::protocol::*;
use probe_config::{BoardProfile, ChipLimits, Topology};

use crate::flash_config::{commit_profile, commit_topology};

/// Max payload bytes per GET_TOPOLOGY response chunk
/// (64-byte packet minus cmd, status, total_len).
const GET_CHUNK: usize = 61;

struct Staged {
    buf: [u8; TOPOLOGY_BUF_LEN],
    filled: usize,
}

/// Shared state behind the config protocol.
pub struct ConfigService {
    staged: Mutex<CriticalSectionRawMutex, Staged>,
    watchdog: Mutex<CriticalSectionRawMutex, Watchdog>,
    info: FirmwareInfo,
    active: Topology,
    limits: ChipLimits,
    /// Current board profile. Updated in place by `CMD_SET_PROFILE` so later
    /// topology commits validate against the new profile without a reboot.
    profile: Mutex<CriticalSectionRawMutex, BoardProfile>,
}

/// What the DAP task should do after sending the response.
pub enum AfterResponse {
    Nothing,
    Reboot,
    /// Reboot into the BOOTSEL bootloader (bootrom `reset_to_usb_boot`).
    Bootsel,
}

impl ConfigService {
    pub fn new(
        watchdog: Watchdog,
        info: FirmwareInfo,
        active: Topology,
        limits: ChipLimits,
        profile: BoardProfile,
    ) -> Self {
        Self {
            staged: Mutex::new(Staged { buf: [0; TOPOLOGY_BUF_LEN], filled: 0 }),
            watchdog: Mutex::new(watchdog),
            info,
            active,
            limits,
            profile: Mutex::new(profile),
        }
    }

    /// Handle a vendor command. Returns `(response_len, after)`.
    /// `request[0]` must already be within `VENDOR_BASE..=VENDOR_END`.
    pub async fn handle(&self, request: &[u8], response: &mut [u8]) -> (usize, AfterResponse) {
        let cmd = request[0];
        response[0] = cmd;
        response[1] = STATUS_OK;
        let mut after = AfterResponse::Nothing;

        let len = match cmd {
            CMD_INFO => match postcard::to_slice(&self.info, &mut response[2..]) {
                Ok(payload) => 2 + payload.len(),
                Err(_) => {
                    response[1] = STATUS_ERR_BAD_REQUEST;
                    2
                }
            },
            CMD_GET_TOPOLOGY => self.get_topology(request, response),
            CMD_SET_TOPOLOGY => self.set_topology(request, response).await,
            CMD_COMMIT => self.commit(request, response).await,
            CMD_GET_PROFILE => self.get_profile(response).await,
            CMD_SET_PROFILE => self.set_profile(request, response).await,
            CMD_REBOOT => {
                after = AfterResponse::Reboot;
                2
            }
            CMD_BOOTSEL => {
                after = AfterResponse::Bootsel;
                2
            }
            _ => {
                response[1] = STATUS_ERR_BAD_REQUEST;
                2
            }
        };
        (len, after)
    }

    fn get_topology(&self, request: &[u8], response: &mut [u8]) -> usize {
        let mut encoded = [0u8; TOPOLOGY_BUF_LEN];
        let encoded: &[u8] = match postcard::to_slice(&self.active, &mut encoded) {
            Ok(e) => e,
            Err(_) => {
                response[1] = STATUS_ERR_DECODE;
                return 2;
            }
        };
        let offset = *request.get(1).unwrap_or(&0) as usize;
        if offset > encoded.len() {
            response[1] = STATUS_ERR_BAD_REQUEST;
            return 2;
        }
        let chunk = (encoded.len() - offset).min(GET_CHUNK);
        response[2] = encoded.len() as u8;
        response[3..3 + chunk].copy_from_slice(&encoded[offset..offset + chunk]);
        3 + chunk
    }

    async fn set_topology(&self, request: &[u8], response: &mut [u8]) -> usize {
        let Some(&offset) = request.get(1) else {
            response[1] = STATUS_ERR_BAD_REQUEST;
            return 2;
        };
        let data = &request[2..];
        let mut staged = self.staged.lock().await;
        if offset == 0 {
            staged.filled = 0;
        }
        if offset as usize != staged.filled || staged.filled + data.len() > TOPOLOGY_BUF_LEN {
            response[1] = STATUS_ERR_BAD_REQUEST;
            return 2;
        }
        let filled = staged.filled;
        staged.buf[filled..filled + data.len()].copy_from_slice(data);
        staged.filled += data.len();
        2
    }

    async fn commit(&self, request: &[u8], response: &mut [u8]) -> usize {
        let Some(&total) = request.get(1) else {
            response[1] = STATUS_ERR_BAD_REQUEST;
            return 2;
        };
        let staged = self.staged.lock().await;
        if staged.filled != total as usize {
            response[1] = STATUS_ERR_BAD_REQUEST;
            return 2;
        }
        let topo: Topology = match postcard::from_bytes(&staged.buf[..staged.filled]) {
            Ok(t) => t,
            Err(_) => {
                response[1] = STATUS_ERR_DECODE;
                return 2;
            }
        };
        if topo.validate(&self.limits, &*self.profile.lock().await).is_err() {
            response[1] = STATUS_ERR_INVALID;
            return 2;
        }
        if commit_topology(topo).await.is_err() {
            response[1] = STATUS_ERR_FLASH;
        }
        2
    }

    async fn get_profile(&self, response: &mut [u8]) -> usize {
        let profile = *self.profile.lock().await;
        match postcard::to_slice(&profile, &mut response[2..]) {
            Ok(payload) => 2 + payload.len(),
            Err(_) => {
                response[1] = STATUS_ERR_DECODE;
                2
            }
        }
    }

    async fn set_profile(&self, request: &[u8], response: &mut [u8]) -> usize {
        let profile: BoardProfile = match postcard::from_bytes(&request[1..]) {
            Ok(p) => p,
            Err(_) => {
                response[1] = STATUS_ERR_DECODE;
                return 2;
            }
        };
        if profile.validate(&self.limits).is_err() {
            response[1] = STATUS_ERR_INVALID;
            return 2;
        }
        if commit_profile(profile).await.is_err() {
            response[1] = STATUS_ERR_FLASH;
            return 2;
        }
        *self.profile.lock().await = profile;
        2
    }

    /// Arm the watchdog to hard-reset after `delay` (sync, for the USB reset
    /// interface's control handler). Returns false if the watchdog is locked,
    /// which only happens when a vendor-command reboot is already in flight.
    pub fn arm_watchdog_reboot(&self, delay: embassy_time::Duration) -> bool {
        match self.watchdog.try_lock() {
            Ok(mut wd) => {
                wd.start(delay);
                true
            }
            Err(_) => false,
        }
    }

    /// Reset the probe (does not return).
    pub async fn reboot(&self) -> ! {
        // Give the host time to collect the response.
        Timer::after_millis(100).await;
        self.watchdog.lock().await.trigger_reset();
        loop {
            core::hint::spin_loop();
        }
    }
}
