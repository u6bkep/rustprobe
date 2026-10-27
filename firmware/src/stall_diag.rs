//! Per-lane reply-stall diagnostics (feature `stall-diag`; issue #4).
//!
//! Separates "the probe answered late" from "the host never collected the
//! answer" when a host reports a CMSIS-DAP timeout. Each lane's
//! `dap_io_task` records:
//!
//! * when a request arrives, whether its previous reply is still armed in the
//!   IN buffer — the host sent a new request without having taken it;
//! * how long the SWD worker took to answer, and how long from request to
//!   reply armed.
//!
//! The counters are read with `DAP_Info` id 0xA0 on the lane's own
//! interface (answered on core 0, like the rest of `DAP_Info`); the host tool
//! is `tools/stress` `mpdiag`. Without the feature every hook below is a
//! no-op and id 0xA0 gets the standard empty `DAP_Info` value.

#[cfg(feature = "stall-diag")]
pub use enabled::*;
#[cfg(not(feature = "stall-diag"))]
pub use disabled::*;

#[cfg(feature = "stall-diag")]
mod enabled {
    use embassy_time::Instant;
    use portable_atomic::{AtomicU32, Ordering};
    use probe_config::MAX_PROBES;

    /// `DAP_Info` id carrying this lane's counters (vendor-unassigned).
    const INFO_ID: u8 = 0xA0;

    /// Reply fields, in wire order (u32 LE each). `tools/stress` mirrors this.
    #[derive(Clone, Copy)]
    enum Field {
        /// Requests handed to the SWD worker.
        Cmds,
        /// Worker answers slower than [`SLOW_US`].
        SlowWorker,
        MaxWorkerUs,
        /// Requests that arrived while our previous reply was still armed.
        Unread,
        /// Age of such an uncollected reply when the next request arrived.
        MaxUnreadMs,
        /// Request-to-reply-armed slower than [`SLOW_US`].
        SlowArm,
        MaxArmUs,
    }
    const FIELDS: usize = 7;
    const SLOW_US: u32 = 100_000;

    /// Written only by the lane's own `dap_io_task` (core 0).
    static COUNTERS: [[AtomicU32; FIELDS]; MAX_PROBES] =
        [const { [const { AtomicU32::new(0) }; FIELDS] }; MAX_PROBES];

    /// When a request arrived.
    pub struct Stamp(Instant);

    /// One lane's diagnostic state, owned by its `dap_io_task`.
    pub struct Lane {
        idx: usize,
        /// IN endpoint index (buffer-control slot) of this lane.
        in_index: usize,
        /// When our last reply was armed; cleared by the next request.
        armed_at: Option<Instant>,
    }

    impl Lane {
        pub fn new(idx: usize, in_index: usize) -> Self {
            Self { idx, in_index, armed_at: None }
        }

        fn add(&self, f: Field) {
            COUNTERS[self.idx][f as usize].fetch_add(1, Ordering::Relaxed);
        }

        fn max(&self, f: Field, v: u32) {
            COUNTERS[self.idx][f as usize].fetch_max(v, Ordering::Relaxed);
        }

        /// A request arrived on the OUT endpoint.
        pub fn request(&mut self) -> Stamp {
            let now = Instant::now();
            if let Some(armed) = self.armed_at.take() {
                let uncollected = embassy_rp::pac::USB_DPRAM
                    .ep_in_buffer_control(self.in_index)
                    .read()
                    .available(0);
                if uncollected {
                    let age_ms = (now - armed).as_millis() as u32;
                    self.add(Field::Unread);
                    self.max(Field::MaxUnreadMs, age_ms);
                    defmt::warn!("lane {}: request arrived with previous reply uncollected for {} ms", self.idx, age_ms);
                }
            }
            Stamp(now)
        }

        /// The SWD worker answered the request stamped `at`.
        pub fn worker_answered(&self, at: &Stamp) {
            let us = at.0.elapsed().as_micros() as u32;
            self.add(Field::Cmds);
            self.max(Field::MaxWorkerUs, us);
            if us > SLOW_US {
                self.add(Field::SlowWorker);
                defmt::warn!("lane {}: worker answer took {} us", self.idx, us);
            }
        }

        /// The reply to the request stamped `at` is armed in the IN buffer.
        pub fn reply_armed(&mut self, at: &Stamp) {
            let us = at.0.elapsed().as_micros() as u32;
            self.max(Field::MaxArmUs, us);
            if us > SLOW_US {
                self.add(Field::SlowArm);
            }
            self.armed_at = Some(Instant::now());
        }

        /// Answer a `DAP_Info` [`INFO_ID`] request with this lane's counters;
        /// `None` for any other request. Returns the response length.
        pub fn info(&self, request: &[u8], response: &mut [u8]) -> Option<usize> {
            if request.get(1) != Some(&INFO_ID) {
                return None;
            }
            response[0] = request[0];
            response[1] = (FIELDS * 4) as u8;
            for (i, c) in COUNTERS[self.idx].iter().enumerate() {
                response[2 + 4 * i..6 + 4 * i].copy_from_slice(&c.load(Ordering::Relaxed).to_le_bytes());
            }
            Some(2 + FIELDS * 4)
        }
    }
}

#[cfg(not(feature = "stall-diag"))]
mod disabled {
    pub struct Stamp;

    pub struct Lane;

    impl Lane {
        pub fn new(_idx: usize, _in_index: usize) -> Self {
            Self
        }
        #[inline(always)]
        pub fn request(&mut self) -> Stamp {
            Stamp
        }
        #[inline(always)]
        pub fn worker_answered(&self, _at: &Stamp) {}
        #[inline(always)]
        pub fn reply_armed(&mut self, _at: &Stamp) {}
        #[inline(always)]
        pub fn info(&self, _request: &[u8], _response: &mut [u8]) -> Option<usize> {
            None
        }
    }
}
