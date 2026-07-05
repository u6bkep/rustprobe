//! Topology persistence in the last flash sectors via sequential-storage.
//!
//! embassy-rp's flash operations must run on core 0 (they return
//! `InvalidCore` elsewhere) and park core 1 for the duration. Vendor
//! commands execute in DAP tasks on core 1, so commits are marshalled to
//! [`flash_worker`], a core-0 task owning the flash; the requesting DAP
//! task awaits the result. DAP transactions on other probes stall briefly
//! while core 1 is parked — acceptable for a config write.

use core::ops::Range;

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use probe_config::protocol::TOPOLOGY_BUF_LEN;
use probe_config::Topology;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{fetch_item, store_item};

/// Total flash size the firmware assumes (Pico: 2 MiB, Pico 2: 4 MiB).
#[cfg(feature = "rp2040")]
pub const FLASH_SIZE: usize = 2 * 1024 * 1024;
#[cfg(not(feature = "rp2040"))]
pub const FLASH_SIZE: usize = 4 * 1024 * 1024;

/// Two 4 KiB sectors at the top of flash hold the config map. On RP2040 the
/// linker script excludes this range from the image; on RP2350 the image
/// region (2 MiB) ends far below it. Boards with less than `FLASH_SIZE`
/// flash need a board profile override (future work).
const CONFIG_RANGE: Range<u32> = (FLASH_SIZE as u32 - 8192)..(FLASH_SIZE as u32);

const KEY_TOPOLOGY: u8 = 1;

pub type ProbeFlash = BlockingAsync<Flash<'static, FLASH, Blocking, FLASH_SIZE>>;

/// Item buffer: postcard topology plus sequential-storage overhead.
type ItemBuf = [u8; TOPOLOGY_BUF_LEN + 32];

/// Load the stored topology; `None` if absent or unreadable.
pub async fn load_topology(flash: &mut ProbeFlash) -> Option<Topology> {
    let mut buf: ItemBuf = [0; TOPOLOGY_BUF_LEN + 32];
    let raw: &[u8] =
        fetch_item(flash, CONFIG_RANGE, &mut NoCache::new(), &mut buf, &KEY_TOPOLOGY)
            .await
            .ok()??;
    postcard::from_bytes(raw).ok()
}

/// Persist a topology from any core: marshals the write to [`flash_worker`]
/// on core 0. The caller must have validated the topology.
pub async fn commit_topology(topo: Topology) -> Result<(), ()> {
    let _guard = COMMIT_LOCK.lock().await;
    COMMIT_RESULT.reset();
    COMMIT_REQUEST.send(topo).await;
    COMMIT_RESULT.wait().await
}

static COMMIT_LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());
static COMMIT_REQUEST: Channel<CriticalSectionRawMutex, Topology, 1> = Channel::new();
static COMMIT_RESULT: Signal<CriticalSectionRawMutex, Result<(), ()>> = Signal::new();

/// Owns the flash after boot; must be spawned on the core-0 executor.
#[embassy_executor::task]
pub async fn flash_worker(mut flash: ProbeFlash) -> ! {
    loop {
        let topo = COMMIT_REQUEST.receive().await;
        COMMIT_RESULT.signal(store_topology(&mut flash, &topo).await);
    }
}

/// Persist a topology (already validated by the caller).
async fn store_topology(flash: &mut ProbeFlash, topo: &Topology) -> Result<(), ()> {
    let mut item: ItemBuf = [0; TOPOLOGY_BUF_LEN + 32];
    let mut encoded = [0u8; TOPOLOGY_BUF_LEN];
    let encoded: &[u8] = postcard::to_slice(topo, &mut encoded).map_err(|_| ())?;
    store_item(flash, CONFIG_RANGE, &mut NoCache::new(), &mut item, &KEY_TOPOLOGY, &encoded)
        .await
        .map_err(|_| ())
}
