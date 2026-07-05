//! Topology persistence in the last flash sectors via sequential-storage.
//!
//! Flash writes pause core 1 (embassy-rp's flash driver parks it), so
//! committing from a USB-facing task while DAP tasks run on core 1 is safe;
//! the affected DAP transactions stall for the write duration.

use core::ops::Range;

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::FLASH;
use probe_config::protocol::TOPOLOGY_BUF_LEN;
use probe_config::Topology;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{fetch_item, store_item};

/// Total flash size the firmware assumes (Pico: 2 MiB, Pico 2: 4 MiB).
#[cfg(feature = "rp2040")]
pub const FLASH_SIZE: usize = 2 * 1024 * 1024;
#[cfg(not(feature = "rp2040"))]
pub const FLASH_SIZE: usize = 4 * 1024 * 1024;

/// Two 4 KiB sectors at the top of flash hold the config map. The linker
/// script caps the image well below this.
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

/// Persist a topology (already validated by the caller).
pub async fn store_topology(flash: &mut ProbeFlash, topo: &Topology) -> Result<(), ()> {
    let mut item: ItemBuf = [0; TOPOLOGY_BUF_LEN + 32];
    let mut encoded = [0u8; TOPOLOGY_BUF_LEN];
    let encoded: &[u8] = postcard::to_slice(topo, &mut encoded).map_err(|_| ())?;
    store_item(flash, CONFIG_RANGE, &mut NoCache::new(), &mut item, &KEY_TOPOLOGY, &encoded)
        .await
        .map_err(|_| ())
}
