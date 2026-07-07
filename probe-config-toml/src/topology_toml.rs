//! Human-readable TOML mirror of [`probe_config::Topology`].
//!
//! The wire type uses `heapless` fixed-capacity vectors; these `std`-backed
//! mirror structs map cleanly to TOML arrays-of-tables and convert to/from the
//! wire type with explicit capacity checks.

use anyhow::{anyhow, Result};
use probe_config::{ProbeConfig, Topology, UartConfig, MAX_PROBES, MAX_UARTS};
use serde::{Deserialize, Serialize};

/// TOML representation of a full topology.
#[derive(Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TomlTopology {
    /// SWD probe instances, in USB interface order.
    #[serde(default)]
    pub probes: Vec<TomlProbe>,
    /// UART bridges, in CDC interface order.
    #[serde(default)]
    pub uarts: Vec<TomlUart>,
}

/// TOML representation of one SWD probe.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TomlProbe {
    /// SWCLK GPIO number.
    pub swclk: u8,
    /// SWDIO GPIO number.
    pub swdio: u8,
    /// Optional nRESET GPIO number.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reset: Option<u8>,
}

/// TOML representation of one UART bridge.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TomlUart {
    /// TX GPIO number (probe → target).
    pub tx: u8,
    /// RX GPIO number (target → probe).
    pub rx: u8,
    /// Initial baud rate.
    pub baud: u32,
}

impl TomlTopology {
    /// Convert into the wire type, rejecting over-capacity inputs.
    pub fn into_topology(self) -> Result<Topology> {
        let mut topo = Topology::default();
        for p in self.probes {
            topo.probes
                .push(ProbeConfig { swclk: p.swclk, swdio: p.swdio, reset: p.reset })
                .map_err(|_| anyhow!("too many probes (max {MAX_PROBES})"))?;
        }
        for u in self.uarts {
            topo.uarts
                .push(UartConfig { tx: u.tx, rx: u.rx, baud: u.baud })
                .map_err(|_| anyhow!("too many uarts (max {MAX_UARTS})"))?;
        }
        Ok(topo)
    }

    /// Build from the wire type.
    pub fn from_topology(topo: &Topology) -> Self {
        TomlTopology {
            probes: topo
                .probes
                .iter()
                .map(|p| TomlProbe { swclk: p.swclk, swdio: p.swdio, reset: p.reset })
                .collect(),
            uarts: topo
                .uarts
                .iter()
                .map(|u| TomlUart { tx: u.tx, rx: u.rx, baud: u.baud })
                .collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_through_toml() {
        let text = "\
[[probes]]
swclk = 2
swdio = 3
reset = 1

[[uarts]]
tx = 4
rx = 5
baud = 115200
";
        let parsed: TomlTopology = toml::from_str(text).unwrap();
        let topo = parsed.into_topology().unwrap();
        assert_eq!(topo.probes.len(), 1);
        assert_eq!(topo.probes[0].reset, Some(1));
        assert_eq!(topo.uarts[0].baud, 115200);

        let back = toml::to_string(&TomlTopology::from_topology(&topo)).unwrap();
        let reparsed: Topology = toml::from_str::<TomlTopology>(&back)
            .unwrap()
            .into_topology()
            .unwrap();
        assert_eq!(topo, reparsed);
    }
}
