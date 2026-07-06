//! Human-readable TOML mirror of [`probe_config::BoardProfile`].
//!
//! The wire type is two `u64` bitmasks; the TOML form uses pin-range strings
//! ("0-15,26-29") so board files stay readable and diffable.

use anyhow::{anyhow, bail, Result};
use probe_config::BoardProfile;
use serde::{Deserialize, Serialize};

/// TOML representation of a board profile.
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TomlBoardProfile {
    /// GPIOs that exist and are wired out, as ranges ("0-15,26-29").
    pub available: String,
    /// GPIOs reserved for non-probe duties (LEDs, ...), as ranges. Reserved
    /// pins are never assignable even if listed available.
    #[serde(default)]
    pub reserved: String,
}

impl TomlBoardProfile {
    /// Convert into the wire type.
    pub fn into_profile(self) -> Result<BoardProfile> {
        Ok(BoardProfile {
            available: parse_pin_ranges(&self.available)?,
            reserved: parse_pin_ranges(&self.reserved)?,
        })
    }

    /// Build from the wire type.
    pub fn from_profile(profile: &BoardProfile) -> Self {
        TomlBoardProfile {
            available: format_pin_ranges(profile.available),
            reserved: format_pin_ranges(profile.reserved),
        }
    }
}

/// Parse "0-15,26-29" (or "" for no pins) into a bitmask. Single pins and
/// ranges may be mixed; ranges are inclusive.
pub fn parse_pin_ranges(text: &str) -> Result<u64> {
    let mut mask = 0u64;
    for part in text.split(',').map(str::trim).filter(|p| !p.is_empty()) {
        let (lo, hi) = match part.split_once('-') {
            Some((lo, hi)) => (parse_pin(lo)?, parse_pin(hi)?),
            None => {
                let pin = parse_pin(part)?;
                (pin, pin)
            }
        };
        if lo > hi {
            bail!("descending pin range \"{part}\"");
        }
        for pin in lo..=hi {
            mask |= 1u64 << pin;
        }
    }
    Ok(mask)
}

fn parse_pin(text: &str) -> Result<u8> {
    let pin: u8 = text
        .trim()
        .parse()
        .map_err(|_| anyhow!("invalid pin number \"{text}\""))?;
    if pin > 63 {
        bail!("pin {pin} out of range (max 63)");
    }
    Ok(pin)
}

/// Format a bitmask as "0-15,26-29" (or "" for no pins).
pub fn format_pin_ranges(mask: u64) -> String {
    let mut parts: Vec<String> = Vec::new();
    let mut pin = 0u8;
    while pin < 64 {
        if mask & (1u64 << pin) == 0 {
            pin += 1;
            continue;
        }
        let start = pin;
        while pin < 63 && mask & (1u64 << (pin + 1)) != 0 {
            pin += 1;
        }
        parts.push(if start == pin {
            format!("{start}")
        } else {
            format!("{start}-{pin}")
        });
        pin += 1;
    }
    parts.join(",")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_ranges_and_singles() {
        assert_eq!(parse_pin_ranges("").unwrap(), 0);
        assert_eq!(parse_pin_ranges("0").unwrap(), 1);
        assert_eq!(parse_pin_ranges("0-3").unwrap(), 0b1111);
        assert_eq!(parse_pin_ranges("1, 4-5").unwrap(), 0b110010);
        assert_eq!(parse_pin_ranges("63").unwrap(), 1u64 << 63);
    }

    #[test]
    fn rejects_bad_ranges() {
        assert!(parse_pin_ranges("5-2").is_err());
        assert!(parse_pin_ranges("64").is_err());
        assert!(parse_pin_ranges("abc").is_err());
        assert!(parse_pin_ranges("1--3").is_err());
    }

    #[test]
    fn formats_ranges() {
        assert_eq!(format_pin_ranges(0), "");
        assert_eq!(format_pin_ranges(0b1), "0");
        assert_eq!(format_pin_ranges(0b1111), "0-3");
        assert_eq!(format_pin_ranges(0b110010), "1,4-5");
        assert_eq!(format_pin_ranges(1u64 << 63), "63");
    }

    #[test]
    fn round_trips_the_pico_profile() {
        let toml_text = toml::to_string(&TomlBoardProfile::from_profile(&BoardProfile::PICO))
            .unwrap();
        let back: TomlBoardProfile = toml::from_str(&toml_text).unwrap();
        assert_eq!(back.into_profile().unwrap(), BoardProfile::PICO);
    }

    #[test]
    fn pico_profile_reads_as_expected() {
        let t = TomlBoardProfile::from_profile(&BoardProfile::PICO);
        assert_eq!(t.available, "0-28");
        assert_eq!(t.reserved, "23-25,29");
    }
}
