//! Host-testable core of the wasm bindings: JSON strings in/out, `String`
//! errors. See the crate docs for the JSON shapes.

use probe_config::protocol::{self, FirmwareInfo};
use probe_config::{BoardProfile, ChipLimits, Topology, ValidationError};
use probe_config_toml::board_toml::{format_pin_ranges, parse_pin_ranges, TomlBoardProfile};
use probe_config_toml::topology_toml::TomlTopology;

pub fn constants() -> String {
    serde_json::json!({
        "protocol_version": protocol::PROTOCOL_VERSION,
        "topology_buf_len": protocol::TOPOLOGY_BUF_LEN,
        "profile_buf_len": protocol::PROFILE_BUF_LEN,
        "cmd": {
            "info": protocol::CMD_INFO,
            "get_topology": protocol::CMD_GET_TOPOLOGY,
            "set_topology": protocol::CMD_SET_TOPOLOGY,
            "commit": protocol::CMD_COMMIT,
            "reboot": protocol::CMD_REBOOT,
            "bootsel": protocol::CMD_BOOTSEL,
            "get_profile": protocol::CMD_GET_PROFILE,
            "set_profile": protocol::CMD_SET_PROFILE,
        },
        "status": {
            "ok": protocol::STATUS_OK,
            "err_decode": protocol::STATUS_ERR_DECODE,
            "err_invalid": protocol::STATUS_ERR_INVALID,
            "err_flash": protocol::STATUS_ERR_FLASH,
            "err_bad_request": protocol::STATUS_ERR_BAD_REQUEST,
        },
    })
    .to_string()
}

pub fn decode_info(bytes: &[u8]) -> Result<String, String> {
    let info: FirmwareInfo =
        postcard::from_bytes(bytes).map_err(|e| format!("decode FirmwareInfo: {e}"))?;
    serde_json::to_string(&info).map_err(|e| e.to_string())
}

fn topology_from_json(json: &str) -> Result<Topology, String> {
    let toml_topo: TomlTopology =
        serde_json::from_str(json).map_err(|e| format!("parse topology JSON: {e}"))?;
    toml_topo.into_topology().map_err(|e| e.to_string())
}

fn topology_to_json(topo: &Topology) -> Result<String, String> {
    serde_json::to_string(&TomlTopology::from_topology(topo)).map_err(|e| e.to_string())
}

pub fn decode_topology(bytes: &[u8]) -> Result<String, String> {
    let topo: Topology =
        postcard::from_bytes(bytes).map_err(|e| format!("decode Topology: {e}"))?;
    topology_to_json(&topo)
}

pub fn encode_topology(json: &str) -> Result<Vec<u8>, String> {
    let topo = topology_from_json(json)?;
    let bytes = postcard::to_stdvec(&topo).map_err(|e| format!("encode Topology: {e}"))?;
    if bytes.len() > protocol::TOPOLOGY_BUF_LEN {
        return Err(format!(
            "encoded topology is {} bytes, firmware staging buffer holds {}",
            bytes.len(),
            protocol::TOPOLOGY_BUF_LEN
        ));
    }
    Ok(bytes)
}

fn profile_from_json(json: &str) -> Result<BoardProfile, String> {
    let toml_profile: TomlBoardProfile =
        serde_json::from_str(json).map_err(|e| format!("parse profile JSON: {e}"))?;
    toml_profile.into_profile().map_err(|e| e.to_string())
}

fn profile_to_json(profile: &BoardProfile) -> Result<String, String> {
    serde_json::to_string(&TomlBoardProfile::from_profile(profile)).map_err(|e| e.to_string())
}

pub fn decode_profile(bytes: &[u8]) -> Result<String, String> {
    let profile: BoardProfile =
        postcard::from_bytes(bytes).map_err(|e| format!("decode BoardProfile: {e}"))?;
    profile_to_json(&profile)
}

pub fn encode_profile(json: &str) -> Result<Vec<u8>, String> {
    let profile = profile_from_json(json)?;
    postcard::to_stdvec(&profile).map_err(|e| format!("encode BoardProfile: {e}"))
}

fn limits_from_json(json: &str) -> Result<ChipLimits, String> {
    serde_json::from_str(json).map_err(|e| format!("parse ChipLimits JSON: {e}"))
}

/// `"null"` for Ok, else `{"code", "pin", "message"}`.
fn validation_json(result: Result<(), ValidationError>) -> String {
    let Err(e) = result else { return "null".into() };
    let (code, pin): (&str, Option<u8>) = match e {
        ValidationError::TooManyProbes => ("TooManyProbes", None),
        ValidationError::TooManyUarts => ("TooManyUarts", None),
        ValidationError::EndpointBudget => ("EndpointBudget", None),
        ValidationError::PinUnavailable(p) => ("PinUnavailable", Some(p)),
        ValidationError::PinConflict(p) => ("PinConflict", Some(p)),
        ValidationError::PioWindow => ("PioWindow", None),
        ValidationError::UartPinMux(p) => ("UartPinMux", Some(p)),
        ValidationError::UartInstanceConflict(i) => ("UartInstanceConflict", Some(i)),
        ValidationError::ProfilePinRange => ("ProfilePinRange", None),
        ValidationError::ProfileEmpty => ("ProfileEmpty", None),
    };
    let message = match e {
        ValidationError::TooManyProbes => {
            "Not enough PIO state machines: each probe needs one, plus one for autobaud when any UART is configured".into()
        }
        ValidationError::TooManyUarts => "At most 2 UART bridges (hardware UARTs)".into(),
        ValidationError::EndpointBudget => {
            "USB endpoint budget exceeded (IN: probes + 2×UARTs, OUT: probes + UARTs, each ≤ 15)".into()
        }
        ValidationError::PinUnavailable(p) => {
            format!("GPIO{p} is not assignable on this board (missing, reserved, or beyond the chip's pins)")
        }
        ValidationError::PinConflict(p) => format!("GPIO{p} is assigned twice"),
        ValidationError::PioWindow => {
            "Probes cannot be grouped into PIO blocks whose pins fit one GPIOBASE window".into()
        }
        ValidationError::UartPinMux(p) => {
            format!("TX GPIO{p} and the RX pin do not select the same hardware UART (UART0: TX 0/12/16/28, RX 1/13/17/29; UART1: TX 4/8/20/24, RX 5/9/21/25)")
        }
        ValidationError::UartInstanceConflict(i) => {
            format!("Two UART bridges both resolve to hardware UART{i}")
        }
        ValidationError::ProfilePinRange => {
            "Profile marks pins available that do not exist on this chip".into()
        }
        ValidationError::ProfileEmpty => {
            "Profile leaves no pin assignable (available minus reserved is empty)".into()
        }
    };
    serde_json::json!({ "code": code, "pin": pin, "message": message }).to_string()
}

pub fn validate_topology(topology: &str, limits: &str, profile: &str) -> Result<String, String> {
    let topo = topology_from_json(topology)?;
    let limits = limits_from_json(limits)?;
    let profile = profile_from_json(profile)?;
    Ok(validation_json(topo.validate(&limits, &profile)))
}

pub fn validate_profile(profile: &str, limits: &str) -> Result<String, String> {
    let profile = profile_from_json(profile)?;
    let limits = limits_from_json(limits)?;
    Ok(validation_json(profile.validate(&limits)))
}

pub fn topology_to_toml(json: &str) -> Result<String, String> {
    let topo = topology_from_json(json)?;
    toml::to_string(&TomlTopology::from_topology(&topo)).map_err(|e| e.to_string())
}

pub fn topology_from_toml(text: &str) -> Result<String, String> {
    let parsed: TomlTopology =
        toml::from_str(text).map_err(|e| format!("parse topology TOML: {e}"))?;
    topology_to_json(&parsed.into_topology().map_err(|e| e.to_string())?)
}

pub fn profile_to_toml(json: &str) -> Result<String, String> {
    let profile = profile_from_json(json)?;
    toml::to_string(&TomlBoardProfile::from_profile(&profile)).map_err(|e| e.to_string())
}

pub fn profile_from_toml(text: &str) -> Result<String, String> {
    let parsed: TomlBoardProfile =
        toml::from_str(text).map_err(|e| format!("parse board profile TOML: {e}"))?;
    profile_to_json(&parsed.into_profile().map_err(|e| e.to_string())?)
}

pub fn pins_in_ranges(text: &str) -> Result<String, String> {
    let mask = parse_pin_ranges(text).map_err(|e| e.to_string())?;
    let pins: Vec<u8> = (0..64).filter(|p| mask & (1u64 << p) != 0).collect();
    serde_json::to_string(&pins).map_err(|e| e.to_string())
}

pub fn pins_to_ranges(json: &str) -> Result<String, String> {
    let pins: Vec<u8> = serde_json::from_str(json).map_err(|e| format!("parse pin array: {e}"))?;
    let mut mask = 0u64;
    for p in pins {
        if p > 63 {
            return Err(format!("pin {p} out of range (max 63)"));
        }
        mask |= 1u64 << p;
    }
    Ok(format_pin_ranges(mask))
}

#[cfg(test)]
mod tests {
    use super::*;
    use probe_config::RP2350A;

    const TOPO_JSON: &str = r#"{"probes":[{"swclk":2,"swdio":3,"reset":1}],"uarts":[{"tx":4,"rx":5,"baud":115200}]}"#;
    const ZERO_PROFILE: &str = r#"{"available":"0-16,26-29","reserved":"16"}"#;

    fn limits_json() -> String {
        serde_json::to_string(&RP2350A).unwrap()
    }

    #[test]
    fn topology_round_trips_json_postcard_toml() {
        let bytes = encode_topology(TOPO_JSON).unwrap();
        let back = decode_topology(&bytes).unwrap();
        assert_eq!(encode_topology(&back).unwrap(), bytes);

        let toml_text = topology_to_toml(TOPO_JSON).unwrap();
        let from_toml = topology_from_toml(&toml_text).unwrap();
        assert_eq!(encode_topology(&from_toml).unwrap(), bytes);
    }

    #[test]
    fn profile_round_trips() {
        let bytes = encode_profile(ZERO_PROFILE).unwrap();
        let back = decode_profile(&bytes).unwrap();
        assert_eq!(encode_profile(&back).unwrap(), bytes);
        assert_eq!(profile_from_toml(&profile_to_toml(ZERO_PROFILE).unwrap()).unwrap(), back);
    }

    #[test]
    fn validation_reports_friendly_errors() {
        assert_eq!(
            validate_topology(TOPO_JSON, &limits_json(), ZERO_PROFILE).unwrap(),
            "null"
        );
        // GP20 is not brought out on the RP2350-Zero.
        let bad = r#"{"probes":[{"swclk":20,"swdio":3}],"uarts":[]}"#;
        let err: serde_json::Value =
            serde_json::from_str(&validate_topology(bad, &limits_json(), ZERO_PROFILE).unwrap())
                .unwrap();
        assert_eq!(err["code"], "PinUnavailable");
        assert_eq!(err["pin"], 20);
    }

    #[test]
    fn info_decodes() {
        use probe_config::protocol::{Chip, FirmwareInfo, PROTOCOL_VERSION};
        let info = FirmwareInfo {
            protocol_version: PROTOCOL_VERSION,
            firmware_version: (0, 1, 0),
            chip: Chip::Rp2350,
            limits: RP2350A,
            active_probes: 9,
            active_uarts: 1,
            config_fault: false,
        };
        let bytes = postcard::to_stdvec(&info).unwrap();
        let json: serde_json::Value =
            serde_json::from_str(&decode_info(&bytes).unwrap()).unwrap();
        assert_eq!(json["chip"], "Rp2350");
        assert_eq!(json["active_probes"], 9);
        assert_eq!(json["limits"]["pio_blocks"], 3);
    }

    #[test]
    fn pin_range_helpers() {
        assert_eq!(pins_in_ranges("0-2,29").unwrap(), "[0,1,2,29]");
        assert_eq!(pins_to_ranges("[0,1,2,29]").unwrap(), "0-2,29");
    }
}
