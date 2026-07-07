//! wasm-bindgen bindings for the web UI.
//!
//! The JS side stays thin: it moves 64-byte packets over WebUSB and renders
//! the UI; everything protocol-shaped — postcard encode/decode, validation,
//! TOML import/export — happens here, in the same code the firmware runs.
//!
//! All functions speak JSON strings (structures mirror the TOML forms:
//! topologies as `{probes: [{swclk, swdio, reset?}], uarts: [{tx, rx,
//! baud}]}`, board profiles as pin-range strings `{available: "0-16,26-29",
//! reserved: "16"}`). Errors become thrown JS exceptions.
//!
//! The core logic lives in `core` with plain `Result<_, String>` signatures
//! so it unit-tests on the host; the `#[wasm_bindgen]` layer only converts
//! errors.

mod core;

use wasm_bindgen::prelude::*;

fn js<T>(r: Result<T, String>) -> Result<T, JsError> {
    r.map_err(|e| JsError::new(&e))
}

/// Protocol constants (command bytes, buffer sizes, protocol version) as
/// JSON, so the JS session layer shares one source of truth with the CLI
/// and firmware.
#[wasm_bindgen]
pub fn constants() -> String {
    core::constants()
}

/// Decode a `CMD_INFO` response payload into `FirmwareInfo` JSON.
#[wasm_bindgen]
pub fn decode_info(bytes: &[u8]) -> Result<String, JsError> {
    js(core::decode_info(bytes))
}

/// Decode postcard `Topology` bytes into topology JSON.
#[wasm_bindgen]
pub fn decode_topology(bytes: &[u8]) -> Result<String, JsError> {
    js(core::decode_topology(bytes))
}

/// Encode topology JSON into postcard bytes (the `CMD_SET_TOPOLOGY` payload).
#[wasm_bindgen]
pub fn encode_topology(json: &str) -> Result<Vec<u8>, JsError> {
    js(core::encode_topology(json))
}

/// Decode postcard `BoardProfile` bytes into profile JSON.
#[wasm_bindgen]
pub fn decode_profile(bytes: &[u8]) -> Result<String, JsError> {
    js(core::decode_profile(bytes))
}

/// Encode profile JSON into postcard bytes (the `CMD_SET_PROFILE` payload).
#[wasm_bindgen]
pub fn encode_profile(json: &str) -> Result<Vec<u8>, JsError> {
    js(core::encode_profile(json))
}

/// Validate topology JSON against `ChipLimits` JSON and profile JSON.
/// Returns `"null"` when valid, else `{"code": "...", "pin": N|null,
/// "message": "human readable"}`.
#[wasm_bindgen]
pub fn validate_topology(
    topology: &str,
    limits: &str,
    profile: &str,
) -> Result<String, JsError> {
    js(core::validate_topology(topology, limits, profile))
}

/// Validate profile JSON against `ChipLimits` JSON. Same result shape as
/// [`validate_topology`].
#[wasm_bindgen]
pub fn validate_profile(profile: &str, limits: &str) -> Result<String, JsError> {
    js(core::validate_profile(profile, limits))
}

/// Render topology JSON as the TOML the CLI reads/writes.
#[wasm_bindgen]
pub fn topology_to_toml(json: &str) -> Result<String, JsError> {
    js(core::topology_to_toml(json))
}

/// Parse topology TOML (a `configs/*.toml` file) into topology JSON.
#[wasm_bindgen]
pub fn topology_from_toml(text: &str) -> Result<String, JsError> {
    js(core::topology_from_toml(text))
}

/// Render profile JSON as the TOML the CLI reads/writes.
#[wasm_bindgen]
pub fn profile_to_toml(json: &str) -> Result<String, JsError> {
    js(core::profile_to_toml(json))
}

/// Parse board-profile TOML (a `configs/boards/*.toml` file) into profile JSON.
#[wasm_bindgen]
pub fn profile_from_toml(text: &str) -> Result<String, JsError> {
    js(core::profile_from_toml(text))
}

/// Expand a pin-range string ("0-15,26-29") into a JSON array of pin numbers.
#[wasm_bindgen]
pub fn pins_in_ranges(text: &str) -> Result<String, JsError> {
    js(core::pins_in_ranges(text))
}

/// Collapse a JSON array of pin numbers into a pin-range string.
#[wasm_bindgen]
pub fn pins_to_ranges(json: &str) -> Result<String, JsError> {
    js(core::pins_to_ranges(json))
}

/// Hardware UART instance `pin` can drive as TX, or -1 if none.
#[wasm_bindgen]
pub fn uart_tx_instance(pin: u8) -> i32 {
    probe_config::uart_tx_instance(pin).map_or(-1, i32::from)
}

/// Hardware UART instance `pin` can receive as RX, or -1 if none.
#[wasm_bindgen]
pub fn uart_rx_instance(pin: u8) -> i32 {
    probe_config::uart_rx_instance(pin).map_or(-1, i32::from)
}
