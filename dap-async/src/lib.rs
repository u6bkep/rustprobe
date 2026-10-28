//! Async, SWD-only CMSIS-DAP v2 command processor.
//!
//! [`Dap::process_command`] takes one CMSIS-DAP request packet and writes
//! the reply. All probe hardware access goes through the [`Driver`] trait,
//! whose methods are `async fn`, so a transfer waiting for bits on the wire
//! yields to other tasks on the same executor (the point of issue #5).
//!
//! Derived from dap-rs (probe-rs project, rev `a867ca7`, MIT OR
//! Apache-2.0); transfer semantics follow the ARM CMSIS-DAP reference
//! implementation. See `README.md`.

#![cfg_attr(not(test), no_std)]
#![allow(async_fn_in_trait)]
#![warn(missing_docs)]

pub mod command;
pub mod dap;
pub mod framing;
pub mod io;
pub mod swd;

pub use dap::Dap;
pub use framing::{request_len, Framing};
pub use swd::{Driver, HostStatus, Pins, Transfer, TransferParams};

#[cfg(test)]
mod tests;
