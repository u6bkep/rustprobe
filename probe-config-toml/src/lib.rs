//! Human-readable TOML mirrors of the `probe-config` wire types, shared by
//! every host frontend (CLI, web UI) so board and topology files parse the
//! same everywhere.

pub mod board_toml;
pub mod topology_toml;
