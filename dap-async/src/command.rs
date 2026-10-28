//! CMSIS-DAP command ids and response status bytes.

/// Command ids (first byte of every request and response).
#[allow(missing_docs)]
pub mod cmd {
    pub const INFO: u8 = 0x00;
    pub const HOST_STATUS: u8 = 0x01;
    pub const CONNECT: u8 = 0x02;
    pub const DISCONNECT: u8 = 0x03;
    pub const TRANSFER_CONFIGURE: u8 = 0x04;
    pub const TRANSFER: u8 = 0x05;
    pub const TRANSFER_BLOCK: u8 = 0x06;
    pub const TRANSFER_ABORT: u8 = 0x07;
    pub const WRITE_ABORT: u8 = 0x08;
    pub const DELAY: u8 = 0x09;
    pub const RESET_TARGET: u8 = 0x0A;
    pub const SWJ_PINS: u8 = 0x10;
    pub const SWJ_CLOCK: u8 = 0x11;
    pub const SWJ_SEQUENCE: u8 = 0x12;
    pub const SWD_CONFIGURE: u8 = 0x13;
    pub const JTAG_SEQUENCE: u8 = 0x14;
    pub const JTAG_CONFIGURE: u8 = 0x15;
    pub const JTAG_IDCODE: u8 = 0x16;
    pub const SWO_TRANSPORT: u8 = 0x17;
    pub const SWO_MODE: u8 = 0x18;
    pub const SWO_BAUDRATE: u8 = 0x19;
    pub const SWO_CONTROL: u8 = 0x1A;
    pub const SWO_STATUS: u8 = 0x1B;
    pub const SWO_DATA: u8 = 0x1C;
    pub const SWD_SEQUENCE: u8 = 0x1D;
    pub const SWO_EXTENDED_STATUS: u8 = 0x1E;
    pub const QUEUE_COMMANDS: u8 = 0x7E;
    pub const EXECUTE_COMMANDS: u8 = 0x7F;
    /// Reply id for an unknown command.
    pub const INVALID: u8 = 0xFF;
}

/// `DAP_Info` ids.
#[allow(missing_docs)]
pub mod info {
    pub const VENDOR: u8 = 0x01;
    pub const PRODUCT: u8 = 0x02;
    pub const SERIAL: u8 = 0x03;
    pub const FW_VERSION: u8 = 0x04;
    pub const TARGET_VENDOR: u8 = 0x05;
    pub const TARGET_NAME: u8 = 0x06;
    pub const CAPABILITIES: u8 = 0xF0;
    pub const TEST_DOMAIN_TIMER: u8 = 0xF1;
    pub const SWO_BUFFER_SIZE: u8 = 0xFD;
    pub const MAX_PACKET_COUNT: u8 = 0xFE;
    pub const MAX_PACKET_SIZE: u8 = 0xFF;
}

/// Status byte: success.
pub const DAP_OK: u8 = 0x00;
/// Status byte: failure.
pub const DAP_ERROR: u8 = 0xFF;

/// `DAP_Info` capabilities byte: SWD only. JTAG, SWO, atomic commands and
/// the test-domain timer are not supported.
pub const CAPABILITIES: u8 = 0x01;
