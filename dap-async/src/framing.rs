//! Request framing for transports whose packets are smaller than the DAP
//! packet.
//!
//! On full-speed USB bulk the endpoint carries 64-byte packets while
//! `DAP_Info` advertises a larger logical packet, so a request may span
//! several USB packets. A short packet ends a request, but a request that
//! is an exact multiple of the USB packet size has no terminator (hosts do
//! not send a zero-length packet), so the transport needs to know when it
//! has the whole command. Every standard CMSIS-DAP request is
//! self-describing; [`request_len`] computes its length from the bytes
//! received so far.

use crate::command::cmd;

/// What [`request_len`] knows about the request so far.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Framing {
    /// The request is `len` bytes long, all of which have been received.
    Complete(usize),
    /// More bytes are needed before the length is known or reached.
    NeedMore,
    /// Not a standard command: the transport must end the request some
    /// other way (short packet, or a single-packet convention).
    Unknown,
}

/// Length of the CMSIS-DAP request that starts in `buf`.
pub fn request_len(buf: &[u8]) -> Framing {
    use Framing::*;
    let Some(&command) = buf.first() else {
        return NeedMore;
    };
    let fixed = |n: usize| if buf.len() >= n { Complete(n) } else { NeedMore };
    match command {
        cmd::DISCONNECT | cmd::TRANSFER_ABORT | cmd::RESET_TARGET | cmd::SWO_STATUS => fixed(1),
        cmd::INFO | cmd::CONNECT | cmd::SWD_CONFIGURE | cmd::JTAG_IDCODE | cmd::SWO_TRANSPORT | cmd::SWO_MODE
        | cmd::SWO_CONTROL | cmd::SWO_EXTENDED_STATUS => fixed(2),
        cmd::HOST_STATUS | cmd::DELAY | cmd::SWO_DATA => fixed(3),
        cmd::SWJ_CLOCK | cmd::SWO_BAUDRATE => fixed(5),
        cmd::TRANSFER_CONFIGURE | cmd::WRITE_ABORT => fixed(6),
        cmd::SWJ_PINS => fixed(7),
        cmd::SWJ_SEQUENCE => match buf.get(1) {
            None => NeedMore,
            Some(&n) => {
                let bits = if n == 0 { 256 } else { n as usize };
                fixed(2 + bits.div_ceil(8))
            }
        },
        cmd::JTAG_CONFIGURE => match buf.get(1) {
            None => NeedMore,
            Some(&n) => fixed(2 + n as usize),
        },
        cmd::TRANSFER_BLOCK => {
            // index, count u16, request; writes carry count words.
            if buf.len() < 5 {
                return NeedMore;
            }
            let count = u16::from_le_bytes([buf[2], buf[3]]) as usize;
            let write = buf[4] & 0x02 == 0;
            fixed(5 + if write { count * 4 } else { 0 })
        }
        cmd::TRANSFER => {
            // index, count, then per transfer: request byte plus a word for
            // writes and for reads with value match.
            if buf.len() < 3 {
                return NeedMore;
            }
            let count = buf[2] as usize;
            let mut at = 3;
            for _ in 0..count {
                let Some(&rv) = buf.get(at) else {
                    return NeedMore;
                };
                let read = rv & 0x02 != 0;
                let match_value = rv & 0x10 != 0;
                at += if read && !match_value { 1 } else { 5 };
            }
            fixed(at)
        }
        cmd::SWD_SEQUENCE | cmd::JTAG_SEQUENCE => {
            // count, then per sequence: info byte plus the data bytes for
            // output sequences (SWD: bit 7 clear; JTAG: always carries TDI).
            let Some(&count) = buf.get(1) else {
                return NeedMore;
            };
            let mut at = 2;
            for _ in 0..count {
                let Some(&info) = buf.get(at) else {
                    return NeedMore;
                };
                let bits = match info & 0x3F {
                    0 => 64,
                    n => n as usize,
                };
                let carries_data = command == cmd::JTAG_SEQUENCE || info & 0x80 == 0;
                at += 1 + if carries_data { bits.div_ceil(8) } else { 0 };
            }
            fixed(at)
        }
        _ => Unknown,
    }
}

#[cfg(test)]
mod tests {
    use super::Framing::*;
    use super::*;

    #[test]
    fn fixed_lengths() {
        assert_eq!(request_len(&[]), NeedMore);
        assert_eq!(request_len(&[0x03]), Complete(1));
        assert_eq!(request_len(&[0x00]), NeedMore);
        assert_eq!(request_len(&[0x00, 0xFF]), Complete(2));
        assert_eq!(request_len(&[0x04, 0, 0, 0, 0]), NeedMore);
        assert_eq!(request_len(&[0x04, 0, 0, 0, 0, 0]), Complete(6));
        assert_eq!(request_len(&[0x10, 0, 0, 0, 0, 0, 0, 9]), Complete(7));
        assert_eq!(request_len(&[0x80, 1, 2]), Unknown);
        assert_eq!(request_len(&[0x7F]), Unknown);
    }

    #[test]
    fn swj_sequence() {
        assert_eq!(request_len(&[0x12]), NeedMore);
        assert_eq!(request_len(&[0x12, 51]), NeedMore);
        let mut r = vec![0x12, 51];
        r.extend([0; 7]);
        assert_eq!(request_len(&r), Complete(9));
        let mut r = vec![0x12, 0];
        r.extend([0; 32]);
        assert_eq!(request_len(&r), Complete(34));
    }

    #[test]
    fn transfer() {
        // read, read-with-match, write: 1 + 5 + 5 bytes after the header.
        let r = [0x05, 0, 3, 0x02, 0x12, 1, 2, 3, 4, 0x00, 5, 6, 7, 8];
        assert_eq!(request_len(&r), Complete(14));
        assert_eq!(request_len(&r[..10]), NeedMore);
        assert_eq!(request_len(&r[..2]), NeedMore);
        assert_eq!(request_len(&[0x05, 0, 0]), Complete(3));
    }

    #[test]
    fn transfer_block() {
        assert_eq!(request_len(&[0x06, 0, 3, 0, 0x02]), Complete(5));
        assert_eq!(request_len(&[0x06, 0, 3, 0]), NeedMore);
        let mut w = vec![0x06, 0, 2, 0, 0x00];
        assert_eq!(request_len(&w), NeedMore);
        w.extend([0; 8]);
        assert_eq!(request_len(&w), Complete(13));
    }

    #[test]
    fn swd_and_jtag_sequences() {
        // SWD: 8-bit output, 5-bit input, 33-bit output.
        let r = [0x1D, 3, 8, 0xBD, 0x80 | 5, 33, 1, 2, 3, 4, 5];
        assert_eq!(request_len(&r), Complete(11));
        assert_eq!(request_len(&r[..6]), NeedMore);
        // JTAG: every sequence carries TDI bytes; 0 means 64 bits.
        let mut j = vec![0x14, 2, 0x80 | 3, 0xFF, 0];
        assert_eq!(request_len(&j), NeedMore);
        j.extend([0; 8]);
        assert_eq!(request_len(&j), Complete(13));
    }
}
