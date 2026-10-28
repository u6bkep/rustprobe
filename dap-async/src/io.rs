//! Bounds-checked request parsing and response assembly.
//!
//! Every accessor returns `None` past the end of the buffer, so a truncated
//! or oversized packet from the host produces an error reply instead of a
//! panic in the firmware.

/// A request payload (the bytes after the command id).
pub struct Request<'a> {
    data: &'a [u8],
}

impl<'a> Request<'a> {
    /// Wrap the payload following the command byte.
    pub fn new(data: &'a [u8]) -> Self {
        Self { data }
    }

    /// Next byte.
    pub fn u8(&mut self) -> Option<u8> {
        let (&v, rest) = self.data.split_first()?;
        self.data = rest;
        Some(v)
    }

    /// Next little-endian `u16`.
    pub fn u16(&mut self) -> Option<u16> {
        let b = self.bytes(2)?;
        Some(u16::from_le_bytes([b[0], b[1]]))
    }

    /// Next little-endian `u32`.
    pub fn u32(&mut self) -> Option<u32> {
        let b = self.bytes(4)?;
        Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// Next `n` bytes.
    pub fn bytes(&mut self, n: usize) -> Option<&'a [u8]> {
        if self.data.len() < n {
            return None;
        }
        let (head, rest) = self.data.split_at(n);
        self.data = rest;
        Some(head)
    }
}

/// A response being assembled; byte 0 is the command id.
pub struct Response<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> Response<'a> {
    /// Start a response for `command`. `buf` must hold at least one byte.
    pub fn new(buf: &'a mut [u8], command: u8) -> Self {
        buf[0] = command;
        Self { buf, len: 1 }
    }

    /// Bytes written so far.
    pub fn len(&self) -> usize {
        self.len
    }

    /// True when nothing but the command id has been written.
    pub fn is_empty(&self) -> bool {
        self.len <= 1
    }

    /// Append a byte.
    pub fn u8(&mut self, v: u8) -> Option<()> {
        *self.buf.get_mut(self.len)? = v;
        self.len += 1;
        Some(())
    }

    /// Append a little-endian `u16`.
    pub fn u16(&mut self, v: u16) -> Option<()> {
        self.slice(&v.to_le_bytes())
    }

    /// Append a little-endian `u32`.
    pub fn u32(&mut self, v: u32) -> Option<()> {
        self.slice(&v.to_le_bytes())
    }

    /// Append bytes.
    pub fn slice(&mut self, data: &[u8]) -> Option<()> {
        let dst = self.buf.get_mut(self.len..self.len + data.len())?;
        dst.copy_from_slice(data);
        self.len += data.len();
        Some(())
    }

    /// Reserve `n` zeroed bytes to be filled in later with `set_*`; returns
    /// the index of the first one.
    pub fn reserve(&mut self, n: usize) -> Option<usize> {
        let at = self.len;
        let dst = self.buf.get_mut(at..at + n)?;
        dst.fill(0);
        self.len += n;
        Some(at)
    }

    /// Reserve `n` bytes and hand them out for filling (e.g. read data).
    pub fn reserve_mut(&mut self, n: usize) -> Option<&mut [u8]> {
        let at = self.reserve(n)?;
        Some(&mut self.buf[at..at + n])
    }

    /// Drop everything written past `len` bytes.
    pub fn truncate(&mut self, len: usize) {
        self.len = self.len.min(len);
    }

    /// Overwrite a byte written earlier.
    pub fn set_u8(&mut self, at: usize, v: u8) {
        self.buf[at] = v;
    }

    /// Overwrite a little-endian `u16` written earlier.
    pub fn set_u16(&mut self, at: usize, v: u16) {
        self.buf[at..at + 2].copy_from_slice(&v.to_le_bytes());
    }
}
