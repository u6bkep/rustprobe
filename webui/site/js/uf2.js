// UF2 parsing: split a .uf2 into 512-byte blocks, filter by family, and
// coalesce payloads into contiguous address ranges for PICOBOOT writes.
//
// Format: https://github.com/microsoft/uf2 — each block is 512 bytes with
// magics at 0/4/508, flags/targetAddr/payloadSize/blockNo/numBlocks, and
// familyID at offset 28 when flags bit 0x2000 is set.

const MAGIC0 = 0x0a324655;
const MAGIC1 = 0x9e5d5157;
const MAGIC_END = 0x0ab16f30;
const FLAG_NOT_MAIN_FLASH = 0x0000_0001;
const FLAG_FAMILY_ID_PRESENT = 0x0000_2000;

// Family IDs from pico-sdk picoboot_constants.h / uf2 family list.
export const FAMILIES = {
  rp2040: 0xe48bff56,
  absolute: 0xe48bff57,
  data: 0xe48bff58,
  "rp2350-arm-s": 0xe48bff59,
  "rp2350-riscv": 0xe48bff5a,
  "rp2350-arm-ns": 0xe48bff5b,
};

export function familyName(id) {
  for (const [name, fid] of Object.entries(FAMILIES)) if (fid === id) return name;
  return `0x${id.toString(16)}`;
}

/// Parse an ArrayBuffer into blocks: {targetAddr, payload, familyID}.
/// Throws on malformed blocks; skips not-main-flash blocks.
export function parseUf2(buffer) {
  if (buffer.byteLength % 512 !== 0) {
    throw new Error(`UF2 length ${buffer.byteLength} is not a multiple of 512`);
  }
  const blocks = [];
  for (let off = 0; off < buffer.byteLength; off += 512) {
    const v = new DataView(buffer, off, 512);
    if (v.getUint32(0, true) !== MAGIC0 || v.getUint32(4, true) !== MAGIC1 ||
        v.getUint32(508, true) !== MAGIC_END) {
      throw new Error(`bad UF2 magic in block at offset ${off}`);
    }
    const flags = v.getUint32(8, true);
    if (flags & FLAG_NOT_MAIN_FLASH) continue;
    const payloadSize = v.getUint32(16, true);
    if (payloadSize > 476) throw new Error(`block at ${off}: payload size ${payloadSize}`);
    blocks.push({
      targetAddr: v.getUint32(12, true),
      payload: new Uint8Array(buffer, off + 32, payloadSize),
      familyID: flags & FLAG_FAMILY_ID_PRESENT ? v.getUint32(28, true) : null,
    });
  }
  return blocks;
}

/// Distinct family IDs present in `blocks`.
export function familiesIn(blocks) {
  return [...new Set(blocks.map((b) => b.familyID).filter((f) => f !== null))];
}

/// Blocks matching `familyID` (blocks with no family pass any filter, per the
/// UF2 convention for pre-family files).
export function filterFamily(blocks, familyID) {
  return blocks.filter((b) => b.familyID === null || b.familyID === familyID);
}

/// Coalesce blocks into sorted contiguous ranges: {addr, data: Uint8Array}.
export function coalesce(blocks) {
  const sorted = [...blocks].sort((a, b) => a.targetAddr - b.targetAddr);
  const ranges = [];
  for (const b of sorted) {
    const last = ranges[ranges.length - 1];
    if (last && last.addr + last.parts.reduce((n, p) => n + p.length, 0) === b.targetAddr) {
      last.parts.push(b.payload);
    } else {
      ranges.push({ addr: b.targetAddr, parts: [b.payload] });
    }
  }
  return ranges.map((r) => {
    const len = r.parts.reduce((n, p) => n + p.length, 0);
    const data = new Uint8Array(len);
    let off = 0;
    for (const p of r.parts) { data.set(p, off); off += p.length; }
    return { addr: r.addr, data };
  });
}
