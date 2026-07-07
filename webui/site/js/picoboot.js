// PICOBOOT client over WebUSB: flash a BOOTSEL-mode RP2040/RP2350.
//
// Protocol per pico-sdk boot_picoboot_headers/include/boot/picoboot.h:
// a 32-byte command packet goes to the bulk OUT endpoint, dTransferLength
// data bytes move IN or OUT (IN when the command id's top bit is set), and
// the device acks with a zero-length packet in the opposite direction.
// On error the device stalls; PICOBOOT_IF_CMD_STATUS reads the reason and
// PICOBOOT_IF_RESET un-stalls the endpoints.

import { FAMILIES } from "./uf2.js";

const MAGIC = 0x431fd10b;

// Bootrom USB PIDs (VID 0x2e8a).
export const BOOTROM_PIDS = { 0x0003: "rp2040", 0x000f: "rp2350" };

export const CHIP_FAMILY = {
  rp2040: FAMILIES.rp2040,
  rp2350: FAMILIES["rp2350-arm-s"],
};

// Command ids.
const PC_EXCLUSIVE_ACCESS = 0x01;
const PC_REBOOT = 0x02; // RP2040 only
const PC_FLASH_ERASE = 0x03;
const PC_READ = 0x84;
const PC_WRITE = 0x05;
const PC_EXIT_XIP = 0x06;
const PC_ENTER_CMD_XIP = 0x07;
const PC_REBOOT2 = 0x0a; // RP2350 only

// PC_EXCLUSIVE_ACCESS argument.
const EXCLUSIVE = 1;

// PC_REBOOT2 flags (picoboot_constants.h).
const REBOOT2_TYPE_NORMAL = 0x0;
const REBOOT2_FLAG_NO_RETURN_ON_SUCCESS = 0x100;

// Control requests on the PICOBOOT interface.
const IF_RESET = 0x41;
const IF_CMD_STATUS = 0x42;

const SECTOR = 4096;
const WRITE_CHUNK = 4096;

const STATUS_NAMES = [
  "ok", "unknown cmd", "invalid cmd length", "invalid transfer length",
  "invalid address", "bad alignment", "interleaved write", "rebooting",
  "unknown error", "invalid state", "not permitted", "invalid arg",
  "buffer too small", "precondition not met", "modified data", "invalid data",
  "not found", "unsupported modification",
];

/// Prompt for a BOOTSEL-mode device. Must be called from a user gesture.
export async function requestBootromDevice() {
  return navigator.usb.requestDevice({
    filters: Object.keys(BOOTROM_PIDS).map((pid) => ({
      vendorId: 0x2e8a, productId: Number(pid),
    })),
  });
}

export class Picoboot {
  constructor(device, log = () => {}) {
    this.device = device;
    this.log = log;
    this.chip = BOOTROM_PIDS[device.productId];
    if (!this.chip) throw new Error(`not a bootrom device (PID 0x${device.productId.toString(16)})`);
    this.token = 1;
  }

  async open() {
    const d = this.device;
    await d.open();
    if (d.configuration === null) await d.selectConfiguration(1);
    // The PICOBOOT interface is the vendor one (the other is mass storage).
    const iface = d.configuration.interfaces.find((i) => {
      const alt = i.alternates[0];
      return alt && alt.interfaceClass === 0xff &&
        alt.endpoints.filter((e) => e.type === "bulk").length === 2;
    });
    if (!iface) throw new Error("no PICOBOOT interface found (is the MSC drive busy?)");
    await d.claimInterface(iface.interfaceNumber);
    this.ifaceNum = iface.interfaceNumber;
    const eps = iface.alternates[0].endpoints;
    this.epOut = eps.find((e) => e.direction === "out").endpointNumber;
    this.epIn = eps.find((e) => e.direction === "in").endpointNumber;
    await this.ifReset();
  }

  async close() {
    try { await this.device.close(); } catch { /* gone */ }
  }

  /// Un-stall endpoints and reset the PICOBOOT interface state machine.
  async ifReset() {
    await this.device.controlTransferOut({
      requestType: "vendor", recipient: "interface",
      request: IF_RESET, value: 0, index: this.ifaceNum,
    });
  }

  /// Read the 16-byte status of the last command.
  async cmdStatus() {
    const r = await this.device.controlTransferIn({
      requestType: "vendor", recipient: "interface",
      request: IF_CMD_STATUS, value: 0, index: this.ifaceNum,
    }, 16);
    const code = r.data.getUint32(4, true);
    return { code, name: STATUS_NAMES[code] ?? `status ${code}`, inProgress: r.data.getUint8(9) };
  }

  async fail(what) {
    let detail = "";
    try {
      const s = await this.cmdStatus();
      detail = ` (bootrom: ${s.name})`;
    } catch { /* status unavailable */ }
    try {
      await this.device.clearHalt("in", this.epIn);
      await this.device.clearHalt("out", this.epOut);
      await this.ifReset();
    } catch { /* leave it stalled */ }
    throw new Error(`${what}${detail}`);
  }

  /// Issue one PICOBOOT command. `args` is ≤16 bytes; `dataOut` sends a data
  /// phase; a command id with the top bit set reads `transferLength` back.
  async cmd(cmdId, args, transferLength, dataOut = null) {
    const pkt = new ArrayBuffer(32);
    const v = new DataView(pkt);
    v.setUint32(0, MAGIC, true);
    v.setUint32(4, this.token++, true);
    v.setUint8(8, cmdId);
    v.setUint8(9, args.length);
    v.setUint32(12, transferLength, true);
    new Uint8Array(pkt, 16, args.length).set(args);

    const out = await this.device.transferOut(this.epOut, pkt);
    if (out.status !== "ok") return this.fail(`command 0x${cmdId.toString(16)} rejected`);

    let dataIn = null;
    if (cmdId & 0x80) {
      const r = await this.device.transferIn(this.epIn, transferLength);
      if (r.status !== "ok") return this.fail(`read for 0x${cmdId.toString(16)} failed`);
      dataIn = new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength);
      // Ack with a zero-length OUT packet.
      const ack = await this.device.transferOut(this.epOut, new ArrayBuffer(0));
      if (ack.status !== "ok") return this.fail("ack failed");
    } else {
      if (dataOut) {
        const w = await this.device.transferOut(this.epOut, dataOut);
        if (w.status !== "ok") return this.fail(`data for 0x${cmdId.toString(16)} rejected`);
      }
      // Device acks with a zero-length IN packet.
      const ack = await this.device.transferIn(this.epIn, 64);
      if (ack.status !== "ok") return this.fail(`command 0x${cmdId.toString(16)} failed`);
    }
    return dataIn;
  }

  args32(...words) {
    const a = new Uint8Array(words.length * 4);
    const v = new DataView(a.buffer);
    words.forEach((w, i) => v.setUint32(i * 4, w, true));
    return a;
  }

  async exclusiveAccess() {
    await this.cmd(PC_EXCLUSIVE_ACCESS, new Uint8Array([EXCLUSIVE]), 0);
  }

  async exitXip() { await this.cmd(PC_EXIT_XIP, new Uint8Array(0), 0); }
  async enterCmdXip() { await this.cmd(PC_ENTER_CMD_XIP, new Uint8Array(0), 0); }

  async flashErase(addr, size) {
    await this.cmd(PC_FLASH_ERASE, this.args32(addr, size), 0);
  }

  async write(addr, data) {
    await this.cmd(PC_WRITE, this.args32(addr, data.length), data.length, data);
  }

  async read(addr, size) {
    return this.cmd(PC_READ, this.args32(addr, size), size);
  }

  /// Reboot into the freshly-flashed firmware.
  async reboot() {
    const delayMs = 500;
    if (this.chip === "rp2040") {
      await this.cmd(PC_REBOOT, this.args32(0, 0, delayMs), 0);
    } else {
      await this.cmd(PC_REBOOT2, this.args32(
        REBOOT2_TYPE_NORMAL | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, delayMs, 0, 0), 0);
    }
  }

  /// Erase + write + verify `ranges` ({addr, data}, from uf2.coalesce), then
  /// reboot. `progress(fraction, label)` is called throughout.
  async flashRanges(ranges, progress = () => {}) {
    // Total work: one unit per sector erased, per chunk written, per chunk
    // verified.
    const sectors = new Set();
    for (const r of ranges) {
      for (let a = r.addr & ~(SECTOR - 1); a < r.addr + r.data.length; a += SECTOR) {
        sectors.add(a);
      }
    }
    const chunkCount = ranges.reduce(
      (n, r) => n + Math.ceil(r.data.length / WRITE_CHUNK), 0);
    const total = sectors.size + 2 * chunkCount;
    let done = 0;
    const tick = (label) => progress(++done / total, label);

    await this.exclusiveAccess();
    await this.exitXip();

    for (const a of [...sectors].sort((x, y) => x - y)) {
      await this.flashErase(a, SECTOR);
      tick(`erase 0x${a.toString(16)}`);
    }
    for (const r of ranges) {
      for (let off = 0; off < r.data.length; off += WRITE_CHUNK) {
        const chunk = r.data.subarray(off, Math.min(off + WRITE_CHUNK, r.data.length));
        await this.write(r.addr + off, chunk);
        tick(`write 0x${(r.addr + off).toString(16)}`);
      }
    }
    await this.enterCmdXip();
    for (const r of ranges) {
      for (let off = 0; off < r.data.length; off += WRITE_CHUNK) {
        const chunk = r.data.subarray(off, Math.min(off + WRITE_CHUNK, r.data.length));
        const back = await this.read(r.addr + off, chunk.length);
        for (let i = 0; i < chunk.length; i++) {
          if (back[i] !== chunk[i]) {
            throw new Error(
              `verify failed at 0x${(r.addr + off + i).toString(16)}: ` +
              `wrote 0x${chunk[i].toString(16)}, read 0x${back[i].toString(16)}`);
          }
        }
        tick(`verify 0x${(r.addr + off).toString(16)}`);
      }
    }
    progress(1, "rebooting");
    await this.reboot().catch(() => { /* device may drop off mid-ack */ });
  }
}
