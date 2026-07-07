// Config-protocol framing over a transport, mirroring cli/src/session.rs.
//
// Command bytes, buffer sizes, and postcard encode/decode all come from the
// wasm module (the same probe-config crate the firmware runs), so this file
// only sequences packets.

const SET_CHUNK = 61; // 64-byte packet minus cmd + offset, mirrors the CLI

export class Session {
  /// `transport` from transport.js, `wasm` the initialized probe-config-wasm
  /// module.
  constructor(transport, wasm) {
    this.transport = transport;
    this.wasm = wasm;
    this.k = JSON.parse(wasm.constants());
  }

  statusStr(status) {
    const s = this.k.status;
    switch (status) {
      case s.ok: return "ok";
      case s.err_decode: return "payload failed to decode";
      case s.err_invalid: return "failed validation";
      case s.err_flash: return "flash write failed";
      case s.err_bad_request: return "unknown command or malformed arguments";
      default: return "unknown status";
    }
  }

  check(resp, cmd) {
    if (resp.length < 2) throw new Error(`short response (${resp.length} bytes)`);
    if (resp[0] !== cmd) {
      throw new Error(
        `response command 0x${resp[0].toString(16)} does not echo request 0x${cmd.toString(16)}`);
    }
    if (resp[1] !== this.k.status.ok) {
      throw new Error(
        `probe reported error: ${this.statusStr(resp[1])} (status 0x${resp[1].toString(16)})`);
    }
  }

  /// FirmwareInfo as a parsed object.
  async info() {
    const resp = await this.transport.transceive(new Uint8Array([this.k.cmd.info]));
    this.check(resp, this.k.cmd.info);
    return JSON.parse(this.wasm.decode_info(resp.subarray(2)));
  }

  /// Active topology as a JSON object {probes: [...], uarts: [...]}.
  async getTopology() {
    const cmd = this.k.cmd.get_topology;
    let buf = new Uint8Array(0);
    for (;;) {
      const resp = await this.transport.transceive(new Uint8Array([cmd, buf.length]));
      this.check(resp, cmd);
      if (resp.length < 3) throw new Error(`short GET_TOPOLOGY response (${resp.length} bytes)`);
      const totalLen = resp[2];
      const chunk = resp.subarray(3);
      if (buf.length >= totalLen) break;
      if (chunk.length === 0) {
        throw new Error("probe returned an empty chunk before the topology was complete");
      }
      const next = new Uint8Array(buf.length + chunk.length);
      next.set(buf); next.set(chunk, buf.length);
      buf = next;
      if (buf.length >= totalLen) { buf = buf.subarray(0, totalLen); break; }
    }
    return JSON.parse(this.wasm.decode_topology(buf));
  }

  /// Stage + commit a topology (JSON object). The firmware re-validates.
  async setTopology(topo) {
    const encoded = this.wasm.encode_topology(JSON.stringify(topo));
    const cmd = this.k.cmd.set_topology;
    // Offset 0 resets the staging buffer, so send it first even when empty.
    let offset = 0;
    for (;;) {
      const end = Math.min(offset + SET_CHUNK, encoded.length);
      const req = new Uint8Array(2 + (end - offset));
      req[0] = cmd; req[1] = offset;
      req.set(encoded.subarray(offset, end), 2);
      const resp = await this.transport.transceive(req);
      this.check(resp, cmd);
      offset = end;
      if (offset >= encoded.length) break;
    }
    const resp = await this.transport.transceive(
      new Uint8Array([this.k.cmd.commit, encoded.length]));
    this.check(resp, this.k.cmd.commit);
  }

  /// Current board profile as {available: "0-16,26-29", reserved: "16"}.
  async getProfile() {
    const cmd = this.k.cmd.get_profile;
    const resp = await this.transport.transceive(new Uint8Array([cmd]));
    this.check(resp, cmd);
    return JSON.parse(this.wasm.decode_profile(resp.subarray(2)));
  }

  /// Validate-and-store a board profile (JSON object).
  async setProfile(profile) {
    const encoded = this.wasm.encode_profile(JSON.stringify(profile));
    const cmd = this.k.cmd.set_profile;
    const req = new Uint8Array(1 + encoded.length);
    req[0] = cmd; req.set(encoded, 1);
    const resp = await this.transport.transceive(req);
    this.check(resp, cmd);
  }

  async reboot() {
    const resp = await this.transport.transceive(new Uint8Array([this.k.cmd.reboot]));
    this.check(resp, this.k.cmd.reboot);
  }

  async rebootBootsel() {
    const resp = await this.transport.transceive(new Uint8Array([this.k.cmd.bootsel]));
    this.check(resp, this.k.cmd.bootsel);
  }
}
