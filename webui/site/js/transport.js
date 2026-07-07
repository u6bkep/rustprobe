// WebUSB transport for the rustprobe config protocol.
//
// Mirrors cli/src/transport.rs: config commands ride on the vendor-bulk
// CMSIS-DAP v2 interfaces (class FF/00/00, interface string "CMSIS-DAP");
// one ≤64-byte packet OUT, one ≤64-byte packet IN. The pico-sdk reset
// interface is FF/00/01, so matching protocol 0 excludes it.

export const VID = 0x2e8a;
export const PID = 0x000c;
const PACKET_SIZE = 64;

export function webusbAvailable() {
  return typeof navigator !== "undefined" && !!navigator.usb;
}

/// Prompt the user to pick a rustprobe. Must be called from a user gesture.
export async function requestProbe() {
  return navigator.usb.requestDevice({
    filters: [{ vendorId: VID, productId: PID }],
  });
}

/// Previously-authorized rustprobes (no prompt needed).
export async function authorizedProbes() {
  const devices = await navigator.usb.getDevices();
  return devices.filter((d) => d.vendorId === VID && d.productId === PID);
}

function isDapInterface(iface) {
  const alt = iface.alternates[0];
  if (!alt || alt.interfaceClass !== 0xff || alt.interfaceSubclass !== 0x00 ||
      alt.interfaceProtocol !== 0x00) {
    return false;
  }
  // The interface string clinches it when the browser exposes it.
  if (alt.interfaceName && !alt.interfaceName.includes("CMSIS-DAP")) return false;
  return true;
}

/// Open `device` and claim its lowest-numbered CMSIS-DAP interface.
/// Returns a transport with a single-packet `transceive`.
export async function openProbe(device, log = () => {}) {
  await device.open();
  if (device.configuration === null) await device.selectConfiguration(1);

  const iface = device.configuration.interfaces
    .filter(isDapInterface)
    .sort((a, b) => a.interfaceNumber - b.interfaceNumber)[0];
  if (!iface) {
    await device.close();
    throw new Error("device exposes no CMSIS-DAP interface");
  }
  await device.claimInterface(iface.interfaceNumber);

  const eps = iface.alternates[0].endpoints.filter((e) => e.type === "bulk");
  const epOut = eps.find((e) => e.direction === "out")?.endpointNumber;
  const epIn = eps.find((e) => e.direction === "in")?.endpointNumber;
  if (epOut === undefined || epIn === undefined) {
    await device.close();
    throw new Error("DAP interface lacks a bulk IN/OUT endpoint pair");
  }

  return {
    device,
    interfaceNumber: iface.interfaceNumber,

    async transceive(request) {
      log(`→ ${hex(request)}`);
      const out = await device.transferOut(epOut, request);
      if (out.status !== "ok") throw new Error(`bulk OUT failed: ${out.status}`);
      const inn = await device.transferIn(epIn, PACKET_SIZE);
      if (inn.status !== "ok") throw new Error(`bulk IN failed: ${inn.status}`);
      const resp = new Uint8Array(inn.data.buffer, inn.data.byteOffset, inn.data.byteLength);
      log(`← ${hex(resp)}`);
      return resp;
    },

    async close() {
      try { await device.close(); } catch { /* already gone */ }
    },
  };
}

export function hex(bytes) {
  return Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join(" ");
}
