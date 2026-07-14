// Flash tab: flash a BOOTSEL-mode board from the bundled UF2 or a picked
// file, then optionally provision it with the editor's board profile and
// topology.

import { parseUf2, filterFamily, familiesIn, familyName, coalesce } from "./uf2.js";
import { requestBootromDevice, Picoboot, CHIP_FAMILY, BOOTROM_PIDS } from "./picoboot.js";
import { requestProbe, openProbe } from "./transport.js";
import { Session } from "./session.js";

export function initFlash(app) {
  const $ = (id) => document.getElementById(id);
  let bakedBuffer = null; // ArrayBuffer of the bundled UF2, lazily fetched
  let fileBuffer = null;
  let boot = null; // open Picoboot

  const status = (msg) => { $("flash-status").textContent = msg; };

  // ---- firmware source -----------------------------------------------

  async function checkBaked() {
    const fw = app.presets.firmware;
    if (!fw || !fw.file) {
      $("baked-info").textContent = "(not bundled in this deployment)";
      $("fw-src-baked").disabled = true;
      $("fw-src-file").checked = true;
      return;
    }
    $("baked-info").textContent =
      `${fw.file.split("/").pop()}${fw.version ? ` — ${fw.version}` : ""}`;
    $("fw-src-baked").checked = true;
  }

  async function selectedUf2() {
    if ($("fw-src-file").checked) {
      if (!fileBuffer) throw new Error("choose a UF2 file first");
      return fileBuffer;
    }
    if (!bakedBuffer) {
      const resp = await fetch(app.presets.firmware.file);
      if (!resp.ok) throw new Error(`fetch bundled firmware: HTTP ${resp.status}`);
      bakedBuffer = await resp.arrayBuffer();
    }
    return bakedBuffer;
  }

  $("fw-file").addEventListener("change", async (ev) => {
    const file = ev.target.files[0];
    if (!file) return;
    fileBuffer = await file.arrayBuffer();
    $("fw-src-file").checked = true;
    try {
      const fams = familiesIn(parseUf2(fileBuffer));
      $("fw-file-info").textContent =
        `${file.name} (${(fileBuffer.byteLength / 1024).toFixed(0)} KiB, ${fams.map(familyName).join(" + ") || "no family"})`;
    } catch (e) {
      $("fw-file-info").textContent = `${file.name} — ${e.message}`;
      fileBuffer = null;
    }
  });

  // ---- bootrom connection --------------------------------------------

  let bootromDevices = []; // paired + attached dedup groups, pushed by app.refreshUsbLists

  /// Connect to a BOOTSEL device. `devices` is one device or a dedup group —
  /// stale Chrome entries fail open() with "Access denied", so try each.
  async function connectBootrom(devices) {
    if (boot) { await boot.close(); boot = null; }
    let lastErr;
    for (const device of [].concat(devices)) {
      try {
        const b = new Picoboot(device, app.log);
        await b.open();
        boot = b;
        break;
      } catch (e) {
        lastErr = e;
      }
    }
    $("bootrom-info").textContent =
      boot ? `connected: ${boot.chip.toUpperCase()} bootrom` : lastErr.message;
    $("btn-flash").disabled = !boot;
    renderBootromList();
  }

  /// Paired-and-attached BOOTSEL devices, one Connect button each; the
  /// chooser (btn-bootrom-connect) is only needed for a first-time grant.
  function renderBootromList() {
    const list = $("bootrom-list");
    list.textContent = "";
    for (const group of bootromDevices) {
      const row = document.createElement("div");
      row.className = "preset-row";
      const name = document.createElement("span");
      name.className = "name";
      name.textContent = group[0].productName ?? "RP2 Boot";
      const kind = document.createElement("span");
      kind.className = "kind";
      kind.textContent = BOOTROM_PIDS[group[0].productId]?.toUpperCase() ?? "";
      row.append(name, kind);
      if (group.includes(boot?.device)) {
        const pill = document.createElement("span");
        pill.className = "pill ok";
        pill.textContent = "connected";
        row.append(pill);
      } else {
        const btn = document.createElement("button");
        btn.textContent = "Connect";
        btn.addEventListener("click", () => connectBootrom(group));
        row.append(btn);
      }
      list.append(row);
    }
  }

  app.renderBootromList = (devices) => {
    bootromDevices = devices;
    renderBootromList();
  };

  $("btn-bootrom-connect").addEventListener("click", async () => {
    let device;
    try {
      device = await requestBootromDevice();
    } catch {
      return; // user cancelled the picker
    }
    await connectBootrom(device);
    app.refreshUsbLists();
  });

  // The connected bootrom drops off the bus when it reboots or is unplugged.
  if (navigator.usb) {
    navigator.usb.addEventListener("disconnect", (ev) => {
      if (boot && ev.device === boot.device) {
        boot = null;
        $("btn-flash").disabled = true;
        $("bootrom-info").textContent = "";
      }
    });
  }

  // ---- flash ----------------------------------------------------------

  $("btn-flash").addEventListener("click", async () => {
    if (!boot) return;
    const progressEl = $("flash-progress");
    try {
      const buffer = await selectedUf2();
      const blocks = filterFamily(parseUf2(buffer), CHIP_FAMILY[boot.chip]);
      if (blocks.length === 0) {
        throw new Error(
          `UF2 has no blocks for ${boot.chip.toUpperCase()} ` +
          `(contains: ${familiesIn(parseUf2(buffer)).map(familyName).join(", ") || "none"})`);
      }
      const ranges = coalesce(blocks);
      const bytes = ranges.reduce((n, r) => n + r.data.length, 0);
      app.log(`flashing ${bytes} bytes in ${ranges.length} range(s) to ${boot.chip}`);

      $("btn-flash").disabled = true;
      progressEl.classList.remove("hidden");
      await boot.flashRanges(ranges, (frac, label) => {
        progressEl.value = frac;
        status(label);
      });
      status("done — the board is rebooting into rustprobe");
      app.log("flash complete");
      await boot.close();
      boot = null;
      $("bootrom-info").textContent = "";
      renderBootromList();
    } catch (e) {
      status(`failed: ${e.message}`);
      app.log(`flash failed: ${e.message}`);
    } finally {
      $("btn-flash").disabled = !boot;
    }
  });

  // ---- provisioning ----------------------------------------------------

  $("btn-provision").addEventListener("click", async () => {
    const say = (msg) => { $("provision-status").textContent = msg; };
    try {
      const device = await requestProbe();
      const transport = await openProbe(device, app.log);
      const session = new Session(transport, app.wasm);
      const info = await session.info();
      say(`connected (${info.chip}); writing board profile…`);
      await session.setProfile(app.boardEditor.profile);
      say("writing topology…");
      await session.setTopology(app.editor.topo);
      say("rebooting…");
      await session.reboot();
      await transport.close();
      say("provisioned — the probe is rebooting with the new configuration");
    } catch (e) {
      say(`failed: ${e.message}`);
    }
  });

  checkBaked();
  app.renderFlash = checkBaked;
}
