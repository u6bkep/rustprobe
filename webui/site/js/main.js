// App bootstrap: wasm init, tab switching, device dashboard, presets
// manifest, share-link intake, protocol console.

import init, * as wasm from "../pkg/probe_config_wasm.js";
import { webusbAvailable, requestProbe, authorizedProbes, openProbe } from "./transport.js";
import { Session } from "./session.js";
import { initEditor, download } from "./editor.js";
import { initBoard } from "./board.js";
import { initFlash } from "./flash.js";
import * as store from "./store.js";

const $ = (id) => document.getElementById(id);

// Chip limits mirror probe-config/src/lib.rs (RP2040 / RP2350A / RP2350B),
// used until a device reports its own via CMD_INFO.
const LIMITS = {
  Rp2040: { gpio_count: 30, pio_blocks: 2, sms_per_block: 4, ep_numbers_per_dir: 15, pio_pin_window: 32 },
  Rp2350: { gpio_count: 30, pio_blocks: 3, sms_per_block: 4, ep_numbers_per_dir: 15, pio_pin_window: 32 },
};

const app = {
  wasm: null,
  session: null,
  transport: null,
  info: null,
  profile: null, // device's board profile
  presets: { topologies: [], boards: [], firmware: null },
  // Editor state; defaults mirror the firmware's built-in fallbacks.
  editor: {
    topo: { probes: [{ swclk: 2, swdio: 3, reset: 1 }], uarts: [{ tx: 4, rx: 5, baud: 115200 }] },
    name: "default",
  },
  boardEditor: {
    profile: { available: "0-28", reserved: "23-25,29" }, // bare Pico
    name: "pico",
  },
  offlineChip: "Rp2350",

  log(line) {
    const el = $("console");
    const t = new Date().toISOString().slice(11, 23);
    el.textContent += `${t} ${line}\n`;
    el.scrollTop = el.scrollHeight;
  },

  currentLimits() {
    return this.info ? this.info.limits : LIMITS[this.offlineChip];
  },

  /// Profile the topology editor validates against: the connected device's,
  /// else the board editor's.
  currentProfile() {
    return this.profile ?? this.boardEditor.profile;
  },

  async refreshDevice() {
    if (!this.session) return;
    this.info = await this.session.info();
    this.profile = this.info.protocol_version >= 2
      ? await this.session.getProfile()
      : { available: "0-28", reserved: "23-25,29" };
    const topo = await this.session.getTopology();
    renderDevice(topo);
    this.renderEditor?.();
    this.renderBoard?.();
  },

  async disconnect(reason) {
    if (this.transport) await this.transport.close();
    this.session = null;
    this.transport = null;
    this.info = null;
    this.profile = null;
    renderDevice(null);
    if (reason) this.log(reason);
    this.renderEditor?.();
    this.renderBoard?.();
  },
};

// ---- tabs -------------------------------------------------------------

document.querySelectorAll("nav#tabs button").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll("nav#tabs button").forEach((b) => b.classList.remove("active"));
    document.querySelectorAll(".tab").forEach((t) => t.classList.remove("active"));
    btn.classList.add("active");
    $(`tab-${btn.dataset.tab}`).classList.add("active");
  });
});

// ---- device tab --------------------------------------------------------

function renderDevice(topo) {
  const connected = !!app.session;
  $("device-none").classList.toggle("hidden", connected);
  $("device-panel").classList.toggle("hidden", !connected);
  for (const id of ["btn-refresh", "btn-reboot", "btn-bootsel", "btn-disconnect"]) {
    $(id).disabled = !connected;
  }
  $("fault-banner").classList.add("hidden");
  if (!connected) return;

  const i = app.info;
  const [maj, min, pat] = i.firmware_version;
  const serial = app.transport.device.serialNumber ?? "?";
  $("info-table").innerHTML = [
    ["serial", serial],
    ["chip", i.chip],
    ["firmware", `${maj}.${min}.${pat}`],
    ["protocol", i.protocol_version],
    ["active probes", i.active_probes],
    ["active UARTs", i.active_uarts],
    ["GPIOs", i.limits.gpio_count],
    ["PIO blocks", `${i.limits.pio_blocks} × ${i.limits.sms_per_block} SMs`],
  ].map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join("");

  if (i.config_fault) {
    const el = $("fault-banner");
    el.classList.remove("hidden");
    el.textContent =
      "Config fault: the stored topology was missing or invalid at boot and " +
      "the firmware fell back to its default. Write a valid topology (and " +
      "check the board profile) to clear this.";
  }

  $("active-toml").textContent =
    app.wasm.topology_to_toml(JSON.stringify(topo)) || "# empty topology";
  $("active-board-toml").textContent = i.protocol_version >= 2
    ? app.wasm.profile_to_toml(JSON.stringify(app.profile))
    : "# firmware predates board profiles (protocol < 2)";
  app.activeTopo = topo;
}

async function connect(device) {
  try {
    app.transport = await openProbe(device, app.log);
    app.session = new Session(app.transport, app.wasm);
    device.addEventListener?.("disconnect", () => app.disconnect("probe disconnected"));
    await app.refreshDevice();
    app.log(`connected to ${device.serialNumber ?? "probe"}`);
  } catch (e) {
    await app.disconnect();
    app.log(`connect failed: ${e.message}`);
    alert(`connect failed: ${e.message}`);
  }
}

$("btn-connect").addEventListener("click", async () => {
  try {
    await app.disconnect();
    await connect(await requestProbe());
  } catch {
    /* user cancelled the picker */
  }
});
$("btn-refresh").addEventListener("click", () => app.refreshDevice().catch((e) => alert(e.message)));
$("btn-disconnect").addEventListener("click", () => app.disconnect("disconnected"));
$("btn-reboot").addEventListener("click", async () => {
  try {
    await app.session.reboot();
    await app.disconnect("reboot requested; reconnect when the probe re-enumerates");
  } catch (e) { alert(e.message); }
});
$("btn-bootsel").addEventListener("click", async () => {
  try {
    await app.session.rebootBootsel();
    await app.disconnect("BOOTSEL reboot requested — head to the Flash tab");
    document.querySelector('nav#tabs button[data-tab="flash"]').click();
  } catch (e) { alert(e.message); }
});
$("btn-edit-active").addEventListener("click", () => {
  if (!app.activeTopo) return;
  app.editor.topo = structuredClone(app.activeTopo);
  app.editor.name = "active";
  app.renderEditor?.();
  document.querySelector('nav#tabs button[data-tab="configure"]').click();
});
$("btn-download-active").addEventListener("click", () => {
  if (!app.activeTopo) return;
  download("topology.toml", app.wasm.topology_to_toml(JSON.stringify(app.activeTopo)));
});
$("btn-edit-board").addEventListener("click", () => {
  if (!app.profile) return;
  app.loadBoardEditor(structuredClone(app.profile), "device");
  document.querySelector('nav#tabs button[data-tab="board"]').click();
});

// USB disconnect events at the bus level.
if (webusbAvailable()) {
  navigator.usb.addEventListener("disconnect", (ev) => {
    if (app.transport && ev.device === app.transport.device) {
      app.disconnect("probe disconnected");
    }
  });
}

// ---- boot --------------------------------------------------------------

async function loadPresets() {
  try {
    const resp = await fetch("presets/manifest.json");
    if (!resp.ok) return;
    const manifest = await resp.json();
    app.presets.topologies = manifest.topologies ?? [];
    app.presets.boards = manifest.boards ?? [];
    app.presets.firmware = manifest.firmware ?? null;
    if (manifest.repo) {
      const link = $("repo-link");
      link.href = manifest.repo;
      link.classList.remove("hidden");
    }
  } catch {
    /* running without a manifest (e.g. straight from the source tree) */
  }
}

async function main() {
  await init();
  app.wasm = wasm;
  const k = JSON.parse(wasm.constants());
  $("version").textContent = `config protocol v${k.protocol_version}`;

  if (!webusbAvailable()) {
    $("usb-support").classList.remove("hidden");
    $("btn-connect").disabled = true;
    $("btn-bootrom-connect").disabled = true;
  }

  await loadPresets();
  initEditor(app);
  initBoard(app);
  initFlash(app);
  app.renderPresets?.();
  app.renderBoardPresets?.();
  app.renderFlash?.();

  // Share link? Load it into the right editor.
  const shared = store.sharedFromUrl();
  if (shared) {
    if (shared.kind === "topology") {
      app.editor.topo = shared.data;
      app.editor.name = shared.name ?? "shared";
      app.renderEditor?.();
      document.querySelector('nav#tabs button[data-tab="configure"]').click();
    } else if (shared.kind === "board") {
      app.loadBoardEditor(shared.data, shared.name ?? "shared");
      document.querySelector('nav#tabs button[data-tab="board"]').click();
    }
    app.log(`loaded shared ${shared.kind} "${shared.name ?? ""}" from URL`);
    history.replaceState(null, "", location.pathname + location.search);
  }

  // Reconnect a previously-authorized probe without prompting.
  if (webusbAvailable()) {
    const known = await authorizedProbes();
    if (known.length === 1) await connect(known[0]);
  }
}

main().catch((e) => {
  console.error(e);
  document.body.insertAdjacentHTML("afterbegin",
    `<div class="banner error" style="margin:1rem">failed to start: ${e.message} — ` +
    `if you are running from the source tree, build the wasm module first ` +
    `(webui/build.sh)</div>`);
});
