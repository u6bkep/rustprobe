// Configure tab: topology editor, live pin map, budget meters, presets,
// library, TOML import/export, share links.

import * as store from "./store.js";

const ROLE_LABELS = { swclk: "CLK", swdio: "DIO", reset: "RST", tx: "TX", rx: "RX" };

export function initEditor(app) {
  const $ = (id) => document.getElementById(id);
  const state = app.editor; // {topo: {probes, uarts}, name}

  // ---- helpers -------------------------------------------------------

  function profile() { return app.currentProfile(); }
  function limits() { return app.currentLimits(); }

  function assignablePins() {
    const avail = new Set(JSON.parse(app.wasm.pins_in_ranges(profile().available)));
    const reserved = new Set(JSON.parse(app.wasm.pins_in_ranges(profile().reserved)));
    return [...avail].filter((p) => !reserved.has(p) && p < limits().gpio_count);
  }

  /// pin -> {role, inst} map for the current topology.
  function pinRoles() {
    const roles = new Map();
    state.topo.probes.forEach((p, i) => {
      roles.set(p.swclk, { role: "swclk", inst: i });
      roles.set(p.swdio, { role: "swdio", inst: i });
      if (p.reset !== undefined && p.reset !== null) roles.set(p.reset, { role: "reset", inst: i });
    });
    state.topo.uarts.forEach((u, i) => {
      roles.set(u.tx, { role: "tx", inst: i });
      roles.set(u.rx, { role: "rx", inst: i });
    });
    return roles;
  }

  function validate() {
    try {
      return JSON.parse(app.wasm.validate_topology(
        JSON.stringify(state.topo),
        JSON.stringify(limits()),
        JSON.stringify(profile()),
      ));
    } catch (e) {
      return { code: "Internal", pin: null, message: String(e) };
    }
  }

  // ---- rendering -----------------------------------------------------

  function pinOptions(current, legal) {
    const opts = [...new Set([...legal, ...(current !== null ? [current] : [])])]
      .sort((a, b) => a - b);
    return opts.map((p) =>
      `<option value="${p}" ${p === current ? "selected" : ""}>GP${p}</option>`).join("");
  }

  function renderInstances() {
    const legal = assignablePins();
    const probesEl = $("probes-list");
    probesEl.innerHTML = state.topo.probes.map((p, i) => `
      <div class="inst" data-kind="probe" data-i="${i}">
        <span class="idx">probe ${i}</span>
        <label>SWCLK <select data-f="swclk">${pinOptions(p.swclk, legal)}</select></label>
        <label>SWDIO <select data-f="swdio">${pinOptions(p.swdio, legal)}</select></label>
        <label>nRESET <select data-f="reset">
          <option value="" ${p.reset == null ? "selected" : ""}>—</option>
          ${pinOptions(p.reset ?? null, legal)}
        </select></label>
        <button class="del" title="remove">✕</button>
      </div>`).join("");

    const txPins = legal.filter((p) => app.wasm.uart_tx_instance(p) >= 0);
    const rxPins = legal.filter((p) => app.wasm.uart_rx_instance(p) >= 0);
    $("uarts-list").innerHTML = state.topo.uarts.map((u, i) => `
      <div class="inst" data-kind="uart" data-i="${i}">
        <span class="idx">uart ${i}</span>
        <label>TX <select data-f="tx">${pinOptions(u.tx, txPins)}</select></label>
        <label>RX <select data-f="rx">${pinOptions(u.rx, rxPins)}</select></label>
        <label>baud <input type="number" data-f="baud" value="${u.baud}" min="110" max="4000000" step="1" style="width:7rem"></label>
        <span class="dim">UART${app.wasm.uart_tx_instance(u.tx) >= 0 ? app.wasm.uart_tx_instance(u.tx) : "?"}</span>
        <button class="del" title="remove">✕</button>
      </div>`).join("");
  }

  function renderBudgets(err) {
    const l = limits();
    const nP = state.topo.probes.length, nU = state.topo.uarts.length;
    const totalSms = l.pio_blocks * l.sms_per_block;
    const sms = nP + (nU > 0 ? 1 : 0);
    const epIn = nP + 2 * nU, epOut = nP + nU;
    const bars = [
      ["PIO state machines", sms, totalSms, nU > 0 ? "(probes + autobaud)" : "(probes)"],
      ["USB IN endpoints", epIn, l.ep_numbers_per_dir, "(probes + 2×UARTs)"],
      ["USB OUT endpoints", epOut, l.ep_numbers_per_dir, "(probes + UARTs)"],
      ["hardware UARTs", nU, 2, ""],
    ];
    $("budgets").innerHTML = bars.map(([name, used, max, note]) => `
      <div class="budget ${used > max ? "full" : ""}">
        ${name}: ${used} / ${max} <span class="dim">${note}</span>
        <div class="bar"><div class="fill" style="width:${Math.min(100, 100 * used / max)}%"></div></div>
      </div>`).join("");

    const banner = $("validation-banner");
    banner.classList.remove("hidden", "error", "ok");
    if (err) {
      banner.classList.add("error");
      banner.textContent = `Invalid: ${err.message}`;
    } else {
      banner.classList.add("ok");
      banner.textContent = `Valid — ${nP} probe${nP === 1 ? "" : "s"}, ${nU} UART${nU === 1 ? "" : "s"}`;
    }
    $("btn-apply").disabled = !!err || !app.session;
    $("btn-load-active").disabled = !app.session;
  }

  function renderPinmap(err) {
    const l = limits();
    const roles = pinRoles();
    const avail = new Set(JSON.parse(app.wasm.pins_in_ranges(profile().available)));
    const reserved = new Set(JSON.parse(app.wasm.pins_in_ranges(profile().reserved)));
    const cells = [];
    for (let p = 0; p < l.gpio_count; p++) {
      const r = roles.get(p);
      let cls = "pin", title = `GP${p}`;
      if (!avail.has(p)) { cls += " absent"; title += " — not on this board"; }
      else if (reserved.has(p)) { cls += " reserved"; title += " — reserved by the board"; }
      if (r) { cls += ` ${r.role}`; title += ` — ${r.role.toUpperCase()} of ${r.role === "tx" || r.role === "rx" ? "uart" : "probe"} ${r.inst}`; }
      if (err && err.pin === p && (err.code === "PinConflict" || err.code === "PinUnavailable")) {
        cls += " conflict";
      }
      const roleTxt = r ? `${ROLE_LABELS[r.role]}${r.inst}` : "";
      cells.push(`<div class="${cls}" title="${title}">GP${p}<span class="role">${roleTxt}</span></div>`);
    }
    $("pinmap").innerHTML = cells.join("");
    $("pinmap-legend").innerHTML = `
      <span class="chip swclk">SWCLK</span><span class="chip swdio">SWDIO</span>
      <span class="chip reset">nRESET</span><span class="chip tx">UART TX</span>
      <span class="chip rx">UART RX</span><span class="chip reserved">reserved</span>
      <span class="chip absent">absent</span>`;
  }

  function render() {
    $("topo-name").textContent = state.name ? `— ${state.name}` : "";
    renderInstances();
    const err = validate();
    renderBudgets(err);
    renderPinmap(err);
    renderLibrary();
  }
  app.renderEditor = render;

  // ---- instance edits ------------------------------------------------

  function firstFree(pins) {
    const used = new Set(pinRoles().keys());
    return pins.find((p) => !used.has(p));
  }

  $("btn-add-probe").addEventListener("click", () => {
    const legal = assignablePins();
    const clk = firstFree(legal);
    const used = new Set([...pinRoles().keys(), clk]);
    const dio = legal.find((p) => !used.has(p));
    if (clk === undefined || dio === undefined) {
      alert("no free assignable pins left");
      return;
    }
    state.topo.probes.push({ swclk: clk, swdio: dio });
    render();
  });

  $("btn-add-uart").addEventListener("click", () => {
    const legal = assignablePins();
    const used = new Set(pinRoles().keys());
    // Find a TX/RX pair on the same free UART instance.
    const usedInst = new Set(state.topo.uarts.map(
      (u) => app.wasm.uart_tx_instance(u.tx)).filter((i) => i >= 0));
    for (const tx of legal) {
      const inst = app.wasm.uart_tx_instance(tx);
      if (inst < 0 || used.has(tx) || usedInst.has(inst)) continue;
      const rx = legal.find((p) => !used.has(p) && p !== tx && app.wasm.uart_rx_instance(p) === inst);
      if (rx !== undefined) {
        state.topo.uarts.push({ tx, rx, baud: 115200 });
        render();
        return;
      }
    }
    alert("no free TX/RX pin pair for an unused hardware UART");
  });

  for (const listId of ["probes-list", "uarts-list"]) {
    $(listId).addEventListener("change", (ev) => {
      const inst = ev.target.closest(".inst");
      if (!inst) return;
      const i = Number(inst.dataset.i);
      const f = ev.target.dataset.f;
      const obj = inst.dataset.kind === "probe" ? state.topo.probes[i] : state.topo.uarts[i];
      if (f === "reset") {
        if (ev.target.value === "") delete obj.reset;
        else obj.reset = Number(ev.target.value);
      } else if (f === "baud") {
        obj.baud = Number(ev.target.value) || 115200;
      } else {
        obj[f] = Number(ev.target.value);
      }
      render();
    });
    $(listId).addEventListener("click", (ev) => {
      if (!ev.target.classList.contains("del")) return;
      const inst = ev.target.closest(".inst");
      const i = Number(inst.dataset.i);
      if (inst.dataset.kind === "probe") state.topo.probes.splice(i, 1);
      else state.topo.uarts.splice(i, 1);
      render();
    });
  }

  // ---- apply ---------------------------------------------------------

  $("btn-apply").addEventListener("click", async () => {
    if (!app.session) return;
    try {
      await app.session.setTopology(state.topo);
      if ($("apply-reboot").checked) {
        await app.session.reboot();
        app.log("topology written; probe rebooting");
        await app.disconnect("probe rebooting — reconnect when it re-enumerates");
      } else {
        app.log("topology written; reboot to apply");
        await app.refreshDevice();
      }
    } catch (e) {
      alert(`write failed: ${e.message}`);
    }
  });

  $("btn-load-active").addEventListener("click", async () => {
    if (!app.session) return;
    try {
      state.topo = await app.session.getTopology();
      state.name = "active";
      render();
    } catch (e) {
      alert(`read failed: ${e.message}`);
    }
  });

  // ---- presets & library ---------------------------------------------

  function renderPresets() {
    const rows = app.presets.topologies.map((p, i) => `
      <div class="preset-row">
        <span class="name">${p.name}</span><span class="kind">preset</span>
        <button data-preset="${i}">Load</button>
      </div>`);
    $("presets-list").innerHTML = rows.join("") ||
      `<div class="dim">no bundled presets</div>`;
  }
  app.renderPresets = renderPresets;

  $("presets-list").addEventListener("click", async (ev) => {
    const i = ev.target.dataset.preset;
    if (i === undefined) return;
    const p = app.presets.topologies[Number(i)];
    try {
      const text = await (await fetch(p.file)).text();
      state.topo = JSON.parse(app.wasm.topology_from_toml(text));
      state.name = p.name;
      render();
    } catch (e) {
      alert(`load preset failed: ${e.message}`);
    }
  });

  function renderLibrary() {
    const rows = store.listConfigs().map((e) => `
      <div class="preset-row" data-name="${e.name}" data-kind="${e.kind}">
        <span class="name">${e.name}</span><span class="kind">${e.kind}</span>
        <button data-act="load">Load</button>
        <button data-act="share">Share</button>
        <button data-act="del" class="danger">✕</button>
      </div>`);
    $("library-list").innerHTML = rows.join("") ||
      `<div class="dim">nothing saved yet</div>`;
  }

  $("library-list").addEventListener("click", (ev) => {
    const row = ev.target.closest(".preset-row");
    if (!row || !ev.target.dataset.act) return;
    const entry = store.listConfigs().find(
      (e) => e.name === row.dataset.name && e.kind === row.dataset.kind);
    if (!entry) return;
    switch (ev.target.dataset.act) {
      case "load":
        if (entry.kind === "topology") {
          state.topo = structuredClone(entry.data);
          state.name = entry.name;
          render();
        } else {
          app.loadBoardEditor(structuredClone(entry.data), entry.name);
          document.querySelector('nav#tabs button[data-tab="board"]').click();
        }
        break;
      case "share":
        navigator.clipboard.writeText(store.shareUrl(entry));
        app.log(`share link for "${entry.name}" copied`);
        break;
      case "del":
        store.deleteConfig(entry.name, entry.kind);
        renderLibrary();
        break;
    }
  });

  $("btn-save-local").addEventListener("click", () => {
    const name = $("save-name").value.trim() || state.name || "unnamed";
    store.saveConfig({ name, kind: "topology", data: structuredClone(state.topo) });
    state.name = name;
    render();
  });

  $("btn-share").addEventListener("click", () => {
    navigator.clipboard.writeText(store.shareUrl({
      name: state.name || "shared", kind: "topology", data: state.topo,
    }));
    app.log("share link copied to clipboard");
  });

  // ---- TOML ----------------------------------------------------------

  const tomlErr = (msg) => {
    const el = $("toml-error");
    el.classList.toggle("hidden", !msg);
    el.textContent = msg ?? "";
  };

  $("btn-toml-render").addEventListener("click", () => {
    try {
      $("toml-box").value = app.wasm.topology_to_toml(JSON.stringify(state.topo));
      tomlErr(null);
    } catch (e) { tomlErr(e.message); }
  });

  $("btn-toml-load").addEventListener("click", () => {
    try {
      state.topo = JSON.parse(app.wasm.topology_from_toml($("toml-box").value));
      state.name = "";
      tomlErr(null);
      render();
    } catch (e) { tomlErr(e.message); }
  });

  $("btn-toml-download").addEventListener("click", () => {
    try {
      const text = app.wasm.topology_to_toml(JSON.stringify(state.topo));
      download(`${state.name || "topology"}.toml`, text);
      tomlErr(null);
    } catch (e) { tomlErr(e.message); }
  });

  $("toml-file").addEventListener("change", async (ev) => {
    const file = ev.target.files[0];
    if (!file) return;
    try {
      state.topo = JSON.parse(app.wasm.topology_from_toml(await file.text()));
      state.name = file.name.replace(/\.toml$/, "");
      tomlErr(null);
      render();
    } catch (e) { tomlErr(e.message); }
    ev.target.value = "";
  });

  renderPresets();
  render();
}

export function download(name, text) {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
  a.download = name;
  a.click();
  URL.revokeObjectURL(a.href);
}
