// Board tab: board-profile editor (click pins through assignable → reserved
// → absent), presets, TOML import/export, write to probe.

import * as store from "./store.js";
import { download } from "./editor.js";

export function initBoard(app) {
  const $ = (id) => document.getElementById(id);
  const state = app.boardEditor; // {profile: {available, reserved}, name}

  function pinSets() {
    return {
      avail: new Set(JSON.parse(app.wasm.pins_in_ranges(state.profile.available))),
      reserved: new Set(JSON.parse(app.wasm.pins_in_ranges(state.profile.reserved))),
    };
  }

  function setsToProfile(avail, reserved) {
    state.profile = {
      available: app.wasm.pins_to_ranges(JSON.stringify([...avail].sort((a, b) => a - b))),
      reserved: app.wasm.pins_to_ranges(JSON.stringify([...reserved].sort((a, b) => a - b))),
    };
  }

  function validate() {
    try {
      return JSON.parse(app.wasm.validate_profile(
        JSON.stringify(state.profile), JSON.stringify(app.currentLimits())));
    } catch (e) {
      return { code: "Internal", pin: null, message: String(e) };
    }
  }

  function render() {
    const l = app.currentLimits();
    const { avail, reserved } = pinSets();
    const cells = [];
    for (let p = 0; p < l.gpio_count; p++) {
      let cls = "pin clickable", label = "";
      if (!avail.has(p)) { cls += " absent"; label = "absent"; }
      else if (reserved.has(p)) { cls += " reserved"; label = "resv"; }
      cells.push(
        `<div class="${cls}" data-pin="${p}" title="GP${p} — click to cycle">GP${p}<span class="role">${label}</span></div>`);
    }
    $("board-pinmap").innerHTML = cells.join("");

    const err = validate();
    const banner = $("board-validation");
    banner.classList.remove("hidden", "error", "ok");
    if (err) {
      banner.classList.add("error");
      banner.textContent = `Invalid: ${err.message}`;
    } else {
      banner.classList.add("ok");
      banner.textContent =
        `Valid — available ${state.profile.available || "(none)"}, reserved ${state.profile.reserved || "(none)"}`;
    }
    const preProfile = app.info && app.info.protocol_version < 2;
    $("btn-board-apply").disabled = !!err || !app.session || preProfile;
    $("btn-board-load-active").disabled = !app.session || preProfile;
    $("board-toml-box").value = app.wasm.profile_to_toml(JSON.stringify(state.profile));

    // Board changes shift what's legal in the topology editor.
    app.renderEditor?.();
  }
  app.renderBoard = render;
  app.loadBoardEditor = (profile, name) => {
    state.profile = profile;
    state.name = name ?? "";
    render();
  };

  $("board-pinmap").addEventListener("click", (ev) => {
    const cell = ev.target.closest(".pin");
    if (!cell) return;
    const p = Number(cell.dataset.pin);
    const { avail, reserved } = pinSets();
    // assignable → reserved → absent → assignable
    if (avail.has(p) && !reserved.has(p)) reserved.add(p);
    else if (avail.has(p)) { avail.delete(p); reserved.delete(p); }
    else avail.add(p);
    setsToProfile(avail, reserved);
    render();
  });

  // Presets dropdown (board files from configs/boards/).
  function renderPresets() {
    $("board-presets").innerHTML = app.presets.boards.map(
      (b, i) => `<option value="${i}">${b.name}</option>`).join("");
  }
  app.renderBoardPresets = renderPresets;

  $("btn-board-preset").addEventListener("click", async () => {
    const b = app.presets.boards[Number($("board-presets").value)];
    if (!b) return;
    try {
      const text = await (await fetch(b.file)).text();
      app.loadBoardEditor(JSON.parse(app.wasm.profile_from_toml(text)), b.name);
    } catch (e) {
      alert(`load preset failed: ${e.message}`);
    }
  });

  $("btn-board-load-active").addEventListener("click", async () => {
    if (!app.session) return;
    try {
      app.loadBoardEditor(await app.session.getProfile(), "device");
    } catch (e) {
      alert(`read failed: ${e.message}`);
    }
  });

  $("btn-board-apply").addEventListener("click", async () => {
    if (!app.session) return;
    try {
      await app.session.setProfile(state.profile);
      app.log("board profile written");
      // Mirror the CLI: warn when the active topology now violates the profile.
      const active = await app.session.getTopology();
      const err = JSON.parse(app.wasm.validate_topology(
        JSON.stringify(active),
        JSON.stringify(app.currentLimits()),
        JSON.stringify(state.profile)));
      if (err) {
        alert(`Written — but the active topology violates the new profile ` +
          `(${err.message}). The probe will fall back to its default topology ` +
          `on next boot unless you write a compatible one.`);
      }
      await app.refreshDevice();
    } catch (e) {
      alert(`write failed: ${e.message}`);
    }
  });

  const tomlErr = (msg) => {
    const el = $("board-toml-error");
    el.classList.toggle("hidden", !msg);
    el.textContent = msg ?? "";
  };

  $("btn-board-toml-render").addEventListener("click", () => {
    try {
      $("board-toml-box").value = app.wasm.profile_to_toml(JSON.stringify(state.profile));
      tomlErr(null);
    } catch (e) { tomlErr(e.message); }
  });

  $("btn-board-toml-load").addEventListener("click", () => {
    try {
      app.loadBoardEditor(JSON.parse(app.wasm.profile_from_toml($("board-toml-box").value)));
      tomlErr(null);
    } catch (e) { tomlErr(e.message); }
  });

  $("btn-board-toml-download").addEventListener("click", () => {
    try {
      download(`${state.name || "board"}.toml`,
        app.wasm.profile_to_toml(JSON.stringify(state.profile)));
      tomlErr(null);
    } catch (e) { tomlErr(e.message); }
  });

  $("board-toml-file").addEventListener("change", async (ev) => {
    const file = ev.target.files[0];
    if (!file) return;
    try {
      app.loadBoardEditor(
        JSON.parse(app.wasm.profile_from_toml(await file.text())),
        file.name.replace(/\.toml$/, ""));
      tomlErr(null);
    } catch (e) { tomlErr(e.message); }
    ev.target.value = "";
  });

  $("btn-board-save-local").addEventListener("click", () => {
    const name = $("board-save-name").value.trim() || state.name || "unnamed board";
    store.saveConfig({ name, kind: "board", data: structuredClone(state.profile) });
    state.name = name;
    app.renderEditor?.(); // library list lives on the Configure tab
    app.log(`board profile "${name}" saved locally`);
  });

  renderPresets();
  render();
}
