// Config library (localStorage) and share-by-URL encoding.
//
// A library entry is {name, kind: "topology"|"board", data, savedAt} where
// `data` is the editor JSON object (same shape the wasm layer speaks).
// Share links carry the same entry, base64url-encoded in the URL fragment,
// so nothing is sent to any server.

const KEY = "rustprobe.configs";

export function listConfigs() {
  try {
    return JSON.parse(localStorage.getItem(KEY)) ?? [];
  } catch {
    return [];
  }
}

export function saveConfig(entry) {
  const all = listConfigs().filter(
    (e) => !(e.name === entry.name && e.kind === entry.kind));
  all.push({ ...entry, savedAt: new Date().toISOString() });
  all.sort((a, b) => a.name.localeCompare(b.name));
  localStorage.setItem(KEY, JSON.stringify(all));
}

export function deleteConfig(name, kind) {
  localStorage.setItem(KEY, JSON.stringify(
    listConfigs().filter((e) => !(e.name === name && e.kind === kind))));
}

function b64urlEncode(text) {
  const bytes = new TextEncoder().encode(text);
  let bin = "";
  for (const b of bytes) bin += String.fromCharCode(b);
  return btoa(bin).replaceAll("+", "-").replaceAll("/", "_").replace(/=+$/, "");
}

function b64urlDecode(text) {
  const bin = atob(text.replaceAll("-", "+").replaceAll("_", "/"));
  const bytes = Uint8Array.from(bin, (c) => c.charCodeAt(0));
  return new TextDecoder().decode(bytes);
}

/// Build a share URL for the current page carrying `entry`.
export function shareUrl(entry) {
  const payload = b64urlEncode(JSON.stringify({
    name: entry.name, kind: entry.kind, data: entry.data,
  }));
  const url = new URL(location.href);
  url.hash = `share=${payload}`;
  return url.toString();
}

/// Parse a share payload from the current URL fragment, or null.
export function sharedFromUrl() {
  const m = location.hash.match(/^#share=(.+)$/);
  if (!m) return null;
  try {
    const entry = JSON.parse(b64urlDecode(m[1]));
    if (!entry.kind || !entry.data) return null;
    return entry;
  } catch {
    return null;
  }
}
