#!/usr/bin/env bash
# Assemble the static web UI into webui/dist: wasm module, site files,
# presets from configs/, and (when present) the firmware UF2s.
#
# Usage: webui/build.sh [--firmware <dir-with-uf2s>] [--version <tag>] [--repo <url>]
# Serve locally with e.g.: python3 -m http.server -d webui/dist
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
dist="$root/webui/dist"
fwdir=""
version="dev"
repo=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --firmware) fwdir="$2"; shift 2 ;;
        --version) version="$2"; shift 2 ;;
        --repo) repo="$2"; shift 2 ;;
        *) echo "unknown argument: $1" >&2; exit 1 ;;
    esac
done

wasm-pack build "$root/probe-config-wasm" --target web --release

rm -rf "$dist"
cp -r "$root/webui/site" "$dist"
mkdir -p "$dist/pkg"
cp "$root/probe-config-wasm/pkg/probe_config_wasm.js" \
   "$root/probe-config-wasm/pkg/probe_config_wasm_bg.wasm" "$dist/pkg/"

# Presets: every topology TOML in configs/ and board TOML in configs/boards/.
mkdir -p "$dist/presets/boards"
cp "$root"/configs/*.toml "$dist/presets/" 2>/dev/null || true
cp "$root"/configs/boards/*.toml "$dist/presets/boards/" 2>/dev/null || true

# Bundled firmware (CI passes the make-uf2.sh output dir).
fw_json="null"
if [[ -n $fwdir ]]; then
    mkdir -p "$dist/firmware"
    cp "$fwdir"/*.uf2 "$dist/firmware/"
    fw_json="{\"file\": \"firmware/rustprobe.uf2\", \"version\": \"$version\"}"
fi

python3 - "$dist" "$fw_json" "$version" "$repo" <<'EOF'
import json, sys
from pathlib import Path

dist, fw_json, version, repo = Path(sys.argv[1]), sys.argv[2], sys.argv[3], sys.argv[4]
manifest = {
    "version": version,
    "repo": repo or None,
    "topologies": [
        {"name": p.stem, "file": f"presets/{p.name}"}
        for p in sorted(dist.glob("presets/*.toml"))
    ],
    "boards": [
        {"name": p.stem, "file": f"presets/boards/{p.name}"}
        for p in sorted(dist.glob("presets/boards/*.toml"))
    ],
    "firmware": json.loads(fw_json),
}
(dist / "presets/manifest.json").write_text(json.dumps(manifest, indent=1))
print(f"manifest: {len(manifest['topologies'])} topologies, "
      f"{len(manifest['boards'])} boards, firmware={manifest['firmware'] is not None}")
EOF

echo "site assembled in webui/dist"
