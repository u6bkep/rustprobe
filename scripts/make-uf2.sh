#!/usr/bin/env bash
# Convert the built firmware ELFs to UF2s and merge a combined RP2040+RP2350
# image. A combined UF2 is a plain concatenation: each block carries its
# family ID and each bootrom ignores foreign-family blocks (verified with
# `picotool info -t uf2` — both program blocks are reported).
#
# Usage: scripts/make-uf2.sh <outdir>
# Expects both release firmwares to be built already (see README) and
# picotool on PATH.
set -euo pipefail

out="${1:?usage: make-uf2.sh <outdir>}"
root="$(cd "$(dirname "$0")/.." && pwd)"
elf2040="$root/target/thumbv6m-none-eabi/release/rustprobe-firmware"
elf2350="$root/target/thumbv8m.main-none-eabihf/release/rustprobe-firmware"

for f in "$elf2040" "$elf2350"; do
    [[ -f $f ]] || { echo "missing $f — build the firmware first" >&2; exit 1; }
done

mkdir -p "$out"
picotool uf2 convert --quiet -t elf "$elf2040" "$out/rustprobe-rp2040.uf2"
picotool uf2 convert --quiet -t elf "$elf2350" "$out/rustprobe-rp2350.uf2"
cat "$out/rustprobe-rp2040.uf2" "$out/rustprobe-rp2350.uf2" > "$out/rustprobe.uf2"

# Sanity-check that picotool sees both families in the combined image.
info="$(picotool info -t uf2 "$out/rustprobe.uf2")"
for fam in "'rp2040'" "'rp2350-arm-s'"; do
    grep -q "family ID $fam" <<<"$info" \
        || { echo "combined UF2 is missing family $fam" >&2; exit 1; }
done
echo "wrote $out/{rustprobe-rp2040,rustprobe-rp2350,rustprobe}.uf2"
