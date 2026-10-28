# rustprobe

Rust rewrite of the multiprobe debugprobe firmware: N SWD probes + M CDC-UART
bridges on RP2040/RP2350, with the topology configured at boot from flash
instead of at compile time.

## Workspace

| Crate | Purpose |
|---|---|
| `probe-config` | `no_std` config schema + validation, shared by firmware and host tools |
| `probe-config-toml` | TOML mirrors of the wire types, shared by the CLI and web UI |
| `probe-config-wasm` | wasm-bindgen bindings so the web UI runs the same encode/decode/validation |
| `autobaud-estimator` | `no_std` baud-rate estimator core (raw edge-timing samples in, baud out), host-testable |
| `firmware` | embassy-based probe firmware (features: `rp2040` \| `rp2350`) |
| `cli` | `rustprobe` host CLI: configure the probe over USB (nusb + `probe-config`) |

The `webui/` directory holds a static web UI (vanilla JS + the wasm crate,
no external dependencies) that does everything the CLI does over WebUSB —
see below.

Embassy comes from the `computer-whisperer/embassy` fork (`raven_merge6`),
which carries not-yet-upstreamed fixes we rely on (PIO GPIOBASE window
selection on RP2350B, RP2350 flash driver QMI save/restore, PIO/DMA TOCTOU).

## Building

```sh
# RP2040 (Pico)
cargo build -p rustprobe-firmware --target thumbv6m-none-eabi --release

# RP2350 (Pico 2)
cargo build -p rustprobe-firmware --target thumbv8m.main-none-eabihf \
    --no-default-features --features rp2350 --release

# Host-side tests (config validation + autobaud estimator)
cargo test --workspace --exclude rustprobe-firmware
```

`cargo run` flashes via probe-rs (see `.cargo/config.toml` runners); defmt
logs arrive over RTT.

## Host CLI

The `rustprobe` binary reads and writes the probe's topology over USB, using
the config protocol (`probe_config::protocol`) carried on the CMSIS-DAP vendor
commands. Config commands are served on any probe interface.

```sh
cargo build -p rustprobe-cli          # binary: target/debug/rustprobe

rustprobe list                        # attached probes (serial, product, DAP interfaces)
rustprobe info                        # chip, fw/protocol version, limits, active probes/uarts
rustprobe get > topology.toml         # dump the active topology as TOML
rustprobe set topology.toml           # validate + stage + commit a new topology
rustprobe set topology.toml --reboot  # ...and reboot so it takes effect
rustprobe get-board                   # dump the current board profile as TOML
rustprobe set-board configs/boards/rp2350-zero.toml  # store a board profile
rustprobe reboot                      # reboot the probe
```

Use `--serial <uid:0>` (from `rustprobe list`) to pick one when several probes
are attached. `set` validates locally against the chip limits and board
profile reported by `info` before staging; the firmware re-validates on
commit.

The board profile — which GPIOs are wired out and which are reserved for the
board itself (LEDs etc.) — is stored in flash alongside the topology and
defaults to a bare Pico. Presets live in `configs/boards/`; the TOML form is
pin-range strings:

```toml
# Waveshare RP2350-Zero
available = "0-16,26-29"
reserved = "16"   # WS2812 LED
```

`set-board` takes effect immediately for later `set` commits and warns if the
active topology violates the new profile (the firmware falls back to its
default topology at the next boot if the stored one no longer validates).

## Web UI

`webui/` is a static single-page app (Chrome/Edge — it needs WebUSB) with
the CLI's functionality plus a visual topology editor: live pin map, PIO
SM / USB endpoint budget meters, board-profile editing, presets bundled from
`configs/`, a local config library (localStorage), TOML import/export, and
share-by-URL (the config rides in the fragment; nothing touches a server).

It can also flash a bare BOOTSEL-mode board: the page speaks PICOBOOT over
WebUSB (erase / write / verify / reboot), using either the UF2 baked into the
deployment or a file you pick, then provisions the fresh probe with a board
profile and topology in one flow.

```sh
webui/build.sh                          # wasm-pack + assemble webui/dist
python3 -m http.server -d webui/dist    # serve locally (WebUSB works on localhost)
scripts/make-uf2.sh out/                # per-chip + combined UF2s (needs picotool)
webui/build.sh --firmware out/          # ...bake them into the page
```

Releases are tag-driven: pushing `v*` runs `.github/workflows/release.yml`,
which tests, builds both firmwares, publishes the UF2s (per-chip and a
combined RP2040+RP2350 image — a plain concatenation; each bootrom ignores
foreign-family blocks) to a GitHub release, and deploys the site to GitHub
Pages with that firmware baked in. One-time repo setup: Settings → Pages →
Source: "GitHub Actions".

Topology TOML mirrors the `Topology` type — arrays-of-tables of probes and
UARTs (`reset` is optional):

```toml
[[probes]]
swclk = 2
swdio = 3
reset = 1

[[uarts]]
tx = 4
rx = 5
baud = 115200
```

## Stress testing

`tools/stress/` holds bench tools for concurrent-lane testing:

- `mpstress` runs bulk, word, churn and uptime load on every lane at once,
  with per-lane throughput, latency and error accounting.
- `mpdiag` reads the per-lane reply-stall counters. These need firmware
  built with `--features stall-diag`.
- `scripts/usb_stalls.py` finds stalls in usbmon captures.

It is a standalone crate outside the workspace, with its own `cargo build` in
that directory. See `tools/stress/README.md` for usage and baselines
(issue #4).

## Current state

Multi-instance topology read from flash at boot (falling back to the stock
debugprobe pico config: SWD probe SWCLK=GP2/SWDIO=GP3/nRESET=GP1 plus a
CDC-UART bridge on TX=GP4/RX=GP5 @ 115200). CMSIS-DAP v2 per probe over a
vendor bulk interface, WinUSB descriptors, per-instance serial `<uid>:<n>`,
config vendor commands + host CLI. DAP endpoint I/O runs on core 0 next to
the USB device task (all endpoint registers are touched from one core, and
`DAP_Info`/config vendor commands are answered at USB latency); the SWD
execution workers run on core 1 behind per-probe request/response channels
(see issue #3). The CMSIS-DAP command processor is `dap-async`, an async,
SWD-only fork of dap-rs vendored in this workspace; the SWD wire engine is
the C firmware's `probe.pio` command FIFO program, driven by hand-written
futures in `firmware/src/dap.rs` that spin through short FIFO waits and
yield to the other lanes' workers on long ones. So a lane clocking at
100 kHz no longer holds every other lane for the duration of its commands
(issue #5): with two lanes at 20 MHz and four at 100 kHz, the fast lanes'
word round trip is ~1.4 ms instead of ~130 ms, and six lanes at 4 MHz
aggregate ~300 KB/s instead of ~175 (numbers and the bench recipe in
`tools/stress/README.md`). `DAP_Info` advertises a 512-byte packet over
the 64-byte full-speed endpoint (issue #6): a `DAP_TransferBlock` carries
~126 words per USB round trip, which lifts single-lane bulk at 20 MHz from
~200 to ~300 KB/s; the firmware reassembles requests from USB packets using
each command's own length and ends exact-multiple replies with a
zero-length packet. Vendor config commands stay single-packet. The command
processor and the per-request I/O path are polled from SRAM-resident
functions: from flash they cost ~85 XIP-cache misses per command, more
than the command's own work. Transfer semantics follow the
CMSIS-DAP reference implementation (posted AP reads, trailing `RDBUFF` read
after writes, `DAP_TransferConfigure` idle cycles honoured, transfer counts
that exclude the failing transfer).

Official Raspberry Pi Debug Probe (RP2040): `configs/DebugProbe.toml` gives
the stock layout, the D connector as one SWD lane (SWCLK GP12, SWDIO on
GP14) and the U connector as the UART bridge. The U connector's buffering is
unidirectional, so it cannot be a second SWD lane. Validated 2026-10-28 with
the async firmware: bulk at 100 kHz / 4 MHz / 20 MHz and word ops, zero
errors or mismatches; 182 KB/s single-lane bulk and ~765 word ops/s at
20 MHz (the RP2350 does 304 KB/s and ~1180/s).

Hardware-validated on an RP2040 Pico (July 2026): probe-rs walks a Pico 2 W's
CoreSight topology and sustains ~110 KB/s verified RAM read/write at 3 MHz
SWD; topology set/get/commit over USB survives reflash. `bcdDevice` reports
2.20 because probe-rs deny-lists `2e8a:000c` below that (picoprobe era).

Reflashing without the BOOTSEL button: `rustprobe reboot --bootsel` (a bulk
vendor command), then `picotool load -v -x <elf>`. The firmware also carries
the pico-sdk USB reset interface (vendor class FF/00/01, added last), but
stock picotool only reaches its reset-interface scan with explicit
`--vid 0x2e8a --pid 0x000c`: unknown PIDs under the Raspberry Pi VID are
classified (and rejected) before that scan. The reset/CDC interface numbers
move when the probe count changes — find the reset interface by its class
triple, never by a cached number.

CDC-UART bridges (ports `debugprobe/src/cdc_uart.c`): one embassy
`CdcAcmClass` + `BufferedUart` per configured UART, bridged bidirectionally on
the core-0 executor. Host line-coding sets baud live (`BufferedUart` has no
live `set_format`, so data-bit/parity/stop-bit changes only take effect at
construction — see `firmware/src/uart_bridge.rs`); DTR deassert pauses
bridging as the C firmware does. Pin/instance legality is enforced by
`probe_config::Topology::validate` (UART0 TX ∈ {0,12,16,28}, UART1 TX ∈
{4,8,20,24}, RX = the matching bank), so the bridge code assumes a valid
topology.

Autobaud (ports `debugprobe/src/autobaud.{pio,c}`): selecting the magic baud
rate (9728) on a CDC port triggers a PIO edge timer that measures the target's
low-pulse widths; a DMA channel streams the timer FIFO into the
`autobaud-estimator` core (hash-binned pulse-width clustering → baud + validity
score), and a confident estimate is applied to the UART with `set_baudrate`.
The estimator is a pure `no_std` crate with host unit tests (synthetic 8N1
edge-timing samples across several bauds, asserting the estimate lands within
0.5%). The capture state machine is the SM `validate` already reserves when a
UART is configured (the next free SM after the probes); it snoops the UART's RX
GPIO via PIO input without disturbing the UART's ownership of the pin. See
`firmware/src/autobaud.rs`.

Not yet: host-driven UART break (embassy's `CdcAcmClass` doesn't surface CDC
`SEND_BREAK` — `TODO(break)`), RP2350-only alternate UART pins, LEDs,
`DAP_TransferAbort` mid-command (the I/O task collects a command's reply
before reading the next request, so an abort is only seen afterwards).
JTAG commands are answered with `DAP_ERROR`, as the reference does when
JTAG is unavailable; `probe-rs info` (0.31) still fails against this
firmware because its espressif chip detection JTAG-scans first and treats
that error as fatal, while `probe-rs run`/`attach` are unaffected.

Multi-instance hardware validation (Aug 2026, 9-probe RP2350 LA fixture,
issue #3): 7 concurrent `probe-rs run` flash+RTT sessions at 100 kHz SWD ran
to completion, the full `pad_selftest` fleet harness passed 14/14 sessions,
and a 5× `USBDEVFS_RESET` storm fired mid-traffic recovered without a power
cycle (the pre-fix firmware deep-wedged, POR-only). Most of the session
failures originally attributed to probe wedging were the harness's 60 s idle
timeout killing slow 7-way 100 kHz flashes — see issue #3 for the split.
