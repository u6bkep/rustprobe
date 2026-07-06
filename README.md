# rustprobe

Rust rewrite of the multiprobe debugprobe firmware: N SWD probes + M CDC-UART
bridges on RP2040/RP2350, with the topology configured at boot from flash
instead of at compile time.

## Workspace

| Crate | Purpose |
|---|---|
| `probe-config` | `no_std` config schema + validation, shared by firmware and host tools |
| `autobaud-estimator` | `no_std` baud-rate estimator core (raw edge-timing samples in, baud out), host-testable |
| `firmware` | embassy-based probe firmware (features: `rp2040` \| `rp2350`) |
| `cli` | `rustprobe` host CLI: configure the probe over USB (nusb + `probe-config`) |

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

## Current state

Multi-instance topology read from flash at boot (falling back to the stock
debugprobe pico config: SWD probe SWCLK=GP2/SWDIO=GP3/nRESET=GP1 plus a
CDC-UART bridge on TX=GP4/RX=GP5 @ 115200). CMSIS-DAP v2 per probe over a
vendor bulk interface, WinUSB descriptors, per-instance serial `<uid>:<n>`,
config vendor commands + host CLI, core-1 execution of the DAP tasks. The SWD
wire engine is the C firmware's `probe.pio` command FIFO program driven
through dap-rs's `Swd`/`Dependencies` traits.

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
`SEND_BREAK` — `TODO(break)`), RP2350-only alternate UART pins, LEDs.
