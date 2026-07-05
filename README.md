# rustprobe

Rust rewrite of the multiprobe debugprobe firmware: N SWD probes + M CDC-UART
bridges on RP2040/RP2350, with the topology configured at boot from flash
instead of at compile time.

## Workspace

| Crate | Purpose |
|---|---|
| `probe-config` | `no_std` config schema + validation, shared by firmware and host tools |
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

# Host-side tests
cargo test -p probe-config
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
rustprobe reboot                      # reboot the probe
```

Use `--serial <uid:0>` (from `rustprobe list`) to pick one when several probes
are attached. `set` validates locally against the chip limits reported by
`info` before staging; the firmware re-validates on commit.

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

## Current state (spike)

Single SWD probe, hardcoded pins (SWCLK=GP2, SWDIO=GP3, nRESET=GP1 — probe 0
of the C multiprobe pico config), CMSIS-DAP v2 over one vendor bulk
interface, WinUSB descriptors, per-instance serial `<uid>:<n>`. The SWD wire
engine is the C firmware's `probe.pio` command FIFO program driven through
dap-rs's `Swd`/`Dependencies` traits.

Not yet: multi-instance topology, flash-stored config (sequential-storage),
config vendor commands + host CLI, CDC-UART bridges, autobaud, core-1
execution, LEDs.
