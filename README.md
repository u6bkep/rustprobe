# rustprobe

Rust rewrite of the multiprobe debugprobe firmware: N SWD probes + M CDC-UART
bridges on RP2040/RP2350, with the topology configured at boot from flash
instead of at compile time.

## Workspace

| Crate | Purpose |
|---|---|
| `probe-config` | `no_std` config schema + validation, shared by firmware and host tools |
| `firmware` | embassy-based probe firmware (features: `rp2040` \| `rp2350`) |

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

## Current state (spike)

Single SWD probe, hardcoded pins (SWCLK=GP2, SWDIO=GP3, nRESET=GP1 — probe 0
of the C multiprobe pico config), CMSIS-DAP v2 over one vendor bulk
interface, WinUSB descriptors, per-instance serial `<uid>:<n>`. The SWD wire
engine is the C firmware's `probe.pio` command FIFO program driven through
dap-rs's `Swd`/`Dependencies` traits.

Not yet: multi-instance topology, flash-stored config (sequential-storage),
config vendor commands + host CLI, CDC-UART bridges, autobaud, core-1
execution, LEDs.
