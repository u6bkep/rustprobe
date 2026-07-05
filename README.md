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

## Current state

Multi-instance topology read from flash at boot (falling back to the stock
debugprobe pico config: SWD probe SWCLK=GP2/SWDIO=GP3/nRESET=GP1 plus a
CDC-UART bridge on TX=GP4/RX=GP5 @ 115200). CMSIS-DAP v2 per probe over a
vendor bulk interface, WinUSB descriptors, per-instance serial `<uid>:<n>`,
config vendor commands + host CLI, core-1 execution of the DAP tasks. The SWD
wire engine is the C firmware's `probe.pio` command FIFO program driven
through dap-rs's `Swd`/`Dependencies` traits.

CDC-UART bridges (ports `debugprobe/src/cdc_uart.c`): one embassy
`CdcAcmClass` + `BufferedUart` per configured UART, bridged bidirectionally on
the core-0 executor. Host line-coding sets baud live (`BufferedUart` has no
live `set_format`, so data-bit/parity/stop-bit changes only take effect at
construction — see `firmware/src/uart_bridge.rs`); DTR deassert pauses
bridging as the C firmware does. Pin/instance legality is enforced by
`probe_config::Topology::validate` (UART0 TX ∈ {0,12,16,28}, UART1 TX ∈
{4,8,20,24}, RX = the matching bank), so the bridge code assumes a valid
topology.

Not yet: autobaud (the `MAGIC_BAUD` 9728 line-coding trigger is stubbed with a
`TODO(autobaud)` hook), host-driven UART break (embassy's `CdcAcmClass`
doesn't surface CDC `SEND_BREAK` — `TODO(break)`), RP2350-only alternate UART
pins, LEDs.
