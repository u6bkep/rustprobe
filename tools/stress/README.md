# rustprobe stress tools

Host-side tools for loading every lane of a rustprobe at once and measuring what
breaks. They were built for the concurrent-lane investigation in issue #4. This
is a standalone crate (its own `[workspace]`, excluded from the firmware
workspace): probe-rs and nusb are heavy, and only a bench needs them.

| Tool | What it does |
|---|---|
| `mpstress` | One thread per lane running bulk / word / churn / uptime / hold load; per-lane throughput, latency percentiles, errors, slow ops; optional per-lane probe-rs trace ring dumped on every error. |
| `mpdiag` | Reads the per-lane reply-stall counters from firmware built with `--features stall-diag` (DAP_Info id 0xA0; see `firmware/src/stall_diag.rs`). |
| `scripts/usb_stalls.py` | Finds stalled bulk transfers in a usbmon capture (needs `tshark`). |

## Safety

Everything `mpstress` does is non-halting and leaves the targets alone, so it can
run against live firmware:

- **bulk** block-reads the boot ROM (immutable) and checks it against a golden
  copy.
- **word** writes and reads back core 0's DCRDR, a debug-only scratch register.
- **churn** attaches with default permissions and reads one ROM word.
- **uptime** reads the raw timer halves (TIMERAWH/TIMERAWL), never the latched
  TIMEHR/TIMELR pair the target firmware may be using.

Supported targets: `--chip rp2350` (default) and `--chip rp2040`. That covers
the ROM size, the timer address and the probe-rs chip name.

## Build and run

```sh
cd tools/stress
cargo build --release              # target/release/{mpstress,mpdiag}

# which lanes have targets: `rustprobe get` (topology) plus your bench map
./target/release/mpstress --lanes 0,1,2 --khz 4000 --secs 30
./target/release/mpstress --lanes 0::20000,1:word:20000,5::100   # mixed speeds
./target/release/mpstress --lanes 0:churn,1:churn,2:churn --khz 4000 --secs 1800 \
    --trace 'probe_rs::probe::cmsisdap=trace' --trace-dir traces
./target/release/mpdiag             # firmware built with --features stall-diag
```

Lane specs are `LANE[:MODE[:KHZ]]`, where LANE is the probe index, which is also
its CMSIS-DAP interface number. `--serial` is needed only when more than one
rustprobe is attached. Each lane's summary line goes to stderr, then one JSON
object per lane goes to stdout. `--slow-ms` (default 500) reports any single op
slower than that; in churn mode it applies to the whole cycle, with per-phase
timing (open / speed / attach / read / detach). Every event and error carries
a wall-clock timestamp, so it lines up with trace dumps and usbmon captures.

## Reading the results

- **`MISMATCH`** means data corruption. Nothing has ever produced one.
- **probe-rs 0.31 errors after a reply stall:** `Timeout in USB communication`,
  then `Unexpected answer to command`, `Only N/M transfers were executed`, or
  `Transfer count larger than requested`. Each reply is now one command
  behind. A `detach` of about 1000 ms in a slow churn cycle is the same thing.
  Use `--trace` to see the raw request and reply bytes.
- **`Target device did not respond` / `FAULT` on one lane only, even when it
  runs alone:** the target is the problem, not the probe. Check with `uptime`
  mode whether it is rebooting.
- **`Not enough data in response from probe`:** a dap-rs bug that hides the
  real ACK (issue #4, section 4).

## Stall counters (`stall-diag` firmware)

Build the firmware with the `stall-diag` feature. For example, RP2350:
`cargo build -p rustprobe-firmware --target thumbv8m.main-none-eabihf
--no-default-features --features rp2350,stall-diag --release`. Each lane then
counts:

- requests handed to its SWD worker;
- worker answers and request-to-reply-armed times over 100 ms, plus the maxima;
- requests that arrived while the lane's previous reply was still armed and
  uncollected, with that reply's age.

`mpdiag` prints them. Lanes held by another process are skipped, so run it
after `mpstress` exits. If `unread` is non-zero while `slow_*` is zero, the
probe answered in time and the host or USB path didn't fetch the reply. A
non-zero `slow_*` means the probe itself was late. Stock firmware returns no
counters.

## Capturing USB (usbmon)

```sh
lsusb | grep 2e8a:000c                     # "Bus BBB Device DDD"
dumpcap -i usbmon<BBB> -s 64 -b filesize:300000 -b files:20 -w cap.pcapng &
# ... run mpstress ...
scripts/usb_stalls.py <DDD> cap_*.pcapng --context 5
```

Reading `/dev/usbmon*` needs the `wireshark` group (or root). At `-s 64` a
6-lane churn run fills about 300 MB every 1.5 minutes. `/tmp` is often RAM, so
size the ring accordingly.

## Baselines (fw 0.1.2, RP2350-Zero, 9 lanes, RP2350 targets; issue #4)

Bulk ROM read, KB/s:

| SWD clock | 1 lane | aggregate, 6 lanes |
|---|---|---|
| 100 kHz | 5.6 | — |
| 1 MHz | 43.5 | 55 |
| 4 MHz | 97 | 168 |
| 20 MHz | 150 | 358 |

- **One lane above ~4 MHz** is bound by the USB round trip. The aggregate is
  bound by core 1, which runs every lane's SWD; lanes share it round-robin, one
  DAP command each.
- **Head-of-line blocking.** One 100 kHz lane drags a 20 MHz neighbour from
  120 to 6.4 KB/s; its word round trip goes from 1.7 to 38 ms.
- **Churn (6 lanes, 4 MHz).** About 7–9.5 open/attach/close cycles per second
  per lane; attach p50 105 ms on a root port.
- **Reply stalls depend on the USB path, not the firmware:**

  | Path | Stalls |
  |---|---|
  | Thunderbolt dock xHCI (JHL7440) + 4 hub tiers | ~1 per 4–6k churn cycles |
  | Laptop root port | 0 in 102k cycles |

  The device-side counters show the firmware arms every reply within about
  1.2 ms. Treat a stalling bench as a USB-path problem first.
