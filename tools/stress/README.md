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
  uncollected, with that reply's age;
- core-1 time spent inside `process_command` for the lane, summed (`busy_us`,
  wraps after ~71 min) and max; `mpdiag` derives `mean_busy_us` per command;
- request-to-answer time as core 0 sees it (`worker_us`, summed; `mean_worker_us`)
  and its two cross-core halves, hand-off to worker start (`inbound_us`) and
  worker end to pickup (`outbound_us`), so `worker - busy - in - out` is the
  worker's own queueing behind other lanes;
- `fifo_yields`: PIO FIFO waits that outlived the spin budget and yielded to
  the other lanes (global across lanes, reported on every lane).

`mpdiag` prints them; `mpdiag --reset` zeroes every lane first, so the usual
sequence is reset, run `mpstress`, read. Lanes held by another process are
skipped, so run it after `mpstress` exits. If `unread` is non-zero while `slow_*` is zero, the
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

## Baselines (RP2350-Zero fixture, RP2350 targets, Genesys hub on an AMD root hub; issue #4/#5)

`--size 4096 --secs 20`, lanes 0, 1, 2, 5, 7, 8. "async" is the issue-#5
firmware (dap-async + yielding workers); 0.1.2 is the blocking engine.

Bulk ROM read, KB/s:

| SWD clock | 1 lane 0.1.2 | 1 lane async | 6 lanes 0.1.2 | 6 lanes async |
|---|---|---|---|---|
| 100 kHz | 5.8 | 6.7 | — | — |
| 1 MHz | 47.4 | 52.9 | 55.6 | 194 (5 lanes; one attach failed) |
| 4 MHz | 118.9 | 124.1 | 175 | 303 |
| 20 MHz | 201.8 | 203.4 | 391 | 466 |

Word op (DCRDR write + readback) on one 20 MHz lane: p50 771 µs / 1284 op/s
(0.1.2) → 723 µs / 1374 op/s (async).

Head-of-line blocking: lane 0 bulk @ 20 MHz, lane 1 word @ 20 MHz, slow
lanes bulk @ 100 kHz.

| Setup | lane 0 bulk KB/s 0.1.2 → async | lane 1 word p50 0.1.2 → async |
|---|---|---|
| 2 fast | 187 → 194 | 1.23 → 1.15 ms |
| + 1 slow | 6.0 → 175 | 37.3 → 1.26 ms |
| + 4 slow | 1.9 → 158 | 130.6 → 1.42 ms |

- **One lane above ~4 MHz** is bound by the USB round trip; behind a hub
  that round trip is quantized to 125 µs microframes, so single-lane
  numbers move in steps with device time.
- **Core-1 cost per 14-word block at 20 MHz** (`busy_us`): 72 µs (0.1.2) vs
  ~83 µs (async), of which ~40 µs is wire time; the cross-core hand-off
  (`inbound_us` + `outbound_us`) is ~13 µs per command.
- **Churn (6 lanes, 4 MHz, 0.1.2).** About 7–9.5 open/attach/close cycles per
  second per lane; attach p50 105 ms on a root port.
- **Reply stalls depend on the USB path, not the firmware:**

  | Path | Stalls |
  |---|---|
  | Thunderbolt dock xHCI (JHL7440) + 4 hub tiers | ~1 per 4–6k churn cycles |
  | Laptop root port | 0 in 102k cycles |

  The device-side counters show the firmware arms every reply within about
  1.2 ms. Treat a stalling bench as a USB-path problem first.
