#!/usr/bin/env python3
"""Find stalled bulk transfers to a rustprobe in usbmon captures.

usage: usb_stalls.py DEVADDR CAPTURE.pcapng... [--threshold S] [--context MS]

A stall is a bulk URB whose completion came more than THRESHOLD seconds
(default 0.5) after its submit. For CMSIS-DAP that is a reply the host did
not get: probe-rs gives up after 1 s and cancels (status -2), and in probe-rs
0.31 every later reply is then one command behind.

DEVADDR is the probe's USB address on the captured bus (`lsusb`: "Bus BBB
Device DDD"). Capture the bus with e.g.
    dumpcap -i usbmon<BBB> -s 64 -b filesize:300000 -b files:20 -w cap.pcapng
(-s 64 keeps URB headers only; usbmon access needs the `wireshark` group or root).

--context MS prints every bulk URB event to the probe within MS ms before
each stall's submit and after its completion, to see what the other lanes
(endpoints) were doing.
"""
import argparse
import subprocess
import sys


def urb_events(dev, path):
    """Yield (time, 'S'|'C', endpoint, status, urb_id, data_len) for bulk URBs to dev."""
    out = subprocess.run(
        ["tshark", "-r", path, "-Y", f"usb.device_address=={dev} && usb.transfer_type==3",
         "-T", "fields", "-E", "separator=|",
         "-e", "frame.time_epoch", "-e", "usb.urb_type", "-e", "usb.endpoint_address",
         "-e", "usb.urb_status", "-e", "usb.urb_id", "-e", "usb.data_len"],
        capture_output=True, text=True, check=True).stdout
    for line in out.splitlines():
        p = line.split("|")
        if len(p) != 6:
            continue  # dissector banners etc.
        try:
            yield float(p[0]), p[1].strip("'"), p[2], int(p[3]), p[4], int(p[5] or 0)
        except ValueError:
            continue


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dev", help="USB device address of the probe")
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--threshold", type=float, default=0.5, help="stall threshold, s")
    ap.add_argument("--context", type=float, default=0, help="show events within this many ms around stalls")
    a = ap.parse_args()

    events = []
    for path in a.captures:
        events.extend(urb_events(a.dev, path))
    events.sort()

    pending, stalls, total = {}, [], 0
    for t, kind, ep, status, urb, _ in events:
        if kind == "S":
            pending[urb] = (t, ep)
        elif urb in pending:
            t_sub, ep_sub = pending.pop(urb)
            total += 1
            if t - t_sub > a.threshold:
                stalls.append((t_sub, t, ep_sub, status))

    span = events[-1][0] - events[0][0] if events else 0
    print(f"{total} completed URBs over {span:.0f} s, {len(stalls)} stalled (> {a.threshold} s)")
    for t_sub, t_done, ep, status in stalls:
        print(f"  {t_sub:.6f} ep {ep} {t_done - t_sub:.3f} s status {status}")
        if a.context:
            w = a.context / 1000
            for t, kind, e, st, _, n in events:
                if t_sub - w <= t <= t_sub + 0.001 or t_done - 0.001 <= t <= t_done + w:
                    ref = t - t_sub if t <= t_sub + 0.001 else t - t_done
                    anchor = "submit" if t <= t_sub + 0.001 else "done"
                    print(f"      {anchor}{ref * 1000:+9.3f} ms  {kind} ep {e} status {st:>4} len {n}")
    return 1 if stalls else 0


if __name__ == "__main__":
    sys.exit(main())
