# dap-async

Async, SWD-only CMSIS-DAP v2 command processor for the rustprobe firmware
(issue #5). The hardware driver implements the `Driver` trait with `async fn`
methods, so a probe's SWD transfer can yield to other probes' tasks while its
bits are on the wire.

Derived from [dap-rs](https://github.com/probe-rs/dap-rs) (probe-rs project,
revision `a867ca7`, MIT OR Apache-2.0, by Emil Fresk and Adam Greig): the
command set, request/response layout and the `DAP_SWD_Sequence` tests come
from there. The transfer loops (`DAP_Transfer`, `DAP_TransferBlock`) follow
the ARM CMSIS-DAP reference implementation (`DAP.c`), including posted AP
reads, the trailing `RDBUFF` read after writes, and transfer counts that
exclude the failing transfer.

Dropped relative to dap-rs: JTAG and SWO (answered with `DAP_ERROR`), the
`usb-device` dependency, the type-state mode machine. Added: bounds-checked
request parsing (a truncated request yields `DAP_ERROR`, never a panic),
`DAP_SWD_Configure` turnaround and data-phase settings, honoured
`DAP_TransferConfigure` idle cycles.
