# Grouped DYNAMIXEL write transport

This package-private plugin copies the Apache-2.0 DYNAMIXEL Hardware Interface
source at `6bb5f93ecad1fe3c231e3ece580344e8f0a6efc3`. `source_manifest.json`
records every original and vendored file hash. Copyright notices and `LICENSE`
are retained. Behavioral patches are `grouped_write.patch` and `read_service.patch`;
C++ namespaces, include paths and plugin registration are isolated from the installed driver.

The supervisor uses this plugin for the arm and end-unit in all three modes. Only the mixed
probe enables `use_grouped_sync_write=true`. It sends the current-controlled joint and position-controlled joints in separate
SyncWrite groups with their existing address/length layouts. Every payload is
validated before transmission; failures clear the pending group buffers.
Groups are sent sequentially, so their commands are not atomic or simultaneous.
All model tables, current/position units and device register definitions still
come from the installed `dynamixel_hardware_interface` package.

`read_service.patch` fixes a queued-read deadlock: the GetDataFromDxl callback runs
inside hardware `read()` and cannot wait for the next `ReadItemBuf()` in that same
thread. It reads directly through the existing bus owner, resolving logical IDs
to configured communication IDs. SDK retries/timeouts bound the transaction;
the request's timeout field is not a separate wall-clock deadline. The runtime
uses these occasional configuration reads before torque ON, not for cyclic data.

HAT power initialization, torque-OFF initialization, mode/unit conversions, and
ordinary homogeneous SyncWrite are inherited unchanged. A separate stock bringup
is unnecessary. No upstream package file is replaced.

The grouped path uses separate SyncWrite transactions for mixed current/position commands.
No firmware watchdog, brake timing, or Goal Update Delay is changed here.

`write_alert.patch` separates Protocol 2's Alert bit from an instruction error.
Alert-only acknowledgements succeed without clearing the drive's Error Code.
Every buffered write is attempted once; one failed axis does not block another
axis's OFF request, and a failed ON is not replayed after a later stop. A failed
write returns ITEM_WRITE_FAIL; only successful acknowledgements update the torque
cache. The supervisor still confirms torque OFF using advancing hardware ticks.

The read-thread service executor is created once per hardware instance. Service callbacks
still execute on the bus-owning thread; no parallel SDK access is introduced. The
end-unit uses the same isolated plugin so its executor setup does not churn at 400 Hz.
Read-stage latency diagnostics distinguish bus, item service, publication and callback time.
