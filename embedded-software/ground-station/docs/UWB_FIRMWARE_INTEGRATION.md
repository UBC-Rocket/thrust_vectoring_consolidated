# UWB Firmware Integration — Design Doc

**Status:** Pending UWB hardware purchase
**Audience:** Firmware author who picks up the rocket-side UWB work
**Written:** 2026-05-17 (after the GCS-side UWB loop landed and the firmware-side was reverted to keep the current rocket safe)
**Companion review notes:** `~/Downloads/gcs_review_2026-05-17.md` (not in repo; the author's session log)

---

## TL;DR

The Ground Control Station (GCS) side of the UWB anchor-layout / position-fix
pipeline is done. The flight-controller side is intentionally NOT implemented
yet — UWB hardware is not purchased, and shipping a half-wired feature would
risk current bench tests and flights. This doc is the handoff: everything the
firmware author needs to wire the loop when the MaUWB modules arrive.

The reference implementation existed briefly in commit `644b1f2 updated
version of GCS` on branch `embedded-software/feat/gcs-map-page` and was
reverted in a subsequent commit. Read that commit's diff for the original
patches — most of what's below restores them with the open questions resolved.

---

## What's already wired (GCS side, on this branch)

These are live. Don't touch them; the firmware author can rely on the wire
format being stable.

### Protocol — `embedded-software/libs/rocket-protocol/proto/tvr/`

```proto
// common.proto
message Vec2 {
  float x = 1;
  float y = 2;
}

// command.proto
message FlightCommand {
  oneof payload {
    StateCommand    state_cmd        = 1;
    SetPidGains     set_pid_gains    = 2;
    SetReference    set_reference    = 3;
    SetConfig       set_config       = 4;
    SetProbeLayout  set_probe_layout = 5;   // ← GCS uplink, FC consumer needed
  }
}

message SetProbeLayout {
  Vec2 anchor_0 = 1;
  Vec2 anchor_1 = 2;
  Vec2 anchor_2 = 3;
  Vec2 anchor_3 = 4;
}

// telemetry.proto — added to existing TelemetryState
message TelemetryState {
  ...
  Vec2 uwb_tag_0 = 10;   // ← FC populates when trilateration produces a fix
  Vec2 uwb_tag_1 = 11;
}
```

Generated headers under `libs/rocket-protocol/generated/tvr/` (regenerated;
do not hand-edit). Regenerate with:

```
cd embedded-software/libs/rocket-protocol
python3 -m venv /tmp/nanopb_venv && /tmp/nanopb_venv/bin/pip install protobuf
cat > /tmp/protoc-gen-nanopb-venv <<'EOF'
#!/bin/bash
exec /tmp/nanopb_venv/bin/python3 \
  $(pwd)/nanopb/generator/nanopb_generator.py --protoc-plugin
EOF
chmod +x /tmp/protoc-gen-nanopb-venv
protoc --plugin=protoc-gen-nanopb=/tmp/protoc-gen-nanopb-venv \
       -I proto/tvr -I nanopb/generator/proto \
       --nanopb_out=generated/tvr proto/tvr/*.proto
```

**Do not pass `--c-style`** — the existing codebase uses CamelCase
(`tvr_Vec2`, `tvr_SetProbeLayout`); `--c-style` would switch to snake_case
and break every consumer.

### GCS encoder — `embedded-software/ground-station/SourceFiles/CommandSender.cpp`

`CommandSender::sendProbeLayout(probes)` encodes the 4 anchor positions
into `tvr_FlightCommand.set_probe_layout` and transmits via
`bridge->sendBinary(txTo, data)`. The QML probe map (`Panel_Probe_Map.qml`)
hands it exactly 4 anchors in the order `A0 = bottom-left, A1 =
bottom-right, A2 = top-right, A3 = top-left`.

### GCS decoder — `embedded-software/ground-station/SourceFiles/SensorDataModel.cpp`

Reads `TelemetryState.uwb_tag_0/1` (with `has_*` validity flags) into
QML-bound properties `uwbTag0X/Y/Valid` and `uwbTag1X/Y/Valid`. The probe
map's geometry canvas draws live red dots at those positions and clears
them when `has_*` is false. CSV recording adds 4 columns: `uwb0_x, uwb0_y,
uwb1_x, uwb1_y`.

### Operational reality today

If the operator clicks **Send** on the probe map right now, the GCS encodes
a valid `SetProbeLayout` packet and writes it to the radio. The
flight-controller decodes the packet, hits the `default:` branch in
`mission_manager.c` (no case for tag 5), and silently discards it. No
crash, no log entry, no behavior change. That's the safe state.

---

## What the firmware author needs to do

Eight pieces of work, roughly in order. Some are independent (parallelize
A+B+C+D); others have dependencies noted.

### A. Restore the `SetProbeLayout` decode case in `mission_manager.c`

**File:** `embedded-software/firmware/ulysses-flight-controller/Core/Src/tasks/mission_manager.c`

**Reference:** commit `644b1f2`, diff hunk on `mission_manager.c`.

Restore four pieces:

1. Forward declaration: `static void handle_probe_layout(const tvr_SetProbeLayout *layout);`
2. Static storage (resolve thread-safety question — see §G below):
   ```c
   static tvr_SetProbeLayout g_probe_layout = tvr_SetProbeLayout_init_zero;
   ```
3. Switch case in `mission_manager_task_start` (under the radio RX dispatch):
   ```c
   case tvr_FlightCommand_set_probe_layout_tag:
       cmd_rx_count++;
       handle_probe_layout(&decoded.payload.set_probe_layout);
       break;
   ```
4. `handle_probe_layout` function body — copy into `g_probe_layout`, write
   to SD log (depends on B), DLOG_PRINT for visibility:
   ```c
   static void handle_probe_layout(const tvr_SetProbeLayout *layout) {
       if (layout == NULL) return;
       g_probe_layout = *layout;   // ← see §G for sync
       log_service_log_probe_layout(...);   // ← see §B
       DLOG_PRINT("[MM] SetProbeLayout received\r\n");
   }
   ```

Optional: keep the `default:` branch silent (matches pre-session
behavior). The reverted commit had a `DLOG_PRINT` on unknown tags — fine
for development, low value in flight.

### B. Restore the `probe_layout` SD log record

**Files:**
- `embedded-software/firmware/libs/log_records/include/log_records/log_records.h`
- `embedded-software/firmware/tools/sd-log/log_schema.py` (auto-regenerated)

**Reference:** commit `644b1f2`, diff hunk on `log_records.h`.

1. Bump `LOG_SCHEMA_VERSION` from 5 to 6.
2. Add the field macro after `LOG_RECORD_FIELDS_CONFIGURATION`:
   ```c
   #define LOG_RECORD_FIELDS_PROBE_LAYOUT(FIELD) \
       FIELD(uint32_t, timestamp_us) \
       FIELD(bool, has_anchor_0) \
       FIELD(float, anchor_0_x) \
       FIELD(float, anchor_0_y) \
       FIELD(bool, has_anchor_1) \
       FIELD(float, anchor_1_x) \
       FIELD(float, anchor_1_y) \
       FIELD(bool, has_anchor_2) \
       FIELD(float, anchor_2_x) \
       FIELD(float, anchor_2_y) \
       FIELD(bool, has_anchor_3) \
       FIELD(float, anchor_3_x) \
       FIELD(float, anchor_3_y)
   ```
3. Add to `LOG_RECORD_LIST`:
   ```c
   APP(0x0F, probe_layout, LOG_RECORD_FIELDS_PROBE_LAYOUT, LOG_ENABLE_ALWAYS) \
   ```
4. Regenerate the Python decoder mirror:
   ```
   cd embedded-software/firmware/tools/sd-log && python3 generate_log_schema.py
   ```

The auto-generated `log_service_log_probe_layout()` call site goes in
`handle_probe_layout` (§A piece 4) — copy the field-by-field assignment
from commit `644b1f2`.

### C. Land the MaUWB driver into the firmware build

**Source branch:** `embedded-software/uwb_driver/stage` (two commits ahead of
the GCS branch in firmware-only paths)

**Files on that branch:**
- `firmware/libs/sensors/include/sensors/mauwb.h` — bus-agnostic AT-command
  builders + response parsers
- `firmware/libs/sensors/src/mauwb.c` — implementation
- `firmware/ulysses-flight-controller/Core/Inc/uwb_drivers/mauwb_device.h` —
  STM32 wrapper struct
- `firmware/ulysses-flight-controller/Core/Src/uwb_drivers/mauwb_device.c` —
  init sequence (AT? → AT+RESTORE → AT+SETCFG → AT+SETCAP → AT+SETRPT(1) →
  AT+SAVE → AT+RESTART)

**What's missing on that branch (per the author's latest commit message):**
"need to work on the receival part by using circular dma plus CHARACTER
MATCHING INTERRUPT". The driver currently can only send AT commands and
parse OK/ERROR responses. It does not yet ingest ranging frames.

**Work to do:**
1. Merge `embedded-software/uwb_driver/stage` into the integration branch.
2. Add `mauwb.c` to `firmware/libs/sensors/CMakeLists.txt` (currently lists
   only bmi088_accel/gyro and ms5611/5607 baros).
3. Add `mauwb_device.c` to the flight-controller's CMakeLists (mirror the
   pattern used by `spi_drivers/bmi088_accel_device.c`, etc.).
4. Complete the RX path: USART circular DMA + character-match interrupt
   on `\n` (NMEA-style line terminator). The driver author's plan.
5. Parse the MaUWB ranging-report format into a structured representation.
   Check the MaUWB documentation for the exact format; the driver's AT
   command set hints at `AT+RANGE` and `AT+RDATA` for reading.

### D. Wire the MaUWB driver into the bring-up flow

**File:** `firmware/ulysses-flight-controller/Core/Src/sensors_init.c` (or
wherever the rest of the sensor inits live)

Call `mauwb_device_init(&dev, &huart_X, timeout_ms, &config)` then
`mauwb_device_setup(&dev)` during board init. The on-board UWB tags
need:

- `device_id` — choose an ID per on-board tag (0 and 1, or 100 and 101 —
  must not collide with ground anchor IDs)
- `role` — `MAUWB_ROLE_TAG`
- `rate` — `MAUWB_RATE_6M8` (or 850K; tune for range vs. throughput)
- `tag_capacity` — number of tags in the system (2 on rocket + ?
  off-rocket). Resolve with §H below.

There are two on-board UWB modules (per the driver author's note about
USART1 + USART6). Currently the driver uses static global state; the
driver author plans to refactor into structs/instances when the new board
is ready.

### E. Bring up the trilateration consumer

**This is the new code, not a restore.** No reference in commit `644b1f2`
— it was intentionally left as a TODO ("see embedded-software/uwb_driver/
stage MaUWB driver, which still needs the RX path completed before it
can produce ranging frames").

**Inputs:**
- `g_probe_layout` from §A (4 ground anchor positions, GCS-supplied)
- Per-tick ranging measurements from §C (range from each on-board tag to
  each ground anchor — 4 ranges × 2 tags = 8 ranges per cycle)

**Output:**
- 2 on-board tag positions in nav-frame meters
- Validity flag per tag (false if the most recent ranging cycle didn't
  produce a usable fix — e.g., < 3 anchor returns in range)

**Algorithm:** standard multilateration. With 4 anchors in a known
geometry and 4 range measurements per tag, the system is over-determined
— use least-squares (e.g., Gauss-Newton or a Kalman-style update). For a
first pass, a closed-form trilateration using 3 anchors and discarding
the 4th as a sanity check is sufficient.

**Where to run it:** new FreeRTOS task, or extend an existing task.
Trilateration at ~10 Hz is cheap (handful of float matrix ops). Don't run
it in an ISR.

**Coordinate frame to match the GCS expectation:**
- Origin: rocket takeoff point (operator-defined, marked as (0,0) in the
  probe-map UI)
- Axes: see §G open question 1 below

### F. Publish the UWB fix into TelemetryState

**File:** `firmware/ulysses-flight-controller/Core/Src/tasks/mission_manager.c`
— specifically `send_telemetry()`

`TelemetryState.uwb_tag_0` and `uwb_tag_1` are optional `Vec2` fields. The
nanopb encoder uses `has_uwb_tag_N` to decide whether to serialize them.

Pattern (mirrors how `position`, `velocity`, etc. are set today):

```c
telem.has_uwb_tag_0 = uwb_fix_0.valid;
if (uwb_fix_0.valid) {
    telem.uwb_tag_0.x = uwb_fix_0.x;
    telem.uwb_tag_0.y = uwb_fix_0.y;
}
telem.has_uwb_tag_1 = uwb_fix_1.valid;
if (uwb_fix_1.valid) {
    telem.uwb_tag_1.x = uwb_fix_1.x;
    telem.uwb_tag_1.y = uwb_fix_1.y;
}
```

Where `uwb_fix_N` is published by the trilateration task (§E) via
`state_exchange` (the existing inter-task data pattern).

### G. Resolve five open questions before merging

These were flagged during GCS-side review and intentionally deferred.
Resolve at integration time so the firmware and GCS agree.

**Q1. Coordinate-frame convention.** TelemetryState.uwb_tag_0/1 are
documented as "nav-frame meters". Pick one and document on
`telemetry.proto`:

- Origin: rocket takeoff point (probe-map (0,0))? Anchor centroid?
- Axes: +x = east, +y = north (geographic)? Or rocket-body convention?
- Units: meters (assumed)

**The GCS overlay maps `uwb_tag_N.{x,y}` directly to the same meters used
for anchor positions.** Whatever convention the firmware picks must match
the operator's mental model of the probe-map rectangle.

**Q2. SetProbeLayout cadence.** Today the GCS only sends on operator
button click. Options:

- (a) Firmware treats the most-recent layout as authoritative; ignore-if-
  not-received (current GCS-side stub behavior)
- (b) Require a layout before ARM (firmware refuses to arm without one)
- (c) GCS periodically re-broadcasts in case the rocket missed it

Recommendation: (b) is safest for launch ops; the firmware can publish a
`pre_arm_check_failed` event if `g_probe_layout` is all-zero when the ARM
command arrives. The GCS already prints "SetProbeLayout sent (4 anchors)"
to confirm send; an explicit pre-arm rejection closes the loop.

**Q3. Anchor ID ↔ corner mapping.** The proto labels anchors 0-3 but the
physical MaUWB units have configurable device IDs (set via `AT+SETCFG`).
Two options:

- (a) **Fixed convention:** device ID N is always corner N. Operator
  physically labels the four ground units 0/1/2/3 to match the
  probe-map corners. No wire changes needed.
- (b) **Configurable in GCS:** add a device-ID field per pole to the
  wire format. Operator picks the ID per corner via the probe map UI.

Recommendation: (a) for first launch. Document on `SetProbeLayout` proto
that anchor_N corresponds to device ID N.

**Q4. SetProbeLayout RAM-state synchronization.** `g_probe_layout` is
written by `mission_manager_task` and will be read by the trilateration
task (§E). Two FreeRTOS tasks, one writer + one reader, 52-byte struct.
Without a primitive, the reader can see torn state mid-copy. Options:

- (a) FreeRTOS mutex — simplest, low overhead at 10 Hz
- (b) Double-buffer + atomic pointer swap — lock-free, faster
- (c) Lock-free with full struct atomic — not portable in C

Recommendation: (a). Initialize the mutex in
`mission_manager_task_start`, take/give around reads and writes. The
update rate is 1 Hz worst case (operator clicks Send); the read rate is
10 Hz (matches trilateration). Negligible contention.

**Q5. Boot recovery.** After a reboot in flight, `g_probe_layout`
re-zeros. PID gains, reference setpoints, and vehicle config have the
same limitation today (firmware logs them but doesn't restore from log
on boot). Two paths:

- (a) Match the existing pattern — log on receipt only; operator must
  re-send after any reboot. Documented in `handle_probe_layout`'s
  comment block.
- (b) On boot, scan the SD log for the most recent `probe_layout`
  record and restore `g_probe_layout` from it.

Recommendation: (a). In-flight reboots are already a pre-arm-abort
scenario; the additional re-send is one operator click. (b) is feasible
but adds a boot-time SD scan that's only valuable in a narrow scenario.

---

## End-to-end test plan (when hardware lands)

Before the first integrated test, verify each layer in isolation:

1. **GCS standalone:** open the probe map, drag a rectangle, click Send,
   confirm `messageSent` signal fires. (Already verified — GCS works
   without any firmware response today.)

2. **Firmware loopback:** flash firmware with §A+B implemented. From a
   serial monitor, hand-craft a `SetProbeLayout` packet (use the
   nanopb encoder or capture one from GCS via the radio console).
   Inject over the radio link. Verify:
   - `cmd_rx_count` increments
   - `[MM] SetProbeLayout received` line appears in debug UART
   - SD log contains a `0x0F` record with the right 4 anchors after
     `decode_log.py`

3. **MaUWB bring-up:** wire the four ground anchors at known surveyed
   positions. Bring up the on-board tags via §D. Verify:
   - `mauwb_device_setup()` returns `MAUWB_DEV_OK`
   - `AT+RANGE` produces frames on the configured USART
   - Range measurements are sane (~ Euclidean distance to each anchor)

4. **Trilateration:** with anchors at known positions and the rocket
   stationary, run §E. Verify the computed `(x, y)` matches a tape
   measurement to < 30 cm.

5. **End-to-end:** GCS sends layout → firmware stores it → trilateration
   uses it → telemetry returns `uwb_tag_0/1` → GCS overlay shows red
   dots in roughly the right place. Move the rocket; verify the dots
   move correspondingly on the probe map.

6. **Edge cases:**
   - GCS sends layout while rocket is in ESTOP → confirm decode still
     happens, layout still stored, but trilateration output is
     irrelevant (or held at last valid).
   - Only 3 anchors visible (one obstructed) → trilateration should
     still produce a fix or mark it invalid.
   - All 4 anchors invalid → `has_uwb_tag_*` = false; GCS overlay
     disappears.
   - Operator re-sends layout mid-flight → firmware accepts, takes
     mutex, swaps `g_probe_layout`, trilateration continues with new
     reference.

---

## Files that will change vs. files already done

### Firmware-side (this work)

```
M  embedded-software/firmware/libs/log_records/include/log_records/log_records.h
M  embedded-software/firmware/tools/sd-log/log_schema.py            (auto-regen)
M  embedded-software/firmware/ulysses-flight-controller/Core/Src/tasks/mission_manager.c
M  embedded-software/firmware/libs/sensors/CMakeLists.txt           (add mauwb.c)
M  embedded-software/firmware/ulysses-flight-controller/CMakeLists.txt   (add mauwb_device.c)
M  embedded-software/firmware/ulysses-flight-controller/Core/Src/sensors_init.c   (call mauwb_device_setup)
A  embedded-software/firmware/ulysses-flight-controller/Core/Src/tasks/trilateration.c   (new task or module)
A  embedded-software/firmware/ulysses-flight-controller/Core/Inc/tasks/trilateration.h
```

Plus whatever merges in from `embedded-software/uwb_driver/stage`.

### GCS-side (no change needed)

```
embedded-software/ground-station/SourceFiles/CommandSender.cpp        ← sendProbeLayout works as-is
embedded-software/ground-station/SourceFiles/SensorDataModel.cpp      ← decoder + overlay live
embedded-software/ground-station/QMLFiles/Panels/Panel_Probe_Map.qml  ← UI live
embedded-software/libs/rocket-protocol/proto/tvr/*.proto              ← wire format live
embedded-software/libs/rocket-protocol/generated/tvr/*.pb.{c,h}       ← regenerated
```

When the firmware is rebuilt against the existing regenerated headers,
the new `tvr_SetProbeLayout` and `tvr_Vec2` types are available
immediately — no further proto work.

---

## References

- **Companion review notes:** `~/Downloads/gcs_review_2026-05-17.md`
  (the session log; §§3-7 cover the GCS-side decisions, §14 records the
  firmware revert)
- **Reference commit:** `644b1f2 updated version of GCS` — the
  three firmware files restored from this commit are the starting point
  for §§A and B above
- **MaUWB driver work-in-progress branch:** `embedded-software/uwb_driver/stage`
- **MaUWB documentation:** see Makerfabs MaUWB DW3000 product page (link
  in `firmware/libs/sensors/include/sensors/mauwb.h` header comment)
- **GCS probe map UI:** `QMLFiles/Panels/Panel_Probe_Map.qml` — read the
  legend + `recomputeBasePoles` to understand the anchor ordering
  convention
