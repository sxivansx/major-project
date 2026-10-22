# Alignment with the hardware plan (plan.md, 22 Oct 2026)

How the hardware teammate's plan maps onto this repo, what already exists, what differs, and what needs a decision.

## Already built here (plan → repo)

| Plan item | Where it lives | State |
|---|---|---|
| Laptop ingestion service, local API/WebSocket, connection state, packet rate, health | `core/obslam/server.py`, `rig.py` | done, 30 tests |
| Calibration file: mount transform, servo zero, axis direction, limits | `core/config.json` via `GET/PUT /api/config`, edited in the dashboard "Mount and filters" panel | done; microphone geometry not yet |
| Polar → 3D using the tilt angle, reject invalid ranges and unsettled points | `geometry.py`, `rig.py` (first rotation after settle is discarded) | done; encoder angle being added now |
| Scan controller: move → settle → dwell → repeat, progress, point count | `rig.py` `JobParams` / `_run_job` | done |
| Raw session retained, PLY/CSV export, replay without hardware | `session.py` (`meta.json` + `sweeps.npz`), `/sessions/{id}/load` | done |
| Browser dashboard: 2D/3D views, tilt, progress, session controls, live/replayed indication | `web/` | done |
| Unit tests for frame codecs incl. corrupted/truncated frames; simulated device; synthetic transform tests | `core/tests/`, `driver/check_frames.py`, `web/scripts/check.ts` | done for the laptop-side formats |
| Faults visible to the operator | alarms watchdog, status bar, log | done |

## Differences

### 1. Where the LiDAR bytes go (the big one)

Plan: `S2M1 → UART → ESP32-S3 → Wi-Fi → laptop`. Built: `S2M1 → Slamtec USB adapter → laptop`, decoded by Slamtec's own SDK in `driver/bridge`.

Both can coexist. `ScanSource` in `core/obslam/sources.py` is an interface; the USB bridge and the simulator are two implementations, a Wi-Fi one is a third. Recommendation, in order:

- **Tonight and Milestone 1:** USB adapter straight into the laptop. Zero new code, validates the LiDAR, the mount geometry, the servo, the encoder and the whole pipeline.
- **Milestone 3 option A (recommended):** the S3 bridges the LiDAR UART to a TCP socket *transparently* and runs the framed protocol from the plan on a second socket for servo/encoder/audio/heartbeat. The Slamtec SDK already supports a TCP channel, so `s2bridge` gains a `--tcp host:port` flag and nothing else on the laptop changes. The S2's packet format stays Slamtec's problem, which matters: the S-series "dense" scan format is undocumented and no open Python decoder exists.
- **Option B:** the S3 parses RPLIDAR packets itself and sends `LIDAR_PACKET` frames. This means porting the S2 decoder to the S3 and writing a second decoder on the laptop. Only worth it if S3-side timestamps per measurement are truly needed; for move-settle-scan they are not, because the laptop timestamps whole rotations and discards the rotation that straddles the move.

Decision needed: A or B.

### 2. Servo and encoder

- Plan: **GS2010MG 180°**, AS5600 encoder on the axle. Repo was written against the DS3218 270° in `full_assembly.step` (P1). Changed: defaults are now 180° / centre 90°; the firmware's `C us_min us_max range` command sets the calibrated travel.
- Measured angle: firmware reads the AS5600 over I2C, reports it in `STATUS` (`meas=`) and `OK` replies, and `W <ms>` waits for mechanical settle using the encoder. The rig uses the measured angle for geometry, stores commanded and measured per point (`phi`, `phi_meas`), and warns when they differ by more than 2°. Without an encoder it falls back to the commanded angle after `settle_ms`.
- Config gains `enc_center_deg` and `enc_invert` (encoder → φ), separate from `center_deg` / `invert` (servo → φ).

### 3. Microphones and acoustic bearing

Not in this repo yet. Staged per the plan's §4: level meters → channel check → paired-channel → 4-channel GCC-PHAT. The dashboard gets an audio tab when Milestone 5 starts. Session format will gain `audio/*.wav` and the microphone geometry goes into the config file. Nothing built here blocks it.

## Smaller notes

- Sweep range: plan says −75°..+75°; job defaults are −60..60 and the panel accepts anything within the servo limits.
- Power tree in the plan matches `firmware/README.md` and `docs/bringup.md`: separate servo rail, star ground, never through the S3.
- "Do not treat the adapter's micro-USB as a serial cable into the S3": agreed. Tonight the adapter's USB goes to the **laptop**, which is the intended use of that port.
- End-to-end latency is not measured yet (plan §6). Easy to add once the Wi-Fi path exists: `HEARTBEAT` round trip and S3 timestamp vs. laptop arrival.
- Replay: `POST /sessions/{id}/load` replays a recorded sweep without hardware, which satisfies the plan's "replay recorded sessions" item for LiDAR. Audio replay comes with Milestone 5.

## Milestone mapping

| Plan milestone | What this repo needs |
|---|---|
| M1 hardware bring-up | nothing new: `docs/bringup.md` |
| M2 S3 acquisition | firmware: AS5600 (done now), audio capture (new) |
| M3 Wi-Fi protocol | decision A/B above; `WifiTiltSource` + `--tcp` bridge (A) |
| M4 point-cloud scan with AS5600 | done once M1 works |
| M5 four-channel audio | new subsystem |
| M6 classroom demo | dashboard audio tab + bearing overlay on the height map |
