# major-project: contracts

Four processes, three contracts. Everything is little-endian.

```
ESP32 (firmware/) ──USB serial, ASCII lines──┐
                                             ├── core server (Python, :8000) ──REST + WS──> web dashboard
RPLIDAR S3 ──USB 1 Mbaud──> s2bridge (C++) ─┘   stdout binary frames
```

## 1. s2bridge (driver/bridge) → core: stdout frames

Binary executable `driver/build/s2bridge`.

```
s2bridge --serial <port> [--baud 1000000]      stream frames to stdout until SIGTERM / stdin closes
s2bridge --serial <port> --info                print one JSON line, exit 0
```

`--info` JSON line:
```json
{"model":"S2M1","model_id":113,"firmware":"1.02","hardware":18,"serial":"ABCD...","health":"good","health_code":0}
```
`model` comes from the SDK's own model table (major id 7 = S2 family); `model_id` is the raw byte. Confirm against the real unit on first run.
```
```
Errors: one JSON line on stderr `{"error":"..."}` and exit 1.

Stream mode: one frame per complete 360° rotation (`grabScanData` + `ascendScanData`), nodes with range 0 dropped.

```
frame header (16 bytes)
  u32  magic   = 0x52463253  ('S2FR')
  u32  n       number of points
  f64  t_unix  seconds, wall clock at grab
then n × point (9 bytes, packed, no padding)
  f32  angle_deg   0..360, lidar native direction (clockwise from the cable mark, viewed from above)
  f32  range_m
  u8   quality
```

Logs go to stderr only. stdout is frames only.

## 2. Firmware ↔ core: servo serial protocol

115200 8N1, `\n`-terminated ASCII, one reply line per command. Angles are **servo degrees** `0..RANGE` (RANGE = 180 or 270). The mapping servo-deg → tilt φ lives in the core config, not in firmware.

| Command | Reply | Notes |
|---|---|---|
| `I` | `OBSLAM-TILT v1` | identify, used for port auto-detect |
| `?` | `STATUS angle=<deg> meas=<deg\|nan> min=<deg> max=<deg> range=<180\|270> us_min=<us> us_max=<us> enc=<ok\|nomag\|weak\|strong\|err\|none> settled=<0\|1>` | `meas` = AS5600 reading on the servo-degree scale |
| `T <deg>` | `OK <deg> [<meas>]` | move immediately, clamped to [min,max]; reply after the pulse is set, not after settle. `<meas>` present only with an encoder |
| `W [ms]` | `OK <meas> settled=<0\|1>` | block until the encoder is stable (\|Δ\| < 0.15° over 150 ms) or `ms` elapses (default 1500, max 10000). Without an encoder: fixed 300 ms, `settled=1` |
| `E` / `E zero` / `E dir` | `ENC raw=<0..4095> deg=<deg> zero=<deg> dir=<1\|-1> status=<…>` / `OK <meas>` | encoder diagnostics; `zero` aligns `meas` to the current commanded angle, `dir` flips direction; both persisted |
| `S <deg> <ms>` | `OK <deg>` | ease to angle over ms, reply when done |
| `H` | `OK <deg>` | go to RANGE/2 |
| `L <min> <max>` | `OK` | soft limits, persisted in NVS |
| `C <us_min> <us_max> <range>` | `OK` | pulse calibration (default 500 2500 270), persisted in NVS |
| anything else | `ERR <msg>` | |

Defaults: GS2010MG 180° servo, signal on **GPIO 13**, 50 Hz, 500–2500 µs, range 180 (home 90). Servo power is a separate 5–6 V supply, GND shared with the ESP32. Never from the ESP32 5 V pin. AS5600 encoder on I2C (ESP32-S3: SDA 8 / SCL 9; ESP32: 21 / 22), optional: without it `meas=nan`, `enc=none`, and the core falls back to the commanded angle after `settle_ms`.

## 3. core ↔ web: REST + WebSocket

Server on `http://127.0.0.1:8000`. In dev, Vite on `:5173` proxies `/api` and `/ws`. In prod the server serves `web/dist`.

### Frames of reference

- **Lidar frame**: origin at the S2 optical centre, x = 0° mark, y = 90° *counter-clockwise* from above, z up. Native angle is clockwise, so `theta_ccw = -angle_deg`.
- **Base frame**: origin on the base plate under the tilt axis, x along the tilt axis, z up. Tilt axis at `z = axis_z_mm` (80 from CAD).
- Point transform: `p_lidar = (r cos θ, r sin θ, 0)` → `Rz(lidar_yaw) · p + (0, 0, h)` → `Rx(φ)` → `+ (0, 0, axis_z)`.
- `φ` (tilt, deg): commanded `φ = sign · (servo_deg − center_deg)`, sign = −1 if `invert`; measured `φ = sign' · (meas_deg − enc_center_deg)`, sign' = −1 if `enc_invert`. φ = 0 means scan plane horizontal. Positive φ = right-hand rotation about +x. The rig uses the measured φ for geometry whenever the encoder reports one, and warns when it differs from the commanded φ by more than 2°.

### Config (GET/PUT `/api/config`)

```json
{
  "mount": {"axis_z_mm": 80, "h_mm": 0, "lidar_yaw_deg": 0, "center_deg": 90, "invert": false,
            "enc_center_deg": 90, "enc_invert": false},
  "grid":  {"cell_m": 0.05, "z_min_m": -0.5, "z_max_m": 2.5, "layer": "max"},
  "lidar": {"min_quality": 10, "min_range_m": 0.1, "max_range_m": 40}
}
```
Persisted to `core/config.json`.

### REST (`/api`)

| Method | Path | Body / query | Returns |
|---|---|---|---|
| GET | `/status` | | Status object (below) |
| GET | `/ports` | | `[{"path":"/dev/cu.usbserial-0001","description":"CP2102N"}]` |
| POST | `/lidar/connect` | `{"source":"s2"\|"sim","port":"...","baud":1000000}` | Status |
| POST | `/lidar/disconnect` | | Status |
| POST | `/servo/connect` | `{"port":"..."}` or `{"port":"sim"}` | Status |
| POST | `/servo/disconnect` | | Status |
| POST | `/servo/move` | `{"servo_deg": 135}` or `{"phi_deg": 0}` | Status |
| POST | `/servo/home` | | Status |
| POST | `/servo/raw` | `{"line":"?"}` | `{"reply":"STATUS ..."}` (calibration console) |
| POST | `/job/start` | `{"name":"bench-1","phi_start":-60,"phi_end":60,"phi_step":2,"rotations":1,"settle_ms":300}` | `{"job_id":1}` |
| POST | `/job/stop` | | Status |
| POST | `/live/clear` | | Status (drops the live cloud) |
| GET | `/scan/snapshot.csv` | `?rotations=1..20` | CSV `rotation,t,phi,angle,range,quality` of the next N raw rotations at the current tilt |
| GET | `/metrics` | `?cell=&z_min=&z_max=` | `{points, steps, points_per_step, duration_s, grid:{cell_m,cells,filled,coverage_pct,points_per_filled_cell}, floor:{points,rms_mm,tilt_deg,offset_mm}\|null}` (floor = plane fit to points within ±10 cm of z=0) |
| GET | `/grid` | `?cell=0.05&layer=max&z_min=&z_max=` | binary grid frame (kind 3 payload) |
| GET | `/ops/findings` | `?cell=0.1&max_step=0.15&max_slope=25` | Findings JSON (below) for the live cloud |
| GET | `/ops/layers` | same query | binary kind-4 frame: classes + height above floor |
| POST | `/ops/route` | `{"from_xy":[x,y],"to_xy":[x,y],"cell":0.1,"max_step":0.15,"max_slope":25}` (send the same params as the layers so the route is judged on the classification shown) | `{"found":true,"start_snapped_m":0.3,"goal_snapped_m":0.4,"length_m":4.2,"cells":40,"marginal_cells":0,"verdict":"passable"\|"marginal","max_step_m":0.08,"path":[[x,y],…]}` or `{"found":false,"reason":"…"}`. The start snaps to the nearest passable cell within 1 m; the goal snaps within 0.6 m but only to cells reachable from the start (an object's flat top is an island, never a destination) |
| GET | `/ai/status` | | `{"available":bool,"reason":"…","models":{"decisions":"typesafe/jev-1.14","prose":"google/gemini-2.5-flash"}}` |
| POST | `/ai/triage` | `{"cell":0.1}` | Jev ranking of findings: `{"items":[{"id":"V1","kind":"void","attention":0.82,"confidence":0.7,"why":"…"}],"rescan":{"recommended":false,"p":0.2,"confidence":0.8},"cost_usd":0.0003}` |
| POST | `/ai/sitrep` | `{"cell":0.1}` | `{"text":"five-line situation report","facts":{…findings…},"model":"…","cost_usd":…}` |
| POST | `/ai/ask` | `{"question":"…"}` | `{"intent":"route\|measure\|list_voids\|list_obstacles\|coverage\|sitrep\|other","confidence":0.9,"target":"O3"\|null,"tool":{"intent":…,"target":…, …result…},"answer":"…","cost_usd":…}`. For `route` the route result's own fields (`found`, `path`, `length_m`…) sit at the top level of `tool`; for `measure` under `tool.measure`; lists under `tool.voids` / `tool.obstacles`; `coverage` under `tool.coverage` + `tool.next_station` |
| GET | `/sessions` | | `[SessionMeta]` |
| GET | `/sessions/{id}` | | SessionMeta |
| GET | `/sessions/{id}/points` | | binary: `u32 n`, n × (f32 x,y,z) base frame |
| GET | `/sessions/{id}/grid` | same query as `/grid` | binary grid frame |
| GET | `/sessions/{id}/export.ply` | | ASCII PLY |
| GET | `/sessions/{id}/export.csv` | | `phi,theta,range,quality,x,y,z` |
| DELETE | `/sessions/{id}` | | 204 |
| POST | `/sessions/{id}/load` | | Status (loads into the live view as source `replay`) |
| GET | `/config` / PUT `/config` | Config | Config |

Status object:
```json
{
  "lidar": {"connected": true, "source": "s2", "port": "/dev/cu.usbserial-0001", "baud": 1000000,
            "model": "S2M1", "serial": "...", "firmware": "1.02", "hardware": 18, "health": "good",
            "scan_hz": 10.1, "pts_per_rev": 3184, "range_min_m": 0.21, "range_max_m": 12.4},
  "servo": {"connected": true, "port": "/dev/cu.usbmodem1101", "servo_deg": 90.0, "phi_deg": 0.0,
            "min_deg": 0, "max_deg": 180, "range": 180,
            "meas_deg": 90.4, "phi_meas_deg": 0.4, "encoder": "ok", "settled": true},
  "job": null,
  "live_points": 0,
  "session_loaded": null,
  "alarms": []
}
```
`alarms` is the current set of watchdog conditions, checked once a second: no frames for >2 s, scan rate <8 Hz, <1000 points per rotation, lidar health not good, lidar stream died, last job failed. Each raise/clear also produces one `log` message. Finished jobs carry `finished` (unix s) and `session_id`.
`job` when running:
```json
{"job_id": 1, "name": "bench-1", "state": "running"|"done"|"stopped"|"error", "step": 12, "n_steps": 61,
 "phi_deg": -36, "points": 38208, "started": 1760000000.0, "eta_s": 24.5, "error": null}
```
SessionMeta:
```json
{"id": "20261022-213000-bench-1", "name": "bench-1", "created": 1760000000.0, "n_points": 190000,
 "n_steps": 61, "phi_start": -60, "phi_end": 60, "phi_step": 2, "rotations": 1,
 "lidar": {"model": "S2M1", "serial": "..."}, "mount": {...}}
```

### WebSocket `/ws`

Server pushes; client sends nothing (may send `{"type":"ping"}`). Text messages are JSON with a `type`; binary messages start with a `u8 kind`.

Text:
- `{"type":"status", ...Status}` on every change and at 1 Hz.
- `{"type":"job", ...job}` on every step.
- `{"type":"log", "level":"info"|"warn"|"error", "msg":"...", "t": 1760000000.0}`

Binary, kind 1 **scan** (live 2D, ≤10 Hz, always on while the lidar is connected):
```
u8 kind=1, u8[3] pad, u32 seq, f64 t_unix, f32 phi_deg, u32 n,
n × f32 angle_deg, n × f32 range_m, n × u8 quality
```
Binary, kind 2 **points** (after each job step; also when a session is loaded, chunked ≤ 50k points):
```
u8 kind=2, u8[3] pad, u32 job_id, u32 step, f32 phi_deg, u32 n, n × (f32 x, f32 y, f32 z)
```
Binary, kind 3 **grid** (at most once per second during a job, and at job end / session load):
```
u8 kind=3, u8 layer (0 max,1 min,2 mean,3 count), u16 pad, u32 w, u32 h,
f32 cell_m, f32 x0_m, f32 y0_m, f32 z_min_m, f32 z_max_m,
h × w × f32 row-major (row index = y), NaN = empty
```
Cell (ix, iy) covers x ∈ [x0 + ix·cell, x0 + (ix+1)·cell), same for y.

Binary, kind 4 **operator layers** (`GET /api/ops/layers` only, never pushed):
```
u8 kind=4, u8 pad, u16 pad, u32 w, u32 h, f32 cell_m, f32 x0_m, f32 y0_m,
h × w × u8 class, then h × w × f32 height above the fitted floor (NaN = not known)
```
The f32 block starts at byte 24 + w·h, which is 4-byte aligned only when w·h is a multiple of 4; clients must copy through a DataView otherwise (the dashboard does).
Classes: 0 unknown (sweep gap too wide, or past the last return), 1 passable, 2 marginal, 3 blocked, 4 shadow (occluded by a nearer object: scan from elsewhere), 5 blind (the stripe along the tilt axle no scan plane can reach: rotate the scanner 90°).

### Findings (`GET /api/ops/findings`)

All deterministic. Bearings are clockwise from +x (the lidar's 0° mark) in degrees; distances from the scanner at the origin.
```json
{
  "scanner": {"x_m": 0, "y_m": 0, "height_above_floor_m": 1.02, "note": "…"},
  "points": 190000,
  "grid": {"cell_m": 0.1, "w": 62, "h": 42, "x0_m": -3.1, "y0_m": -2.1},
  "floor": {"fitted": true, "points": 51000, "z_at_origin_m": -0.02, "tilt_deg": 0.3, "rms_mm": 11.2},
  "coverage": {"filled_cells": 1872, "interpolated_cells": 300, "shadow_cells": 139, "blind_cells": 585, "unknown_cells": 40,
               "observable_cells": 2400, "area_seen_m2": 21.7, "coverage_pct": 90.5, "floor_reach_m": 3.0,
               "blind_stripe": {"half_width_m": 0.57, "cells": 585, "note": "…"}},
  "traversability": {"passable": 1500, "marginal": 80, "blocked": 300, "shadow": 139, "blind": 585, "params": {"max_step_m": 0.15, "max_slope_deg": 25}},
  "obstacles":   [{"id": "O1", "cells": 13, "area_m2": 0.13, "height_m": 0.49, "x_m": 1.07, "y_m": 1.07, "distance_m": 1.51, "bearing_deg": 315, "bbox_m": [0.8, 0.9, 1.6, 1.5]}],
  "voids":       [{"id": "V1", "cells": 80, "area_m2": 0.8, "clearance_m": 0.6, "x_m": 1.0, "y_m": -1.0, "distance_m": 1.41, "bearing_deg": 45, "bbox_m": […]}],
  "depressions": [{"id": "D1", "…": "same shape, depth_m"}],
  "shadows":     [{"id": "S1", "cells": 60, "area_m2": 0.6, "x_m": 2.2, "y_m": 1.6, "distance_m": 2.7, "bearing_deg": 324, "bbox_m": […]}],
  "next_station": {"bearing_deg": 324, "distance_m": 1.4, "reason": "largest occluded region S1: 0.6 m² at 2.7 m"}
}
```
A void is a cell whose returns have a vertical gap ≥ 0.4 m starting within 0.15 m of the floor (a slab over open space), never a wall. Objects are split where neighbouring heights differ by more than 0.5 m, so a box against a wall stays its own finding.

### AI layer (`/api/ai/*`)

All three consume the findings JSON, never points. Two keys in `core/.env` (see `core/.env.example`) or the environment: `TYPESAFE_API_KEY` sends Jev questions straight to `https://api.typesafe.ai/v1/systemone` (model `jev-latest`, cost computed from input tokens at $0.042/M); `OPENROUTER_API_KEY` is used for Gemini (`google/gemini-3.5-flash`) and, if there is no TypeSafe key, for Jev via OpenRouter's Decisions API (`typesafe/jev-1.14`). `/ai/status` reports each half (`decisions`, `prose`) separately; `available` means decisions work. Without decisions, `/ai/triage` and `/ai/ask` return 503; without prose, `/ai/sitrep` and the `answer` in `/ai/ask` are deterministic templates (`"model": "template"`). Upstream errors surface as 502 with the provider's message. Override models with `OBSLAM_DECISIONS_MODEL` / `OBSLAM_PROSE_MODEL`. Measured costs (22 Oct 2026): a full Jev triage ≈ $0.00016, an intent decision ≈ $0.00013, a Gemini answer or SITREP ≈ $0.001 with `reasoning.effort: minimal` (the model's reasoning cannot be disabled; "low" costs 5× more and can truncate). Guardrails: every ranking `why` is a template over the finding's own numbers (Jev cannot write text); prose that mentions a survivor, victim or body is discarded for the template; the UI labels all of it as assistant output.

### 3.5 Handheld room map (`core/obslam/slam2d.py`, `room.py`)

Works with the tilt level (φ 0°) and the scanner carried by hand: each rotation is matched to the map built so far and the floor plan fills in. Map frame: origin where mapping started, +x = LiDAR 0° mark at the start, +y to its left, metres; `theta_deg` CCW from map +x.

- Matching: 0.5° sectors (median range, 0.15–12 m) scored against a chamfer distance field (distance to the nearest wall cell, clipped 0.3 m fine / 0.6 m coarse at 2× cells). Brute force ±0.2 m / ±8° on the coarse field around a constant-velocity guess, then Gauss-Newton (Huber) on the fine one. ~20 ms per rotation on an M-series Mac.
- Map: log-odds occupancy (hit +0.9, free −0.4 along the beam, clamp ±4), 5 cm cells, 24 m square around the start. Updated from keyframes only (moved ≥ 10 cm or turned ≥ 4°).
- Lost: fewer than 35 % of points within 10 cm of a wall → the scan is not used, `lost: true`, and the next scans search ±0.6 m / ±30° around the last good pose.

REST: `POST /api/room/start {"cell_m": 0.05}` (resets; 400 without a LiDAR), `POST /api/room/stop` (keeps the map), `GET /api/room` → room JSON + `grid {w,h,cell,x0,y0}`, `GET /api/room/grid` → the kind-6 frame.

WS binary kind 6 (≤ 2 Hz while mapping): `<BBHIIfff` kind=6, pad, pad, w, h, cell, x0, y0 (24 bytes), then u8 × w·h row-major, row = y: 0 unknown, 1 free, 2 occupied (cropped to what has been seen + 0.5 m).
WS JSON (≤ 5 Hz): `{"type": "room", state: "idle"|"mapping", pose {x, y, theta_deg}, path [[x, y]] (≤ 500), keyframes, scans, area_m2, match {rms_mm, inliers, ok}, lost, note}`.

## 4. Session on disk

`sessions/<id>/meta.json` (SessionMeta) + `sessions/<id>/sweeps.npz` with equal-length arrays:
`step u16, phi f32 (commanded tilt, deg), phi_meas f32 (encoder tilt, deg, NaN without an encoder), theta f32 (deg, native clockwise), range f32 (m), quality u8, t f64`.
Points are derived on load from the mount config in `meta.json`, using `phi_meas` where finite and `phi` otherwise, so re-calibrating `h` or a centre never needs a re-scan.

## 5. Acoustic layer (four INMP441 microphones)

Goal: detect impulsive sounds (knocks, shouts, tapping) and estimate their **direction** from the scanner, with a confidence, then intersect that bearing with the LiDAR findings to list where a person could be. The array is 2–4 cm across, so it gives **direction only, never distance**; a position needs bearings from two scanner stations.

### 5.1 Hardware (pin_connection.md §3)

| Signal | GPIO | Notes |
|---|---|---|
| BCLK (all mics) | 4 | driven by I2S0 (master) |
| WS (all mics) | 5 | driven by I2S0 (master) |
| SD mics 1 + 2 | 7 | I2S0 RX data. Mic 1 L/R=GND → left slot, mic 2 L/R=3V3 → right slot |
| SD mics 3 + 4 | 6 | I2S1 RX data (slave, clocked from the same BCLK/WS). Mic 3 left, mic 4 right |

Both I2S units sample on the same BCLK/WS edges, so all four channels are simultaneous; I2S1 is started before I2S0 so both begin on the same WS frame.

### 5.2 Firmware → core: audio stream (TCP 8901 'AUD1' or UDP 8902 'AUD2')

One client at a time (a new one replaces the old). Streaming runs only while a client is connected. Little-endian frames:

```
u32  magic        0x31445541  ('AUD1')
u32  seq          increments per frame; a gap = frames lost
u64  t_us         esp_timer_get_time() at the first sample of this frame
u32  sample_rate  16000 | 24000 | 48000 (default 24000, set with `A rate`)
u16  channels     4   (order: mic1, mic2, mic3, mic4)
u16  n            samples per channel in this frame = sample_rate / 100 (10 ms: 160 / 240 / 480)
u16  shift        right shift applied to the 32-bit I2S word (default 14)
u16  flags        bit0 = I2S read timeout in this frame, bit1 = frames dropped before this one (seq shows how many)
n × 4 × i16       interleaved: s0m1 s0m2 s0m3 s0m4 s1m1 ...
```

Frame size = 28 + n × 8 bytes: 128 KB/s at 16 kHz, 193 KB/s at 24 kHz, 384 KB/s at 48 kHz. Frames are never older than ~300 ms when sent: if the link is slower than the stream, the firmware skips the backlog down to the newest 50 ms, adds the skipped samples to `dropped`, and sets bit1 on the next frame sent (the `seq` gap gives the count). Over TCP the ESP32's lwIP send buffer (5,744 B) caps throughput at about 5,744 B / RTT, e.g. ~150 KB/s at a 38 ms Wi-Fi round trip, so on a slow link use 16 kHz.

Tilt protocol additions (port 8900 / USB):
- `A` → `AUDIO rate=<16000|24000|48000> shift=<8..16> clients=<0|1> frames=<n> dropped=<n> i2s_timeouts=<n> udp=<ip:port|none>`. `frames` = frames captured since boot; `clients` = TCP 8901 clients; `dropped` = samples per channel not delivered (TCP backlog skipped, ring full, or UDP datagrams the stack refused); `udp` = the current UDP subscriber.
- `A rate <16000|24000|48000>` → `OK` (persisted; applied within ~20 ms; the stream's next frames carry the new rate and n).
- `A shift <8..16>` → `OK` (gain; lower shift = louder, clips sooner; persisted).
- `A raw` → one diagnostic line: `RAW rate=… n=… fmt=philips data_bits=32 slot_bits=32 mask=both ws_width=32 ws_pol=0 bit_shift=1 left_align=1 big_endian=0 lsb_first=0 mclk_x=256 slave_bclk_div=8 pins=… shift=… pads_hi/4000=bclk4:…,ws5:…,din7:…,din6:… | i2s0.L(mic1) first=<8 hex words> min=… max=… nz=… ones=… toggle=<hex> | i2s0.R(mic2) … | i2s1.L(mic3) … | i2s1.R(mic4) …`. Raw 32-bit words of one block before the shift; `ones` counts 0xFFFFFFFF words, `toggle` ORs every bit that ever differed from the first word, `pads_hi` counts how often each pad read high in 4000 quick samples. Not for parsers.
- `?` gains `audio=<on|off>`.

#### UDP 8902: 'AUD2' datagrams (preferred over Wi-Fi)

TCP is limited to lwIP's 5,744 B send window per round trip; UDP has no window, so the stream is limited only by Wi-Fi airtime and nothing queues behind a slow ACK.

Subscription (no tilt link needed): the receiver sends the ASCII datagram `AUDS <port>` (e.g. `AUDS 50123`) to the scanner's UDP **8902** at least every 2 s. The scanner streams to the **source IP of that datagram** and the given port until ~5 s after the last `AUDS`, or until it receives `AUDX`. One subscriber at a time: the newest `AUDS` wins. While a UDP subscriber exists it gets the audio and a TCP 8901 client receives nothing (its `seq` jumps when TCP resumes).

Each 10 ms frame is sent as `parts` datagrams of at most 1,400 B of samples (175 samples × 4 ch × 2 B), split evenly. Little-endian, 34-byte header:

```
u32  magic        0x32445541  ('AUD2')
u32  seq          frame number (same counter as AUD1); all parts of a frame share it
u64  t_us         esp_timer_get_time() at the first sample of the FRAME
u32  sample_rate  16000 | 24000 | 48000
u16  channels     4
u16  n            samples per channel in the whole frame (= sample_rate / 100)
u16  shift        right shift applied to the 32-bit I2S word
u16  flags        bit0 = I2S read timeout in this frame, bit1 = datagrams dropped before this frame
u8   part         0 .. parts-1
u8   parts        datagrams in this frame
u16  first        index (per channel) of this part's first sample within the frame
u16  count        samples per channel in this part
count × 4 × i16   interleaved: s(first)m1 m2 m3 m4, s(first+1)m1 ...
```

| Rate | n | parts × count | Datagram | Datagrams/s | Payload rate |
|---|---|---|---|---|---|
| 16 kHz | 160 | 1 × 160 | 1,314 B | 100 | 131 KB/s |
| 24 kHz | 240 | 2 × 120 | 994 B | 200 | 199 KB/s |
| 48 kHz | 480 | 3 × 160 | 1,314 B | 300 | 394 KB/s |

All fit one 1,472 B UDP payload (no IP fragmentation). Receiver: a frame is complete when all `parts` arrived; place each part's samples at `first`; allow a few frames (~50 ms) of reordering, then treat a missing or partial frame as lost (a `seq` gap). Datagrams are built straight from the DMA block and sent non-blocking: nothing is queued on the scanner (latency ≈ 10 ms frame + Wi-Fi), and a datagram the stack cannot take at that moment is dropped, counted in `dropped`, and flagged (bit1) on the next frame.

### 5.3 Core

Config (`/api/config`, new `audio` block):
```json
"audio": {
  "port": "udp://192.168.1.49:8902",   // udp://ip[:8902] (AUD2, recommended over Wi-Fi) or tcp://ip[:8901] (AUD1)
  "mics_mm": [[0,0,0],[0,0,0],[0,0,0],[0,0,0]],   // sound-port positions in the array frame, mm
  "array_yaw_deg": 0,          // bearing of the array's +x axis, clockwise from base +x (same convention as every bearing)
  "calibrated": false,
  "trigger_db": 12,            // spike = short-term energy this many dB above the running noise floor
  "min_gap_ms": 300,           // dead time between events
  "speed_of_sound": 343.0
}
```
Endpoints:

| Method | Path | Returns |
|---|---|---|
| POST | `/audio/connect` | `{"source":"udp"\|"tcp"\|"sim","port":"udp://…:8902"}` → Status. `udp` (the default) binds an ephemeral local port, sends `AUDS <port>` every 1 s, reassembles AUD2 parts (5 frames of reordering; an incomplete frame counts as lost), reports `link: "connecting"` after 2 s of silence while it keeps subscribing, and sends `AUDX` on disconnect. Without `port` the configured one is used; a configured address of the other transport lends its host. Status gains `transport_note` recommending UDP while on TCP |
| POST | `/audio/disconnect` | Status |
| GET | `/audio/events?limit=50` | `[AudioEvent]`, newest first |
| POST | `/audio/clear` | 204 |
| POST | `/audio/sim/emit` | `{"bearing_deg":40,"elevation_deg":0,"snr_db":20}` (sim source only): inject a test knock |

Status gains `"audio": {"connected", "source", "port", "rate", "frames", "lost_frames", "levels_dbfs":[4], "noise_dbfs":[4], "clipping":[4 bool], "calibrated"}`.

AudioEvent:
```json
{"id": 12, "t": 1761250000.123, "t_us": 123456789,
 "peak_dbfs": -18.2, "snr_db": 24.5,
 "bearing_deg": 41.0,            // clockwise from base +x, like every bearing in findings; null if unresolved
 "bearing_sigma_deg": 6.5,       // 1σ uncertainty
 "elevation_deg": 12.0,          // |elevation|; a planar array cannot tell above from below
 "confidence": 0.82,             // 0..1: GCC peak sharpness × pair consistency × SNR
 "tdoa_us": {"12": -41.2, "13": 18.0, "14": -22.9, "23": 59.1, "24": 18.3, "34": -40.8},
 "candidates": [ {"kind":"void","id":"V2","distance_m":1.8,"bearing_deg":43}, {"kind":"shadow","id":"S1", ...} ],
 "label": "impulse"}
```
`candidates` = findings whose centroid lies inside bearing ± 2σ (wrap-safe), grouped by `kind` in this order: `void`, `shadow`, `unknown`, `obstacle` (depressions are not candidates), nearest first within each group. Never "person": these are places a source could be.

Conventions: `tdoa_us["ij"]` = t_i − t_j (positive when mic i hears the sound later). `bearing_deg`, `bearing_sigma_deg` and `elevation_deg` are all null when the direction is unresolved (with `"reason"`). `status.audio` is `{"connected": false}` when nothing is connected. `calibrated` lives in config; status mirrors it. `/audio/sim/emit` returns the detected AudioEvent (or `{"detected": false, "reason": …}` if the knock stayed under the trigger) and is 409 unless the source is `sim`. Event ages use the core clock (`t`); `t_us` is the ESP32 clock, kept for the raw record only.

### 5.4 WebSocket additions

Binary, kind 5 **audio preview** (≤ 20 Hz while audio is connected): per channel min/max envelopes of the last 100 ms at 1 kHz, for the waveform view:
```
u8 kind=5, u8 channels=4, u16 n (points per channel, 100), u32 seq, f64 t_unix,
channels × f32 level_dbfs (RMS of the block), then channels × n × (i16 min, i16 max)
```
Channel-major (all of mic1's n points, then mic2's …). `t_unix` is the time of the **last** point; frames may overlap (each covers the last 100 ms), so clients de-duplicate by time. min/max are the stream's i16 values after `shift`, ±32767 = full scale (0 dBFS).
Text: `{"type":"audio_event", ...AudioEvent}` on every detection.

### 5.5 Calibration and additions (implemented)

- Config `audio.channel_offsets_us` (4 floats): fixed per-channel capture delays fitted by calibration and removed before direction finding. Event `tdoa_us` stays the **raw** measurement so stored events can be reused for calibration.
- `POST /audio/calibrate` `{"clicks":[{"event_id":12 | "tdoa_us":{…}, "bearing_deg":0, "elevation_deg":0}], "apply":false, "refine_positions":false}` → fitted `array_yaw_deg`, `channel_offsets_us`, optional refined `mics_mm`, per-click residuals, a channel-order check (e.g. ch2/ch3 swapped) with the suggested fix. `apply: true` writes the config and sets `calibrated: true` only if the channel order is confirmed and the residual is under 6 µs. Clap at array height (or give `elevation_deg`); ≥ 4 known bearings, ≥ 6 to refine positions.
- `POST /audio/sim/emit` accepts `"kind": "knock" | "voice"`.
- Events also carry `reasons[]`, `coherence[6]`, `residual_us`, `calibrated`, `map_age_s`; candidates carry `off_axis_deg`. Candidate window is at least ±3°, a wide region straddling the bearing counts, and events with confidence < 0.1 get none.
- Status `audio` also carries `link`, `note`, `array_yaw_deg`, `map_age_s` and CPU stats.
- The array is assumed fixed to the base, not the tilting cradle. With this left–right symmetric layout a ch2/ch3 swap gives a perfectly fitting but mirrored bearing; only the clap calibration catches it.

### 5.6 Sound sources (what the operator sees)

Raw events are noisy (echoes, one clap detected twice, a noisy mic). The operator-facing unit is a **source**: events that keep coming from the same direction, grouped over time. Repeated tapping from one direction is the signal that matters.

Tracking (core): events with `confidence ≥ 0.1` and a bearing join the active source whose bearing is within `max(15°, 3 × spread)` and whose last event is < 60 s old; otherwise they start a new source. Events below 0.1 are kept but marked `"noise": true` and never form sources. A source expires 120 s after its last event (kept in history). Bearing = confidence/σ²-weighted circular mean; `spread_deg` = weighted circular standard deviation. `kind` = `"tapping"` when ≥ 3 events arrive 0.3–6 s apart, else `"sound"`. `confidence` grows with the number of consistent events and drops with spread: `1 − Π(1 − cᵢ)` × `exp(−spread/20°)`, capped at 0.99.

`GET /audio/sources` → `{"active":[Source], "history":[Source]}`; WS text `{"type":"audio_sources","active":[…]}` on every change; `status.audio.sources_active` = count.
```json
{"id": 4, "kind": "tapping", "bearing_deg": 112.0, "spread_deg": 6.1, "sigma_deg": 3.0,
 "n_events": 6, "first_t": 1761250000.1, "last_t": 1761250040.3, "rate_per_min": 9.0,
 "confidence": 0.86, "peak_dbfs": -9.0, "event_ids": [42, 45, 52],
 "candidates": [{"kind":"void","id":"V2","distance_m":1.8,"bearing_deg":110,"off_axis_deg":2}],
 "active": true}
```

Channel health: a mic whose noise floor is > 12 dB above the median of the others is **excluded** from direction finding (its pairs dropped; 3 mics still give a 2-D bearing). `status.audio.excluded_mics` lists them with a reason; events carry `mics_used`.

### 5.7 Spoken calls for help (`core/obslam/speech.py`)

Speech is cut out of the same 4-channel stream by an energy gate (pre-emphasised mean of the used mics, floor = 20th percentile of 10 ms block energy over 3 s; opens 9 dB above it for 40 ms, closes after 450 ms below +5 dB; ≥ 180 ms of voice, ≤ 6 s), so knocks and claps are never sent. Each segment:

1. **Direction:** the loudest 1 s goes through the event DOA (`locate`, same geometry, offsets and excluded mics).
2. **Transcript:** Sarvam `POST https://api.sarvam.ai/speech-to-text`, `saaras:v4`, `language_code=unknown`, keyterms biased toward the help words. Key `SARVAM_API_KEY` in `core/.env`. At most 12 calls a minute.
3. **Flag:** exact words (`save me/us`, `help`, `bachao`/बचाओ, `madad`/मदद, `kaapadi`/ಕಾಪಾಡಿ, `sahaya`/ಸಹಾಯ), sound-alikes of them (rough phonetic key + edit distance, first sound must agree: "safe me", "shave me", "bachaav", "hep me"), or Jev's `distress` probability ≥ 0.6.

Out: WS `{"type": "speech_event", id, t, duration_s, transcript, language, matched[], sounds_like[{word, heard}], distress_p, flagged, bearing_deg, bearing_sigma_deg, confidence, calibrated, reasons[], stt_ms, jev_ms, jev_error, cost_usd}`; a flagged event also raises alarm `speech_help` for 30 s and a `warn` log line. `GET /api/audio/speech?limit=` → `{status, events}`; `POST /api/audio/speech {"enabled": bool}`; `POST /api/audio/speech/clear`. `status.audio.speech` carries the counters. On by default when the key is set: speech leaves the rig for Sarvam's API only while enabled.
