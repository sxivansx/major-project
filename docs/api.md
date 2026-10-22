# major-project: contracts

Four processes, three contracts. Everything is little-endian.

```
ESP32 (firmware/) ──USB serial, ASCII lines──┐
                                             ├── core server (Python, :8000) ──REST + WS──> web dashboard
RPLIDAR S2M1 ──USB 1 Mbaud──> s2bridge (C++) ─┘   stdout binary frames
```

## 1. s2bridge (driver/bridge) → core: stdout frames

Binary executable `driver/build/s2bridge`.

```
s2bridge --serial <port> [--baud 1000000]      stream frames to stdout until SIGTERM / stdin closes
s2bridge --serial <port> --info                print one JSON line, exit 0
```

`--info` JSON line:
```json
{"model":"S2M1","model_id":97,"firmware":"1.02","hardware":18,"serial":"ABCD...","health":"good","health_code":0}
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
| `?` | `STATUS angle=<deg> min=<deg> max=<deg> range=<180\|270> us_min=<us> us_max=<us>` | |
| `T <deg>` | `OK <deg>` | move immediately, clamped to [min,max]; reply after the pulse is set, not after settle |
| `S <deg> <ms>` | `OK <deg>` | ease to angle over ms, reply when done |
| `H` | `OK <deg>` | go to RANGE/2 |
| `L <min> <max>` | `OK` | soft limits, persisted in NVS |
| `C <us_min> <us_max> <range>` | `OK` | pulse calibration (default 500 2500 270), persisted in NVS |
| anything else | `ERR <msg>` | |

Defaults: DS3218 signal on **GPIO 13**, 50 Hz, 500–2500 µs, range 270. Servo power is a separate 5–6 V supply, GND shared with the ESP32. Never from the ESP32 5 V pin.

## 3. core ↔ web: REST + WebSocket

Server on `http://127.0.0.1:8000`. In dev, Vite on `:5173` proxies `/api` and `/ws`. In prod the server serves `web/dist`.

### Frames of reference

- **Lidar frame**: origin at the S2 optical centre, x = 0° mark, y = 90° *counter-clockwise* from above, z up. Native angle is clockwise, so `theta_ccw = -angle_deg`.
- **Base frame**: origin on the base plate under the tilt axis, x along the tilt axis, z up. Tilt axis at `z = axis_z_mm` (80 from CAD).
- Point transform: `p_lidar = (r cos θ, r sin θ, 0)` → `Rz(lidar_yaw) · p + (0, 0, h)` → `Rx(φ)` → `+ (0, 0, axis_z)`.
- `φ` (tilt, deg): `φ = sign · (servo_deg − center_deg)`, sign = −1 if `invert`. φ = 0 means scan plane horizontal. Positive φ = right-hand rotation about +x.

### Config (GET/PUT `/api/config`)

```json
{
  "mount": {"axis_z_mm": 80, "h_mm": 0, "lidar_yaw_deg": 0, "center_deg": 135, "invert": false},
  "grid":  {"cell_m": 0.05, "z_min_m": -0.5, "z_max_m": 2.5, "layer": "max"},
  "lidar": {"min_quality": 10, "min_range_m": 0.1, "max_range_m": 30}
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
| GET | `/grid` | `?cell=0.05&layer=max&z_min=&z_max=` | binary grid frame (kind 3 payload) |
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
  "servo": {"connected": true, "port": "/dev/cu.usbmodem1101", "servo_deg": 135.0, "phi_deg": 0.0,
            "min_deg": 0, "max_deg": 270, "range": 270},
  "job": null,
  "live_points": 0,
  "session_loaded": null
}
```
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

## 4. Session on disk

`sessions/<id>/meta.json` (SessionMeta) + `sessions/<id>/sweeps.npz` with equal-length arrays:
`step u16, phi f32 (deg), theta f32 (deg, native clockwise), range f32 (m), quality u8, t f64`.
Points are derived on load from the mount config in `meta.json`, so re-calibrating `h` or `center_deg` never needs a re-scan.
