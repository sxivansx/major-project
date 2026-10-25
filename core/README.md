# core: the Python backend

The core is the part that runs on the laptop. It takes data from the rig's LiDAR, servo and microphones (or from the simulator), builds the maps, finds sounds and calls for help, and serves the dashboard over REST and a WebSocket. Every format it reads or writes is defined in [../docs/api.md](../docs/api.md).

## Run it

```bash
cd core
uv sync                          # Python 3.12 venv and dependencies
uv run obslam-server --sim       # http://127.0.0.1:8000, simulated LiDAR and servo already connected
uv run obslam-server             # real hardware: connect from the dashboard
uv run obslam-server --port 8001 # another port
uv run pytest -q                 # all tests; they run on the simulator, no hardware needed
```

For simulated microphones, pick the `sim` source on the dashboard's Sound tab.

If `web/dist` exists (`cd web && bun run build`), the server also serves the dashboard at `/`. Otherwise run `bun run dev` in `web/` and open http://localhost:5173.

Some timing tests (for example `tests/test_rig.py::test_job_end_to_end`) can fail once on a busy machine. Rerun before debugging.

## Settings and data

| File | Holds | In git? |
|---|---|---|
| `core/.env` | API keys: `TYPESAFE_API_KEY`, `OPENROUTER_API_KEY`, `SARVAM_API_KEY`. All optional; changes are picked up without a restart | no (copy `.env.example`) |
| `core/config.json` | mount geometry, grid defaults, LiDAR filters, microphone positions; edited from the dashboard | no |
| `../sessions/<id>/` | saved sweeps (`meta.json` + `sweeps.npz`), replayable and exportable | no |

Without keys the AI features fall back to fixed templates and the speech listener stays off. Everything else works the same.

## Modules

`obslam/` grouped by job:

**Inputs**

| Module | What it does |
|---|---|
| `sources.py` | `ScanSource`: the interface for anything that produces one frame per LiDAR rotation |
| `s2.py` | the real RPLIDAR, through the `driver/build/s2bridge` subprocess (serial or Wi-Fi) |
| `sim.py` | simulated rig: a ray-cast room with rubble boxes, using the same mount geometry |
| `servo.py` | the tilt servo over the ESP32 line protocol, plus a simulated one |
| `audio_source.py` | four-channel audio from the ESP32 (UDP 8902 or TCP 8901) or the simulator |
| `audio_wire.py` | audio framing on the wire and the waveform preview sent to the dashboard |

**Geometry and maps**

| Module | What it does |
|---|---|
| `config.py` | persistent settings: mount geometry, servo-to-tilt mapping, grid, filters, mic array |
| `geometry.py` | LiDAR frame to base frame, using the tilt angle |
| `grid.py` | 2.5D height grid (max, min, mean and count per cell) |
| `slam2d.py` | handheld 2D room mapping: scan matching against a distance field, log-odds grid |
| `room.py` | the room-mapping session: feeds live rotations to the mapper, station mode, next-spot hints |

**Rig and sessions**

| Module | What it does |
|---|---|
| `rig.py` | the bench rig: connections, the live point cloud, the step-and-scan sweep job, alarms |
| `session.py` | saved sweeps on disk, PLY and CSV export |

**Understanding the scene**

| Module | What it does |
|---|---|
| `analysis.py` | floor fit, walkable or blocked, obstacles, voids, unseen shadows, the blind stripe along the tilt axle, A\* routes. Plain numpy, no AI |
| `acoustic.py` | sound detection and direction finding (GCC-PHAT) for the four-mic array |
| `audio_pipeline.py` | the acoustic layer as the server sees it: detector, directions, grouped sound sources, mic health |
| `speech.py` | cuts speech out of the mic stream, transcribes it (Sarvam), decides if it is a call for help (keywords, then Jev), gives it a direction |
| `ai.py` | Jev (typed yes/no and ranking decisions) and Gemini Flash (written summaries), each with a template fallback |

**Server**

| Module | What it does |
|---|---|
| `server.py` | FastAPI: REST under `/api`, WebSocket at `/ws`, the built dashboard at `/` |
| `wire.py` | binary encodings for the WebSocket and session endpoints |

## Scripts

```bash
uv run python scripts/soak.py --minutes 10                  # back-to-back sweeps against `make sim`, then a report
uv run python scripts/mic_geometry.py --dist 40.1 40.1 80   # what a microphone layout can resolve
```

The report from the 10-minute soak run is in [../docs/soak-2026-10-22.md](../docs/soak-2026-10-22.md).
