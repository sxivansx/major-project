# core

Python side: lidar sources (S2 bridge, simulator), servo tilt, geometry, 2.5D grid, sessions, and the FastAPI server that the dashboard talks to. Contract in `../docs/api.md`.

```bash
cd core
uv sync                         # Python 3.12 venv + deps
uv run pytest -q                # all of it runs against the simulator, no hardware
uv run obslam-server --sim     # http://127.0.0.1:8000, sim lidar + sim servo pre-connected
uv run obslam-server           # real hardware: connect from the dashboard
```

With `web/dist` built (`cd web && bun run build`) the server serves the dashboard at `/`. Otherwise run `bun run dev` in `web/` and open `http://localhost:5173`.

Config (mount offsets, grid defaults, lidar filters) persists in `core/config.json`; sessions in `../sessions/`.

Layout:

```
obslam/config.py    mount geometry + servo↔phi mapping, grid and lidar filters
obslam/geometry.py  lidar frame → base frame
obslam/grid.py      2.5D grid (max/min/mean/count layers)
obslam/wire.py      binary WS/REST frames
obslam/sources.py   ScanSource base + ScanStats
obslam/sim.py       ray-cast room simulator
obslam/s2.py        S2 via driver/build/s2bridge
obslam/servo.py     ESP32 tilt protocol + SimTilt
obslam/session.py   sessions/<id>/{meta.json,sweeps.npz}, PLY/CSV export
obslam/rig.py       the rig: connections, live cloud, step-and-scan job
obslam/server.py    FastAPI REST + WebSocket
```
