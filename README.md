# major-project (OBSLAM payload tool)

Internal tool for the OBSLAM LiDAR payload: drives an RPLIDAR S2M1 on a servo-tilted cradle, captures step-and-scan sweeps, builds a 2.5D elevation map and shows it on a dashboard.

```
driver/    C++ bridge over Slamtec rplidar_sdk (S2 needs 1 Mbaud + the S-series scan format)
core/      Python: scan/tilt sources, simulator, geometry, 2.5D grid, session format, FastAPI server
web/       React + react-three-fiber dashboard (Bun)
firmware/  ESP32 sketch that tilts the DS3218 over a serial line protocol
docs/      API and protocol contracts
sessions/  captured sweeps (gitignored)
```

## Quick start

```bash
make setup                      # builds the S2 bridge, python venv, web deps (needs clang, uv, bun)
make sim                        # simulated lidar + servo; dashboard at http://localhost:5173
make dev                        # real hardware; connect from the dashboard
make build && make serve        # single process on http://127.0.0.1:8000
make flash PORT=/dev/cu.usbmodemXXXX   # ESP32 servo firmware (FQBN=esp32:esp32:esp32s3 for an S3)
make test
```

First night with the rig: follow `docs/bringup.md` top to bottom.
