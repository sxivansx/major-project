# major-project

Internal tool for the OBSLAM LiDAR payload: drives an RPLIDAR S2M1 on a servo-tilted cradle, captures step-and-scan sweeps, builds a 2.5D elevation map and shows it on a dashboard.

```
driver/    C++ bridge over Slamtec rplidar_sdk (S2 needs 1 Mbaud + the S-series scan format)
core/      Python: scan/tilt sources, simulator, geometry, 2.5D grid, session format, FastAPI server
web/       React + react-three-fiber dashboard (Bun)
firmware/  ESP32 sketch that tilts the DS3218 over a serial line protocol
docs/      API and protocol contracts
sessions/  captured sweeps (gitignored)
```
