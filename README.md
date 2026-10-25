# obslam: On-board SLAM for GPS-Denied Indoor Search

Bench tool for On-board SLAM, a search-and-rescue payload: an RPLIDAR S3 on a servo-tilted cradle and a 4-mic array on an ESP32-S3, streamed over Wi-Fi to a laptop that maps the room, finds sound directions, flags spoken calls for help and shows it all on a dashboard.

```
driver/    C++ bridge over Slamtec rplidar_sdk (S-series scan format, 1 Mbaud, serial or TCP)
core/      Python 3.12 (uv): FastAPI + WebSocket server, mapping, acoustics, speech, AI layer
web/       React + react-three-fiber dashboard (Bun + Vite)
firmware/  ESP32-S3 sketch: LiDAR relay, tilt servo, I2S mics, Wi-Fi AP `SLAM-Test`
docs/      API contract (api.md), bring-up, dashboard guide
sessions/  captured sweeps (gitignored)
```

## Dashboard tabs

| Tab | What it gives |
|---|---|
| Operator | the map for decisions: walkable / blocked, possible voids, next scan station, sound wedges, call-for-help banner, AI summary |
| Live scan | the raw LiDAR view (polar plot), connection and servo controls |
| Sound | four microphone waveforms, sound directions on a compass, red dots for calls for help, heard speech |
| Point cloud and height map | 3D point cloud and the 2.5D height map; the room map in 3D |
| Room map | handheld / six-station floor plan with next-spot guidance |
| Sessions | saved sweeps: load, export PLY / CSV |
| Log | everything the server reported |

## Quick start

```bash
git clone --recursive https://github.com/sxivansx/obslam.git && cd obslam
make setup                       # driver + python venv + web deps (needs Xcode CLT, uv, bun)
cp core/.env.example core/.env   # then paste the API keys
# power the rig, join Wi-Fi SLAM-Test (password SLAMtest24)
make dev                         # open http://localhost:5173
make sim                         # same without hardware (simulated LiDAR + servo)
```

Full instructions for a fresh Mac, manual commands, Wi-Fi with internet, curl recipes and troubleshooting: **[SETUP.md](SETUP.md)**. First bring-up of new hardware: `docs/bringup.md`. Contracts: `docs/api.md`.

Other targets: `make test` (Python tests + web type check), `make build && make serve` (single process on :8000), `make flash PORT=… FQBN=esp32:esp32:esp32s3:PSRAM=opi`.
