# Setup on a fresh Mac (Apple silicon)

Everything needed to clone this repo on a new Mac, connect to the rig and run the dashboard. About 10 minutes the first time, most of it downloads.

## 1. Tools (once per Mac)

```bash
xcode-select --install                                  # clang, make, git (skip if already installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"   # Homebrew, if missing
brew install uv                                         # Python 3.12 + venv manager
curl -fsSL https://bun.sh/install | bash                # Bun (web); then open a new terminal tab
```

Check: `clang --version`, `uv --version`, `bun --version` all print a version.

## 2. Get the code

```bash
git clone --recursive https://github.com/sxivansx/obslam.git
cd obslam
```

Already cloned without `--recursive`? Run `git submodule update --init` (the Slamtec SDK lives in `driver/rplidar_sdk`).

## 3. Build and install

```bash
make setup        # = make driver (Slamtec SDK + s2bridge) + cd core && uv sync + cd web && bun install
```

The driver build takes under a minute. `make test` runs the Python tests and the web type check (about 40 s). The tests include timing checks: on a busy machine one can fail once; rerun before worrying.

## 4. API keys (`core/.env`)

```bash
cp core/.env.example core/.env
open -e core/.env          # paste the keys, save
```

| Key | Used for | Without it |
|---|---|---|
| `TYPESAFE_API_KEY` | Jev: typed decisions (triage, distress yes/no) | falls back to OpenRouter for Jev, else templates |
| `OPENROUTER_API_KEY` | Gemini 3.5 Flash: SITREP and answers | deterministic text templates |
| `SARVAM_API_KEY` | Sarvam `saaras:v4` speech to text ("save me", "bachao") | speech listener stays off |

`core/.env` is gitignored: never commit it. Keys pasted while the server runs are picked up without a restart.

## 5. Connect to the rig (Wi-Fi)

1. Power the ESP32 rig from the **powerbank only**. Never connect the laptop USB and the powerbank to the ESP at the same time.
2. Wait about 10 s for the ESP32 to boot. It always starts its own access point.
3. On the Mac, join Wi-Fi **`SLAM-Test`**, password **`SLAMtest24`**. The rig is then at **192.168.4.1** (LiDAR on TCP 8899, tilt on 8900, audio on 8901 TCP / 8902 UDP).
4. Internet at the same time (Sarvam, Jev and Gemini need it): plug in the iPhone, turn on **Personal Hotspot**, and in **System Settings → Network → ⋯ → Set Service Order** drag **iPhone USB** above **Wi-Fi**. The Mac then reaches the rig over Wi-Fi and the internet over the iPhone.
5. Quick check: `ping -c 3 192.168.4.1` answers, and `curl -sI https://api.sarvam.ai | head -1` prints an HTTP line.

Optional, the ESP32's stored network (its NVS): the rig can also join a normal Wi-Fi network as well as its own AP. Send it once over the tilt port, it is saved and survives reboots (SSID and password without spaces):

```bash
printf 'N myhotspot mypassword\n' | nc -w 3 192.168.4.1 8900     # OK, then NET ip=… once joined
printf '?\n' | nc -w 2 192.168.4.1 8900                         # status
```

The AP `SLAM-Test` stays on regardless, so the steps above always work.

## 6. Run

```bash
make dev          # backend on :8000 + dashboard on :5173; Ctrl-C stops both
```

Open **http://localhost:5173**. The dashboard opens on the **Operator** tab. Then connect the hardware (the Live scan and Sound tabs have Connect buttons, or use the curl commands below).

No rig at hand? `make sim` starts the same thing with a simulated LiDAR and servo already connected.

### Manual commands (two terminal tabs instead of `make dev`)

```bash
# tab 1: backend (FastAPI, http://127.0.0.1:8000, API under /api, WebSocket /ws)
cd core && uv run obslam-server            # add --sim for the simulator, --port 8001 to move it

# tab 2: dashboard (Vite, http://localhost:5173, proxies /api and /ws to :8000)
cd web && bun run dev                       # CORE_URL=http://127.0.0.1:8001 bun run dev if the backend moved
```

Single process instead (built dashboard served by the backend on http://127.0.0.1:8000): `make serve`.

### Connect the rig from the terminal

```bash
H='content-type: application/json'
curl -s -XPOST localhost:8000/api/lidar/connect -H "$H" -d '{"source":"s2","port":"tcp://192.168.4.1:8899"}'
curl -s -XPOST localhost:8000/api/servo/connect -H "$H" -d '{"port":"tcp://192.168.4.1:8900"}'
curl -s -XPOST localhost:8000/api/audio/connect -H "$H" -d '{"source":"tcp","port":"tcp://192.168.4.1:8901"}'
curl -s localhost:8000/api/status | python3 -m json.tool | head -40      # lidar scan_hz ≈ 10, audio link up
```

### Room map: six-station sprint

```bash
curl -s -XPOST localhost:8000/api/live/clear                                  # clear the old cloud
curl -s -XPOST localhost:8000/api/room/start -H "$H" -d '{"mode":"stations","planned":6}'
# set the scanner down, level and still, then:
curl -s -XPOST localhost:8000/api/room/station                                # repeat at each spot (≤ 3 m apart)
curl -s -XPOST localhost:8000/api/room/stop
```

The **Room map** tab does the same with buttons (Start station sprint, Add station N of 6) and shows the next spot as an amber ring. The **Point cloud** tab shows the room in 3D.

### Speech ("save me") check

```bash
say -v Samantha "Save me! Please save me!"          # the Mac speaks near the mics
curl -s "localhost:8000/api/audio/speech?limit=3" | python3 -m json.tool
```

A flagged call shows as a red dot on the Sound tab compass and a red banner on Operator.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `address already in use` on 8000 or 5173 | an old run is still up: `lsof -ti :8000 -ti :5173 \| xargs kill` |
| LiDAR "not streaming", 0 Hz | rig powered? on `SLAM-Test`? `ping 192.168.4.1`; reconnect the LiDAR; only one client can hold port 8899 |
| No audio / many lost frames | powerbank must feed the mics; stay on the rig's AP, a home router drops frames |
| "AI unavailable" or speech off | keys missing in `core/.env`, or no internet (check the iPhone USB service order) |
| Servo accepts commands but does not tilt | it needs its own 5–6 V, ≥ 3 A supply with ground shared with the ESP32; a shared powerbank browns out |
| `make driver` fails | `xcode-select --install`, then `git submodule update --init` |
| `uv` / `bun` not found | open a new terminal tab after installing (PATH) |

## Flash the firmware (not needed for the demo)

The rig already runs the firmware. Only to reflash (ESP32 on USB, powerbank disconnected):

```bash
softwareupdate --install-rosetta --agree-to-license     # arduino-cli's ESP32 tools need Rosetta on Apple silicon
brew install arduino-cli
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install ESP32Servo
make flash PORT=/dev/cu.usbmodem1101 FQBN=esp32:esp32:esp32s3:PSRAM=opi   # port from: arduino-cli board list
```

Details: `firmware/README.md`. API contract: `docs/api.md`. First bring-up of new hardware: `docs/bringup.md`.
