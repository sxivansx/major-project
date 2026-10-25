# web: the dashboard

The browser dashboard for obslam. It talks to the core server over REST and a WebSocket ([../docs/api.md](../docs/api.md), section 3). It's built with React 18, react-three-fiber (3D) and zustand (state), and bundled with Bun and Vite.

For what each part of each screen means from an operator's point of view, see [../docs/dashboard-guide.pdf](../docs/dashboard-guide.pdf). This file is for people working on the code.

## Run

```bash
bun install        # once
bun run mock       # a fake core on http://127.0.0.1:8000, no hardware or Python needed
bun run dev        # Vite on http://127.0.0.1:5173, proxies /api and /ws to :8000
bun run build      # production bundle in web/dist (the core serves it with `make serve`)
```

- If the core runs on another port: `CORE_URL=http://127.0.0.1:8001 bun run dev`.
- `bun run typecheck` checks both tsconfigs.
- `bun run check` is a headless REST and WebSocket contract check against whatever is on :8000. Against the real core it moves the servo and runs a short sweep.

## One layout for every tab

Every tab uses the same shell (`src/components/Shell.tsx`):

1. **Numbers:** a row of three or four large figures with plain labels.
2. **Main view:** on the left, about 70 % of the width.
3. **Side column:** on the right, in plain sentences.
4. **Action bar:** the buttons for that tab.
5. **Advanced ▸:** a drawer under the action bar with the engineering panels. It scrolls on its own, takes at most about 45 % of the height, and its open or closed state is remembered per tab.

There is no sidebar. The connection, servo, mount and scan-job panels live in the Advanced drawers. A status bar at the bottom of every tab shows the WebSocket state (it reconnects on its own) and a one-line summary of the LiDAR, servo, job and cloud size.

Tabs can be linked directly: `#ops`, `#live`, `#sound`, `#cloud`, `#room`, `#sessions`, `#log`.

## The tabs

### Operator (`#ops`, the default)

For the person at the deployment point.

- **Numbers:** Seen (coverage %), Possible voids, Next (for example "1.3 m toward 206°" or "rotate scanner 90°"), and the strongest sound source (for example "Tapping 112° ± 6°", grey when its confidence is under 0.4).
- **Map:** top-down, with a 1 m grid; drag to pan, wheel to zoom, Fit. Green is walkable (lighter where marginal), red is blocked, and one hatch covers everything not seen yet. Voids (V1, V2, …) are outlined and labelled, and an arrow shows the next station. Each sound source is a wedge from the scanner, with its candidate regions outlined. Hovering shows x, y and the class.
- **What we have:** the situation report from `POST /api/ai/sitrep`, fetched after every analysis. A model-written report says "assistant summary" with its cost; without a key the core answers with a template. Below it is an Ask box (`POST /api/ai/ask`); a route answer is drawn on the map.
- **Actions:** Scan (shows "scanning… step 12 of 61"), Route (click a start, then a goal: "4.2 m, passable" or "no passable route: …"), Report (map PNG + findings JSON).
- **Advanced:** layer toggles, cell size and max step, Refresh analysis, Stop sweep, the Scan job form, the full situation block, route details, the finding lists (click to highlight and pan), and the assistant ranking (`POST /api/ai/triage`). With Advanced open, the map also labels obstacles, depressions and shadows.

Analysis never refreshes during a sweep. It refreshes on mount, on Refresh, and when a sweep's grid arrives after the job stops. Until then the previous analysis stays up and the numbers turn grey.

### Live scan (`#live`)

Is the hardware alive, and which way is it pointing?

- **Numbers:** Scan rate, Points per scan, Range (min–max), Health.
- **Main:** the polar plot (range rings, native clockwise angle, points coloured by quality).
- **Sensor column:** status in sentences, for example "RPLIDAR S3 on tcp://192.168.4.1:8899, healthy, 10.0 Hz, 2,860 points per scan." The connect controls appear here while something is not connected. Port fields take a serial port or `tcp://host:8899` / `tcp://host:8900` for the rig's Wi-Fi relay, and remember the last value. Also here: the tilt slider and Level.
- **Actions:** Disconnect LiDAR / servo, Level, Snapshot CSV.
- **Advanced:** the raw values table, the servo console (send `?`, `T …`, `L …`, `C …` and see the reply), connection details, alarms, and **Mount and filters** (the calibration from [../docs/bringup.md](../docs/bringup.md#3-calibrate-the-mount)).

### Sound (`#sound`)

Four microphones, [../docs/api.md](../docs/api.md) section 5.

- **Numbers:** Level (loudest mic, dBFS), Noise floor, Sources, Calibrated.
- **Main:** four stacked waveforms (the last 5 s), a level meter per mic, clipping in red, a marker at each detection. Under them, a mic-health strip shows which mic is excluded as too noisy, and why.
- **Side:** Sources first (one card per active source: kind, bearing ± spread, knocks, last heard, confidence, "Show on map"), then a compass with sources as wedges, red dots for calls for help, and a "Heard" panel with the transcribed speech.
- **Actions:** source (`tcp` or `sim`), port `tcp://host:8901`, Connect / Disconnect, Test knock (sim only), Clear events.
- **Advanced:** the **clap calibration** wizard (clap once at 0°, 90°, 180° and 270°, 1 m away at mic height, then Compute and Apply), the per-pair time differences, trigger threshold and dead time, and the mic geometry editor.

The array gives a direction only, never a distance, so nothing draws a sound as a point.

### Point cloud and height map (`#cloud`)

- **Numbers:** Points, Coverage, Floor RMS, Session ("live" or the loaded session's name).
- **Main:** the 3D point cloud (react-three-fiber), with a grid at floor level, axes, a surface mesh from the height grid, and the room map as a 3D layer.
- **Side:** the top-down 2.5D height map (hover shows x, y, z) with a metrics line under it.
- **Actions:** Scan, Fit, Surface on/off, PNG, PLY, Clear.
- **Advanced:** 3D view settings, the height-map query (cell size, layer, z range), the Scan job form (φ start, end and step, rotations, settle) and Mount and filters.

### Room map (`#room`)

A floor plan built from the level LiDAR ([../docs/api.md](../docs/api.md) section 3.5).

- **Handheld:** Start walking map, then carry the scanner slowly; each rotation is matched to the map so far. If it says "Lost track", move back to an area already on the map.
- **Station sprint:** Start station sprint (6), then set the scanner down still and level at each spot and press Add station. The next suggested spot shows as an amber ring.
- **Actions:** Stop, and PNG of the floor plan. The same map also appears as a 3D layer on the Point cloud tab.

### Sessions (`#sessions`) and Log (`#log`)

- **Sessions:** every saved sweep, with load, grid, PLY / CSV export and delete.
- **Log:** the last 200 messages from the server and the dashboard, with the latest error and warning on top.

## The mock backend

`bun run mock` runs a fake core, so the dashboard works with no hardware and no Python.

- `mock/server.ts`, `mock/world.ts`: a fake room ray-cast at the current tilt.
- `mock/ops.ts`: a TypeScript port of the core's `analysis.py` (floor, classes, obstacles, voids, shadows, blind stripe, A\* route) plus canned AI answers. `MOCK_AI=off bun run mock` behaves like a core without API keys.
- `mock/audio.ts`: a simulated array. Mic 3 is about 14 dB noisier, so it gets excluded (`MOCK_NOISY_MIC=off` to disable). A burst of six knocks comes from the first void's direction every 45 s, plus single knocks, echoes and noise.
- The mock also simulates speech and the room map.

## Code layout

| Path | What it holds |
|---|---|
| `src/api/types.ts`, `src/api/frames.ts` | contract types and the little-endian binary codec (shared with the mock and the check script) |
| `src/api/ws.ts` | WebSocket client with backoff; parses frames off the render path, throttles scan pushes to 10 Hz |
| `src/store/store.ts` | zustand store: server state, live data, the shared scan-job form, view settings |
| `src/store/actions.ts` | server calls shared by action bars and drawers (sweeps, connect, servo, height-map query) |
| `src/store/ops.ts` | Operator tab state and the sweep-aware analysis refresh |
| `src/store/audio.ts` | microphone state: waveform ring buffers (outside React), events, levels |
| `src/store/room.ts` | room-map state: the latest status and occupancy grid, start / stop |
| `src/store/metrics.ts` | refetches `/api/metrics` when a new grid arrives |
| `src/store/hooks.ts` | throttled store subscriptions for 10 Hz panels |
| `src/store/cloud.ts` | the point buffer, kept outside React; the 3D view uploads only the new slice each frame |
| `src/components/Shell.tsx` | the shared tab layout |
| `src/components/*Tab.tsx` | one file per tab; the other components are the pieces they place |
| `src/components/Ops*.tsx` | the Operator map, summary, findings, assistant and Advanced panel |
| `src/styles.css` | the only stylesheet: dark, neutral grey, one accent colour |
| `mock/` | the fake core (above) |
| `scripts/check.ts` | the headless contract check |
