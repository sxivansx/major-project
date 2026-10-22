# web: obslam bench dashboard

Internal bench tool for the RPLIDAR S2M1 on the servo-tilted cradle. Talks to the core server (`docs/api.md`, section 3) over REST and a WebSocket.

```
bun install        # once
bun run mock       # fake core on http://127.0.0.1:8000 (no hardware needed)
bun run dev        # Vite on http://127.0.0.1:5173, proxies /api and /ws to :8000
bun run build      # production bundle in web/dist (the core serves it in prod)
```

Also: `bun run typecheck` (both tsconfigs) and `bun run check` (headless REST + WS contract check against whatever is on :8000; against the real core it moves the servo and runs a short sweep).

## Panels

The left sidebar holds the controls: **Connection** (port list from `/api/ports` with refresh, source `s2` or `sim`, baud, connect/disconnect, and the device info from the status object: model, serial, firmware, health, scan Hz, points per rev, range; the servo has the same with a `sim` entry), **Servo** (slider in servo degrees within the reported limits, numeric tilt φ, home, and a raw console for `?`, `C ...`, `L ...` calibration lines showing the reply), and **Scan job** (name, φ start/end/step, rotations, settle ms, start/stop, progress bar, step x of n, points, ETA, state). The main area has four tabs. **Live scan** draws the latest kind-1 frame on a canvas polar plot (range rings, angle ticks, native clockwise angle, points coloured by quality) next to big numbers (scan Hz, points, min/max range) and a virtualised raw table with an angle filter. **Point cloud and height map** is a react-three-fiber scene that accumulates kind-2 frames into a preallocated 2M-point buffer coloured by z (orbit, grid at z=0, axes, reticle at the tilt axis, clear button, optional surface mesh from the grid) beside the top-down 2.5D height map from kind-3 frames (NaN transparent, hover shows x/y/z, cell size, layer and z range controls that PUT `/api/config` and GET `/api/grid`). **Sessions** lists `/api/sessions` with load, grid, PLY/CSV export and delete. **Log** shows the last 200 log messages; the status bar at the bottom shows the WebSocket state with auto-reconnect and a one-line summary of lidar, servo, job and cloud size.

## Layout of the code

- `src/api/types.ts`, `src/api/frames.ts`: the contract types and the little-endian binary codec (shared with the mock and the check script).
- `src/api/ws.ts`: WebSocket client with exponential backoff; parses frames off the render path and throttles scan pushes to 10 Hz.
- `src/store/store.ts`: zustand store. `src/store/cloud.ts`: the point buffer, kept outside React; the 3D view uploads only the new slice each frame.
- `src/components/*`: one file per panel. `src/styles.css`: the only stylesheet (dark, neutral grey, one accent).
- `mock/server.ts`, `mock/world.ts`: Bun mock of the core with a fake room that is ray-cast at the current tilt.
- `scripts/check.ts`: headless contract check.

Tabs are deep-linkable: `#live`, `#cloud`, `#sessions`, `#log`.
