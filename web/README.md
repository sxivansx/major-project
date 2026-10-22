# web: obslam bench dashboard

Internal bench tool for the RPLIDAR S2M1 on the servo-tilted cradle. Talks to the core server (`docs/api.md`, section 3) over REST and a WebSocket.

```
bun install        # once
bun run mock       # fake core on http://127.0.0.1:8000 (no hardware needed)
bun run dev        # Vite on http://127.0.0.1:5173, proxies /api and /ws to :8000 (or CORE_URL=http://127.0.0.1:8001)
bun run build      # production bundle in web/dist (the core serves it in prod)
```

Also: `bun run typecheck` (both tsconfigs) and `bun run check` (headless REST + WS contract check against whatever is on :8000; against the real core it moves the servo and runs a short sweep).

## Panels

The first tab, **Operator**, is for the person at the deployment point rather than the bench: it answers what is out there, where someone could be, whether we can get there and what has not been seen. It is the default tab. The map (a top-down canvas, scanner at the origin with the +x cable-mark arrow, 1 m grid, drag to pan, wheel to zoom, Fit) draws the kind-4 layers from `GET /api/ops/layers`: terrain as the same height ramp as the height map, traversability as passable / marginal / blocked fills, shadow cells as a diagonal hatch, the blind stripe along the tilt axle as a horizontal hatch, unknown left dark; toggles for each. Findings from `GET /api/ops/findings` are outlined and labelled (voids V1… solid blue, obstacles O1… thin, depressions D1… dotted, shadows S1… dashed) and the next-station suggestion is an arrow at its bearing (bearings are clockwise from +x). Hover shows x, y, height above the floor and the class. **Route** mode takes a start and a goal click, posts `/api/ops/route` and draws the path with its length, verdict, max step and start snap; a failed route shows the reason. The findings column has the situation block (coverage, area seen, floor reach, shadow and blind cells with their notes, floor fit, scanner height, traversability counts, next station), the four finding lists (click to highlight and pan), the cell-size select and max-step input (both re-query), and at the bottom the **assistant** block: `GET /api/ai/status`, then Rank findings (`/api/ai/triage`, sorted by attention with a confidence bar and the reason), Situation report (`/api/ai/sitrep`, a monospace block labelled as an assistant summary) and Ask (`/api/ai/ask`, intent, confidence, tool result, answer; a route answer is drawn on the map). Every assistant result is labelled as assistant output and shows its cost. The action bar under the map has Scan (same form and store as the Scan job panel, progress inline), Route, Clear route, Refresh and Report (downloads the map as PNG and the findings as JSON); status alarms show as a red line above the map. Findings and layers refresh themselves whenever a kind-3 grid frame arrives (end of sweep, session load), at most once per 2 s.

The left sidebar holds the controls: **Connection** (port list from `/api/ports` with refresh, source `s2` or `sim`, baud, connect/disconnect, and the device info from the status object: model, serial, firmware, health, scan Hz, points per rev, range; the servo has the same with a `sim` entry), **Servo** (slider in servo degrees within the reported limits, numeric tilt φ, home, and a raw console for `?`, `C ...`, `L ...` calibration lines showing the reply), and **Scan job** (name, φ start/end/step, rotations, settle ms, start/stop, progress bar, step x of n, points, ETA, state). The other tabs are the engineering views. **Live scan** draws the latest kind-1 frame on a canvas polar plot (range rings, angle ticks, native clockwise angle, points coloured by quality) next to big numbers (scan Hz, points, min/max range) and a virtualised raw table with an angle filter. **Point cloud and height map** is a react-three-fiber scene that accumulates kind-2 frames into a preallocated 2M-point buffer coloured by z (orbit, grid at z=0, axes, reticle at the tilt axis, clear button, optional surface mesh from the grid) beside the top-down 2.5D height map from kind-3 frames (NaN transparent, hover shows x/y/z, cell size, layer and z range controls that PUT `/api/config` and GET `/api/grid`). **Sessions** lists `/api/sessions` with load, grid, PLY/CSV export and delete. **Log** shows the last 200 log messages; the status bar at the bottom shows the WebSocket state with auto-reconnect and a one-line summary of lidar, servo, job and cloud size.

## Layout of the code

- `src/api/types.ts`, `src/api/frames.ts`: the contract types and the little-endian binary codec (shared with the mock and the check script).
- `src/api/ws.ts`: WebSocket client with exponential backoff; parses frames off the render path and throttles scan pushes to 10 Hz.
- `src/store/store.ts`: zustand store. `src/store/cloud.ts`: the point buffer, kept outside React; the 3D view uploads only the new slice each frame.
- `src/components/*`: one file per panel. `src/styles.css`: the only stylesheet (dark, neutral grey, one accent).
- `src/store/ops.ts`: state for the Operator tab (parameters, findings, layers, route, selection) and the debounced refresh. `src/store/actions.ts`: sweep start/stop shared by the job panel and the operator action bar.
- `mock/server.ts`, `mock/world.ts`: Bun mock of the core with a fake room that is ray-cast at the current tilt. `mock/ops.ts`: a port of the core's `analysis.py` (floor fit, classes, obstacles, voids, shadows, blind stripe, A* route) over the mock cloud, plus canned `/api/ai/*` answers built from the findings.
- `scripts/check.ts`: headless contract check, including the kind-4 codec round trip, `/api/ops/*` and `/api/ai/*`.

Tabs are deep-linkable: `#ops`, `#live`, `#cloud`, `#sessions`, `#log`.
