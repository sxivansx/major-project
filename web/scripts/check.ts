// Headless end-to-end check of the REST + WS contract against a running
// server (the mock by default). Exercises everything the panels use.
//
//   bun run mock          # in another terminal
//   bun run check         # or CHECK_URL=http://127.0.0.1:8000 bun run check
//
// Against the real core this will move the servo and run a short sweep.

import { encodeAudio, encodeOpsLayers, parseAudio, parseFrame, parseGrid, parseOpsLayers, parseSessionPoints } from "../src/api/frames";
import type { AiAsk, AiSitrep, AiStatus, AiTriage, AudioEvent, BinaryFrame, Config, Findings, Job, Metrics, PortInfo, RouteResult, SessionMeta, Status } from "../src/api/types";
import { FINDING_KINDS } from "../src/api/types";

const BASE = process.env.CHECK_URL ?? "http://127.0.0.1:8000";
const WS_URL = BASE.replace(/^http/, "ws") + "/ws";

let failures = 0;
function ok(cond: unknown, msg: string) {
  if (cond) console.log(`  ok   ${msg}`);
  else {
    failures++;
    console.log(`  FAIL ${msg}`);
  }
}

async function req<T>(method: string, path: string, body?: unknown): Promise<T> {
  const r = await fetch(BASE + "/api" + path, {
    method,
    headers: body !== undefined ? { "content-type": "application/json" } : undefined,
    body: body !== undefined ? JSON.stringify(body) : undefined,
  });
  if (!r.ok) throw new Error(`${method} ${path} -> ${r.status} ${await r.text()}`);
  if (r.status === 204) return undefined as T;
  return (await r.json()) as T;
}
async function reqBin(path: string): Promise<ArrayBuffer> {
  const r = await fetch(BASE + "/api" + path);
  if (!r.ok) throw new Error(`GET ${path} -> ${r.status}`);
  return r.arrayBuffer();
}
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

// ---- ws collector ----
interface Collected {
  text: Record<string, unknown>[];
  frames: BinaryFrame[];
  errors: string[];
}
const col: Collected = { text: [], frames: [], errors: [] };
const ws = new WebSocket(WS_URL);
ws.binaryType = "arraybuffer";
ws.onmessage = (ev) => {
  if (typeof ev.data === "string") col.text.push(JSON.parse(ev.data));
  else if (ev.data instanceof ArrayBuffer) {
    try {
      col.frames.push(parseFrame(ev.data));
    } catch (e) {
      col.errors.push((e as Error).message);
    }
  }
};
await new Promise<void>((resolve, reject) => {
  ws.onopen = () => resolve();
  ws.onerror = () => reject(new Error(`cannot open ${WS_URL}`));
});
console.log(`connected to ${WS_URL}`);

async function waitFor(pred: () => boolean, timeoutMs: number, what: string): Promise<boolean> {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    if (pred()) return true;
    await sleep(50);
  }
  console.log(`  (timeout waiting for ${what})`);
  return false;
}

// ---- 1. REST basics ----
console.log("\nREST basics");
const status0 = await req<Status>("GET", "/status");
ok(status0 && "lidar" in status0 && "servo" in status0 && "job" in status0, "GET /status has lidar/servo/job");
const ports = await req<PortInfo[]>("GET", "/ports");
ok(Array.isArray(ports) && ports.every((p) => typeof p.path === "string"), `GET /ports -> ${ports.length} ports`);
const config = await req<Config>("GET", "/config");
ok(config.mount && config.grid && config.lidar, "GET /config has mount/grid/lidar");
ok(typeof config.mount.axis_z_mm === "number" && typeof config.grid.cell_m === "number", "config field types");
const sessions0 = await req<SessionMeta[]>("GET", "/sessions");
ok(Array.isArray(sessions0), `GET /sessions -> ${sessions0.length}`);

// ---- 2. connect ----
console.log("\nConnect");
const s1 = await req<Status>("POST", "/lidar/connect", { source: "sim", port: "sim", baud: 1000000 });
ok(s1.lidar.connected === true, "lidar connect (sim)");
ok(typeof s1.lidar.model === "string", `lidar model ${s1.lidar.model} serial ${s1.lidar.serial} fw ${s1.lidar.firmware}`);
const s2 = await req<Status>("POST", "/servo/connect", { port: "sim" });
ok(s2.servo.connected === true, "servo connect (sim)");
ok(typeof s2.servo.min_deg === "number" && typeof s2.servo.max_deg === "number", `servo limits ${s2.servo.min_deg}..${s2.servo.max_deg}`);

// ---- 3. scan stream ----
console.log("\nScan stream (1.5 s)");
col.frames.length = 0;
await sleep(1500);
const scans = col.frames.filter((f) => f.kind === 1);
ok(scans.length >= 10 && scans.length <= 20, `${scans.length} kind-1 frames in 1.5 s (expect ~15)`);
if (scans.length) {
  const f = scans[scans.length - 1];
  if (f.kind === 1) {
    ok(f.n > 2000 && f.n <= 3400, `scan n=${f.n}`);
    ok(f.angle.length === f.n && f.range.length === f.n && f.quality.length === f.n, "scan arrays sized n");
    let ascending = true;
    let inRange = true;
    for (let i = 0; i < f.n; i++) {
      if (f.angle[i] < 0 || f.angle[i] >= 360) inRange = false;
      if (i && f.angle[i] < f.angle[i - 1]) ascending = false;
    }
    ok(inRange, "angles in [0, 360)");
    ok(ascending, "angles ascending");
    let rmin = Infinity;
    let rmax = 0;
    for (let i = 0; i < f.n; i++) {
      rmin = Math.min(rmin, f.range[i]);
      rmax = Math.max(rmax, f.range[i]);
    }
    ok(rmin > 0 && rmax < 31, `range ${rmin.toFixed(3)}..${rmax.toFixed(3)} m`);
    ok(scans.every((s) => s.kind === 1 && s.seq > 0), "seq set");
  }
}
ok(col.errors.length === 0, `no frame parse errors (${col.errors.length})`);
ok(col.text.some((m) => m.type === "status"), "status text messages seen");

// ---- 4. servo ----
console.log("\nServo");
const s3 = await req<Status>("POST", "/servo/move", { phi_deg: 10 });
ok(Math.abs((s3.servo.phi_deg ?? NaN) - 10) < 1e-6, `move phi_deg=10 -> phi ${s3.servo.phi_deg}, servo ${s3.servo.servo_deg}`);
const s4 = await req<Status>("POST", "/servo/move", { servo_deg: 100 });
ok(s4.servo.servo_deg === 100, `move servo_deg=100 -> servo ${s4.servo.servo_deg}, phi ${s4.servo.phi_deg}`);
const s5 = await req<Status>("POST", "/servo/home");
ok(typeof s5.servo.servo_deg === "number", `home -> servo ${s5.servo.servo_deg}`);
const raw = await req<{ reply: string }>("POST", "/servo/raw", { line: "?" });
ok(raw.reply.startsWith("STATUS"), `raw ? -> ${raw.reply}`);
const rawBad = await req<{ reply: string }>("POST", "/servo/raw", { line: "ZZZ" });
ok(rawBad.reply.startsWith("ERR"), `raw ZZZ -> ${rawBad.reply}`);
await sleep(300);
const tilted = col.frames.filter((f) => f.kind === 1 && Math.abs(f.phi_deg - (s5.servo.phi_deg ?? 0)) < 1e-3);
ok(tilted.length > 0, "scan frames carry phi_deg of the servo");

// ---- 5. job ----
console.log("\nJob");
col.frames.length = 0;
col.text.length = 0;
const started = await req<{ job_id: number }>("POST", "/job/start", {
  name: "check",
  phi_start: -6,
  phi_end: 6,
  phi_step: 3,
  rotations: 1,
  settle_ms: 50,
});
ok(typeof started.job_id === "number", `job_id ${started.job_id}`);
const done = await waitFor(
  () => col.text.some((m) => m.type === "job" && (m as unknown as Job).state === "done"),
  15000,
  "job done",
);
ok(done, "job reached state done");
const jobMsgs = col.text.filter((m) => m.type === "job") as unknown as Job[];
ok(jobMsgs.length >= 5, `${jobMsgs.length} job messages`);
const lastJob = jobMsgs[jobMsgs.length - 1];
ok(lastJob && lastJob.n_steps === 5 && lastJob.step === 5, `step ${lastJob?.step} of ${lastJob?.n_steps}`);
ok(typeof lastJob?.points === "number" && lastJob.points > 10000, `points ${lastJob?.points}`);
const pts = col.frames.filter((f) => f.kind === 2);
ok(pts.length === 5, `${pts.length} kind-2 points frames (expect 5)`);
if (pts.length && pts[0].kind === 2) {
  const p = pts[0];
  ok(p.job_id === started.job_id && p.xyz.length === p.n * 3, `points frame job_id ${p.job_id} n ${p.n}`);
  let finite = true;
  for (let i = 0; i < p.xyz.length; i++) if (!Number.isFinite(p.xyz[i])) finite = false;
  ok(finite, "points are finite");
}
const grids = col.frames.filter((f) => f.kind === 3);
ok(grids.length >= 1, `${grids.length} kind-3 grid frames`);
if (grids.length && grids[grids.length - 1].kind === 3) {
  const g = grids[grids.length - 1];
  if (g.kind === 3) {
    ok(g.data.length === g.w * g.h, `grid ${g.w}x${g.h} cell ${g.cell_m} layer ${g.layer}`);
    let filled = 0;
    for (let i = 0; i < g.data.length; i++) if (Number.isFinite(g.data[i])) filled++;
    ok(filled > 0 && filled < g.data.length, `grid has ${filled} filled cells and some NaN`);
  }
}
const stAfter = await req<Status>("GET", "/status");
ok(stAfter.job?.state === "done", `status.job.state ${stAfter.job?.state}`);

// ---- 6. grid endpoint ----
console.log("\nGrid endpoint");
const g2 = parseGrid(await reqBin("/grid?cell=0.1&layer=mean&z_min=-1&z_max=2"));
ok(g2.layer === "mean" && Math.abs(g2.cell_m - 0.1) < 1e-6, `GET /grid -> ${g2.w}x${g2.h} ${g2.layer} cell ${g2.cell_m}`);
const g3 = parseGrid(await reqBin("/grid?layer=count"));
ok(g3.layer === "count", "GET /grid?layer=count");
const mt = await req<Metrics>("GET", "/metrics?cell=0.1");
ok(mt.points > 0 && mt.grid && mt.grid.cells > 0 && mt.grid.filled > 0 && mt.grid.filled <= mt.grid.cells, `GET /metrics -> ${mt.points} points, ${mt.grid?.filled}/${mt.grid?.cells} cells`);
ok(mt.grid.coverage_pct >= 0 && mt.grid.coverage_pct <= 100, `metrics coverage ${mt.grid.coverage_pct}%`);
ok(mt.floor === null || (typeof mt.floor.rms_mm === "number" && typeof mt.floor.tilt_deg === "number" && typeof mt.floor.offset_mm === "number"), `metrics floor ${mt.floor ? `${mt.floor.points} pts, RMS ${mt.floor.rms_mm} mm` : "null"}`);

// ---- 6b. operator products ----
console.log("\nOperator products");
{
  // codec round trip; 3x3 cells so the f32 block is not 4-byte aligned
  const classes = new Uint8Array([0, 1, 2, 3, 4, 5, 1, 1, 3]);
  const height = new Float32Array([NaN, 0.1, 0.2, 0.9, NaN, NaN, 0, 0.05, 1.5]);
  const buf = encodeOpsLayers({ w: 3, h: 3, cell_m: 0.1, x0_m: -0.3, y0_m: 0.2, classes, height });
  ok(buf.byteLength === 24 + 9 * 5, `kind-4 frame is ${buf.byteLength} bytes (24 + 9 + 36)`);
  const f = parseOpsLayers(buf);
  ok(f.kind === 4 && f.w === 3 && f.h === 3 && Math.abs(f.cell_m - 0.1) < 1e-6 && Math.abs(f.x0_m + 0.3) < 1e-6, "kind-4 header round trip");
  ok(Array.from(f.classes).join() === Array.from(classes).join(), "kind-4 classes round trip");
  let same = f.height.length === 9;
  for (let i = 0; i < 9; i++) {
    if (Number.isNaN(height[i]) ? !Number.isNaN(f.height[i]) : Math.abs(f.height[i] - height[i]) > 1e-6) same = false;
  }
  ok(same, "kind-4 heights round trip through the unaligned f32 block");
  ok(parseFrame(buf).kind === 4, "parseFrame dispatches kind 4");
}
// a full sweep so the floor is actually sampled (the job above is ±6°); a void
// needs a floor return and a slab-top return in the same cell, so the step
// matters: 4° gives sweep lines every few cells at 1-3 m
col.text.length = 0;
const opsJob = await req<{ job_id: number }>("POST", "/job/start", { name: "ops-check", phi_start: -60, phi_end: 60, phi_step: 4, rotations: 1, settle_ms: 50 });
ok(await waitFor(() => col.text.some((m) => m.type === "job" && (m as unknown as Job).job_id === opsJob.job_id && (m as unknown as Job).state === "done"), 40000, "ops-check job done"), "ops-check sweep (-60..60 by 4) done");
await sleep(300);
const OPS_Q = "?cell=0.1&max_step=0.15&max_slope=25";
const fd = await req<Findings>("GET", "/ops/findings" + OPS_Q);
ok(fd.grid && fd.grid.w > 0 && fd.grid.h > 0 && Math.abs(fd.grid.cell_m - 0.1) < 1e-6, `findings grid ${fd.grid?.w}x${fd.grid?.h} @ ${fd.grid?.cell_m} m`);
ok(fd.points > 0, `findings points ${fd.points}`);
ok(fd.floor && typeof fd.floor.fitted === "boolean", `floor fitted=${fd.floor?.fitted} rms ${fd.floor?.rms_mm} mm tilt ${fd.floor?.tilt_deg}°`);
const cov = fd.coverage;
ok(cov.coverage_pct >= 0 && cov.coverage_pct <= 100 && cov.area_seen_m2 >= 0, `coverage ${cov.coverage_pct}% area ${cov.area_seen_m2} m² reach ${cov.floor_reach_m} m`);
ok(cov.observable_cells <= fd.grid.w * fd.grid.h && cov.filled_cells + cov.interpolated_cells <= cov.observable_cells, "coverage counts consistent");
const tv = fd.traversability;
ok(tv.passable + tv.marginal + tv.blocked + tv.shadow + tv.blind <= fd.grid.w * fd.grid.h, `traversability p ${tv.passable} m ${tv.marginal} b ${tv.blocked} s ${tv.shadow} bl ${tv.blind}`);
ok(tv.params && tv.params.max_step_m === 0.15 && tv.params.max_slope_deg === 25, "traversability params echo the query");
let shapeOk = true;
const ids = new Set<string>();
for (const kind of FINDING_KINDS) {
  const prefix = kind[0].toUpperCase();
  if (!Array.isArray(fd[kind])) shapeOk = false;
  for (const it of fd[kind] ?? []) {
    ids.add(it.id);
    if (!it.id.startsWith(prefix) || !(it.cells > 0) || !(it.area_m2 > 0)) shapeOk = false;
    if (!(it.bearing_deg >= 0 && it.bearing_deg < 360) || !(it.distance_m >= 0)) shapeOk = false;
    if (!Array.isArray(it.bbox_m) || it.bbox_m.length !== 4 || it.bbox_m[0] > it.bbox_m[2] || it.bbox_m[1] > it.bbox_m[3]) shapeOk = false;
    if (Math.abs(Math.hypot(it.x_m, it.y_m) - it.distance_m) > 0.02) shapeOk = false;
    const want = kind === "voids" ? "clearance_m" : kind === "obstacles" ? "height_m" : kind === "depressions" ? "depth_m" : null;
    if (want && typeof it[want] !== "number") shapeOk = false;
  }
}
ok(shapeOk, `finding items well formed: ${fd.voids.length} voids, ${fd.obstacles.length} obstacles, ${fd.depressions.length} depressions, ${fd.shadows.length} shadows`);
ok(fd.obstacles.length >= 1, "at least one obstacle (room walls and boxes)");
// the mock room has a slab (a void); the core's sim room does not, so only the shape is asserted
ok(fd.voids.every((v) => typeof v.clearance_m === "number" && v.clearance_m >= 0.4), `voids well formed (${fd.voids.length}; the mock slab gives ≥1, the core sim room 0)`);
ok(fd.shadows.length >= 1, "at least one shadow region behind a box");
// at the bench rig's 8 cm the stripe is ±4 cm and may cover no cell; on a 1 m tripod it is ±58 cm
ok(cov.blind_stripe !== null && cov.blind_stripe.half_width_m > 0, `blind stripe ±${cov.blind_stripe?.half_width_m} m, ${cov.blind_cells} cells`);
ok(fd.next_station === null || typeof fd.next_station.reason === "string", `next station ${fd.next_station ? `${fd.next_station.bearing_deg}° ${fd.next_station.distance_m} m` : "none"}`);
ok(fd.scan === undefined || (typeof fd.scan.in_progress === "boolean" && fd.scan.in_progress === false), `findings.scan ${fd.scan ? JSON.stringify(fd.scan) : "absent (older core)"}`);

const ly = parseOpsLayers(await reqBin("/ops/layers" + OPS_Q));
ok(ly.w === fd.grid.w && ly.h === fd.grid.h && Math.abs(ly.x0_m - fd.grid.x0_m) < 1e-5 && Math.abs(ly.y0_m - fd.grid.y0_m) < 1e-5, `layers grid matches findings grid (${ly.w}x${ly.h})`);
{
  const counts = [0, 0, 0, 0, 0, 0];
  let bad = 0;
  let mismatch = 0;
  let blindOutside = 0;
  for (let k = 0; k < ly.w * ly.h; k++) {
    const c = ly.classes[k];
    if (c > 5) bad++;
    else counts[c]++;
    const known = c === 1 || c === 2 || c === 3;
    if (known !== Number.isFinite(ly.height[k])) mismatch++;
    if (c === 5 && cov.blind_stripe) {
      const iy = Math.floor(k / ly.w);
      const y = ly.y0_m + (iy + 0.5) * ly.cell_m;
      if (Math.abs(y) >= cov.blind_stripe.half_width_m + 1e-6) blindOutside++;
    }
  }
  ok(bad === 0, "all classes in 0..5");
  ok(counts[1] === tv.passable && counts[2] === tv.marginal && counts[3] === tv.blocked && counts[4] === tv.shadow && counts[5] === tv.blind, "class counts match traversability");
  ok(mismatch === 0, "height is finite exactly on passable/marginal/blocked cells");
  ok(blindOutside === 0, "blind cells lie within the blind stripe");
}
// route from the scanner to a far floor-level passable cell. The top of a wall or a
// box is flat and so classes passable too, but sits inside a blocked ring; and the
// blind stripe splits the floor in two, so the half the start snaps into is luck:
// try a spread of goals, farthest first, and expect one to work.
{
  const cands: { xy: [number, number]; d: number }[] = [];
  let blockedGoal: [number, number] | null = null;
  for (let k = 0; k < ly.w * ly.h; k++) {
    const iy = Math.floor(k / ly.w);
    const ix = k - iy * ly.w;
    const x = ly.x0_m + (ix + 0.5) * ly.cell_m;
    const y = ly.y0_m + (iy + 0.5) * ly.cell_m;
    if (ly.classes[k] === 1 && Math.abs(ly.height[k]) < 0.15) cands.push({ xy: [x, y], d: Math.hypot(x, y) });
    if (ly.classes[k] === 3 && !blockedGoal) blockedGoal = [x, y];
  }
  cands.sort((a, b) => b.d - a.d);
  let rt: RouteResult = { found: false, reason: "no passable cells" };
  let goal: [number, number] = [0, 0];
  let far = 0;
  let tries = 0;
  const stride = Math.max(1, Math.floor(cands.length / 12));
  for (const c of cands.filter((_, i) => i % stride === 0).slice(0, 12)) {
    tries++;
    goal = c.xy;
    far = c.d;
    rt = await req<RouteResult>("POST", "/ops/route", { from_xy: [0, 0], to_xy: goal, cell: 0.1, max_step: 0.15, max_slope: 25 });
    if (rt.found) break;
  }
  ok(rt.found === true, `route to (${goal[0].toFixed(2)}, ${goal[1].toFixed(2)}) at ${far.toFixed(2)} m found=${rt.found} after ${tries} goal${tries === 1 ? "" : "s"}${rt.found ? "" : " " + rt.reason}`);
  if (rt.found) {
    ok(rt.path.length >= 2 && rt.cells === rt.path.length, `path ${rt.path.length} cells, ${rt.length_m} m, ${rt.verdict}, max step ${rt.max_step_m} m, snapped ${rt.start_snapped_m} m`);
    ok(rt.start_snapped_m >= 0 && rt.start_snapped_m <= 1.0, "start snap within 1 m");
    const last = rt.path[rt.path.length - 1];
    ok(Math.hypot(last[0] - goal[0], last[1] - goal[1]) < ly.cell_m, "path ends at the goal cell");
    const first = rt.path[0];
    ok(Math.hypot(first[0], first[1]) <= rt.start_snapped_m + ly.cell_m, "path starts at the (snapped) start");
    ok(rt.length_m >= far - rt.start_snapped_m - 2 * ly.cell_m, "length at least the straight line");
    ok(rt.verdict === "passable" || rt.verdict === "marginal", "verdict is passable or marginal");
    let onPassable = true;
    for (const [x, y] of rt.path) {
      const ix = Math.floor((x - ly.x0_m) / ly.cell_m);
      const iy = Math.floor((y - ly.y0_m) / ly.cell_m);
      const c = ly.classes[iy * ly.w + ix];
      if (c !== 1 && c !== 2) onPassable = false;
    }
    ok(onPassable, "every path cell is passable or marginal (route goes around blocked cells)");
  }
  if (blockedGoal) {
    const rb = await req<RouteResult>("POST", "/ops/route", { from_xy: [0, 0], to_xy: blockedGoal, cell: 0.1 });
    ok(rb.found === false && typeof rb.reason === "string" && rb.reason.length > 0, `route to a blocked cell: found=false, reason "${rb.found ? "" : rb.reason}"`);
  }
  const rOut = await fetch(BASE + "/api/ops/route", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ from_xy: [0], to_xy: [1, 1] }) });
  ok(rOut.status === 400 || rOut.status === 422, `malformed route body -> ${rOut.status}`);
}
// assistant layer
const ai = await req<AiStatus>("GET", "/ai/status");
ok(typeof ai.available === "boolean", `GET /ai/status available=${ai.available}${ai.reason ? " (" + ai.reason + ")" : ""}`);
if (ai.available) {
  const tr = await req<AiTriage>("POST", "/ai/triage", { cell: 0.1 });
  ok(Array.isArray(tr.items) && tr.items.every((i) => i.attention >= 0 && i.attention <= 1 && i.confidence >= 0 && i.confidence <= 1 && typeof i.why === "string"), `triage ${tr.items.length} items with attention/confidence in 0..1`);
  ok(tr.items.every((i) => ids.has(i.id)), "triage ids exist in the findings");
  ok(tr.rescan && typeof tr.rescan.recommended === "boolean" && tr.rescan.p >= 0 && tr.rescan.p <= 1, `rescan recommended=${tr.rescan?.recommended} p=${tr.rescan?.p}`);
  const sr = await req<AiSitrep>("POST", "/ai/sitrep", { cell: 0.1 });
  ok(typeof sr.text === "string" && sr.text.trim().length > 0, `sitrep ${sr.text.split("\n").length} lines`);
  ok(!/survivor/i.test(sr.text), "sitrep never says survivor");
  const target = fd.voids[0]?.id ?? fd.obstacles[0]?.id ?? "V1";
  const ak = await req<AiAsk>("POST", "/ai/ask", { question: `route to ${target}` });
  ok(typeof ak.intent === "string" && typeof ak.answer === "string" && ak.confidence >= 0 && ak.confidence <= 1, `ask -> intent ${ak.intent} confidence ${ak.confidence}`);
  ok(ak.intent === "route" && !!ak.tool && typeof (ak.tool as { found?: unknown }).found === "boolean", "ask route carries a route result as tool");
} else {
  // without a key the situation report is still expected (a template), ask is not
  const sr = await fetch(BASE + "/api/ai/sitrep", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ cell: 0.1 }) });
  ok(sr.status === 200 || sr.status === 503, `sitrep without a key -> ${sr.status}`);
  if (sr.ok) {
    const j = (await sr.json()) as AiSitrep;
    ok(typeof j.text === "string" && j.text.length > 0 && j.model === "template", `template sitrep (model ${j.model})`);
    ok(!/survivor/i.test(j.text), "template sitrep never says survivor");
  }
  const ak = await fetch(BASE + "/api/ai/ask", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ question: "coverage" }) });
  ok(ak.status === 503, `ask without a key -> ${ak.status}`);
}

// ---- 6c. acoustic layer (docs/api.md §5) ----
console.log("\nAcoustic layer");
{
  // kind-5 codec round trip, odd n so nothing lines up on 4 or 8 bytes
  const n = 3;
  const mins = [0, 1, 2, 3].map((c) => new Int16Array([-100 - c, -32768, -1]));
  const maxs = [0, 1, 2, 3].map((c) => new Int16Array([100 + c, 32767, 0]));
  const buf = encodeAudio({ channels: 4, n, seq: 7, t_unix: 1761250000.25, levels_dbfs: new Float32Array([-40, -41.5, -39, -60]), mins, maxs });
  ok(buf.byteLength === 16 + 4 * 4 + 4 * n * 4, `kind-5 frame is ${buf.byteLength} bytes (16 + 16 + 4·3·4)`);
  const f = parseAudio(buf);
  ok(f.kind === 5 && f.channels === 4 && f.n === n && f.seq === 7 && Math.abs(f.t_unix - 1761250000.25) < 1e-6, "kind-5 header round trip");
  ok(Math.abs(f.levels_dbfs[1] + 41.5) < 1e-6 && f.levels_dbfs[3] === -60, "kind-5 levels round trip");
  let same = true;
  for (let c = 0; c < 4; c++) for (let i = 0; i < n; i++) if (f.mins[c][i] !== mins[c][i] || f.maxs[c][i] !== maxs[c][i]) same = false;
  ok(same, "kind-5 min/max envelopes round trip (channel-major, incl. ±full scale)");
  ok(parseFrame(buf).kind === 5, "parseFrame dispatches kind 5");
}
const stA = await req<Status>("GET", "/status");
if (!stA.audio) {
  console.log("  (skipped: this core reports no audio in its status)");
} else {
  const cfgA = await req<Config>("GET", "/config");
  ok(!!cfgA.audio && Array.isArray(cfgA.audio.mics_mm) && cfgA.audio.mics_mm.length === 4 && typeof cfgA.audio.trigger_db === "number", "config has the audio block");
  const badSrc = await fetch(BASE + "/api/audio/connect", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ source: "usb" }) });
  ok(badSrc.status === 400 || badSrc.status === 422, `audio/connect with a bad source -> ${badSrc.status}`);
  col.frames.length = 0;
  col.text.length = 0;
  const sa = await req<Status>("POST", "/audio/connect", { source: "sim", port: "sim" });
  ok(sa.audio?.connected === true && sa.audio.source === "sim", "audio connect (sim)");
  await sleep(1200);
  const af = col.frames.filter((x) => x.kind === 5);
  ok(af.length >= 10 && af.length <= 30, `${af.length} kind-5 frames in 1.2 s (≤ 20 Hz)`);
  if (af.length && af[0].kind === 5) {
    const a0 = af[af.length - 1];
    if (a0.kind === 5) {
      ok(a0.channels === 4 && a0.n === 100, `audio frame ${a0.channels} ch × ${a0.n} points`);
      let okEnv = true;
      for (let c = 0; c < a0.channels; c++) for (let i = 0; i < a0.n; i++) if (a0.mins[c][i] > a0.maxs[c][i]) okEnv = false;
      ok(okEnv, "min ≤ max everywhere");
      ok(Array.from(a0.levels_dbfs).every((v) => Number.isFinite(v) && v <= 0), `levels ${Array.from(a0.levels_dbfs).map((v) => v.toFixed(1)).join(" ")} dBFS`);
      let seqUp = true;
      for (let i = 1; i < af.length; i++) {
        const p = af[i - 1];
        const q = af[i];
        if (p.kind === 5 && q.kind === 5 && q.seq <= p.seq) seqUp = false;
      }
      ok(seqUp, "seq increases");
    }
  }
  const stB = await req<Status>("GET", "/status");
  const au = stB.audio!;
  ok(au.levels_dbfs?.length === 4 && au.noise_dbfs?.length === 4 && au.clipping?.length === 4, "status.audio has levels, noise and clipping per mic");
  // a knock from the direction of a known finding must list it as a candidate
  const target = fd.voids[0] ?? fd.shadows[0] ?? fd.obstacles[0];
  const bearing = target ? target.bearing_deg : 40;
  col.text.length = 0;
  const em = await req<AudioEvent>("POST", "/audio/sim/emit", { bearing_deg: bearing, elevation_deg: 10, snr_db: 22 });
  await waitFor(() => col.text.some((m) => m.type === "audio_event"), 2000, "audio_event");
  const wsEv = col.text.find((m) => m.type === "audio_event") as unknown as AudioEvent | undefined;
  ok(!!wsEv, "audio_event arrives on the WebSocket");
  const ev = wsEv ?? em;
  ok(typeof ev.id === "number" && typeof ev.t === "number" && typeof ev.peak_dbfs === "number" && typeof ev.snr_db === "number", `event #${ev.id} peak ${ev.peak_dbfs} dBFS snr ${ev.snr_db} dB`);
  ok(ev.bearing_deg !== null && ev.bearing_sigma_deg !== null && ev.bearing_sigma_deg > 0, `bearing ${ev.bearing_deg}° ± ${ev.bearing_sigma_deg}°`);
  const diff = ev.bearing_deg === null ? 999 : Math.abs((((ev.bearing_deg - bearing) % 360) + 540) % 360 - 180);
  ok(diff <= 2 * (ev.bearing_sigma_deg ?? 0), `emitted ${bearing}° lies within ± 2σ (off by ${diff.toFixed(1)}°)`);
  ok(ev.confidence >= 0 && ev.confidence <= 1, `confidence ${ev.confidence}`);
  ok(Object.keys(ev.tdoa_us).sort().join(",") === "12,13,14,23,24,34", "tdoa_us has the six pairs");
  ok(ev.candidates.every((c) => ids.has(c.id) && typeof c.distance_m === "number" && typeof c.kind === "string"), `${ev.candidates.length} candidates, all from the findings`);
  if (target) ok(ev.candidates.some((c) => c.id === target.id), `candidates include ${target.id} (the finding the knock was aimed at)`);
  const order = ev.candidates.map((c) => (c.kind === "void" ? 0 : c.kind === "obstacle" ? 2 : 1));
  ok(order.every((v, i) => i === 0 || v >= order[i - 1]), "candidates: voids, then unseen regions, then obstacles");
  ok(!JSON.stringify(ev).toLowerCase().includes("person") && !JSON.stringify(ev).toLowerCase().includes("survivor"), "event never names a person");
  const evs = await req<AudioEvent[]>("GET", "/audio/events?limit=5");
  ok(Array.isArray(evs) && evs.length >= 1 && evs.length <= 5 && evs[0].id === ev.id, `GET /audio/events -> ${evs.length}, newest first`);
  ok(evs.every((e, i) => i === 0 || e.t <= evs[i - 1].t), "events sorted newest first");
  const cfgB = await req<Config>("PUT", "/config", { ...cfgA, audio: { ...cfgA.audio!, trigger_db: 15, min_gap_ms: 500 } });
  ok(cfgB.audio?.trigger_db === 15 && cfgB.audio?.min_gap_ms === 500, "PUT /config audio block echoes the change");
  await req<Config>("PUT", "/config", cfgA);
  const clr = await fetch(BASE + "/api/audio/clear", { method: "POST" });
  ok(clr.status === 204 || clr.status === 200, `POST /audio/clear -> ${clr.status}`);
  const evs2 = await req<AudioEvent[]>("GET", "/audio/events");
  ok(evs2.every((e) => e.id !== ev.id), "events cleared");
  const sd = await req<Status>("POST", "/audio/disconnect");
  ok(sd.audio?.connected === false, "audio disconnect");
  const emOff = await fetch(BASE + "/api/audio/sim/emit", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ bearing_deg: 10 }) });
  ok(emOff.status === 409 || emOff.status === 400, `sim/emit while disconnected -> ${emOff.status}`);
}

// ---- 7. config ----
console.log("\nConfig");
const c2 = await req<Config>("PUT", "/config", { ...config, grid: { ...config.grid, cell_m: 0.08 } });
ok(Math.abs(c2.grid.cell_m - 0.08) < 1e-9, "PUT /config echoes change");
await req<Config>("PUT", "/config", config);

// ---- 8. sessions ----
console.log("\nSessions");
const sessions1 = await req<SessionMeta[]>("GET", "/sessions");
ok(sessions1.length >= sessions0.length + 1, `sessions now ${sessions1.length} (job added one)`);
const mine = sessions1.find((s) => s.name === "check");
ok(!!mine, `session for job found: ${mine?.id}`);
if (mine) {
  const meta = await req<SessionMeta>("GET", `/sessions/${mine.id}`);
  ok(meta.id === mine.id && meta.n_points === mine.n_points, `GET /sessions/{id} n_points ${meta.n_points}`);
  const xyz = parseSessionPoints(await reqBin(`/sessions/${mine.id}/points`));
  ok(xyz.length === mine.n_points * 3, `GET /sessions/{id}/points -> ${xyz.length / 3} points`);
  const sg = parseGrid(await reqBin(`/sessions/${mine.id}/grid?cell=0.2`));
  ok(sg.w > 0 && sg.h > 0, `GET /sessions/{id}/grid -> ${sg.w}x${sg.h}`);
  const ply = await (await fetch(`${BASE}/api/sessions/${mine.id}/export.ply`)).text();
  ok(ply.startsWith("ply\nformat ascii 1.0"), "export.ply is ASCII PLY");
  const csv = await (await fetch(`${BASE}/api/sessions/${mine.id}/export.csv`)).text();
  ok(csv.startsWith("phi,phi_meas,theta,range,quality,x,y,z"), "export.csv header");

  col.frames.length = 0;
  const ld = await req<Status>("POST", `/sessions/${mine.id}/load`);
  ok(ld.session_loaded === mine.id, `load -> session_loaded ${ld.session_loaded}`);
  await waitFor(() => col.frames.filter((f) => f.kind === 3).length >= 1, 5000, "replay grid");
  const replayPts = col.frames.filter((f) => f.kind === 2);
  const replayN = replayPts.reduce((a, f) => a + (f.kind === 2 ? f.n : 0), 0);
  ok(replayN === mine.n_points, `replay sent ${replayN} points in ${replayPts.length} chunks`);
  ok(replayPts.every((f) => f.kind === 2 && f.n <= 50000), "chunks <= 50k");

  await req<void>("DELETE", `/sessions/${mine.id}`);
  const sessions2 = await req<SessionMeta[]>("GET", "/sessions");
  ok(!sessions2.some((s) => s.id === mine.id), "DELETE removed the session");
}

// ---- 9. disconnect ----
console.log("\nDisconnect");
const d1 = await req<Status>("POST", "/lidar/disconnect");
ok(d1.lidar.connected === false, "lidar disconnect");
const d2 = await req<Status>("POST", "/servo/disconnect");
ok(d2.servo.connected === false, "servo disconnect");
{
  // after the acoustic checks: they aim knocks at the regions of this sweep's map
  const mineOps = (await req<SessionMeta[]>("GET", "/sessions")).find((s) => s.name === "ops-check");
  if (mineOps) await req<void>("DELETE", `/sessions/${mineOps.id}`);
}

ok(col.errors.length === 0, `no frame parse errors overall (${col.errors.length})`);

ws.close();
console.log(failures === 0 ? "\nALL CHECKS PASSED" : `\n${failures} CHECK(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
