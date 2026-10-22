// Headless end-to-end check of the REST + WS contract against a running
// server (the mock by default). Exercises everything the panels use.
//
//   bun run mock          # in another terminal
//   bun run check         # or CHECK_URL=http://127.0.0.1:8000 bun run check
//
// Against the real core this will move the servo and run a short sweep.

import { parseFrame, parseGrid, parseSessionPoints } from "../src/api/frames";
import type { BinaryFrame, Config, Job, PortInfo, SessionMeta, Status } from "../src/api/types";

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
  ok(csv.startsWith("phi,theta,range,quality,x,y,z"), "export.csv header");

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
ok(col.errors.length === 0, `no frame parse errors overall (${col.errors.length})`);

ws.close();
console.log(failures === 0 ? "\nALL CHECKS PASSED" : `\n${failures} CHECK(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
