// Mock of the core server (docs/api.md section 3) so the dashboard can be
// built and tested before the real FastAPI server exists.
//
//   bun run mock            # http://127.0.0.1:8000, ws://127.0.0.1:8000/ws
//   PORT=8001 bun run mock
//
// Everything is in memory. The "world" is a fake room with boxes
// (mock/world.ts); the lidar ray-casts it at the current tilt, so moving the
// servo visibly changes the live 2D scan, and a sweep produces a sensible 3D
// cloud and height map.

import type { ServerWebSocket } from "bun";
import { encodeGrid, encodeOpsLayers, encodePoints, encodeScan, encodeSessionPoints } from "../src/api/frames";
import type { Config, GridLayer, Job, JobRequest, SessionMeta, Status } from "../src/api/types";
import { AI_MODELS, OPS_DEFAULTS, aiAsk, aiSitrep, aiTriage, analyze, fitPlaneNear, nearestPassable, planRoute, type OpsParams, type OpsResult } from "./ops";
import { computeGrid, fakeScan, filterQuality, scanToPoints, type FakeScan, type Mount } from "./world";
import { AUDIO_DEFAULT, AudioSim } from "./audio";
import { SpeechSim } from "./speech";
import { RoomConflict, RoomSim } from "./room";

const PORT = Number(process.env.PORT ?? 8000);
// MOCK_AI=off mimics a core without an OpenRouter key: /ai/status unavailable,
// /ai/sitrep answers with a template, /ai/ask and /ai/triage return 503.
const MOCK_AI = (process.env.MOCK_AI ?? "on") !== "off";
const SCAN_PERIOD_MS = 100;
const POINTS_CHUNK = 50_000;

// ---- state ----

const config: Config = {
  mount: { axis_z_mm: 80, h_mm: 0, lidar_yaw_deg: 0, center_deg: 90, invert: false, enc_center_deg: 90, enc_invert: false },
  grid: { cell_m: 0.05, z_min_m: -0.5, z_max_m: 2.5, layer: "max" },
  lidar: { min_quality: 10, min_range_m: 0.1, max_range_m: 30 },
  audio: { ...AUDIO_DEFAULT, mics_mm: AUDIO_DEFAULT.mics_mm.map((p) => [...p] as [number, number, number]) },
};

const status: Status = {
  lidar: { connected: false },
  servo: { connected: false },
  job: null,
  live_points: 0,
  session_loaded: null,
};

// four-microphone array (docs/api.md §5); status.audio always reflects the sim
const audio = new AudioSim(
  () => config.audio!,
  () => opsResult(OPS_DEFAULTS).findings,
  (buf) => broadcast(buf),
  (ev) => broadcast(JSON.stringify({ type: "audio_event", ...ev })),
  (active) => broadcast(JSON.stringify({ type: "audio_sources", active })),
);
// spoken calls for help (§5.7): speech status rides on status.audio (null without audio)
const speech = new SpeechSim(
  () => {
    const f = opsResult(OPS_DEFAULTS).findings;
    return (f.voids[0] ?? f.shadows[0])?.bearing_deg ?? 144;
  },
  () => !!config.audio?.calibrated,
  (ev) => broadcast(JSON.stringify({ type: "speech_event", ...ev })),
  (level, msg) => sendLog(level, msg),
);
Object.defineProperty(status, "audio", {
  get: () => (audio.status.connected ? { ...audio.status, speech: speech.status() } : { ...audio.status, speech: null }),
  enumerable: true,
});
// alarms: the speech_help alarm text while a call for help is fresh
Object.defineProperty(status, "alarms", {
  get: () => {
    const a = speech.alarm();
    return a ? [a] : [];
  },
  enumerable: true,
});

// handheld room mapping
const room = new RoomSim(
  (r) => broadcast(JSON.stringify({ type: "room", ...r })),
  (buf) => broadcast(buf),
);

// The ESP32-S3 Wi-Fi relay: tcp://host:8899 (lidar) and tcp://host:8900 (servo)
const isTcpPort = (p: string | undefined) => /^tcp:\/\/[^/\s:]+:\d+$/i.test(p ?? "");

const FAKE_PORTS = [
  { path: "/dev/cu.usbserial-0001", description: "CP2102N USB to UART Bridge Controller" },
  { path: "/dev/cu.usbmodem1101", description: "USB JTAG/serial debug unit" },
  { path: "/dev/cu.Bluetooth-Incoming-Port", description: "n/a" },
];

interface ServoSim {
  servo_deg: number;
  min_deg: number;
  max_deg: number;
  range: number;
  us_min: number;
  us_max: number;
}
const servoSim: ServoSim = { servo_deg: 135, min_deg: 0, max_deg: 270, range: 270, us_min: 500, us_max: 2500 };

interface SessionData {
  meta: SessionMeta;
  xyz: Float32Array;
  phi: Float32Array;
  theta: Float32Array;
  range: Float32Array;
  quality: Uint8Array;
}
const sessions = new Map<string, SessionData>();

let scanSeq = 0;
let lastScan: FakeScan | null = null;
let scanTimer: ReturnType<typeof setInterval> | null = null;
let nextJobId = 1;
let stopRequested = false;
let loadJobId = 1_000_000; // job_id used for replayed session chunks
let jobPhiMax = 60; // |φ| reach of the last job's sweep (sets the blind stripe width)
let lastJobSteps = 0; // for /api/metrics
let lastJobDuration: number | null = null;
let sessionPhiMax = 60; // same for the loaded session
let cloudVersion = 0; // bumped whenever the live cloud changes (analysis cache key)

const sockets = new Set<ServerWebSocket<unknown>>();

// ---- helpers ----

const now = () => Date.now() / 1000;
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

function phiFromServo(servoDeg: number): number {
  const sign = config.mount.invert ? -1 : 1;
  return sign * (servoDeg - config.mount.center_deg);
}
function servoFromPhi(phi: number): number {
  const sign = config.mount.invert ? -1 : 1;
  return config.mount.center_deg + sign * phi;
}
function currentPhi(): number {
  return status.servo.connected ? phiFromServo(servoSim.servo_deg) : 0;
}

function broadcast(data: string | ArrayBuffer) {
  for (const ws of sockets) {
    try {
      ws.send(data);
    } catch {
      // socket went away
    }
  }
}
function sendStatus() {
  broadcast(JSON.stringify({ type: "status", ...status }));
}
function sendLog(level: "info" | "warn" | "error", msg: string) {
  console.log(`[${level}] ${msg}`);
  broadcast(JSON.stringify({ type: "log", level, msg, t: now() }));
}
function sendJob() {
  if (status.job) broadcast(JSON.stringify({ type: "job", ...status.job }));
}

function refreshServoStatus() {
  if (!status.servo.connected) return;
  status.servo.servo_deg = servoSim.servo_deg;
  status.servo.phi_deg = phiFromServo(servoSim.servo_deg);
  status.servo.min_deg = servoSim.min_deg;
  status.servo.max_deg = servoSim.max_deg;
  status.servo.range = servoSim.range;
}

function startScanning() {
  if (scanTimer) return;
  scanTimer = setInterval(() => {
    if (!status.lidar.connected) return;
    const s = fakeScan(config.mount as Mount, currentPhi(), 3200, config.lidar.max_range_m);
    lastScan = s;
    scanSeq++;
    let rmin = Infinity;
    let rmax = 0;
    for (let i = 0; i < s.n; i++) {
      if (s.range[i] < rmin) rmin = s.range[i];
      if (s.range[i] > rmax) rmax = s.range[i];
    }
    status.lidar.pts_per_rev = s.n;
    status.lidar.range_min_m = +rmin.toFixed(3);
    status.lidar.range_max_m = +rmax.toFixed(3);
    status.lidar.scan_hz = +(10 + (Math.random() - 0.5) * 0.2).toFixed(2);
    broadcast(
      encodeScan({
        seq: scanSeq,
        t_unix: now(),
        phi_deg: currentPhi(),
        n: s.n,
        angle: s.angle,
        range: s.range,
        quality: s.quality,
      }),
    );
  }, SCAN_PERIOD_MS);
}
function stopScanning() {
  if (scanTimer) clearInterval(scanTimer);
  scanTimer = null;
  lastScan = null;
}

setInterval(sendStatus, 1000);

// ---- fake sessions ----

function buildSweep(name: string, phiStart: number, phiEnd: number, phiStep: number, rotations: number) {
  const steps: number[] = [];
  const dir = phiEnd >= phiStart ? 1 : -1;
  for (let k = 0; ; k++) {
    const phi = phiStart + dir * k * phiStep;
    if (dir * (phi - phiEnd) > 1e-9) break;
    steps.push(phi);
    if (k > 10_000) break;
  }
  const chunks: { phi: number; scan: FakeScan; xyz: Float32Array }[] = [];
  let total = 0;
  for (const phi of steps) {
    for (let r = 0; r < rotations; r++) {
      const scan = filterQuality(fakeScan(config.mount as Mount, phi, 3200, config.lidar.max_range_m), config.lidar.min_quality);
      chunks.push({ phi, scan, xyz: scanToPoints(config.mount as Mount, phi, scan) });
      total += scan.n;
    }
  }
  const xyz = new Float32Array(total * 3);
  const phiA = new Float32Array(total);
  const theta = new Float32Array(total);
  const range = new Float32Array(total);
  const quality = new Uint8Array(total);
  let o = 0;
  for (const c of chunks) {
    xyz.set(c.xyz, o * 3);
    phiA.fill(c.phi, o, o + c.scan.n);
    theta.set(c.scan.angle, o);
    range.set(c.scan.range, o);
    quality.set(c.scan.quality, o);
    o += c.scan.n;
  }
  const created = now();
  const id = `${new Date(created * 1000).toISOString().replace(/[-:]/g, "").slice(0, 15).replace("T", "-")}-${name}`;
  const meta: SessionMeta = {
    id,
    name,
    created,
    n_points: total,
    n_steps: steps.length,
    phi_start: phiStart,
    phi_end: phiEnd,
    phi_step: phiStep,
    rotations,
    lidar: { model: "S3", serial: "A1B2C3D4E5F6" },
    mount: { ...config.mount },
  };
  return { meta, xyz, phi: phiA, theta, range, quality } as SessionData;
}

{
  const demo = buildSweep("demo-room", -60, 60, 4, 1);
  demo.meta.id = "20261022-203000-demo-room";
  demo.meta.created = now() - 3600;
  sessions.set(demo.meta.id, demo);
}

function toPly(s: SessionData): string {
  const n = s.meta.n_points;
  const lines: string[] = [
    "ply",
    "format ascii 1.0",
    `comment obslam session ${s.meta.id}`,
    `element vertex ${n}`,
    "property float x",
    "property float y",
    "property float z",
    "end_header",
  ];
  for (let i = 0; i < n; i++) {
    lines.push(`${s.xyz[i * 3].toFixed(4)} ${s.xyz[i * 3 + 1].toFixed(4)} ${s.xyz[i * 3 + 2].toFixed(4)}`);
  }
  return lines.join("\n") + "\n";
}

function toCsv(s: SessionData): string {
  const n = s.meta.n_points;
  const lines: string[] = ["phi,phi_meas,theta,range,quality,x,y,z"];
  for (let i = 0; i < n; i++) {
    lines.push(
      `${s.phi[i]},${s.phi[i]},${s.theta[i].toFixed(3)},${s.range[i].toFixed(4)},${s.quality[i]},` +
        `${s.xyz[i * 3].toFixed(4)},${s.xyz[i * 3 + 1].toFixed(4)},${s.xyz[i * 3 + 2].toFixed(4)}`,
    );
  }
  return lines.join("\n") + "\n";
}

function gridFrame(xyz: Float32Array, n: number, q: URLSearchParams): ArrayBuffer {
  const cell = Number(q.get("cell") ?? config.grid.cell_m) || config.grid.cell_m;
  const layerQ = q.get("layer");
  const layer: GridLayer = layerQ === "min" || layerQ === "mean" || layerQ === "count" || layerQ === "max" ? layerQ : config.grid.layer;
  const zmin = q.get("z_min") ? Number(q.get("z_min")) : config.grid.z_min_m;
  const zmax = q.get("z_max") ? Number(q.get("z_max")) : config.grid.z_max_m;
  return encodeGrid(computeGrid(xyz, n, cell, layer, zmin, zmax));
}

// ---- job runner ----

let jobXyz = new Float32Array(0);
let jobN = 0;

// ---- operator products ----

function liveCloud(): { xyz: Float32Array; n: number } {
  if (status.session_loaded && sessions.has(status.session_loaded)) {
    const s = sessions.get(status.session_loaded)!;
    return { xyz: s.xyz, n: s.meta.n_points };
  }
  return { xyz: jobXyz, n: jobN };
}

function opsParamsFrom(get: (k: string) => string | number | null | undefined): OpsParams {
  const num = (k: string, d: number) => {
    const v = get(k);
    if (v === null || v === undefined || v === "") return d;
    const x = Number(v);
    if (!Number.isFinite(x)) throw new RangeError(`${k} must be a number`);
    return x;
  };
  const p = { cell: num("cell", OPS_DEFAULTS.cell), maxStep: num("max_step", OPS_DEFAULTS.maxStep), maxSlope: num("max_slope", OPS_DEFAULTS.maxSlope) };
  if (!(p.cell > 0)) throw new RangeError("cell must be > 0");
  return p;
}

let opsCache: { key: string; res: OpsResult } | null = null;
function opsResult(p: OpsParams): OpsResult {
  const key = `${p.cell}|${p.maxStep}|${p.maxSlope}|${cloudVersion}|${status.session_loaded ?? ""}`;
  if (opsCache && opsCache.key === key) return opsCache.res;
  const { xyz, n } = liveCloud();
  const scannerZ = (config.mount.axis_z_mm + config.mount.h_mm) / 1000;
  const phiMax = status.session_loaded && sessions.has(status.session_loaded) ? sessionPhiMax : jobPhiMax;
  const res = analyze(xyz, n, p, scannerZ, phiMax);
  opsCache = { key, res };
  return res;
}

async function runJob(req: JobRequest, jobId: number) {
  const steps: number[] = [];
  const dir = req.phi_end >= req.phi_start ? 1 : -1;
  for (let k = 0; ; k++) {
    const phi = req.phi_start + dir * k * req.phi_step;
    if (dir * (phi - req.phi_end) > 1e-9) break;
    steps.push(phi);
    if (k > 10_000) break;
  }
  const perStepS = (req.settle_ms + req.rotations * SCAN_PERIOD_MS) / 1000;
  const job: Job = {
    job_id: jobId,
    name: req.name,
    state: "running",
    step: 0,
    n_steps: steps.length,
    phi_deg: steps[0] ?? 0,
    points: 0,
    started: now(),
    eta_s: steps.length * perStepS,
    error: null,
  };
  status.job = job;
  status.session_loaded = null;
  jobXyz = new Float32Array(steps.length * req.rotations * 3400 * 3);
  jobN = 0;
  jobPhiMax = Math.max(Math.abs(req.phi_start), Math.abs(req.phi_end)) || 60;
  cloudVersion++;
  sendLog("info", `job ${jobId} "${req.name}": ${steps.length} steps from ${req.phi_start} to ${req.phi_end} by ${req.phi_step}`);
  sendStatus();
  sendJob();

  const rec: { phi: number; scan: FakeScan }[] = [];
  let lastGrid = 0;
  for (let i = 0; i < steps.length; i++) {
    if (stopRequested) {
      job.state = "stopped";
      break;
    }
    const phi = steps[i];
    servoSim.servo_deg = clamp(servoFromPhi(phi), servoSim.min_deg, servoSim.max_deg);
    refreshServoStatus();
    job.phi_deg = phi;
    await sleep(req.settle_ms);
    let pts = new Float32Array(0);
    for (let r = 0; r < req.rotations; r++) {
      await sleep(SCAN_PERIOD_MS);
      const scan = filterQuality(lastScan ?? fakeScan(config.mount as Mount, phi, 3200, config.lidar.max_range_m), config.lidar.min_quality);
      const xyz = scanToPoints(config.mount as Mount, phi, scan);
      rec.push({ phi, scan });
      const merged = new Float32Array(pts.length + xyz.length);
      merged.set(pts);
      merged.set(xyz, pts.length);
      pts = merged;
    }
    if (stopRequested) {
      job.state = "stopped";
      break;
    }
    const n = pts.length / 3;
    if ((jobN + n) * 3 > jobXyz.length) {
      const bigger = new Float32Array(Math.max(jobXyz.length * 2, (jobN + n) * 3));
      bigger.set(jobXyz.subarray(0, jobN * 3));
      jobXyz = bigger;
    }
    jobXyz.set(pts, jobN * 3);
    jobN += n;
    cloudVersion++;
    job.step = i + 1;
    job.points += n;
    job.eta_s = +((steps.length - job.step) * perStepS).toFixed(1);
    status.live_points = jobN;
    broadcast(encodePoints({ job_id: jobId, step: job.step, phi_deg: phi, n, xyz: pts }));
    sendJob();
    if (Date.now() - lastGrid > 1000) {
      lastGrid = Date.now();
      broadcast(gridFrame(jobXyz, jobN, new URLSearchParams()));
    }
  }
  if (job.state === "running") job.state = "done";
  job.eta_s = 0;
  lastJobSteps = job.step;
  lastJobDuration = +(now() - job.started).toFixed(1);
  broadcast(gridFrame(jobXyz, jobN, new URLSearchParams()));
  sendJob();
  if (job.state === "done" || (job.state === "stopped" && jobN > 0)) {
    // save as a session (what the real core does with sweeps.npz)
    let total = 0;
    for (const r of rec) total += r.scan.n;
    const xyz = jobXyz.slice(0, total * 3);
    const phiA = new Float32Array(total);
    const theta = new Float32Array(total);
    const range = new Float32Array(total);
    const quality = new Uint8Array(total);
    let o = 0;
    for (const r of rec) {
      phiA.fill(r.phi, o, o + r.scan.n);
      theta.set(r.scan.angle, o);
      range.set(r.scan.range, o);
      quality.set(r.scan.quality, o);
      o += r.scan.n;
    }
    const created = now();
    const id = `${new Date(created * 1000).toISOString().replace(/[-:]/g, "").slice(0, 15).replace("T", "-")}-${req.name}`;
    sessions.set(id, {
      meta: {
        id,
        name: req.name,
        created,
        n_points: total,
        n_steps: job.step,
        phi_start: req.phi_start,
        phi_end: req.phi_end,
        phi_step: req.phi_step,
        rotations: req.rotations,
        lidar: { model: "S3", serial: "A1B2C3D4E5F6" },
        mount: { ...config.mount },
      },
      xyz,
      phi: phiA,
      theta,
      range,
      quality,
    });
    sendLog("info", `job ${jobId} ${job.state}: ${total} points saved as session ${id}`);
  } else {
    sendLog("warn", `job ${jobId} ${job.state} with no points`);
  }
  sendStatus();
}

// ---- servo raw protocol ----

function servoRaw(line: string): string {
  const parts = line.trim().split(/\s+/);
  const cmd = (parts[0] ?? "").toUpperCase();
  const num = (i: number) => Number(parts[i]);
  switch (cmd) {
    case "I":
      return "OBSLAM-TILT v1";
    case "?":
      return `STATUS angle=${servoSim.servo_deg} min=${servoSim.min_deg} max=${servoSim.max_deg} range=${servoSim.range} us_min=${servoSim.us_min} us_max=${servoSim.us_max}`;
    case "T":
    case "S": {
      if (!Number.isFinite(num(1))) return "ERR bad angle";
      servoSim.servo_deg = clamp(num(1), servoSim.min_deg, servoSim.max_deg);
      return `OK ${servoSim.servo_deg}`;
    }
    case "H":
      servoSim.servo_deg = servoSim.range / 2;
      return `OK ${servoSim.servo_deg}`;
    case "L": {
      if (!Number.isFinite(num(1)) || !Number.isFinite(num(2)) || num(1) >= num(2)) return "ERR bad limits";
      servoSim.min_deg = clamp(num(1), 0, servoSim.range);
      servoSim.max_deg = clamp(num(2), 0, servoSim.range);
      return "OK";
    }
    case "C": {
      if (!Number.isFinite(num(1)) || !Number.isFinite(num(2)) || (num(3) !== 180 && num(3) !== 270)) return "ERR usage: C us_min us_max 180|270";
      servoSim.us_min = num(1);
      servoSim.us_max = num(2);
      servoSim.range = num(3);
      servoSim.max_deg = Math.min(servoSim.max_deg, servoSim.range);
      return "OK";
    }
    default:
      return `ERR unknown command ${parts[0] ?? ""}`;
  }
}

// ---- http ----

const json = (body: unknown, init: ResponseInit = {}) =>
  new Response(JSON.stringify(body), { ...init, headers: { "content-type": "application/json", ...(init.headers ?? {}) } });
const err = (code: number, detail: string) => json({ detail }, { status: code });
const bin = (buf: ArrayBuffer) => new Response(buf, { headers: { "content-type": "application/octet-stream" } });

async function body<T>(req: Request): Promise<T> {
  const text = await req.text();
  if (!text) return {} as T;
  return JSON.parse(text) as T;
}

function handleSession(id: string, rest: string, method: string, req: Request): Response {
  const s = sessions.get(id);
  if (!s) return err(404, `session ${id} not found`);
  if (rest === "" && method === "GET") return json(s.meta);
  if (rest === "" && method === "DELETE") {
    sessions.delete(id);
    if (status.session_loaded === id) status.session_loaded = null;
    sendLog("info", `session ${id} deleted`);
    return new Response(null, { status: 204 });
  }
  if (rest === "/points" && method === "GET") return bin(encodeSessionPoints(s.xyz));
  if (rest === "/grid" && method === "GET") return bin(gridFrame(s.xyz, s.meta.n_points, new URL(req.url).searchParams));
  if (rest === "/export.ply" && method === "GET")
    return new Response(toPly(s), {
      headers: { "content-type": "text/plain", "content-disposition": `attachment; filename="${id}.ply"` },
    });
  if (rest === "/export.csv" && method === "GET")
    return new Response(toCsv(s), {
      headers: { "content-type": "text/csv", "content-disposition": `attachment; filename="${id}.csv"` },
    });
  if (rest === "/load" && method === "POST") {
    if (status.job?.state === "running") return err(409, "a job is running");
    status.session_loaded = id;
    status.live_points = s.meta.n_points;
    sessionPhiMax = Math.max(Math.abs(s.meta.phi_start), Math.abs(s.meta.phi_end)) || 60;
    cloudVersion++;
    const jobId = ++loadJobId;
    // replay as kind-2 chunks (<= 50k points), then a grid
    queueMicrotask(async () => {
      const n = s.meta.n_points;
      let step = 0;
      for (let o = 0; o < n; o += POINTS_CHUNK) {
        const k = Math.min(POINTS_CHUNK, n - o);
        broadcast(encodePoints({ job_id: jobId, step: step++, phi_deg: s.phi[o], n: k, xyz: s.xyz.subarray(o * 3, (o + k) * 3) }));
        await sleep(20);
      }
      broadcast(gridFrame(s.xyz, n, new URLSearchParams()));
      sendLog("info", `session ${id} loaded (${n} points)`);
      sendStatus();
    });
    return json(status);
  }
  return err(404, "not found");
}

async function handleApi(req: Request, path: string): Promise<Response> {
  const m = req.method;
  const url = new URL(req.url);

  if (path === "/status" && m === "GET") return json(status);
  if (path === "/ports" && m === "GET") return json(FAKE_PORTS);

  if (path === "/lidar/connect" && m === "POST") {
    const b = await body<{ source?: string; port?: string; baud?: number }>(req);
    if (b.source !== "s2" && b.source !== "sim") return err(422, "source must be s2 or sim");
    if (b.source === "s2" && !FAKE_PORTS.some((p) => p.path === b.port) && !isTcpPort(b.port)) return err(404, `port ${b.port ?? ""} not found`);
    await sleep(300);
    status.lidar = {
      connected: true,
      source: b.source,
      port: b.source === "sim" ? "sim" : b.port,
      baud: b.baud ?? 1000000,
      model: "S3",
      serial: "A1B2C3D4E5F6",
      firmware: "1.02",
      hardware: 18,
      health: "good",
      scan_hz: 10,
      pts_per_rev: 0,
      range_min_m: 0,
      range_max_m: 0,
    };
    startScanning();
    sendLog("info", `lidar connected (${b.source}${b.source === "s2" ? " on " + b.port : ""})`);
    sendStatus();
    return json(status);
  }
  if (path === "/lidar/disconnect" && m === "POST") {
    if (status.job?.state === "running") return err(409, "stop the job first");
    stopScanning();
    status.lidar = { connected: false };
    sendLog("info", "lidar disconnected");
    sendStatus();
    return json(status);
  }

  if (path === "/servo/connect" && m === "POST") {
    const b = await body<{ port?: string }>(req);
    if (b.port !== "sim" && !FAKE_PORTS.some((p) => p.path === b.port) && !isTcpPort(b.port)) return err(404, `port ${b.port ?? ""} not found`);
    await sleep(200);
    status.servo = { connected: true, port: b.port };
    refreshServoStatus();
    sendLog("info", `servo connected (${b.port}): OBSLAM-TILT v1`);
    sendStatus();
    return json(status);
  }
  if (path === "/servo/disconnect" && m === "POST") {
    if (status.job?.state === "running") return err(409, "stop the job first");
    status.servo = { connected: false };
    sendStatus();
    return json(status);
  }
  if (path === "/servo/move" && m === "POST") {
    if (!status.servo.connected) return err(409, "servo not connected");
    const b = await body<{ servo_deg?: number; phi_deg?: number }>(req);
    let target: number;
    if (typeof b.servo_deg === "number") target = b.servo_deg;
    else if (typeof b.phi_deg === "number") target = servoFromPhi(b.phi_deg);
    else return err(422, "servo_deg or phi_deg required");
    servoSim.servo_deg = clamp(target, servoSim.min_deg, servoSim.max_deg);
    refreshServoStatus();
    sendStatus();
    return json(status);
  }
  if (path === "/servo/home" && m === "POST") {
    if (!status.servo.connected) return err(409, "servo not connected");
    servoSim.servo_deg = servoSim.range / 2;
    refreshServoStatus();
    sendStatus();
    return json(status);
  }
  if (path === "/servo/raw" && m === "POST") {
    if (!status.servo.connected) return err(409, "servo not connected");
    const b = await body<{ line?: string }>(req);
    const reply = servoRaw(String(b.line ?? ""));
    refreshServoStatus();
    sendStatus();
    return json({ reply });
  }

  if (path === "/job/start" && m === "POST") {
    if (!status.lidar.connected) return err(409, "lidar not connected");
    if (!status.servo.connected) return err(409, "servo not connected");
    if (status.job?.state === "running") return err(409, "a job is already running");
    const b = await body<Partial<JobRequest>>(req);
    const reqJob: JobRequest = {
      name: (b.name || "job").replace(/[^\w.-]+/g, "-"),
      phi_start: Number(b.phi_start ?? -60),
      phi_end: Number(b.phi_end ?? 60),
      phi_step: Number(b.phi_step ?? 2),
      rotations: Math.max(1, Math.floor(Number(b.rotations ?? 1))),
      settle_ms: Math.max(0, Number(b.settle_ms ?? 300)),
    };
    if (!(reqJob.phi_step > 0)) return err(422, "phi_step must be > 0");
    stopRequested = false;
    const id = nextJobId++;
    void runJob(reqJob, id);
    return json({ job_id: id });
  }
  if (path === "/job/stop" && m === "POST") {
    if (status.job?.state === "running") {
      stopRequested = true;
      sendLog("warn", "job stop requested");
    }
    return json(status);
  }

  if (path === "/grid" && m === "GET") {
    if (status.session_loaded && sessions.has(status.session_loaded)) {
      const s = sessions.get(status.session_loaded)!;
      return bin(gridFrame(s.xyz, s.meta.n_points, url.searchParams));
    }
    return bin(gridFrame(jobXyz, jobN, url.searchParams));
  }

  // ---- evaluation numbers ----
  if (path === "/metrics" && m === "GET") {
    const { xyz, n } = liveCloud();
    const cell = Number(url.searchParams.get("cell") ?? config.grid.cell_m) || config.grid.cell_m;
    if (!(cell > 0)) return err(400, "cell must be > 0");
    const zmin = url.searchParams.get("z_min") ? Number(url.searchParams.get("z_min")) : config.grid.z_min_m;
    const zmax = url.searchParams.get("z_max") ? Number(url.searchParams.get("z_max")) : config.grid.z_max_m;
    const g = computeGrid(xyz, n, cell, "max", zmin, zmax);
    let filled = 0;
    for (let i = 0; i < g.data.length; i++) if (Number.isFinite(g.data[i])) filled++;
    const cells = g.w * g.h;
    const loaded = status.session_loaded ? sessions.get(status.session_loaded) : undefined;
    const steps = loaded ? loaded.meta.n_steps : lastJobSteps || null;
    const floor = fitPlaneNear(xyz, n, 0, 0.1);
    return json({
      points: n,
      steps,
      points_per_step: steps ? Math.round(n / steps) : null,
      duration_s: loaded ? null : lastJobDuration,
      grid: {
        cell_m: cell,
        cells,
        filled,
        coverage_pct: cells ? +((100 * filled) / cells).toFixed(1) : 0,
        points_per_filled_cell: filled ? +(n / filled).toFixed(1) : null,
      },
      floor: floor ? { points: floor.points, rms_mm: floor.rms_mm, tilt_deg: floor.tilt_deg, offset_mm: Math.round(floor.coef[2] * 1000) } : null,
    });
  }

  // ---- operator products ----
  if ((path === "/ops/findings" || path === "/ops/layers") && m === "GET") {
    let p: OpsParams;
    try {
      p = opsParamsFrom((k) => url.searchParams.get(k));
    } catch (e) {
      return err(400, (e as Error).message);
    }
    const res = opsResult(p);
    if (path === "/ops/findings")
      return json({
        ...res.findings,
        scan: {
          in_progress: status.job?.state === "running",
          step: status.job?.step ?? null,
          n_steps: status.job?.n_steps ?? null,
          session: status.session_loaded,
        },
      });
    return bin(encodeOpsLayers({ w: res.w, h: res.h, cell_m: res.cell, x0_m: res.x0, y0_m: res.y0, classes: res.classes, height: res.rel }));
  }
  if (path === "/ops/route" && m === "POST") {
    const b = await body<{ from_xy?: unknown; to_xy?: unknown; cell?: number; max_step?: number; max_slope?: number }>(req);
    const xy = (v: unknown): [number, number] | null =>
      Array.isArray(v) && v.length === 2 && v.every((x) => typeof x === "number" && Number.isFinite(x)) ? [v[0], v[1]] : null;
    const from = xy(b.from_xy);
    const to = xy(b.to_xy);
    if (!from || !to) return err(400, "from_xy and to_xy are [x, y] in metres");
    let p: OpsParams;
    try {
      p = opsParamsFrom((k) => (b as Record<string, number | undefined>)[k]);
    } catch (e) {
      return err(400, (e as Error).message);
    }
    return json(planRoute(opsResult(p), from, to));
  }

  // ---- acoustic layer ----
  if (path === "/audio/connect" && m === "POST") {
    const b = await body<{ source?: string; port?: string }>(req);
    if (b.source !== "tcp" && b.source !== "sim") return err(422, "source must be tcp or sim");
    if (b.source === "tcp" && !/^tcp:\/\/[^/\s:]+:\d+$/i.test(b.port ?? "")) return err(422, "port must be tcp://host:8901");
    await sleep(200);
    audio.start(b.source, b.source === "sim" ? "sim" : b.port!);
    speech.start();
    if (b.source === "tcp") config.audio!.port = b.port!;
    sendLog("info", `audio connected (${b.source}${b.source === "tcp" ? " " + b.port : ""})`);
    sendStatus();
    return json(status);
  }
  if (path === "/audio/disconnect" && m === "POST") {
    audio.stop();
    speech.stop();
    sendLog("info", "audio disconnected");
    sendStatus();
    return json(status);
  }
  if (path === "/audio/events" && m === "GET") {
    const limit = Math.max(1, Math.min(500, Number(url.searchParams.get("limit") ?? 50) || 50));
    return json(audio.events.slice(0, limit));
  }
  if (path === "/audio/clear" && m === "POST") {
    audio.clear();
    return new Response(null, { status: 204 });
  }
  if (path === "/audio/sim/emit" && m === "POST") {
    if (!audio.status.connected) return err(409, "audio not connected");
    if (audio.status.source !== "sim") return err(409, "sim source only");
    const b = await body<{ bearing_deg?: number; elevation_deg?: number; snr_db?: number; kind?: string }>(req);
    const bearing = Number(b.bearing_deg ?? 0);
    if (!Number.isFinite(bearing)) return err(422, "bearing_deg must be a number");
    const kind = b.kind === "voice" ? "voice" : "knock";
    return json(audio.emit(((bearing % 360) + 360) % 360, Number(b.elevation_deg ?? 0) || 0, Number(b.snr_db ?? 20) || 20, kind));
  }
  if (path === "/audio/sources" && m === "GET") return json(audio.sources());

  // ---- spoken calls for help (§5.7) ----
  if (path === "/audio/speech" && m === "GET") {
    const limit = Math.max(0, Math.min(100, Number(url.searchParams.get("limit") ?? 50) || 50));
    return json({ status: audio.status.connected ? speech.status() : speech.status(), events: speech.events.slice(0, limit) });
  }
  if (path === "/audio/speech" && m === "POST") {
    const b = await body<{ enabled?: unknown }>(req);
    if (typeof b.enabled !== "boolean") return err(422, "enabled must be true or false");
    speech.setEnabled(b.enabled);
    sendStatus();
    return json(speech.status());
  }
  if (path === "/audio/speech/clear" && m === "POST") {
    speech.clear();
    return new Response(null, { status: 204 });
  }

  // ---- room map (handheld 2D mapping) ----
  if (path === "/room/start" && m === "POST") {
    const b = await body<{ cell_m?: number; mode?: string; planned?: number }>(req);
    if (b.cell_m !== undefined && !(Number(b.cell_m) > 0)) return err(422, "cell_m must be > 0");
    if (b.mode !== undefined && b.mode !== "walk" && b.mode !== "stations") return err(422, "mode must be walk or stations");
    const mode = b.mode === "stations" ? "stations" : "walk";
    room.start(b.cell_m !== undefined ? Number(b.cell_m) : undefined, mode, Number(b.planned ?? 6));
    sendLog("info", mode === "walk" ? "room mapping started (walk mode)" : `room mapping started (station mode, ${Number(b.planned ?? 6)} stations planned)`);
    return json(room.snapshot(true));
  }
  if (path === "/room/station" && m === "POST") {
    try {
      const r = await room.addStation();
      if (r.ok) sendLog("info", `station ${r.n} added at x ${r.x.toFixed(2)} m, y ${r.y.toFixed(2)} m (${Math.round(r.inliers * 100)}% matched)`);
      else sendLog("warn", `station not added: ${r.reason}`);
      return json(r);
    } catch (e) {
      if (e instanceof RoomConflict) return json({ error: e.message }, { status: 409 });
      throw e;
    }
  }
  if (path === "/room/stop" && m === "POST") {
    room.stop();
    return json(room.snapshot(true));
  }
  if (path === "/room" && m === "GET") return json(room.snapshot(true));
  if (path === "/room/grid" && m === "GET") return bin(room.gridFrame());
  if (path === "/audio/calibrate" && m === "POST") {
    const b = await body<{ clicks?: { bearing_deg: number; elevation_deg?: number; event_id?: number; tdoa_us?: Record<string, number> }[]; apply?: boolean }>(req);
    if (!Array.isArray(b.clicks)) return err(422, "clicks required");
    let res;
    try {
      res = audio.calibrate(b.clicks);
    } catch (e) {
      return err(400, (e as Error).message);
    }
    const out: Record<string, unknown> = { ...res, applied: false };
    if (b.apply) {
      const a = config.audio!;
      a.array_yaw_deg = res.array_yaw_deg;
      a.channel_offsets_us = res.channel_offsets_us;
      a.calibrated = res.ok;
      out.applied = true;
      out.calibrated = a.calibrated;
      sendLog("info", `audio calibration applied: yaw ${res.array_yaw_deg}°, residual ${res.residual_us} µs, ${res.ok ? "calibrated" : "not calibrated"}`);
      sendStatus();
    }
    return json(out);
  }

  // ---- assistant layer (canned, reads the findings only) ----
  if (path === "/ai/status" && m === "GET")
    return json(
      MOCK_AI
        ? { available: true, reason: "mock provider, canned answers", models: AI_MODELS }
        : { available: false, reason: "no OPENROUTER_API_KEY in the environment or core/.env", models: AI_MODELS },
    );
  if (path === "/ai/triage" && m === "POST") {
    if (!MOCK_AI) return err(503, "assistant unavailable: no OPENROUTER_API_KEY");
    const b = await body<{ cell?: number }>(req);
    await sleep(400);
    return json(aiTriage(opsResult({ ...OPS_DEFAULTS, cell: b.cell && b.cell > 0 ? b.cell : OPS_DEFAULTS.cell }).findings));
  }
  if (path === "/ai/sitrep" && m === "POST") {
    const b = await body<{ cell?: number }>(req);
    const f = opsResult({ ...OPS_DEFAULTS, cell: b.cell && b.cell > 0 ? b.cell : OPS_DEFAULTS.cell }).findings;
    if (!MOCK_AI) return json({ text: aiSitrep(f).text, facts: f, model: "template" }); // written from the findings, no model
    await sleep(600);
    return json(aiSitrep(f));
  }
  if (path === "/ai/ask" && m === "POST") {
    if (!MOCK_AI) return err(503, "assistant unavailable: no OPENROUTER_API_KEY");
    const b = await body<{ question?: string }>(req);
    if (!b.question || typeof b.question !== "string") return err(422, "question required");
    await sleep(500);
    const res = opsResult(OPS_DEFAULTS);
    return json(aiAsk(b.question, res.findings, (from, to) => planRoute(res, from, to), (x, y) => nearestPassable(res, x, y)));
  }

  if (path === "/sessions" && m === "GET") {
    return json([...sessions.values()].map((s) => s.meta).sort((a, b) => b.created - a.created));
  }
  const sm = path.match(/^\/sessions\/([^/]+)(\/.*)?$/);
  if (sm) return handleSession(decodeURIComponent(sm[1]), sm[2] ?? "", m, req);

  if (path === "/config" && m === "GET") return json(config);
  if (path === "/config" && m === "PUT") {
    const b = await body<Partial<Config>>(req);
    if (b.mount) Object.assign(config.mount, b.mount);
    if (b.grid) Object.assign(config.grid, b.grid);
    if (b.lidar) Object.assign(config.lidar, b.lidar);
    if (b.audio) Object.assign(config.audio!, b.audio);
    refreshServoStatus();
    sendLog("info", "config updated");
    sendStatus();
    return json(config);
  }

  return err(404, `no route for ${m} /api${path}`);
}

const server = Bun.serve<unknown>({
  port: PORT,
  hostname: "127.0.0.1",
  async fetch(req, srv) {
    const url = new URL(req.url);
    if (url.pathname === "/ws") {
      if (srv.upgrade(req, { data: {} })) return undefined as unknown as Response;
      return new Response("websocket upgrade failed", { status: 400 });
    }
    if (url.pathname.startsWith("/api/")) {
      try {
        return await handleApi(req, url.pathname.slice(4));
      } catch (e) {
        console.error(e);
        return err(500, (e as Error).message);
      }
    }
    return new Response("obslam mock core. REST under /api, stream at /ws. Run the dashboard with `bun run dev`.", {
      headers: { "content-type": "text/plain" },
    });
  },
  websocket: {
    open(ws) {
      sockets.add(ws);
      ws.send(JSON.stringify({ type: "status", ...status }));
      ws.send(JSON.stringify({ type: "log", level: "info", msg: "mock core: client connected", t: now() }));
    },
    message(ws, msg) {
      if (typeof msg === "string") {
        try {
          const j = JSON.parse(msg);
          if (j.type === "ping") ws.send(JSON.stringify({ type: "pong" }));
        } catch {
          // ignore
        }
      }
    },
    close(ws) {
      sockets.delete(ws);
    },
  },
});

console.log(`mock core listening on http://${server.hostname}:${server.port}  (ws://${server.hostname}:${server.port}/ws)`);
