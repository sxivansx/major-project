// Simulated four-microphone array for the mock core (docs/api.md §5): a noise
// floor per mic, a knock every ~6 s from a random direction, kind-5 preview
// frames at 20 Hz, and AudioEvents with a bearing, its uncertainty, a
// confidence, pair TDOAs and candidate regions drawn from the findings.

import { encodeAudio } from "../src/api/frames";
import type { AudioCandidate, AudioConfig, AudioEvent, AudioStatus, Finding, Findings } from "../src/api/types";

const CH = 4;
const RING = 4000; // ms of generated envelope kept
const FRAME_MS = 50; // 20 Hz
const BLOCK = 100; // points per frame = last 100 ms at 1 kHz
const FULL = 32767;

export const AUDIO_DEFAULT: AudioConfig = {
  port: "tcp://192.168.4.7:8901",
  mics_mm: [
    [15, 15, 0],
    [-15, 15, 0],
    [-15, -15, 0],
    [15, -15, 0],
  ],
  array_yaw_deg: 0,
  calibrated: false,
  trigger_db: 12,
  min_gap_ms: 300,
  speed_of_sound: 343.0,
};

function gauss(): number {
  let u = 0;
  let v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
const r = (v: number, d: number) => Math.round(v * 10 ** d) / 10 ** d;
const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));
const angDiff = (a: number, b: number) => Math.abs((((a - b) % 360) + 540) % 360 - 180);

// "ij" = t_i − t_j in µs: positive when mic i hears the source later.
export function tdoaFor(cfg: AudioConfig, bearingDeg: number, elevationDeg: number): Record<string, number> {
  const b = (bearingDeg * Math.PI) / 180;
  const e = (elevationDeg * Math.PI) / 180;
  const ux = Math.cos(b) * Math.cos(e);
  const uy = -Math.sin(b) * Math.cos(e);
  const uz = Math.sin(e);
  const y = (cfg.array_yaw_deg * Math.PI) / 180;
  const ax = Math.cos(y) * ux + Math.sin(y) * uy;
  const ay = -Math.sin(y) * ux + Math.cos(y) * uy;
  const arr = cfg.mics_mm.map((p) => -(p[0] * ax + p[1] * ay + p[2] * uz) / 1000 / cfg.speed_of_sound);
  const out: Record<string, number> = {};
  for (let i = 0; i < arr.length; i++)
    for (let j = i + 1; j < arr.length; j++) out[`${i + 1}${j + 1}`] = r((arr[i] - arr[j]) * 1e6 + gauss() * 2, 1);
  return out;
}

// Findings whose centroid lies inside bearing ± 2σ: voids first, then unseen
// regions, then obstacles; nearest first within each group.
export function candidatesFor(f: Findings | null, bearing: number, sigma: number): AudioCandidate[] {
  if (!f || f.grid.w === 0) return [];
  const pick = (items: Finding[], kind: string) =>
    items
      .filter((it) => angDiff(it.bearing_deg, bearing) <= 2 * sigma)
      .sort((a, b) => a.distance_m - b.distance_m)
      .map((it) => ({ kind, id: it.id, distance_m: it.distance_m, bearing_deg: it.bearing_deg }));
  return [...pick(f.voids, "void"), ...pick(f.shadows, "shadow"), ...pick(f.obstacles, "obstacle")];
}

interface Knock {
  t0: number; // ms
  amp: number; // peak envelope, counts
  decay: number; // ms
}

export class AudioSim {
  status: AudioStatus = { connected: false };
  events: AudioEvent[] = []; // newest first
  private nextId = 1;
  private seq = 0;
  private frames = 0;
  private timer: ReturnType<typeof setInterval> | null = null;
  private nextKnockAt = 0;
  private lastEventAt = 0;
  private knocks: Knock[] = [];
  private genMs = 0;
  private mins = Array.from({ length: CH }, () => new Int16Array(RING));
  private maxs = Array.from({ length: CH }, () => new Int16Array(RING));
  private noiseAmp = [260, 300, 240, 330];
  private noiseDb = [-60, -60, -60, -60];
  private clipUntil = [0, 0, 0, 0];
  private startMs = Date.now();

  constructor(
    private cfg: () => AudioConfig,
    private findings: () => Findings | null,
    private sendFrame: (buf: ArrayBuffer) => void,
    private sendEvent: (ev: AudioEvent) => void,
  ) {}

  start(source: "tcp" | "sim", port: string): void {
    this.stop();
    this.status = { connected: true, source, port, rate: 48000, frames: 0, lost_frames: 0, calibrated: this.cfg().calibrated };
    this.genMs = Date.now() - RING + 1;
    this.nextKnockAt = Date.now() + 2500;
    this.timer = setInterval(() => this.tick(), FRAME_MS);
  }

  stop(): void {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
    this.status = { connected: false };
    this.knocks = [];
  }

  clear(): void {
    this.events = [];
  }

  // A knock from a given direction, detected now (Test knock / sim emit).
  emit(bearingDeg: number, elevationDeg = 0, snrDb = 20): AudioEvent {
    const t0 = Date.now();
    this.addKnock(t0, snrDb);
    return this.detect(t0, bearingDeg, elevationDeg, snrDb, true);
  }

  private addKnock(t0: number, snrDb: number) {
    const noise = this.noiseAmp.reduce((a, b) => a + b, 0) / CH;
    this.knocks.push({ t0, amp: noise * 10 ** (snrDb / 20), decay: 25 + Math.random() * 25 });
  }

  private detect(t0: number, trueBearing: number, elevation: number, snrDb: number, forced: boolean): AudioEvent {
    const cfg = this.cfg();
    const unresolved = !forced && snrDb < 9 && Math.random() < 0.3;
    const sigma = clamp(2.5 + 70 / Math.max(snrDb, 1) + elevation / 10, 3, 30);
    // the error stays within 1σ so the true direction is always inside ± 2σ
    const bearing = ((trueBearing + (Math.random() * 2 - 1) * sigma) % 360 + 360) % 360;
    const confidence = unresolved ? 0.12 : clamp((0.25 + snrDb / 40) * (0.85 + Math.random() * 0.15), 0.05, 0.97);
    const peak = Math.min(FULL, (this.noiseAmp.reduce((a, b) => a + b, 0) / CH) * 10 ** (snrDb / 20));
    const ev: AudioEvent = {
      id: this.nextId++,
      t: t0 / 1000,
      t_us: (t0 - this.startMs) * 1000,
      peak_dbfs: r(20 * Math.log10(peak / 32768), 1),
      snr_db: r(snrDb, 1),
      bearing_deg: unresolved ? null : r(bearing, 1),
      bearing_sigma_deg: unresolved ? null : r(sigma, 1),
      elevation_deg: unresolved ? null : r(Math.abs(elevation), 1),
      confidence: r(confidence, 2),
      tdoa_us: tdoaFor(cfg, trueBearing, elevation),
      candidates: unresolved ? [] : candidatesFor(this.findings(), bearing, sigma),
      label: "impulse",
    };
    this.events.unshift(ev);
    if (this.events.length > 200) this.events.length = 200;
    this.lastEventAt = t0;
    this.sendEvent(ev);
    return ev;
  }

  private generate(upTo: number) {
    if (upTo - this.genMs > RING) this.genMs = upTo - RING;
    this.knocks = this.knocks.filter((k) => upTo - k.t0 < 400);
    for (let ms = this.genMs + 1; ms <= upTo; ms++) {
      const k = ((ms % RING) + RING) % RING;
      let burst = 0;
      for (const kn of this.knocks) if (ms >= kn.t0) burst += kn.amp * Math.exp(-(ms - kn.t0) / kn.decay) * (0.6 + 0.4 * Math.random());
      for (let c = 0; c < CH; c++) {
        const n = this.noiseAmp[c] * (0.7 + 0.5 * Math.abs(gauss()));
        const b = burst * (0.85 + 0.3 * Math.random());
        const hi = Math.min(FULL, Math.round(n + b));
        const lo = Math.max(-FULL, -Math.round(n * (0.8 + 0.4 * Math.random()) + b * 0.9));
        this.maxs[c][k] = hi;
        this.mins[c][k] = lo;
        if (hi >= 32700 || lo <= -32700) this.clipUntil[c] = ms + 1000;
      }
    }
    this.genMs = upTo;
  }

  private tick() {
    const now = Date.now();
    const cfg = this.cfg();
    // scheduled knocks; detected only above the trigger and outside the dead time
    if (now >= this.nextKnockAt) {
      this.nextKnockAt = now + 5000 + Math.random() * 2500;
      const snr = 5 + Math.random() * 28 + (Math.random() < 0.1 ? 14 : 0); // the odd one clips
      this.addKnock(now, snr);
      if (snr >= cfg.trigger_db && now - this.lastEventAt >= cfg.min_gap_ms) {
        const bearing = Math.random() * 360;
        const elev = Math.random() * 35;
        setTimeout(() => this.detect(now, bearing, elev, snr, false), 40);
      }
    }
    this.generate(now);
    const mins: Int16Array[] = [];
    const maxs: Int16Array[] = [];
    const levels = new Float32Array(CH);
    for (let c = 0; c < CH; c++) {
      const mn = new Int16Array(BLOCK);
      const mx = new Int16Array(BLOCK);
      let acc = 0;
      for (let i = 0; i < BLOCK; i++) {
        const ms = now - (BLOCK - 1 - i);
        const k = ((ms % RING) + RING) % RING;
        mn[i] = this.mins[c][k];
        mx[i] = this.maxs[c][k];
        const a = (mx[i] - mn[i]) / 2 / Math.SQRT2;
        acc += a * a;
      }
      levels[c] = r(20 * Math.log10(Math.max(1, Math.sqrt(acc / BLOCK)) / 32768), 1);
      // noise floor follows the level slowly, and only downwards quickly
      this.noiseDb[c] = levels[c] < this.noiseDb[c] ? this.noiseDb[c] * 0.7 + levels[c] * 0.3 : this.noiseDb[c] * 0.995 + levels[c] * 0.005;
      mins.push(mn);
      maxs.push(mx);
    }
    this.seq++;
    this.frames += 5; // one preview frame covers five 10 ms stream frames
    this.status = {
      ...this.status,
      frames: this.frames,
      lost_frames: 0,
      levels_dbfs: Array.from(levels),
      noise_dbfs: this.noiseDb.map((v) => r(v, 1)),
      clipping: this.clipUntil.map((t) => t > now),
      calibrated: cfg.calibrated,
    };
    this.sendFrame(encodeAudio({ channels: CH, n: BLOCK, seq: this.seq, t_unix: now / 1000, levels_dbfs: levels, mins, maxs }));
  }
}
