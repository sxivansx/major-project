// Simulated four-microphone array for the mock core (docs/api.md §5): a noise
// floor per mic (mic 3 noisy, so it gets excluded), kind-5 preview frames at
// 20 Hz, AudioEvents with a bearing, its uncertainty, a confidence, pair TDOAs
// and candidate regions from the findings, grouped into sources (§5.6), and a
// clap calibration fit (§5.5).
//
// What happens after connect (sim): a burst of tapping from one direction
// (aimed at the first void, else 112°) every ~45 s, a single knock from a
// random direction every 9–14 s (sometimes with an echo detected again from a
// wrong direction), and scattered low-confidence noise detections.

import { encodeAudio } from "../src/api/frames";
import type {
  AudioCalibrateClick,
  AudioCalibrateResult,
  AudioCandidate,
  AudioConfig,
  AudioEvent,
  AudioSource,
  AudioStatus,
  Finding,
  Findings,
} from "../src/api/types";

const CH = 4;
const RING = 4000; // ms of generated envelope kept
const FRAME_MS = 50; // 20 Hz
const BLOCK = 100; // points per frame = last 100 ms at 1 kHz
const FULL = 32767;
const NOISY_MIC = process.env.MOCK_NOISY_MIC === "off" ? -1 : 2; // 0-based: mic 3

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
  channel_offsets_us: [0, 0, 0, 0],
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
const norm360 = (a: number) => ((a % 360) + 360) % 360;
const angDiff = (a: number, b: number) => Math.abs(((a - b) % 360 + 540) % 360 - 180);
const DEG = Math.PI / 180;
const PAIRS = ["12", "13", "14", "23", "24", "34"];

// "ij" = t_i − t_j in µs: positive when mic i hears the source later.
function tdoaClean(mics: number[][], yawDeg: number, c: number, bearingDeg: number, elevationDeg: number): number[] {
  const b = bearingDeg * DEG;
  const e = elevationDeg * DEG;
  const ux = Math.cos(b) * Math.cos(e);
  const uy = -Math.sin(b) * Math.cos(e);
  const uz = Math.sin(e);
  const y = yawDeg * DEG;
  const ax = Math.cos(y) * ux + Math.sin(y) * uy;
  const ay = -Math.sin(y) * ux + Math.cos(y) * uy;
  const arr = mics.map((p) => -(p[0] * ax + p[1] * ay + (p[2] ?? 0) * uz) / 1000 / c);
  return PAIRS.map((p) => (arr[+p[0] - 1] - arr[+p[1] - 1]) * 1e6);
}

export function tdoaFor(cfg: AudioConfig, bearingDeg: number, elevationDeg: number): Record<string, number> {
  const v = tdoaClean(cfg.mics_mm, cfg.array_yaw_deg, cfg.speed_of_sound, bearingDeg, elevationDeg);
  const out: Record<string, number> = {};
  PAIRS.forEach((p, i) => (out[p] = r(v[i] + gauss() * 2, 1)));
  return out;
}

// Findings whose centroid lies inside bearing ± window: voids first, then
// unseen regions, then obstacles; nearest first within each group.
export function candidatesFor(f: Findings | null, bearing: number, window: number): AudioCandidate[] {
  if (!f || f.grid.w === 0) return [];
  const w = Math.max(3, window);
  const pick = (items: Finding[], kind: string) =>
    items
      .filter((it) => angDiff(it.bearing_deg, bearing) <= w)
      .sort((a, b) => a.distance_m - b.distance_m)
      .map((it) => ({ kind, id: it.id, distance_m: it.distance_m, bearing_deg: it.bearing_deg, off_axis_deg: r(angDiff(it.bearing_deg, bearing), 1) }));
  return [...pick(f.voids, "void"), ...pick(f.shadows, "shadow"), ...pick(f.obstacles, "obstacle")];
}

// ---- source tracking (§5.6) ----

interface Track {
  src: AudioSource;
  evs: AudioEvent[];
}

const SOURCE_JOIN_S = 60;
const SOURCE_EXPIRE_S = 120;

function summarise(t: Track, findings: Findings | null) {
  let sx = 0;
  let sy = 0;
  let sw = 0;
  let inv = 0;
  for (const e of t.evs) {
    const sig = Math.max(1, e.bearing_sigma_deg ?? 10);
    const w = e.confidence / (sig * sig);
    sx += w * Math.cos(e.bearing_deg! * DEG);
    sy += w * Math.sin(e.bearing_deg! * DEG);
    sw += w;
    inv += 1 / (sig * sig);
  }
  const mean = norm360(Math.atan2(sy, sx) / DEG);
  const R = Math.min(1, Math.hypot(sx, sy) / sw);
  const spread = t.evs.length > 1 ? Math.sqrt(Math.max(0, -2 * Math.log(Math.max(R, 1e-9)))) / DEG : 0;
  let miss = 1;
  for (const e of t.evs) miss *= 1 - e.confidence;
  const times = t.evs.map((e) => e.t).sort((a, b) => a - b);
  let run = 1;
  let tapping = false;
  for (let i = 1; i < times.length; i++) {
    const gap = times[i] - times[i - 1];
    run = gap >= 0.3 && gap <= 6 ? run + 1 : 1;
    if (run >= 3) tapping = true;
  }
  const span = times[times.length - 1] - times[0];
  const sigma = Math.sqrt(1 / inv);
  const s = t.src;
  s.bearing_deg = r(mean, 1);
  s.spread_deg = r(spread, 1);
  s.sigma_deg = r(sigma, 1);
  s.n_events = t.evs.length;
  s.first_t = times[0];
  s.last_t = times[times.length - 1];
  s.rate_per_min = span > 0 ? r(((t.evs.length - 1) / span) * 60, 1) : 0;
  s.confidence = r(Math.min(0.99, (1 - miss) * Math.exp(-spread / 20)), 2);
  s.peak_dbfs = Math.max(...t.evs.map((e) => e.peak_dbfs));
  s.event_ids = t.evs.map((e) => e.id);
  s.kind = tapping ? "tapping" : "sound";
  s.candidates = candidatesFor(findings, s.bearing_deg, 2 * Math.max(s.spread_deg, s.sigma_deg));
}

// ---- calibration fit (§5.5, simplified: yaw by grid search, offsets as the mean misfit) ----

function fitYaw(mics: number[][], c: number, clicks: { bearing: number; elev: number; tau: number[] }[]) {
  let best = { yaw: 0, rms: Infinity, off: [0, 0, 0, 0] };
  const tryYaw = (yaw: number) => {
    // per-channel offsets: least squares of the pair residuals, ch1 fixed at 0
    const res = clicks.map((k) => {
      const p = tdoaClean(mics, yaw, c, k.bearing, k.elev);
      return k.tau.map((t, i) => t - p[i]);
    });
    const mean = PAIRS.map((_, i) => res.reduce((a, rr) => a + rr[i], 0) / res.length);
    // tdoa_ij gains δ_i − δ_j: with δ1 = 0, δj ≈ −mean(1j)
    const off = [0, -mean[0], -mean[1], -mean[2]];
    let ss = 0;
    let n = 0;
    for (const rr of res)
      PAIRS.forEach((p, i) => {
        const d = rr[i] - (off[+p[0] - 1] - off[+p[1] - 1]);
        ss += d * d;
        n++;
      });
    const rms = Math.sqrt(ss / n);
    if (rms < best.rms) best = { yaw, rms, off };
  };
  for (let y = 0; y < 360; y += 0.5) tryYaw(y);
  const y0 = best.yaw;
  for (let y = y0 - 0.5; y <= y0 + 0.5; y += 0.05) tryYaw(norm360(y));
  return best;
}

export function calibrateMock(cfg: AudioConfig, clicks: { bearing: number; elev: number; tau: number[] }[]): Omit<AudioCalibrateResult, "applied"> {
  if (clicks.length < 4) throw new Error("calibration needs at least 4 claps from known bearings");
  let spread = 0;
  for (const a of clicks) for (const b of clicks) spread = Math.max(spread, angDiff(a.bearing, b.bearing));
  if (spread < 60) throw new Error("claps must come from bearings at least 60° apart (ideally all around)");
  const c = cfg.speed_of_sound;
  const fit = fitYaw(cfg.mics_mm, c, clicks);
  // rival: channels 2 and 3 swapped (the mirror a symmetric board allows)
  const swapped = [cfg.mics_mm[0], cfg.mics_mm[2], cfg.mics_mm[1], cfg.mics_mm[3]];
  const rival = fitYaw(swapped, c, clicks);
  const confirmed = fit.rms <= rival.rms && rival.rms > 1.5 * fit.rms;
  const per_click = clicks.map((k) => {
    let bb = 0;
    let br = Infinity;
    for (let b = 0; b < 360; b += 0.5) {
      const p = tdoaClean(cfg.mics_mm, fit.yaw, c, b, k.elev);
      let ss = 0;
      PAIRS.forEach((pp, i) => {
        const d = k.tau[i] - (fit.off[+pp[0] - 1] - fit.off[+pp[1] - 1]) - p[i];
        ss += d * d;
      });
      if (ss < br) {
        br = ss;
        bb = b;
      }
    }
    return { bearing_deg: r(k.bearing, 1), fitted_bearing_deg: r(bb, 1), error_deg: r(((bb - k.bearing + 540) % 360) - 180, 1), residual_us: r(Math.sqrt(br / 6), 2) };
  });
  const verdict = fit.rms <= rival.rms
    ? confirmed
      ? "channel order confirmed"
      : "channel order plausible but not decisive: add claps from more bearings"
    : "channel order looks wrong: best fit with ch2 = position of mic 3, ch3 = position of mic 2";
  return {
    n_clicks: clicks.length,
    array_yaw_deg: r(fit.yaw, 2),
    channel_offsets_us: fit.off.map((v) => r(v, 2)),
    channel_offsets_samples: fit.off.map((v) => r((v * 48000) / 1e6, 2)),
    residual_us: r(fit.rms, 2),
    per_click,
    order: {
      identity_residual_us: r(fit.rms, 2),
      best_rival_residual_us: r(rival.rms, 2),
      best_rival_channel_to_mic: [1, 3, 2, 4],
      best_rival_mirrored: false,
      confirmed,
      verdict,
    },
    ok: fit.rms < 6 && confirmed,
  };
}

// ---- the simulator ----

interface Knock {
  t0: number; // ms
  amp: number; // peak envelope, counts
  decay: number; // ms
}

export class AudioSim {
  status: AudioStatus = { connected: false };
  events: AudioEvent[] = []; // newest first
  private tracks: Track[] = []; // active
  private history: AudioSource[] = [];
  private nextId = 1;
  private nextSourceId = 1;
  private seq = 0;
  private frames = 0;
  private timer: ReturnType<typeof setInterval> | null = null;
  private timeouts: ReturnType<typeof setTimeout>[] = [];
  private nextKnockAt = 0;
  private nextNoiseAt = 0;
  private nextTapBurstAt = 0;
  private lastEventAt = 0;
  private knocks: Knock[] = [];
  private genMs = 0;
  private mins = Array.from({ length: CH }, () => new Int16Array(RING));
  private maxs = Array.from({ length: CH }, () => new Int16Array(RING));
  private noiseAmp = [260, 300, 240, 330].map((v, i) => (i === NOISY_MIC ? v * 6.5 : v)); // ~14 dB above the median of the others
  private noiseDb = [NaN, NaN, NaN, NaN];
  private clipUntil = [0, 0, 0, 0];
  private startMs = Date.now();

  constructor(
    private cfg: () => AudioConfig,
    private findings: () => Findings | null,
    private sendFrame: (buf: ArrayBuffer) => void,
    private sendEvent: (ev: AudioEvent) => void,
    private sendSources: (active: AudioSource[]) => void,
  ) {}

  start(source: "tcp" | "sim", port: string): void {
    this.stop();
    this.status = { connected: true, source, port, link: "up", rate: 48000, frames: 0, lost_frames: 0, calibrated: this.cfg().calibrated, sources_active: 0, excluded_mics: [] };
    this.genMs = Date.now() - RING + 1;
    this.noiseDb = [NaN, NaN, NaN, NaN];
    const now = Date.now();
    this.nextKnockAt = now + 6000;
    this.nextNoiseAt = now + 2000;
    this.nextTapBurstAt = source === "sim" ? now + 3000 : Infinity;
    this.timer = setInterval(() => this.tick(), FRAME_MS);
  }

  stop(): void {
    if (this.timer) clearInterval(this.timer);
    for (const t of this.timeouts) clearTimeout(t);
    this.timeouts = [];
    this.timer = null;
    this.status = { connected: false };
    this.knocks = [];
  }

  clear(): void {
    this.events = [];
    this.history.push(...this.tracks.map((t) => ({ ...t.src, active: false })));
    this.tracks = [];
    this.sendSources([]);
  }

  sources(): { active: AudioSource[]; history: AudioSource[] } {
    this.expire();
    return { active: this.tracks.map((t) => t.src), history: this.history.slice(-50).reverse() };
  }

  excludedMics(): number[] {
    const ex = this.status.excluded_mics ?? [];
    return ex.map((x) => (typeof x === "number" ? x : (x.mic ?? 0)));
  }

  // A knock (or voice) from a given direction, detected now (Test knock / sim emit).
  emit(bearingDeg: number, elevationDeg = 0, snrDb = 20, kind: "knock" | "voice" = "knock"): AudioEvent {
    const t0 = Date.now();
    this.addKnock(t0, snrDb, kind === "voice" ? 180 : 35);
    return this.detect(t0, bearingDeg, elevationDeg, snrDb, { forced: true, label: kind === "voice" ? "voice" : "impulse" });
  }

  private later(ms: number, fn: () => void) {
    const h = setTimeout(() => {
      this.timeouts = this.timeouts.filter((x) => x !== h);
      if (this.status.connected) fn();
    }, ms);
    this.timeouts.push(h);
  }

  private addKnock(t0: number, snrDb: number, decay = 35) {
    const quiet = [...this.noiseAmp].sort((a, b) => a - b)[1];
    this.knocks.push({ t0, amp: quiet * 10 ** (snrDb / 20), decay: decay * (0.7 + Math.random() * 0.6) });
  }

  private detect(
    t0: number,
    trueBearing: number,
    elevation: number,
    snrDb: number,
    opts: { forced?: boolean; label?: string; confidence?: number; bearingOverride?: number | null } = {},
  ): AudioEvent {
    const cfg = this.cfg();
    const excluded = this.excludedMics();
    const unresolved = opts.bearingOverride === null || (!opts.forced && snrDb < 9 && Math.random() < 0.3);
    const sigma = clamp(2.5 + 70 / Math.max(snrDb, 1) + elevation / 10 + (excluded.length ? 2 : 0), 3, 40);
    // the error stays within 1σ so the true direction is always inside ± 2σ
    const bearing = opts.bearingOverride ?? norm360(trueBearing + (Math.random() * 2 - 1) * sigma);
    const confidence = opts.confidence ?? (unresolved ? 0.06 : clamp((0.25 + snrDb / 40) * (0.85 + Math.random() * 0.15), 0.12, 0.97));
    const noise = confidence < 0.1;
    const quiet = [...this.noiseAmp].sort((a, b) => a - b)[1];
    const peak = Math.min(FULL, quiet * 10 ** (snrDb / 20));
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
      candidates: unresolved || noise ? [] : candidatesFor(this.findings(), bearing, 2 * sigma),
      label: opts.label ?? "impulse",
      noise,
      source_id: null,
      mics_used: [1, 2, 3, 4].filter((m) => !excluded.includes(m)),
      reasons: noise ? ["low GCC peak sharpness", "pairs disagree"] : [],
      calibrated: cfg.calibrated,
    };
    this.events.unshift(ev);
    if (this.events.length > 300) this.events.length = 300;
    this.lastEventAt = t0;
    this.track(ev);
    this.sendEvent(ev);
    return ev;
  }

  private expire() {
    const now = Date.now() / 1000;
    const keep: Track[] = [];
    let changed = false;
    for (const t of this.tracks) {
      if (now - t.src.last_t > SOURCE_EXPIRE_S) {
        this.history.push({ ...t.src, active: false });
        changed = true;
      } else keep.push(t);
    }
    this.tracks = keep;
    if (changed) this.sendSources(this.tracks.map((t) => t.src));
  }

  private track(ev: AudioEvent) {
    this.expire();
    if (ev.noise || ev.bearing_deg === null || ev.confidence < 0.1) return;
    let best: Track | null = null;
    for (const t of this.tracks) {
      if (ev.t - t.src.last_t >= SOURCE_JOIN_S) continue;
      const lim = Math.max(15, 3 * t.src.spread_deg);
      const d = angDiff(t.src.bearing_deg, ev.bearing_deg);
      if (d <= lim && (!best || d < angDiff(best.src.bearing_deg, ev.bearing_deg))) best = t;
    }
    if (!best) {
      best = {
        src: {
          id: this.nextSourceId++,
          kind: "sound",
          bearing_deg: ev.bearing_deg,
          spread_deg: 0,
          sigma_deg: ev.bearing_sigma_deg ?? 10,
          n_events: 0,
          first_t: ev.t,
          last_t: ev.t,
          confidence: 0,
          event_ids: [],
          candidates: [],
          active: true,
        },
        evs: [],
      };
      this.tracks.push(best);
    }
    best.evs.push(ev);
    ev.source_id = best.src.id;
    summarise(best, this.findings());
    this.sendSources(this.tracks.map((t) => t.src));
  }

  private tapBearing(): number {
    const f = this.findings();
    const v = f?.voids[0] ?? f?.shadows[0];
    return v ? v.bearing_deg : 112;
  }

  private generate(upTo: number) {
    if (upTo - this.genMs > RING) this.genMs = upTo - RING;
    this.knocks = this.knocks.filter((k) => upTo - k.t0 < 1200);
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
    // repeated tapping from one direction: 6 knocks 1.5–3.5 s apart, every ~45 s
    if (now >= this.nextTapBurstAt) {
      this.nextTapBurstAt = now + 45000;
      const b = this.tapBearing();
      let at = 0;
      for (let i = 0; i < 6; i++) {
        this.later(at, () => {
          const snr = 16 + Math.random() * 10;
          this.addKnock(Date.now(), snr);
          this.later(40, () => this.detect(Date.now() - 40, b, 5 + Math.random() * 10, snr));
        });
        at += 1500 + Math.random() * 2000;
      }
    }
    // a single knock from anywhere, sometimes heard again from a wrong direction (echo)
    if (now >= this.nextKnockAt) {
      this.nextKnockAt = now + 9000 + Math.random() * 5000;
      const snr = 5 + Math.random() * 28 + (Math.random() < 0.1 ? 14 : 0); // the odd one clips
      this.addKnock(now, snr);
      if (snr >= cfg.trigger_db && now - this.lastEventAt >= cfg.min_gap_ms) {
        const bearing = Math.random() * 360;
        const elev = Math.random() * 35;
        this.later(40, () => this.detect(now, bearing, elev, snr));
        if (Math.random() < 0.35)
          this.later(40 + cfg.min_gap_ms + 60, () =>
            this.detect(now + cfg.min_gap_ms + 60, norm360(bearing + 60 + Math.random() * 120), elev, snr - 10, { confidence: 0.12 + Math.random() * 0.15 }),
          );
      }
    }
    // scattered noise detections: never part of a source
    if (now >= this.nextNoiseAt) {
      this.nextNoiseAt = now + 3000 + Math.random() * 5000;
      this.addKnock(now, 6 + Math.random() * 4, 15);
      const withBearing = Math.random() < 0.6;
      this.later(40, () =>
        this.detect(now, Math.random() * 360, 0, 7, { confidence: 0.02 + Math.random() * 0.07, bearingOverride: withBearing ? r(Math.random() * 360, 1) : null }),
      );
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
      // the noise floor follows the level slowly, and downwards quickly
      if (!Number.isFinite(this.noiseDb[c])) this.noiseDb[c] = levels[c];
      this.noiseDb[c] = levels[c] < this.noiseDb[c] ? this.noiseDb[c] * 0.7 + levels[c] * 0.3 : this.noiseDb[c] * 0.995 + levels[c] * 0.005;
      mins.push(mn);
      maxs.push(mx);
    }
    // channel health: a floor > 12 dB above the median of the others is excluded
    const excluded: { mic: number; reason: string }[] = [];
    for (let c = 0; c < CH; c++) {
      const others = this.noiseDb.filter((_, j) => j !== c).sort((a, b) => a - b);
      const med = others[1];
      const over = this.noiseDb[c] - med;
      if (over > 12) excluded.push({ mic: c + 1, reason: `${Math.round(over)} dB noisier than the others` });
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
      array_yaw_deg: cfg.array_yaw_deg,
      note: cfg.calibrated ? null : "uncalibrated: channel order, array yaw to the LiDAR 0° mark and capture offsets not yet verified by a click test",
      sources_active: this.tracks.length,
      excluded_mics: excluded,
    };
    if (this.seq % 20 === 0) this.expire();
    this.sendFrame(encodeAudio({ channels: CH, n: BLOCK, seq: this.seq, t_unix: now / 1000, levels_dbfs: levels, mins, maxs }));
  }

  // POST /audio/calibrate
  calibrate(clicks: AudioCalibrateClick[]): Omit<AudioCalibrateResult, "applied"> {
    const cfg = this.cfg();
    const rows = clicks.map((k) => {
      let t = k.tdoa_us;
      if (!t && k.event_id !== undefined) {
        const ev = this.events.find((e) => e.id === k.event_id);
        if (!ev || Object.keys(ev.tdoa_us).length !== 6) throw new Error(`no stored event ${k.event_id} with six tdoas`);
        t = ev.tdoa_us;
      }
      if (!t) throw new Error("each click needs event_id or tdoa_us");
      return { bearing: Number(k.bearing_deg), elev: Number(k.elevation_deg ?? 0), tau: PAIRS.map((p) => Number(t![p])) };
    });
    return calibrateMock(cfg, rows);
  }
}
