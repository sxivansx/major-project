import { useEffect, useRef, useState } from "react";
import { api, errorText } from "../api/rest";
import type { AudioConfig, AudioEvent } from "../api/types";
import { fmt, fmtTime } from "../lib/format";
import {
  CLIP,
  LOW_CONFIDENCE,
  RECENT_TEXT_S,
  bearingText,
  candidateWord,
  predictTdoa,
  useAudio,
  wave,
} from "../store/audio";
import { clientLog, useStore } from "../store/store";

// Pieces of the Sound tab: the live waveforms, the bearing compass, the event
// list, and the Advanced editors. Bearings are drawn like on the Operator map:
// 0° (+x, the lidar cable mark) to the right, clockwise.

const C = {
  bg: "#1c1c1c",
  lane: "#212121",
  grid: "#2e2e2e",
  text: "#d9d9d9",
  dim: "#8c8c8c",
  wave: "#b8b8b8",
  accent: "#4d9de0",
  err: "#e06060",
  mono: "11px ui-monospace, Menlo, monospace",
};
const WINDOW_MS = 5000;
const DEG = Math.PI / 180;

// ---- waveforms ----

const SCALES = [
  { v: 1, label: "full scale" },
  { v: 10, label: "×10" },
  { v: 100, label: "×100" },
];

export function Waveforms() {
  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [scale, setScale] = useState(10);
  const scaleRef = useRef(scale);
  const noise = useStore((s) => s.status?.audio?.noise_dbfs ?? null);
  const clipping = useStore((s) => s.status?.audio?.clipping ?? null);
  const connected = useStore((s) => !!s.status?.audio?.connected);
  const events = useAudio((s) => s.events);
  const selectedId = useAudio((s) => s.selectedId);
  const ext = useRef({ noise, clipping, connected, events, selectedId });
  const dirty = useRef(true);

  useEffect(() => {
    scaleRef.current = scale;
    ext.current = { noise, clipping, connected, events, selectedId };
    dirty.current = true;
  }, [scale, noise, clipping, connected, events, selectedId]);

  useEffect(() => {
    const wrap = wrapRef.current;
    const canvas = canvasRef.current;
    if (!wrap || !canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    let W = 0;
    let H = 0;
    let dpr = 1;
    const resize = () => {
      dpr = Math.min(2, window.devicePixelRatio || 1);
      W = wrap.clientWidth;
      H = wrap.clientHeight;
      canvas.width = Math.max(1, Math.round(W * dpr));
      canvas.height = Math.max(1, Math.round(H * dpr));
      dirty.current = true;
    };
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(wrap);

    let seen = -1;
    let raf = 0;
    const draw = () => {
      raf = requestAnimationFrame(draw);
      if (wave.version === seen && !dirty.current) return;
      seen = wave.version;
      dirty.current = false;
      const { noise, clipping, connected, events, selectedId } = ext.current;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = C.bg;
      ctx.fillRect(0, 0, W, H);
      const ch = wave.channels;
      const AXIS = 18;
      const METER = 46;
      const plotW = Math.max(10, W - METER - 8);
      const laneH = (H - AXIS) / ch;
      const end = wave.latestMs;
      const start = end - WINDOW_MS;
      const k = scaleRef.current;
      ctx.font = C.mono;
      ctx.textBaseline = "alphabetic";

      // time ticks every second
      ctx.lineWidth = 1;
      for (let s = 0; s <= WINDOW_MS / 1000; s++) {
        const x = Math.round(plotW - (s * 1000 * plotW) / WINDOW_MS) + 0.5;
        ctx.strokeStyle = C.grid;
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, H - AXIS);
        ctx.stroke();
        ctx.fillStyle = C.dim;
        ctx.textAlign = s === 0 ? "right" : s === WINDOW_MS / 1000 ? "left" : "center";
        ctx.fillText(s === 0 ? "now" : `−${s} s`, x, H - 5);
      }

      const msPerPx = WINDOW_MS / plotW;
      for (let c = 0; c < ch; c++) {
        const top = c * laneH;
        const mid = top + laneH / 2;
        const half = laneH / 2 - 3;
        ctx.strokeStyle = C.grid;
        ctx.beginPath();
        ctx.moveTo(0, Math.round(top + laneH) + 0.5);
        ctx.lineTo(W, Math.round(top + laneH) + 0.5);
        ctx.moveTo(0, Math.round(mid) + 0.5);
        ctx.lineTo(plotW, Math.round(mid) + 0.5);
        ctx.stroke();

        if (end > 0) {
          const mins = wave.mins[c];
          const maxs = wave.maxs[c];
          for (let px = 0; px < plotW; px++) {
            const a = Math.floor(start + px * msPerPx) + 1;
            const b = Math.floor(start + (px + 1) * msPerPx);
            let lo = Infinity;
            let hi = -Infinity;
            for (let ms = a; ms <= b; ms++) {
              const i = ((ms % wave.stamp.length) + wave.stamp.length) % wave.stamp.length;
              if (wave.stamp[i] !== ms) continue;
              if (mins[i] < lo) lo = mins[i];
              if (maxs[i] > hi) hi = maxs[i];
            }
            if (lo === Infinity) continue;
            const clip = hi >= CLIP || lo <= -CLIP;
            const y1 = mid - Math.max(-1, Math.min(1, (hi / 32768) * k)) * half;
            const y2 = mid - Math.max(-1, Math.min(1, (lo / 32768) * k)) * half;
            ctx.fillStyle = clip ? C.err : C.wave;
            ctx.fillRect(px, y1, 1, Math.max(1, y2 - y1));
          }
        }

        // lane label
        ctx.fillStyle = C.dim;
        ctx.textAlign = "left";
        ctx.fillText(`Mic ${c + 1}`, 6, top + 13);

        // level meter: −60 … 0 dBFS, noise floor tick, clipping in red
        const lvl = wave.levels[c];
        const mx = plotW + 12;
        const mh = laneH - 22;
        const my = top + 6;
        ctx.strokeStyle = "#4a4a4a";
        ctx.strokeRect(mx + 0.5, my + 0.5, 10, mh);
        const isClip = !!clipping?.[c];
        if (connected && Number.isFinite(lvl)) {
          const f = Math.max(0, Math.min(1, (lvl + 60) / 60));
          ctx.fillStyle = isClip ? C.err : C.accent;
          ctx.fillRect(mx + 1, my + 1 + mh * (1 - f), 9, mh * f - 1);
        }
        const nf = noise?.[c];
        if (nf !== undefined && nf !== null && Number.isFinite(nf)) {
          const y = my + mh * (1 - Math.max(0, Math.min(1, (nf + 60) / 60)));
          ctx.strokeStyle = C.text;
          ctx.beginPath();
          ctx.moveTo(mx - 2, Math.round(y) + 0.5);
          ctx.lineTo(mx + 12, Math.round(y) + 0.5);
          ctx.stroke();
        }
        ctx.fillStyle = isClip ? C.err : C.dim;
        ctx.textAlign = "right";
        ctx.fillText(isClip ? "clip" : connected && Number.isFinite(lvl) ? `${Math.round(lvl)}` : "–", W - 4, top + laneH - 6);
      }

      // event markers at detection times
      if (end > 0) {
        for (const ev of events) {
          const tms = ev.t * 1000;
          if (tms < start || tms > end) continue;
          const x = Math.round(((tms - start) / WINDOW_MS) * plotW) + 0.5;
          const sel = ev.id === selectedId;
          ctx.strokeStyle = C.accent;
          ctx.lineWidth = sel ? 2 : 1;
          ctx.beginPath();
          ctx.moveTo(x, 0);
          ctx.lineTo(x, H - AXIS);
          ctx.stroke();
          ctx.lineWidth = 1;
          const label = ev.bearing_deg === null ? "?" : `${Math.round(ev.bearing_deg)}°`;
          ctx.font = C.mono;
          const w = ctx.measureText(label).width;
          ctx.fillStyle = "rgba(28,28,28,0.85)";
          ctx.fillRect(x + 3, 2, w + 4, 13);
          ctx.fillStyle = C.accent;
          ctx.textAlign = "left";
          ctx.fillText(label, x + 5, 12);
        }
      } else {
        ctx.fillStyle = C.dim;
        ctx.font = "13px system-ui, sans-serif";
        ctx.textAlign = "center";
        ctx.fillText(connected ? "waiting for audio" : "audio not connected", plotW / 2, (H - AXIS) / 2);
      }
    };
    raf = requestAnimationFrame(draw);
    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
    };
  }, []);

  return (
    <>
      <div className="canvas-toolbar">
        <label>
          Scale
          <select value={scale} onChange={(e) => setScale(Number(e.target.value))}>
            {SCALES.map((s) => (
              <option key={s.v} value={s.v}>
                {s.label}
              </option>
            ))}
          </select>
        </label>
        <span className="spacer" />
        <span className="mono dim">min/max at 1 kHz · last 5 s · meter −60…0 dBFS, tick = noise floor</span>
      </div>
      <div className="heightmap-wrap" ref={wrapRef}>
        <canvas ref={canvasRef} />
      </div>
    </>
  );
}

// ---- compass ----

function wedgePath(cx: number, cy: number, r: number, bearing: number, half: number): string {
  const a0 = (bearing - half) * DEG;
  const a1 = (bearing + half) * DEG;
  const large = half * 2 > 180 ? 1 : 0;
  return `M ${cx} ${cy} L ${cx + r * Math.cos(a0)} ${cy + r * Math.sin(a0)} A ${r} ${r} 0 ${large} 1 ${cx + r * Math.cos(a1)} ${cy + r * Math.sin(a1)} Z`;
}

export function Compass({ events, now }: { events: AudioEvent[]; now: number }) {
  const selectedId = useAudio((s) => s.selectedId);
  const size = 232;
  const cx = size / 2;
  const cy = size / 2;
  const R = size / 2 - 30;
  const shown = events.filter((e) => e.bearing_deg !== null && (now - e.t < RECENT_TEXT_S || e.id === selectedId));
  const newest = shown[0];
  return (
    <svg className="compass" viewBox={`0 0 ${size} ${size}`} width={size} height={size} role="img" aria-label="Sound directions">
      <circle cx={cx} cy={cy} r={R} fill="none" stroke="#383838" />
      <circle cx={cx} cy={cy} r={R / 2} fill="none" stroke="#2e2e2e" />
      {[0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330].map((a) => {
        const x = cx + Math.cos(a * DEG) * R;
        const y = cy + Math.sin(a * DEG) * R;
        const lx = cx + Math.cos(a * DEG) * (R + 15);
        const ly = cy + Math.sin(a * DEG) * (R + 13) + 4;
        return (
          <g key={a}>
            <line x1={cx} y1={cy} x2={x} y2={y} stroke={a % 90 === 0 ? "#383838" : "#2a2a2a"} />
            {a % 90 === 0 && (
              <text x={lx} y={ly} textAnchor="middle" className="compass-label">
                {a}°
              </text>
            )}
          </g>
        );
      })}
      {[...shown].reverse().map((e) => {
        const age = Math.max(0, now - e.t);
        const fade = Math.max(0.15, 1 - age / RECENT_TEXT_S);
        const sel = e.id === selectedId;
        const isNew = e === newest;
        const half = Math.min(180, 2 * (e.bearing_sigma_deg ?? 10));
        const low = e.confidence < LOW_CONFIDENCE;
        return (
          <g key={e.id}>
            <path
              d={wedgePath(cx, cy, R, e.bearing_deg!, half)}
              fill={low ? "#8c8c8c" : "#4d9de0"}
              fillOpacity={(isNew || sel ? 0.3 : 0.12) * fade}
              stroke={sel ? "#ffffff" : "none"}
            />
            <line
              x1={cx}
              y1={cy}
              x2={cx + Math.cos(e.bearing_deg! * DEG) * R}
              y2={cy + Math.sin(e.bearing_deg! * DEG) * R}
              stroke={sel ? "#ffffff" : low ? "#8c8c8c" : "#4d9de0"}
              strokeOpacity={fade}
              strokeWidth={isNew || sel ? 2.5 : 1}
            />
          </g>
        );
      })}
      <circle cx={cx} cy={cy} r={3} fill="#4d9de0" />
      <text x={cx + R - 2} y={cy - 6} textAnchor="end" className="compass-label">
        +x
      </text>
    </svg>
  );
}

// ---- event list ----

export function EventList({ events }: { events: AudioEvent[] }) {
  const selectedId = useAudio((s) => s.selectedId);
  const setSelected = useAudio((s) => s.setSelected);
  if (events.length === 0) return <div className="hint">No sounds detected yet.</div>;
  return (
    <table className="list ops-list audio-events">
      <thead>
        <tr>
          <th>time</th>
          <th className="num">bearing</th>
          <th className="num">elev</th>
          <th>confidence</th>
          <th>along it</th>
        </tr>
      </thead>
      <tbody>
        {events.map((e) => (
          <tr
            key={e.id}
            className={(e.id === selectedId ? "sel " : "") + (e.confidence < LOW_CONFIDENCE ? "dim" : "")}
            onClick={() => setSelected(e.id === selectedId ? null : e.id)}
          >
            <td className="mono">{fmtTime(e.t)}</td>
            <td className="num">{bearingText(e)}</td>
            <td className="num">{e.elevation_deg === null ? "–" : `${Math.round(e.elevation_deg)}°`}</td>
            <td>
              <span className="conf" title={`confidence ${fmt(e.confidence, 2)}`}>
                <div style={{ width: `${Math.max(0, Math.min(1, e.confidence)) * 100}%` }} />
              </span>{" "}
              <span className="mono">{fmt(e.confidence, 2)}</span>
            </td>
            <td className="cands" title={e.candidates.map((c) => `${candidateWord(c.kind)} ${c.id} at ${fmt(c.distance_m, 1)} m`).join(", ")}>
              {e.candidates.length ? e.candidates.map((c) => c.id).join(" ") : "–"}
            </td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}

// ---- Advanced: TDOA table ----

export function TdoaTable({ ev }: { ev: AudioEvent | null }) {
  const cfg = useStore((s) => s.config?.audio);
  if (!ev) return <div className="hint">Select an event in the list.</div>;
  const geometryKnown = !!cfg && cfg.mics_mm.some((p) => p.some((v) => v !== 0));
  const pred =
    geometryKnown && ev.bearing_deg !== null
      ? predictTdoa(cfg!.mics_mm, cfg!.array_yaw_deg, ev.bearing_deg, ev.elevation_deg ?? 0, cfg!.speed_of_sound)
      : null;
  const pairs = Object.keys(ev.tdoa_us).sort();
  return (
    <>
      <dl className="kv">
        <dt>Event</dt>
        <dd>
          #{ev.id} at {fmtTime(ev.t)} · {ev.label ?? "–"}
        </dd>
        <dt>Peak / SNR</dt>
        <dd>
          {fmt(ev.peak_dbfs, 1)} dBFS · {fmt(ev.snr_db, 1)} dB
        </dd>
        <dt>Bearing</dt>
        <dd>
          {bearingText(ev)} · elevation {ev.elevation_deg === null ? "–" : `${fmt(ev.elevation_deg, 0)}°`}
        </dd>
      </dl>
      <table className="list" style={{ marginTop: 6 }}>
        <thead>
          <tr>
            <th>pair</th>
            <th style={{ textAlign: "right" }}>measured µs</th>
            <th style={{ textAlign: "right" }}>from bearing µs</th>
            <th style={{ textAlign: "right" }}>residual µs</th>
          </tr>
        </thead>
        <tbody>
          {pairs.map((p) => {
            const m = ev.tdoa_us[p];
            const q = pred?.[p];
            return (
              <tr key={p}>
                <td className="mono">
                  {p[0]}–{p[1]}
                </td>
                <td className="num">{fmt(m, 1)}</td>
                <td className="num">{q === undefined ? "–" : fmt(q, 1)}</td>
                <td className="num">{q === undefined ? "–" : fmt(m - q, 1)}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
      <div className="hint">
        "ij" taken as t_i − t_j (positive when mic i hears it later).{" "}
        {geometryKnown ? "Predicted values use the mic geometry below." : "Enter the mic geometry to compare."}
      </div>
    </>
  );
}

// ---- Advanced: trigger and geometry editors (PUT /api/config audio block) ----

type Draft = {
  trigger_db: string;
  min_gap_ms: string;
  mics: [string, string][];
  z: number[];
  yaw: string;
  calibrated: boolean;
};

function toDraft(a: AudioConfig): Draft {
  return {
    trigger_db: String(a.trigger_db),
    min_gap_ms: String(a.min_gap_ms),
    mics: a.mics_mm.map((p) => [String(p[0]), String(p[1])] as [string, string]),
    z: a.mics_mm.map((p) => p[2] ?? 0),
    yaw: String(a.array_yaw_deg),
    calibrated: a.calibrated,
  };
}

async function saveAudio(patch: Partial<AudioConfig>): Promise<void> {
  const st = useStore.getState();
  const config = st.config;
  if (!config?.audio) throw new Error("this core has no audio config");
  const saved = await api.putConfig({ ...config, audio: { ...config.audio, ...patch } });
  st.setConfig(saved);
  clientLog("info", "audio config saved");
}

function useAudioDraft(): [Draft | null, (d: Draft) => void, AudioConfig | null] {
  const cfg = useStore((s) => s.config?.audio ?? null);
  const [draft, setDraft] = useState<Draft | null>(cfg ? toDraft(cfg) : null);
  const seeded = useRef(!!cfg);
  useEffect(() => {
    if (cfg && !seeded.current) {
      seeded.current = true;
      setDraft(toDraft(cfg));
    }
  }, [cfg]);
  return [draft, setDraft, cfg];
}

export function TriggerForm() {
  const [draft, setDraft, cfg] = useAudioDraft();
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  if (!cfg || !draft) return <div className="hint">This core has no audio config.</div>;
  const trig = Number(draft.trigger_db);
  const gap = Number(draft.min_gap_ms);
  const valid = draft.trigger_db !== "" && draft.min_gap_ms !== "" && trig > 0 && gap >= 0;
  const dirty = trig !== cfg.trigger_db || gap !== cfg.min_gap_ms;
  return (
    <>
      <div className="field" title="A spike is short-term energy this many dB above the running noise floor">
        <label>Trigger dB</label>
        <input type="number" step={1} min={1} value={draft.trigger_db} onChange={(e) => setDraft({ ...draft, trigger_db: e.target.value })} />
      </div>
      <div className="field" title="Dead time after an event before the next can trigger">
        <label>Dead time ms</label>
        <input type="number" step={50} min={0} value={draft.min_gap_ms} onChange={(e) => setDraft({ ...draft, min_gap_ms: e.target.value })} />
      </div>
      <div className="btn-row">
        <button
          disabled={!valid || !dirty || busy}
          onClick={() => {
            setErr(null);
            setBusy(true);
            saveAudio({ trigger_db: trig, min_gap_ms: gap })
              .catch((e) => setErr(errorText(e)))
              .finally(() => setBusy(false));
          }}
        >
          Save
        </button>
      </div>
      {err && <div className="error">{err}</div>}
    </>
  );
}

export function GeometryForm() {
  const [draft, setDraft, cfg] = useAudioDraft();
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  if (!cfg || !draft) return <div className="hint">This core has no audio config.</div>;
  const nums = draft.mics.map(([x, y]) => [Number(x), Number(y)]);
  const yaw = Number(draft.yaw);
  const valid = draft.mics.every(([x, y]) => x !== "" && y !== "") && nums.every(([x, y]) => Number.isFinite(x) && Number.isFinite(y)) && Number.isFinite(yaw);
  const setMic = (i: number, j: 0 | 1, v: string) => {
    const mics = draft.mics.map((m) => [...m] as [string, string]);
    mics[i][j] = v;
    setDraft({ ...draft, mics });
  };
  return (
    <div className="geom">
      <div className="geom-form">
        <table className="list">
          <thead>
            <tr>
              <th>mic</th>
              <th>x mm</th>
              <th>y mm</th>
            </tr>
          </thead>
          <tbody>
            {draft.mics.map(([x, y], i) => (
              <tr key={i}>
                <td className="mono">{i + 1}</td>
                <td>
                  <input type="number" step={0.5} value={x} onChange={(e) => setMic(i, 0, e.target.value)} />
                </td>
                <td>
                  <input type="number" step={0.5} value={y} onChange={(e) => setMic(i, 1, e.target.value)} />
                </td>
              </tr>
            ))}
          </tbody>
        </table>
        <div className="field" style={{ marginTop: 6 }} title="Array x-axis relative to the lidar 0° mark, degrees counter-clockwise">
          <label>Array yaw °</label>
          <input type="number" step={1} value={draft.yaw} onChange={(e) => setDraft({ ...draft, yaw: e.target.value })} />
        </div>
        <label className="inline-check">
          <input type="checkbox" checked={draft.calibrated} onChange={(e) => setDraft({ ...draft, calibrated: e.target.checked })} />
          calibrated
        </label>
        <div className="btn-row">
          <button
            disabled={!valid || busy}
            onClick={() => {
              setErr(null);
              setBusy(true);
              saveAudio({
                mics_mm: nums.map(([x, y], i) => [x, y, draft.z[i] ?? 0] as [number, number, number]),
                array_yaw_deg: yaw,
                calibrated: draft.calibrated,
              })
                .catch((e) => setErr(errorText(e)))
                .finally(() => setBusy(false));
            }}
          >
            Save geometry
          </button>
          <button disabled={busy} onClick={() => setDraft(toDraft(cfg))}>
            Reset
          </button>
        </div>
        {err && <div className="error">{err}</div>}
        <div className="hint">z (height) of each mic is kept as stored; edit it in config.json if the array is not flat.</div>
      </div>
      <MicSketch mics={valid ? (nums as [number, number][]) : []} yawDeg={Number.isFinite(yaw) ? yaw : 0} />
    </div>
  );
}

// Top-down sketch in the base frame (+x right, +y up): mic positions after the
// array yaw, the array x-axis, and the lidar 0° direction.
export function MicSketch({ mics, yawDeg }: { mics: [number, number][]; yawDeg: number }) {
  const size = 180;
  const c = size / 2;
  const y = yawDeg * DEG;
  const base = mics.map(([x, yy]) => [x * Math.cos(y) - yy * Math.sin(y), x * Math.sin(y) + yy * Math.cos(y)]);
  const extent = Math.max(20, ...base.flat().map((v) => Math.abs(v)));
  const k = (size / 2 - 26) / extent;
  const ax = Math.cos(y);
  const ay = Math.sin(y);
  return (
    <svg className="mic-sketch" viewBox={`0 0 ${size} ${size}`} width={size} height={size} role="img" aria-label="Mic positions">
      <line x1={c} y1={c} x2={size - 10} y2={c} stroke="#4d9de0" />
      <path d={`M ${size - 10} ${c} l -7 -4 l 0 8 z`} fill="#4d9de0" />
      <text x={size - 10} y={c - 8} textAnchor="end" className="compass-label accent">
        lidar 0°
      </text>
      <line x1={c} y1={c} x2={c + ax * (size / 2 - 18)} y2={c - ay * (size / 2 - 18)} stroke="#8c8c8c" strokeDasharray="3 3" />
      <text x={c + ax * (size / 2 - 40)} y={c - ay * (size / 2 - 40) + 14} textAnchor="middle" className="compass-label">
        array x
      </text>
      {base.map(([x, yy], i) => (
        <g key={i}>
          <circle cx={c + x * k} cy={c - yy * k} r={5} fill="#d9d9d9" />
          <text x={c + x * k + 8} y={c - yy * k + 4} className="compass-label">
            {i + 1}
          </text>
        </g>
      ))}
      <text x={6} y={size - 6} className="compass-label">
        {extent.toFixed(0)} mm
      </text>
    </svg>
  );
}
