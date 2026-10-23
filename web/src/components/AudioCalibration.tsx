import { useEffect, useState } from "react";
import { api, errorText } from "../api/rest";
import type { AudioCalibrateResult, AudioEvent } from "../api/types";
import { fmt, fmtTime } from "../lib/format";
import { coreNowSec, useAudio } from "../store/audio";
import { clientLog, useStore } from "../store/store";

// Clap calibration (docs/api.md §5.5): claps from known directions fit the
// array yaw, the per-channel capture offsets and check the channel order.
// One clap per step, then Compute, then Apply.

const DEG = Math.PI / 180;

const PLACE: Record<number, string> = {
  0: "in front of the LiDAR cable mark",
  45: "front-right, halfway between the cable mark and the right side",
  90: "to the right of the cable mark (a quarter turn clockwise, seen from above)",
  135: "back-right",
  180: "behind the scanner, opposite the cable mark",
  225: "back-left",
  270: "to the left of the cable mark (a quarter turn anticlockwise, seen from above)",
  315: "front-left",
};

interface Captured {
  bearing: number;
  event: AudioEvent;
}

// Where to stand: scanner in the middle, cable mark to the right, the target dot.
function Spot({ bearing, done }: { bearing: number; done: number[] }) {
  const size = 120;
  const c = size / 2;
  const R = size / 2 - 14;
  const at = (b: number, r: number) => [c + Math.cos(b * DEG) * r, c + Math.sin(b * DEG) * r];
  const [tx, ty] = at(bearing, R);
  return (
    <svg className="mic-sketch" viewBox={`0 0 ${size} ${size}`} width={size} height={size} role="img" aria-label="Where to stand">
      <circle cx={c} cy={c} r={R} fill="none" stroke="#383838" strokeDasharray="3 3" />
      <line x1={c} y1={c} x2={c + 22} y2={c} stroke="#4d9de0" />
      <path d={`M ${c + 22} ${c} l -6 -3.5 l 0 7 z`} fill="#4d9de0" />
      <circle cx={c} cy={c} r={4} fill="#4d9de0" />
      {done.map((b) => {
        const [x, y] = at(b, R);
        return <circle key={b} cx={x} cy={y} r={3.5} fill="#6fbf73" />;
      })}
      <circle cx={tx} cy={ty} r={6} fill="none" stroke="#ffffff" strokeWidth={2} />
      <text x={size - 4} y={c - 6} textAnchor="end" className="compass-label accent">
        0°
      </text>
    </svg>
  );
}

export function CalibrationWizard() {
  const audio = useStore((s) => s.status?.audio);
  const events = useAudio((s) => s.events);
  const [phase, setPhase] = useState<"intro" | "step" | "review">("intro");
  const [with45, setWith45] = useState(false);
  const [idx, setIdx] = useState(0);
  const [since, setSince] = useState(0); // core time the current step started listening
  const [captured, setCaptured] = useState<Captured[]>([]);
  const [result, setResult] = useState<AudioCalibrateResult | null>(null);
  const [applied, setApplied] = useState<AudioCalibrateResult | null>(null);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const bearings = with45 ? [0, 45, 90, 135, 180, 225, 270, 315] : [0, 90, 180, 270];
  const bearing = bearings[idx];
  const connected = !!audio?.connected;
  const isSim = audio?.source === "sim";

  // the first usable clap since this step started: not noise, six time differences
  const heard = phase === "step" ? ([...events].reverse().find((e) => e.t >= since && !e.noise && Object.keys(e.tdoa_us ?? {}).length === 6) ?? null) : null;

  useEffect(() => {
    if (phase === "step") setSince(coreNowSec() - 0.2);
  }, [phase, idx]);

  const restart = () => {
    setPhase("intro");
    setIdx(0);
    setCaptured([]);
    setResult(null);
    setApplied(null);
    setErr(null);
  };

  const useClap = () => {
    if (!heard) return;
    const next = [...captured.filter((c) => c.bearing !== bearing), { bearing, event: heard }];
    setCaptured(next);
    if (idx + 1 < bearings.length) setIdx(idx + 1);
    else setPhase("review");
  };

  const compute = async (apply: boolean) => {
    setErr(null);
    setBusy(true);
    try {
      const r = await api.audioCalibrate({ clicks: captured.map((c) => ({ event_id: c.event.id, bearing_deg: c.bearing, elevation_deg: 0 })), apply });
      if (apply) {
        setApplied(r);
        try {
          useStore.getState().setConfig(await api.config());
        } catch {
          // the next status brings it anyway
        }
        clientLog("info", `audio calibration applied: yaw ${r.array_yaw_deg}°, residual ${r.residual_us} µs`);
      } else setResult(r);
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };

  if (!connected) return <div className="hint">Connect audio first (action bar), then come back here.</div>;

  if (phase === "intro")
    return (
      <div className="wizard">
        <ol className="wizard-list">
          <li>Find the LiDAR cable mark: that direction is 0°. Directions go clockwise seen from above (90° = to its right).</li>
          <li>For each step, stand about 1 m from the scanner in the shown direction, hands at the height of the microphones, and clap once.</li>
          <li>Check the clap was heard, then go to the next direction. Keep the room quiet.</li>
        </ol>
        <label className="inline-check">
          <input type="checkbox" checked={with45} onChange={(e) => setWith45(e.target.checked)} />
          also the 45° directions (8 claps, a better fit)
        </label>
        <div className="btn-row">
          <button className="primary" onClick={() => setPhase("step")}>
            Start calibration
          </button>
        </div>
      </div>
    );

  if (phase === "step")
    return (
      <div className="wizard">
        <div className="wizard-step">
          <Spot bearing={bearing} done={captured.map((c) => c.bearing)} />
          <div className="wizard-text">
            <div className="wizard-title">
              Step {idx + 1} of {bearings.length} · {bearing}°
            </div>
            <div>
              Stand 1 m away, {PLACE[bearing] ?? `at ${bearing}°`}, hands at mic height. <b>Clap once.</b>
            </div>
            {heard ? (
              <div className="ok" style={{ marginTop: 6 }}>
                Heard a clap at {fmtTime(heard.t)} (confidence {fmt(heard.confidence, 2)}
                {heard.bearing_deg !== null ? `; the array currently reads ${Math.round(heard.bearing_deg)}°` : ""}).
              </div>
            ) : (
              <div className="dim" style={{ marginTop: 6 }}>
                Listening…
              </div>
            )}
            <div className="btn-row">
              <button className="primary" disabled={!heard} onClick={useClap}>
                {idx + 1 < bearings.length ? "Use this clap, next" : "Use this clap, finish"}
              </button>
              <button disabled={!heard} onClick={() => setSince(coreNowSec())}>
                Listen again
              </button>
              {isSim && (
                <button
                  title="Simulator only: a clap from this direction"
                  onClick={() => void api.audioEmit({ bearing_deg: bearing, elevation_deg: 0, snr_db: 24 }).catch((e) => setErr(errorText(e)))}
                >
                  Simulate a clap here
                </button>
              )}
              <button onClick={restart}>Cancel</button>
            </div>
          </div>
        </div>
        {err && <div className="error">{err}</div>}
      </div>
    );

  // review / result
  const r = applied ?? result;
  const good = r ? r.residual_us < 6 : false;
  return (
    <div className="wizard">
      <div className="wizard-title">
        {captured.length} claps recorded: {captured.map((c) => `${c.bearing}°`).join(", ")}
      </div>
      {!result && (
        <div className="btn-row">
          <button className="primary" disabled={busy} onClick={() => void compute(false)}>
            {busy ? "Computing" : "Compute"}
          </button>
          <button onClick={restart}>Start over</button>
        </div>
      )}
      {r && (
        <>
          <dl className="kv">
            <dt>Channel order</dt>
            <dd className={r.order?.confirmed ? "ok" : "warn"}>{r.order?.verdict ?? "not checked"}</dd>
            <dt>Residual</dt>
            <dd className={good ? "ok" : "warn"}>
              {fmt(r.residual_us, 1)} µs ({good ? "good, under 6 µs" : "too high: repeat the claps more carefully"})
            </dd>
            <dt>Array yaw</dt>
            <dd>{fmt(r.array_yaw_deg > 180 ? r.array_yaw_deg - 360 : r.array_yaw_deg, 1)}° (array x-axis to the LiDAR 0° mark, anticlockwise)</dd>
            <dt>Capture offsets</dt>
            <dd>{r.channel_offsets_us.map((v, i) => `ch${i + 1} ${fmt(v, 1)}`).join(" · ")} µs</dd>
          </dl>
          <table className="list" style={{ marginTop: 6 }}>
            <thead>
              <tr>
                <th>clap at</th>
                <th style={{ textAlign: "right" }}>fitted</th>
                <th style={{ textAlign: "right" }}>error</th>
                <th style={{ textAlign: "right" }}>residual µs</th>
              </tr>
            </thead>
            <tbody>
              {r.per_click.map((p, i) => (
                <tr key={i}>
                  <td className="mono">{fmt(p.bearing_deg, 0)}°</td>
                  <td className="num">{p.fitted_bearing_deg === null ? "–" : `${fmt(p.fitted_bearing_deg, 1)}°`}</td>
                  <td className="num">{p.error_deg === null ? "–" : `${fmt(p.error_deg, 1)}°`}</td>
                  <td className="num">{fmt(p.residual_us, 1)}</td>
                </tr>
              ))}
            </tbody>
          </table>
          {applied ? (
            <div className={applied.calibrated ? "ok" : "warn"} style={{ marginTop: 6 }}>
              Applied. Calibrated: {applied.calibrated ? "yes" : "no (yaw and offsets saved; the order or the residual did not pass)"}.
            </div>
          ) : (
            <div className="hint">Apply saves the yaw and offsets. "Calibrated" is set only when the channel order is confirmed and the residual is under 6 µs.</div>
          )}
          <div className="btn-row">
            {!applied && (
              <button className="primary" disabled={busy} onClick={() => void compute(true)}>
                {busy ? "Applying" : "Apply"}
              </button>
            )}
            <button onClick={restart}>Start over</button>
          </div>
        </>
      )}
      {err && <div className="error">{err}</div>}
    </div>
  );
}
