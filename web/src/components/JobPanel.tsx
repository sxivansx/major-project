import { useState } from "react";
import { errorText } from "../api/rest";
import type { JobRequest } from "../api/types";
import { startSweep, stopSweep, sweepSteps } from "../store/actions";
import { useStore } from "../store/store";
import { fmt, fmtDuration, fmtInt } from "../lib/format";

export function JobPanel() {
  const status = useStore((s) => s.status);
  const form = useStore((s) => s.jobForm);
  const setJobForm = useStore((s) => s.setJobForm);
  const [err, setErr] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const job = status?.job ?? null;
  const running = job?.state === "running";
  const ready = !!status?.lidar.connected && !!status?.servo.connected;

  const set = <K extends keyof JobRequest>(k: K, v: JobRequest[K]) => setJobForm({ [k]: v });
  const num = (k: keyof JobRequest) => (e: React.ChangeEvent<HTMLInputElement>) =>
    set(k, Number(e.target.value) as never);

  const start = async () => {
    setErr(null);
    setBusy(true);
    try {
      await startSweep();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };

  const stop = async () => {
    setErr(null);
    setBusy(true);
    try {
      await stopSweep();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };

  const nSteps = sweepSteps(form);
  const estS = (nSteps * (form.settle_ms + form.rotations * 100 + 50)) / 1000;
  const pct = job && job.n_steps > 0 ? Math.min(100, (100 * job.step) / job.n_steps) : 0;

  return (
    <section className="panel">
      <div className="panel-head">
        <span>Scan job</span>
        <span className={"sub " + stateClass(job?.state)}>{job ? job.state : "idle"}</span>
      </div>
      <div className="panel-body">
        {job && (
          <>
            <div className="progress" style={{ marginTop: 0 }} title={`${fmt(pct, 0)}%`}>
              <div style={{ width: `${pct}%` }} />
            </div>
            <dl className="kv" style={{ marginBottom: 10 }}>
              <dt>Job</dt>
              <dd>
                #{job.job_id} {job.name}
              </dd>
              <dt>Step</dt>
              <dd>
                {job.step} of {job.n_steps}
              </dd>
              <dt>φ now</dt>
              <dd>{fmt(job.phi_deg, 1, "°")}</dd>
              <dt>Points</dt>
              <dd>{fmtInt(job.points)}</dd>
              <dt>ETA</dt>
              <dd>{job.state === "running" ? fmtDuration(job.eta_s) : "–"}</dd>
              <dt>State</dt>
              <dd className={stateClass(job.state)}>{job.state}</dd>
              {job.error && (
                <>
                  <dt>Error</dt>
                  <dd className="err">{job.error}</dd>
                </>
              )}
            </dl>
          </>
        )}
        <div className="field">
          <label>Name</label>
          <input value={form.name} disabled={running} onChange={(e) => set("name", e.target.value)} />
        </div>
        <div className="field">
          <label>φ start / end</label>
          <div className="row">
            <input type="number" step={1} value={form.phi_start} disabled={running} onChange={num("phi_start")} />
            <input type="number" step={1} value={form.phi_end} disabled={running} onChange={num("phi_end")} />
          </div>
        </div>
        <div className="field">
          <label>φ step</label>
          <input type="number" step={0.5} min={0.1} value={form.phi_step} disabled={running} onChange={num("phi_step")} />
        </div>
        <div className="field">
          <label>Rotations</label>
          <input type="number" step={1} min={1} value={form.rotations} disabled={running} onChange={num("rotations")} />
        </div>
        <div className="field">
          <label>Settle ms</label>
          <input type="number" step={50} min={0} value={form.settle_ms} disabled={running} onChange={num("settle_ms")} />
        </div>
        <div className="hint">
          {nSteps} steps, roughly {fmtDuration(estS)} at 10 Hz
          {!ready && " · connect lidar and servo first"}
        </div>
        <div className="btn-row">
          <button className="primary" disabled={running || busy || !ready || nSteps === 0} onClick={start}>
            Start sweep
          </button>
          <button className="danger" disabled={!running || busy} onClick={stop}>
            Stop
          </button>
        </div>
        {err && <div className="error">{err}</div>}
      </div>
    </section>
  );
}

function stateClass(state: string | undefined): string {
  switch (state) {
    case "running":
      return "ok";
    case "error":
      return "err";
    case "stopped":
      return "warn";
    default:
      return "";
  }
}
