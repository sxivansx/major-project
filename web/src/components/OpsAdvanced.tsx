import { FINDING_KINDS } from "../api/types";
import { errorText } from "../api/rest";
import { fmt, fmtTime } from "../lib/format";
import { stopSweep } from "../store/actions";
import { refreshOps, useOps, type LayerToggles } from "../store/ops";
import { useStore } from "../store/store";
import { RankFindings } from "./OpsAssistant";
import { FindingList, ParamControls, Situation } from "./OpsFindings";
import { useState } from "react";

// Everything behind the Advanced disclosure: layer toggles, analysis
// parameters, the full situation block, the finding lists, route details and
// the assistant ranking.

const TOGGLES: { k: keyof LayerToggles; label: string }[] = [
  { k: "terrain", label: "terrain" },
  { k: "traversability", label: "traversability" },
  { k: "shadow", label: "shadow" },
  { k: "blind", label: "blind" },
  { k: "findings", label: "findings" },
  { k: "station", label: "next station" },
];

export function OpsAdvanced() {
  const status = useStore((s) => s.status);
  const findings = useOps((s) => s.findings);
  const updatedAt = useOps((s) => s.updatedAt);
  const loading = useOps((s) => s.loading);
  const toggles = useOps((s) => s.toggles);
  const setToggle = useOps((s) => s.setToggle);
  const fit = useOps((s) => s.fit);
  const route = useOps((s) => s.route);
  const routeSource = useOps((s) => s.routeSource);
  const [stopErr, setStopErr] = useState<string | null>(null);
  const running = status?.job?.state === "running";
  const job = status?.job;

  return (
    <div className="ops-advanced">
      <div className="canvas-toolbar">
        {TOGGLES.map((t) => (
          <label key={t.k} className="inline-check">
            <input type="checkbox" checked={toggles[t.k]} onChange={(e) => setToggle(t.k, e.target.checked)} />
            {t.label}
          </label>
        ))}
        <span className="dim">|</span>
        <ParamControls />
        <button className="small" disabled={loading} onClick={() => void refreshOps()} title="GET /api/ops/findings and /api/ops/layers">
          Refresh analysis
        </button>
        <button className="small" onClick={fit}>
          Fit
        </button>
        {running && (
          <button
            className="small danger"
            onClick={() => {
              setStopErr(null);
              stopSweep().catch((e) => setStopErr(errorText(e)));
            }}
          >
            Stop sweep
          </button>
        )}
        {stopErr && <span className="err">{stopErr}</span>}
        <span className="spacer" />
        <span className="mono dim">
          {findings && findings.grid.w > 0 ? `${findings.grid.w}×${findings.grid.h} @ ${+findings.grid.cell_m.toFixed(3)} m` : "no analysis"}
          {updatedAt ? ` · ${fmtTime(updatedAt)}` : ""}
          {job ? ` · job ${job.state} ${job.step}/${job.n_steps}` : ""}
        </span>
      </div>
      <div className="ops-advanced-grid">
        <div className="ops-advanced-col">
          {findings && findings.grid.w > 0 ? <Situation f={findings} /> : <div className="empty">no analysis yet</div>}
          <div className="ops-block">
            <div className="ops-block-title">Route</div>
            {route ? (
              route.found ? (
                <dl className="kv">
                  <dt>Length</dt>
                  <dd>{fmt(route.length_m, 2)} m</dd>
                  <dt>Verdict</dt>
                  <dd className={route.verdict === "passable" ? "ok" : "warn"}>{route.verdict}</dd>
                  <dt>Max step</dt>
                  <dd>{fmt(route.max_step_m, 3)} m</dd>
                  <dt>Cells</dt>
                  <dd>
                    {route.cells} ({route.marginal_cells} marginal)
                  </dd>
                  <dt>Start snapped</dt>
                  <dd>{fmt(route.start_snapped_m, 2)} m</dd>
                  <dt>Source</dt>
                  <dd>{routeSource ?? "–"}</dd>
                </dl>
              ) : (
                <div className="hint">{route.reason}</div>
              )
            ) : (
              <div className="hint">none</div>
            )}
          </div>
        </div>
        <div className="ops-advanced-col">
          {findings && findings.grid.w > 0 && FINDING_KINDS.map((k) => <FindingList key={k} kind={k} items={findings[k]} />)}
          <RankFindings />
        </div>
      </div>
    </div>
  );
}
