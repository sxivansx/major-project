import { useEffect, useState } from "react";
import { FINDING_KINDS, type Finding, type FindingKind, type Findings } from "../api/types";
import { fmt, fmtInt, fmtTime } from "../lib/format";
import { refreshOps, useOps } from "../store/ops";
import { OpsAssistant } from "./OpsAssistant";

const CELL_OPTIONS = [0.05, 0.1, 0.2];

const KIND_TITLE: Record<FindingKind, string> = {
  voids: "Voids",
  obstacles: "Obstacles",
  depressions: "Depressions",
  shadows: "Shadows",
};
const KIND_VALUE: Record<FindingKind, { key: keyof Finding; label: string } | null> = {
  voids: { key: "clearance_m", label: "clearance" },
  obstacles: { key: "height_m", label: "height" },
  depressions: { key: "depth_m", label: "depth" },
  shadows: null,
};

function Situation({ f }: { f: Findings }) {
  const c = f.coverage;
  const t = f.traversability;
  const ns = f.next_station;
  return (
    <div className="ops-block">
      <div className="ops-block-title">Situation</div>
      <dl className="kv">
        <dt>Coverage</dt>
        <dd>
          {fmt(c.coverage_pct, 1)} % of {fmtInt(c.observable_cells)} cells
        </dd>
        <dt>Area seen</dt>
        <dd>
          {fmt(c.area_seen_m2, 1)} m² ({fmtInt(c.filled_cells)} filled, {fmtInt(c.interpolated_cells)} bridged)
        </dd>
        <dt>Floor reach</dt>
        <dd>{fmt(c.floor_reach_m, 1)} m</dd>
        <dt>Shadow cells</dt>
        <dd>{fmtInt(c.shadow_cells)}</dd>
        <dt>Blind cells</dt>
        <dd>{fmtInt(c.blind_cells)}</dd>
        <dt>Unknown cells</dt>
        <dd>{fmtInt(c.unknown_cells)}</dd>
        <dt>Floor</dt>
        <dd>
          {f.floor.fitted
            ? `RMS ${fmt(f.floor.rms_mm, 1)} mm · tilt ${fmt(f.floor.tilt_deg, 2)}° · z0 ${fmt(f.floor.z_at_origin_m, 2)} m`
            : `not fitted (${fmtInt(f.floor.points)} pts)`}
        </dd>
        <dt>Scanner height</dt>
        <dd>{f.scanner.height_above_floor_m === null ? "–" : `${fmt(f.scanner.height_above_floor_m, 2)} m above floor`}</dd>
        <dt>Traversable</dt>
        <dd>
          {fmtInt(t.passable)} passable · {fmtInt(t.marginal)} marginal · {fmtInt(t.blocked)} blocked
        </dd>
        <dt>Limits</dt>
        <dd>
          step ≤ {fmt(t.params.max_step_m, 2)} m · slope ≤ {fmt(t.params.max_slope_deg, 0)}°
        </dd>
        <dt>Points</dt>
        <dd>{fmtInt(f.points)}</dd>
        <dt>Next station</dt>
        <dd>{ns ? (ns.bearing_deg === null ? "rotate in place" : `bearing ${fmt(ns.bearing_deg, 0)}° · ${fmt(ns.distance_m, 1)} m`) : "–"}</dd>
      </dl>
      {c.shadow_cells > 0 && <div className="hint">Shadow: occluded by a nearer object, scan from elsewhere.</div>}
      {c.blind_stripe && c.blind_stripe.cells > 0 && <div className="hint">Blind: {c.blind_stripe.note}</div>}
      {ns && <div className="hint">Next station: {ns.reason}</div>}
    </div>
  );
}

function FindingList({ kind, items }: { kind: FindingKind; items: Finding[] }) {
  const selected = useOps((s) => s.selected);
  const select = useOps((s) => s.select);
  const focusOn = useOps((s) => s.focusOn);
  const value = KIND_VALUE[kind];
  return (
    <div className="ops-block">
      <div className="ops-block-title">
        {KIND_TITLE[kind]} <span className="dim mono">{items.length}</span>
      </div>
      {items.length === 0 ? (
        <div className="hint">none</div>
      ) : (
        <table className="list ops-list">
          <thead>
            <tr>
              <th>id</th>
              <th className="num">m²</th>
              {value && <th className="num">{value.label} m</th>}
              <th className="num">dist m</th>
              <th className="num">bearing</th>
            </tr>
          </thead>
          <tbody>
            {items.map((f) => (
              <tr
                key={f.id}
                className={f.id === selected ? "sel" : ""}
                onClick={() => {
                  select(f.id);
                  focusOn(f.x_m, f.y_m);
                }}
              >
                <td className="mono">{f.id}</td>
                <td className="num">{fmt(f.area_m2, 2)}</td>
                {value && <td className="num">{fmt(f[value.key] as number | undefined, 2)}</td>}
                <td className="num">{fmt(f.distance_m, 2)}</td>
                <td className="num">{fmt(f.bearing_deg, 0)}°</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </div>
  );
}

export function OpsFindings() {
  const findings = useOps((s) => s.findings);
  const loading = useOps((s) => s.loading);
  const error = useOps((s) => s.error);
  const updatedAt = useOps((s) => s.updatedAt);
  const cell = useOps((s) => s.cell);
  const maxStep = useOps((s) => s.maxStep);
  const setParams = useOps((s) => s.setParams);
  const [stepText, setStepText] = useState(String(maxStep));
  useEffect(() => setStepText(String(maxStep)), [maxStep]);

  const commitStep = () => {
    const v = Number(stepText);
    if (!(v > 0)) return setStepText(String(maxStep));
    if (v !== maxStep) {
      setParams({ maxStep: v });
      void refreshOps();
    }
  };

  return (
    <section className="panel ops-col">
      <div className="panel-head">
        <span>Findings</span>
        <span className="sub">{loading ? "analysing" : updatedAt ? `updated ${fmtTime(updatedAt)}` : "no analysis"}</span>
      </div>
      <div className="canvas-toolbar">
        <label>
          Cell m
          <select
            value={String(cell)}
            onChange={(e) => {
              setParams({ cell: Number(e.target.value) });
              void refreshOps();
            }}
          >
            {CELL_OPTIONS.map((c) => (
              <option key={c} value={String(c)}>
                {c}
              </option>
            ))}
          </select>
        </label>
        <label title="Largest step a person can climb; cells above it are blocked">
          Max step m
          <input
            type="number"
            step={0.05}
            min={0.01}
            value={stepText}
            onChange={(e) => setStepText(e.target.value)}
            onBlur={commitStep}
            onKeyDown={(e) => {
              if (e.key === "Enter") (e.target as HTMLInputElement).blur();
            }}
          />
        </label>
        <span className="spacer" />
        <button className="small primary" disabled={loading} onClick={() => void refreshOps()} title="GET /api/ops/findings and /api/ops/layers">
          Refresh analysis
        </button>
      </div>
      <div className="panel-body ops-scroll">
        {error && <div className="error" style={{ padding: "6px 10px", margin: 0 }}>{error}</div>}
        {!findings && !error && <div className="empty">no analysis yet: scan or load a session</div>}
        {findings && findings.grid.w === 0 && <div className="empty">no points in the live cloud: scan or load a session</div>}
        {findings && findings.grid.w > 0 && (
          <>
            <Situation f={findings} />
            {FINDING_KINDS.map((k) => (
              <FindingList key={k} kind={k} items={findings[k]} />
            ))}
          </>
        )}
        <OpsAssistant />
      </div>
    </section>
  );
}
