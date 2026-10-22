import { useStore } from "../store/store";
import { fmt, fmtInt } from "../lib/format";

// Evaluation numbers for the current cloud (GET /api/metrics, refetched by
// store/metrics.ts whenever a new grid arrives).
export function MetricsLine() {
  const m = useStore((s) => s.metrics);
  const err = useStore((s) => s.metricsError);

  if (err) return <div className="error">metrics: {err}</div>;
  if (!m) return null;
  const g = m.grid;
  return (
    <div className="hint mono" style={{ padding: "0 8px 6px" }} title="GET /api/metrics: coverage of the grid's bounding box, sweep throughput, plane fit to points within ±10 cm of z=0">
      coverage {fmt(g.coverage_pct, 1)}% of {fmtInt(g.cells)} cells
      {m.steps ? ` · ${m.steps} steps · ${fmtInt(m.points_per_step)} pts/step` : ""}
      {m.duration_s !== null ? ` · ${fmt(m.duration_s, 1)} s` : ""}
      {m.floor
        ? ` · floor ${fmtInt(m.floor.points)} pts, RMS ${fmt(m.floor.rms_mm, 1)} mm, tilt ${fmt(m.floor.tilt_deg, 2)}°, offset ${fmt(m.floor.offset_mm, 0)} mm`
        : " · floor: no points within ±10 cm of z=0"}
    </div>
  );
}
