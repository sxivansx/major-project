import { useEffect, useState } from "react";
import { api, errorText } from "../api/rest";
import type { Metrics } from "../api/types";
import { useStore } from "../store/store";
import { fmt, fmtInt } from "../lib/format";

// Evaluation numbers for the current cloud, refreshed whenever a new grid arrives
// (end of each sweep second, job end, session load).
export function MetricsLine() {
  const grid = useStore((s) => s.grid);
  const [m, setM] = useState<Metrics | null>(null);
  const [err, setErr] = useState<string | null>(null);

  useEffect(() => {
    if (!grid) {
      setM(null);
      return;
    }
    let cancelled = false;
    api
      .metrics({ cell: grid.cell_m, z_min: grid.z_min_m, z_max: grid.z_max_m })
      .then((r) => {
        if (!cancelled) {
          setM(r);
          setErr(null);
        }
      })
      .catch((e) => {
        if (!cancelled) setErr(errorText(e));
      });
    return () => {
      cancelled = true;
    };
  }, [grid]);

  if (err) return <div className="error">metrics: {err}</div>;
  if (!m) return null;
  const g = m.grid;
  return (
    <div className="hint mono" title="GET /api/metrics: coverage of the grid's bounding box, sweep throughput, plane fit to points within ±10 cm of z=0">
      coverage {fmt(g.coverage_pct, 1)}% of {fmtInt(g.cells)} cells
      {m.steps ? ` · ${m.steps} steps · ${fmtInt(m.points_per_step)} pts/step` : ""}
      {m.duration_s !== null ? ` · ${fmt(m.duration_s, 1)} s` : ""}
      {m.floor
        ? ` · floor ${fmtInt(m.floor.points)} pts, RMS ${fmt(m.floor.rms_mm, 1)} mm, tilt ${fmt(m.floor.tilt_deg, 2)}°, offset ${fmt(m.floor.offset_mm, 0)} mm`
        : " · floor: no points within ±10 cm of z=0"}
    </div>
  );
}
