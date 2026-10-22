// GET /api/metrics follows the grid: whenever a new grid frame lands in the
// store (every sweep second, job end, session load, Apply), the evaluation
// numbers are refetched once. Started from App, shared by every tab.

import { api, errorText } from "../api/rest";
import type { GridFrame } from "../api/types";
import { useStore } from "./store";

export function startMetricsFollow(): () => void {
  let seq = 0;
  const run = async (grid: GridFrame | null) => {
    const my = ++seq;
    const st = useStore.getState();
    if (!grid) {
      st.setMetrics(null, null);
      return;
    }
    try {
      const m = await api.metrics({ cell: grid.cell_m, z_min: grid.z_min_m, z_max: grid.z_max_m });
      if (my === seq) useStore.getState().setMetrics(m, null);
    } catch (e) {
      if (my === seq) useStore.getState().setMetrics(null, errorText(e));
    }
  };
  void run(useStore.getState().grid);
  return useStore.subscribe((s, prev) => {
    if (s.grid !== prev.grid) void run(s.grid);
  });
}
