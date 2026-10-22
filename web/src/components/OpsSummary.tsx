import type { Findings } from "../api/types";
import { fmtTime, fmtUsd } from "../lib/format";
import { decisionsAvailable, useOps } from "../store/ops";
import { AskBox } from "./OpsAssistant";

// "What we have": the situation report, refreshed after every analysis, and
// one question box. The report comes from POST /api/ai/sitrep; a core without
// a key answers with a template (model "template"), and if the call fails
// altogether a few lines are composed here from the findings.

function fromFindings(f: Findings): string {
  const c = f.coverage;
  const lines: string[] = [];
  lines.push(`Seen ${c.coverage_pct}% of the area that could be seen from here (${c.area_seen_m2} m²).`);
  lines.push(
    f.voids.length
      ? `Possible voids: ${f.voids.map((v) => `${v.id} ${v.clearance_m} m clear, ${v.distance_m} m away toward ${v.bearing_deg}°`).join("; ")}.`
      : "No covered voids found.",
  );
  if (f.obstacles.length) lines.push(`${f.obstacles.length} obstacle${f.obstacles.length > 1 ? "s" : ""} above 0.25 m.`);
  if (f.next_station)
    lines.push(
      f.next_station.bearing_deg === null
        ? `Next: ${f.next_station.reason}.`
        : `Next: move ${f.next_station.distance_m} m toward ${f.next_station.bearing_deg}° (${f.next_station.reason}).`,
    );
  return lines.join("\n");
}

export function OpsSummary() {
  const findings = useOps((s) => s.findings);
  const updatedAt = useOps((s) => s.updatedAt);
  const sitrep = useOps((s) => s.sitrep);
  const sitrepBusy = useOps((s) => s.sitrepBusy);
  const sitrepError = useOps((s) => s.sitrepError);
  const aiStatus = useOps((s) => s.aiStatus);
  const aiStatusError = useOps((s) => s.aiStatusError);

  const haveAnalysis = !!findings && findings.grid.w > 0;
  const text = sitrep?.text ?? (haveAnalysis && sitrepError ? fromFindings(findings!) : null);
  const assistantWritten = !!sitrep && !!sitrep.model && sitrep.model !== "template";

  let ask: React.ReactNode;
  if (aiStatusError) ask = <div className="hint">Assistant off: {aiStatusError}</div>;
  else if (!aiStatus) ask = null;
  else if (!decisionsAvailable(aiStatus)) ask = <div className="hint">Assistant off: no API key</div>;
  else ask = <AskBox />;

  return (
    <section className="panel ops-right">
      <div className="panel-head">
        <span>What we have</span>
        <span className="sub">{sitrepBusy ? "updating" : updatedAt ? fmtTime(updatedAt) : ""}</span>
      </div>
      <div className="panel-body ops-summary">
        {!text && !sitrepBusy && <div className="hint">{haveAnalysis ? "Writing the summary." : "Nothing yet: scan, or load a session."}</div>}
        {!text && sitrepBusy && <div className="hint">Writing the summary.</div>}
        {text && <div className="ops-sitrep">{text}</div>}
        {assistantWritten && (
          <div className="ai-label">
            assistant summary{sitrep?.cost_usd !== undefined ? ` · ${fmtUsd(sitrep.cost_usd)}` : ""}
          </div>
        )}
        {text && !sitrep && <div className="ai-label">from the findings (summary service: {sitrepError})</div>}
        <div className="ops-ask-wrap">{ask}</div>
      </div>
    </section>
  );
}
