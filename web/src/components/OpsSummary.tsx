import type { AudioEvent, Findings } from "../api/types";
import { fmt, fmtTime, fmtUsd } from "../lib/format";
import { LOW_CONFIDENCE, RECENT_TEXT_S, bearingText, candidateWord, coreNowSec, eventWord, useAudio, useCoreNow } from "../store/audio";
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

// One plain sentence per detected sound. Directions only: never a position.
export function soundSentence(ev: AudioEvent): string {
  const when = `${eventWord(ev)} at ${fmtTime(ev.t)}`;
  const conf = `confidence ${fmt(ev.confidence, 2)}`;
  if (ev.bearing_deg === null) return `${when}: direction not resolved (${conf}).`;
  const along = ev.candidates.length
    ? `Along that direction: ${ev.candidates
        .slice(0, 4)
        .map((c) => `${candidateWord(c.kind)} ${c.id}${c.kind === "void" ? ` at ${fmt(c.distance_m, 1)} m` : ""}`)
        .join(", ")}${ev.candidates.length > 4 ? ` and ${ev.candidates.length - 4} more` : ""}.`
    : "Nothing mapped along that direction yet.";
  if (ev.confidence < LOW_CONFIDENCE) return `${when} from about ${bearingText(ev)}, low confidence (${fmt(ev.confidence, 2)}): treat the direction as a guess. ${along}`;
  return `${when} from ${bearingText(ev)} (${conf}). ${along}`;
}

function SoundLines() {
  const events = useAudio((s) => s.events);
  const active = events.length > 0 && coreNowSec() - events[0].t < RECENT_TEXT_S + 2;
  const now = useCoreNow(active, 5000);
  const recent = events.filter((e) => now - e.t < RECENT_TEXT_S).slice(0, 5);
  if (recent.length === 0) return null;
  return (
    <div className="ops-sound">
      {recent.map((e) => (
        <div key={e.id} className={e.confidence < LOW_CONFIDENCE || e.bearing_deg === null ? "dim" : ""}>
          {soundSentence(e)}
        </div>
      ))}
      <div className="ai-label">from the microphones: a direction from the scanner, not a distance</div>
    </div>
  );
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
        <SoundLines />
        <div className="ops-ask-wrap">{ask}</div>
      </div>
    </section>
  );
}
