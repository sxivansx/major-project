import type { AudioSource } from "../api/types";
import { fmt } from "../lib/format";
import {
  LOW_CONFIDENCE,
  agoText,
  candidateWord,
  excludedList,
  knocksText,
  sourceBearingText,
  sourceKindWord,
  sourceRank,
  useAudio,
} from "../store/audio";
import { useStore } from "../store/store";

// Sources (§5.6): what the operator should look at. One card per active
// source, strongest first; "Show on map" switches to the Operator tab with
// that source's wedge highlighted.

export function SourceCards({ sources, now }: { sources: AudioSource[]; now: number }) {
  const highlight = useAudio((s) => s.highlightSourceId);
  if (sources.length === 0)
    return (
      <div className="hint">
        No sources right now. A source appears when sounds keep coming from one direction; repeated knocks from one direction show as tapping.
      </div>
    );
  const sorted = [...sources].sort(sourceRank);
  return (
    <div className="source-cards">
      {sorted.map((s) => {
        const low = s.confidence < LOW_CONFIDENCE;
        return (
          <div key={s.id} className={"source-card" + (s.id === highlight ? " sel" : "") + (low ? " weak" : "")}>
            <div className="source-head">
              <span className="source-kind">{sourceKindWord(s)}</span>
              <span className="mono">{sourceBearingText(s)}</span>
              <span className="spacer" />
              <span className="mono dim">#{s.id}</span>
            </div>
            <div className="source-line">
              {knocksText(s.n_events, s.kind)} · last heard {agoText(now - s.last_t)}
              {s.rate_per_min ? ` · ${fmt(s.rate_per_min, 0)} per min` : ""}
            </div>
            <div className="source-line">
              <span className="conf" title={`confidence ${fmt(s.confidence, 2)}`}>
                <div style={{ width: `${Math.max(0, Math.min(1, s.confidence)) * 100}%` }} />
              </span>{" "}
              <span className="mono">{fmt(s.confidence, 2)}</span>
              {low && <span className="dim"> · weak, direction is a guess</span>}
              {s.n_events === 1 && !low && <span className="dim"> · heard once</span>}
            </div>
            <div className="source-line dim">
              {s.candidates.length
                ? `Along it: ${s.candidates
                    .slice(0, 4)
                    .map((c) => `${candidateWord(c.kind)} ${c.id}${c.kind === "void" ? ` ${fmt(c.distance_m, 1)} m` : ""}`)
                    .join(", ")}${s.candidates.length > 4 ? ` +${s.candidates.length - 4}` : ""}`
                : "Nothing mapped along it yet."}
            </div>
            <button
              className="linklike"
              onClick={() => {
                useAudio.setState({ highlightSourceId: s.id });
                useStore.getState().setTab("ops");
              }}
            >
              Show on map
            </button>
          </div>
        );
      })}
    </div>
  );
}

// One line per mic: noise floor, and why a mic is excluded from direction finding.
export function MicHealth() {
  const status = useStore((s) => s.status);
  const audio = status?.audio;
  if (!audio?.connected) return null;
  const noise = audio.noise_dbfs ?? [];
  const clip = audio.clipping ?? [];
  const excluded = excludedList(status);
  const n = Math.max(noise.length, 4);
  return (
    <div className="legend mic-health">
      <span className="dim">Noise floor</span>
      {Array.from({ length: n }, (_, i) => {
        const ex = excluded.find((x) => x.mic === i + 1);
        return (
          <span key={i} className={ex || clip[i] ? "err" : ""} title={ex?.reason || undefined}>
            mic {i + 1} {noise[i] !== undefined ? `${fmt(noise[i], 0)} dBFS` : "–"}
            {clip[i] ? " · clipping" : ""}
            {ex ? ` · excluded${ex.reason ? `: ${ex.reason}` : ""}` : ""}
          </span>
        );
      })}
      {excluded.length > 0 && <span className="dim">direction from the other {n - excluded.length} mics</span>}
    </div>
  );
}
