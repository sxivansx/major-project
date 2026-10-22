import { useState } from "react";
import { api, errorText } from "../api/rest";
import type { AiAsk, AiTriage } from "../api/types";
import { fmt, fmtUsd } from "../lib/format";
import { decisionsAvailable, findingById, isRouteResult, useOps } from "../store/ops";

// The assistant layer (/api/ai/*). Everything it returns is rendered as
// assistant output, labelled as such, and never replaces the deterministic
// findings next to it. AskBox lives in the simple view, RankFindings behind
// Advanced.

export function AskBox() {
  const setRoute = useOps((s) => s.setRoute);
  const [question, setQuestion] = useState("");
  const [ask, setAsk] = useState<AiAsk | null>(null);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const onAsk = async (e: React.FormEvent) => {
    e.preventDefault();
    const q = question.trim();
    if (!q || busy) return;
    setErr(null);
    setBusy(true);
    try {
      const r = await api.aiAsk(q);
      setAsk(r);
      // a route answer is drawn on the map like a manual one
      if (r.intent === "route" && isRouteResult(r.tool)) {
        const t = r.tool;
        const ends = t.found && t.path.length > 1 ? { from: t.path[0], to: t.path[t.path.length - 1] } : undefined;
        setRoute(t, "assistant", ends);
      }
    } catch (e2) {
      setErr(errorText(e2));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="ops-ask">
      <form className="ai-ask" onSubmit={(e) => void onAsk(e)}>
        <input value={question} placeholder="Ask: route to V1, how far is O2, coverage" disabled={busy} onChange={(e) => setQuestion(e.target.value)} />
        <button className="small" type="submit" disabled={busy || !question.trim()} title="POST /api/ai/ask">
          {busy ? "…" : "Ask"}
        </button>
      </form>
      {err && <div className="error">{err}</div>}
      {ask && (
        <div className="ai-section">
          <div className="ops-answer">{ask.answer}</div>
          <div className="ai-label">
            assistant answer · {ask.intent} · confidence {fmt(ask.confidence, 2)}
            {ask.cost_usd !== undefined ? ` · ${fmtUsd(ask.cost_usd)}` : ""}
            {ask.intent === "route" && isRouteResult(ask.tool) ? (ask.tool.found ? " · route drawn on the map" : "") : ""}
          </div>
        </div>
      )}
    </div>
  );
}

export function RankFindings() {
  const cell = useOps((s) => s.cell);
  const findings = useOps((s) => s.findings);
  const aiStatus = useOps((s) => s.aiStatus);
  const select = useOps((s) => s.select);
  const focusOn = useOps((s) => s.focusOn);
  const [triage, setTriage] = useState<AiTriage | null>(null);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const available = decisionsAvailable(aiStatus);
  const run = async () => {
    setErr(null);
    setBusy(true);
    try {
      setTriage(await api.aiTriage({ cell }));
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  const jump = (id: string) => {
    if (!findings) return;
    const hit = findingById(findings, id);
    if (!hit) return;
    select(id);
    focusOn(hit.item.x_m, hit.item.y_m);
  };
  const items = triage ? [...triage.items].sort((a, b) => b.attention - a.attention) : [];

  return (
    <div className="ops-block ops-assistant">
      <div className="ops-block-title">Assistant ranking</div>
      {aiStatus && (
        <div className="hint mono">
          decisions {aiStatus.models?.decisions ?? "–"} · prose {aiStatus.models?.prose ?? "–"}
        </div>
      )}
      {!available ? (
        <div className="hint">Assistant off: {aiStatus?.reason || "no API key"}</div>
      ) : (
        <div className="btn-row">
          <button className="small" disabled={busy || !findings || findings.grid.w === 0} onClick={() => void run()} title="POST /api/ai/triage">
            {busy ? "Ranking" : "Rank findings"}
          </button>
        </div>
      )}
      {err && <div className="error">{err}</div>}
      {triage && (
        <div className="ai-section">
          <div className="ai-label">
            assistant output · ranking by attention
            {triage.cost_usd !== undefined ? ` · ${fmtUsd(triage.cost_usd)}` : ""}
          </div>
          {items.length === 0 && <div className="hint">nothing to rank</div>}
          {items.map((it) => (
            <div key={it.id} className="ai-item" onClick={() => jump(it.id)} title="Show on the map">
              <div className="ai-item-head">
                <span className="mono">{it.id}</span>
                <span className="dim">{it.kind}</span>
                <span className="mono">attention {fmt(it.attention, 2)}</span>
                <span className="conf" title={`confidence ${fmt(it.confidence, 2)}`}>
                  <div style={{ width: `${Math.max(0, Math.min(1, it.confidence)) * 100}%` }} />
                </span>
                <span className="mono dim">{fmt(it.confidence, 2)}</span>
              </div>
              <div className="why">{it.why}</div>
            </div>
          ))}
          <div className="hint">
            rescan {triage.rescan.recommended ? "recommended" : "not recommended"} · p {fmt(triage.rescan.p, 2)} · confidence {fmt(triage.rescan.confidence, 2)}
          </div>
        </div>
      )}
    </div>
  );
}
