import { useEffect, useState } from "react";
import { api, errorText } from "../api/rest";
import type { AiAsk, AiSitrep, AiStatus, AiTriage } from "../api/types";
import { fmt, fmtUsd } from "../lib/format";
import { findingById, isRouteResult, useOps } from "../store/ops";

// The assistant layer (/api/ai/*). Everything it returns is rendered as
// assistant output, labelled as such, and never replaces the deterministic
// findings next to it.

function Cost({ v }: { v: number | undefined }) {
  return v === undefined || v === null ? null : <span className="dim"> · {fmtUsd(v)}</span>;
}

export function OpsAssistant() {
  const cell = useOps((s) => s.cell);
  const findings = useOps((s) => s.findings);
  const select = useOps((s) => s.select);
  const focusOn = useOps((s) => s.focusOn);
  const setRoute = useOps((s) => s.setRoute);

  const [status, setStatus] = useState<AiStatus | null>(null);
  const [statusErr, setStatusErr] = useState<string | null>(null);
  const [triage, setTriage] = useState<AiTriage | null>(null);
  const [sitrep, setSitrep] = useState<AiSitrep | null>(null);
  const [ask, setAsk] = useState<AiAsk | null>(null);
  const [question, setQuestion] = useState("");
  const [busy, setBusy] = useState<"triage" | "sitrep" | "ask" | null>(null);
  const [err, setErr] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    api
      .aiStatus()
      .then((s) => {
        if (!cancelled) setStatus(s);
      })
      .catch((e) => {
        if (!cancelled) setStatusErr(errorText(e));
      });
    return () => {
      cancelled = true;
    };
  }, []);

  const run = async <T,>(what: "triage" | "sitrep" | "ask", call: () => Promise<T>, done: (r: T) => void) => {
    setErr(null);
    setBusy(what);
    try {
      done(await call());
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };

  const onAsk = (e: React.FormEvent) => {
    e.preventDefault();
    const q = question.trim();
    if (!q) return;
    void run("ask", () => api.aiAsk(q), (r) => {
      setAsk(r);
      // a route answer is drawn on the map like a manual one
      if (r.intent === "route" && isRouteResult(r.tool)) {
        const t = r.tool;
        const ends = t.found && t.path.length > 1 ? { from: t.path[0], to: t.path[t.path.length - 1] } : undefined;
        setRoute(t, "assistant", ends);
      }
    });
  };

  const jump = (id: string) => {
    if (!findings) return;
    const hit = findingById(findings, id);
    if (!hit) return;
    select(id);
    focusOn(hit.item.x_m, hit.item.y_m);
  };

  let body: React.ReactNode;
  if (statusErr) body = <div className="hint">unavailable: {statusErr}</div>;
  else if (!status) body = <div className="hint">checking assistant status</div>;
  else if (!status.available) body = <div className="hint">unavailable: {status.reason || "no reason given"}</div>;
  else {
    const items = triage ? [...triage.items].sort((a, b) => b.attention - a.attention) : [];
    body = (
      <>
        <div className="hint mono">
          decisions {status.models?.decisions ?? "–"} · prose {status.models?.prose ?? "–"}
        </div>
        <div className="btn-row">
          <button className="small" disabled={busy !== null || !findings} onClick={() => void run("triage", () => api.aiTriage({ cell }), setTriage)}>
            {busy === "triage" ? "Ranking" : "Rank findings"}
          </button>
          <button className="small" disabled={busy !== null || !findings} onClick={() => void run("sitrep", () => api.aiSitrep({ cell }), setSitrep)}>
            {busy === "sitrep" ? "Writing" : "Situation report"}
          </button>
        </div>
        {err && <div className="error">{err}</div>}

        {triage && (
          <div className="ai-section">
            <div className="ai-label">
              assistant output · ranking by attention
              <Cost v={triage.cost_usd} />
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
              rescan {triage.rescan.recommended ? "recommended" : "not recommended"} · p {fmt(triage.rescan.p, 2)} · confidence{" "}
              {fmt(triage.rescan.confidence, 2)}
            </div>
          </div>
        )}

        {sitrep && (
          <div className="ai-section">
            <div className="ai-label">
              assistant summary{sitrep.model ? ` · ${sitrep.model}` : ""}
              <Cost v={sitrep.cost_usd} />
            </div>
            <pre className="ai-out">{sitrep.text}</pre>
          </div>
        )}

        <form className="ai-ask" onSubmit={onAsk}>
          <input
            value={question}
            placeholder="Ask: route to V1, how far is O2, coverage"
            disabled={busy !== null}
            onChange={(e) => setQuestion(e.target.value)}
          />
          <button className="small" type="submit" disabled={busy !== null || !question.trim()}>
            {busy === "ask" ? "Asking" : "Ask"}
          </button>
        </form>
        {ask && (
          <div className="ai-section">
            <div className="ai-label">
              assistant answer · intent {ask.intent} · confidence {fmt(ask.confidence, 2)}
              <Cost v={ask.cost_usd} />
            </div>
            {ask.intent === "route" && isRouteResult(ask.tool) ? (
              <div className="hint">
                {ask.tool.found
                  ? `route drawn on the map: ${fmt(ask.tool.length_m, 1)} m, ${ask.tool.verdict}, max step ${fmt(ask.tool.max_step_m, 2)} m`
                  : `no route: ${ask.tool.reason}`}
              </div>
            ) : ask.tool !== undefined && ask.tool !== null ? (
              <pre className="ai-out dim">{compact(ask.tool)}</pre>
            ) : null}
            <pre className="ai-out">{ask.answer}</pre>
          </div>
        )}
      </>
    );
  }

  return (
    <div className="ops-block ops-assistant">
      <div className="ops-block-title">Assistant</div>
      {body}
    </div>
  );
}

function compact(v: unknown): string {
  const s = JSON.stringify(v, null, 1);
  return s.length > 1200 ? s.slice(0, 1200) + "\n…" : s;
}
