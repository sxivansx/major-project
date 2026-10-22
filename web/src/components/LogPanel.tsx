import { useEffect, useRef } from "react";
import { LOG_MAX, useStore } from "../store/store";
import { fmtTime } from "../lib/format";

export function LogPanel() {
  const log = useStore((s) => s.log);
  const clearLog = useStore((s) => s.clearLog);
  const ref = useRef<HTMLDivElement>(null);
  const stick = useRef(true);

  useEffect(() => {
    const el = ref.current;
    if (el && stick.current) el.scrollTop = el.scrollHeight;
  }, [log]);

  return (
    <section className="panel full-panel">
      <div className="panel-head">
        <span>Log</span>
        <span style={{ display: "flex", gap: 6, alignItems: "center" }}>
          <span className="sub">
            last {LOG_MAX} · {log.length} shown
          </span>
          <button className="small" onClick={clearLog}>
            Clear
          </button>
        </span>
      </div>
      <div
        className="panel-body"
        ref={ref}
        onScroll={(e) => {
          const el = e.currentTarget;
          stick.current = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
        }}
      >
        {log.length === 0 ? (
          <div className="empty">Nothing logged yet.</div>
        ) : (
          <div className="log-list">
            {log.map((l) => (
              <div className="log-line" key={l.id}>
                <span className="t">{fmtTime(l.t)}</span>
                <span className={l.level === "error" ? "err" : l.level === "warn" ? "warn" : "dim"}>{l.level}</span>
                <span className="dim">{l.src}</span>
                <span>{l.msg}</span>
              </div>
            ))}
          </div>
        )}
      </div>
    </section>
  );
}
