import { useEffect, useRef } from "react";
import { LOG_MAX, useStore } from "../store/store";
import { fmtInt, fmtTime } from "../lib/format";
import { Shell, type BigNumber } from "./Shell";

export function LogPanel() {
  const log = useStore((s) => s.log);
  const clearLog = useStore((s) => s.clearLog);
  const ref = useRef<HTMLDivElement>(null);
  const stick = useRef(true);

  useEffect(() => {
    const el = ref.current;
    if (el && stick.current) el.scrollTop = el.scrollHeight;
  }, [log]);

  const errors = log.filter((l) => l.level === "error");
  const warnings = log.filter((l) => l.level === "warn");
  const lastError = errors[errors.length - 1];
  const lastWarning = warnings[warnings.length - 1];
  const numbers: BigNumber[] = [
    { label: "Errors", value: fmtInt(errors.length), className: errors.length ? "err" : "" },
    { label: "Warnings", value: fmtInt(warnings.length), className: warnings.length ? "warn" : "" },
    { label: "Entries", value: fmtInt(log.length) },
  ];

  return (
    <Shell
      numbers={numbers}
      main={
        <section className="panel ops-mappanel">
          <div className="panel-head">
            <span>Log</span>
            <span className="sub">
              last {LOG_MAX} · {log.length} shown
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
      }
      side={
        <section className="panel ops-right">
          <div className="panel-head">
            <span>Latest</span>
          </div>
          <div className="panel-body ops-summary">
            <div className="sensor-text">{lastError ? `Last error at ${fmtTime(lastError.t)}: ${lastError.msg}` : "No errors."}</div>
            <div className="sensor-text">{lastWarning ? `Last warning at ${fmtTime(lastWarning.t)}: ${lastWarning.msg}` : "No warnings."}</div>
            <div className="hint">Server messages come over the WebSocket; client lines are this dashboard's own calls.</div>
          </div>
        </section>
      }
      actions={
        <button disabled={log.length === 0} onClick={clearLog}>
          Clear
        </button>
      }
    />
  );
}
