import { useEffect, useMemo, useRef, useState } from "react";
import { useThrottled } from "../store/hooks";

// Virtualised table of the latest scan with an angle filter (Live scan tab,
// Advanced drawer). The table re-renders at most 4x per second, the scan
// itself arrives at 10 Hz.

const ROW_H = 20;
const OVERSCAN = 6;
const REFRESH_MS = 250;

export function RawValues() {
  const scan = useThrottled((s) => s.scan, REFRESH_MS);

  const [aMin, setAMin] = useState("");
  const [aMax, setAMax] = useState("");
  const [scrollTop, setScrollTop] = useState(0);
  const [viewH, setViewH] = useState(400);
  const scrollRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const el = scrollRef.current;
    if (!el) return;
    const measure = () => setViewH(el.clientHeight || 400);
    measure();
    const ro = new ResizeObserver(measure);
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const idx = useMemo(() => {
    if (!scan) return [] as number[];
    const lo = aMin === "" ? -Infinity : Number(aMin);
    const hi = aMax === "" ? Infinity : Number(aMax);
    const wrap = Number.isFinite(lo) && Number.isFinite(hi) && lo > hi;
    const out: number[] = [];
    for (let i = 0; i < scan.n; i++) {
      const a = scan.angle[i];
      const inRange = wrap ? a >= lo || a <= hi : a >= lo && a <= hi;
      if (inRange) out.push(i);
    }
    return out;
  }, [scan, aMin, aMax]);

  const total = idx.length;
  const first = Math.max(0, Math.floor(scrollTop / ROW_H) - OVERSCAN);
  const last = Math.min(total, Math.ceil((scrollTop + viewH) / ROW_H) + OVERSCAN);

  const rows = [];
  if (scan) {
    for (let k = first; k < last; k++) {
      const i = idx[k];
      rows.push(
        <div className="raw-row" key={k} style={{ top: k * ROW_H }}>
          <span>{i}</span>
          <span>{scan.angle[i].toFixed(2)}</span>
          <span>{scan.range[i].toFixed(3)}</span>
          <span>{scan.quality[i]}</span>
        </div>,
      );
    }
  }

  return (
    <section className="panel raw-wrap">
      <div className="panel-head">
        <span>Raw values</span>
        <span className="sub">
          {scan ? `${total} of ${scan.n} rows` : "no scan"}
          {scan && aMin !== "" && aMax !== "" && Number(aMin) > Number(aMax) ? " (wrapped)" : ""}
        </span>
      </div>
      <div className="raw-toolbar">
        <span className="dim">Angle filter</span>
        <input type="number" placeholder="from" value={aMin} min={0} max={360} onChange={(e) => setAMin(e.target.value)} />
        <span className="dim">to</span>
        <input type="number" placeholder="to" value={aMax} min={0} max={360} onChange={(e) => setAMax(e.target.value)} />
        {(aMin !== "" || aMax !== "") && (
          <button
            className="small"
            onClick={() => {
              setAMin("");
              setAMax("");
            }}
          >
            Clear
          </button>
        )}
      </div>
      <div className="raw-head">
        <span>#</span>
        <span>angle °</span>
        <span>range m</span>
        <span>quality</span>
      </div>
      <div className="raw-scroll" ref={scrollRef} onScroll={(e) => setScrollTop(e.currentTarget.scrollTop)}>
        <div style={{ height: total * ROW_H, position: "relative" }}>{rows}</div>
      </div>
    </section>
  );
}
