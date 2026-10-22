import { SnapshotButton } from "./SnapshotButton";
import { useEffect, useMemo, useRef, useState } from "react";
import { useStore } from "../store/store";
import { useThrottled } from "../store/hooks";
import { fmt, fmtInt } from "../lib/format";

const ROW_H = 20;
const OVERSCAN = 6;
const REFRESH_MS = 250;

export function RawValues() {
  // the table re-renders at most 4x per second, the scan itself arrives at 10 Hz
  const scan = useThrottled((s) => s.scan, REFRESH_MS);
  const hzMeasured = useThrottled((s) => s.scanHzMeasured, 1000);
  const lidar = useStore((s) => s.status?.lidar);

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

  const { idx, rmin, rmax } = useMemo(() => {
    if (!scan) return { idx: [] as number[], rmin: NaN, rmax: NaN };
    const lo = aMin === "" ? -Infinity : Number(aMin);
    const hi = aMax === "" ? Infinity : Number(aMax);
    const wrap = Number.isFinite(lo) && Number.isFinite(hi) && lo > hi;
    const idx: number[] = [];
    let rmin = Infinity;
    let rmax = -Infinity;
    for (let i = 0; i < scan.n; i++) {
      const r = scan.range[i];
      if (r > 0) {
        if (r < rmin) rmin = r;
        if (r > rmax) rmax = r;
      }
      const a = scan.angle[i];
      const inRange = wrap ? a >= lo || a <= hi : a >= lo && a <= hi;
      if (inRange) idx.push(i);
    }
    return { idx, rmin: idx.length || scan.n ? rmin : NaN, rmax };
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

  const hz = lidar?.scan_hz ?? null;

  return (
    <>
      <div className="bignums">
        <div className="bignum">
          <div className="label">Scan rate</div>
          <div className="value">
            {fmt(hz ?? (hzMeasured > 0 ? hzMeasured : null), 1)}
            <small>Hz{hz != null ? "" : " (measured)"}</small>
          </div>
        </div>
        <div className="bignum">
          <div className="label">Points in scan</div>
          <div className="value">{scan ? fmtInt(scan.n) : "–"}</div>
        </div>
        <div className="bignum">
          <div className="label">Min range</div>
          <div className="value">
            {Number.isFinite(rmin) ? fmt(rmin, 3) : "–"}
            <small>m</small>
          </div>
        </div>
        <div className="bignum">
          <div className="label">Max range</div>
          <div className="value">
            {Number.isFinite(rmax) ? fmt(rmax, 2) : "–"}
            <small>m</small>
          </div>
        </div>
      </div>
      <section className="panel raw-wrap">
        <div className="raw-toolbar">
          <span className="dim">Angle filter</span>
          <input
            type="number"
            placeholder="from"
            value={aMin}
            min={0}
            max={360}
            onChange={(e) => setAMin(e.target.value)}
          />
          <span className="dim">to</span>
          <input
            type="number"
            placeholder="to"
            value={aMax}
            min={0}
            max={360}
            onChange={(e) => setAMax(e.target.value)}
          />
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
          <span className="spacer" style={{ flex: 1 }} />
          <SnapshotButton />
          <span className="mono dim">
            {scan ? `${total} of ${scan.n} rows` : "no scan"}
            {scan && aMin !== "" && aMax !== "" && Number(aMin) > Number(aMax) ? " (wrapped)" : ""}
          </span>
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
    </>
  );
}
