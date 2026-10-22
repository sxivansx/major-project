"""Soak run against a live server on synthetic data: back-to-back sweeps for N minutes while a
WebSocket client counts frames, then a report with timings, throughput, memory and alarms.

    uv run python scripts/soak.py --minutes 10            # against http://127.0.0.1:8000 (make sim)
    uv run python scripts/soak.py --minutes 10 --ai       # also exercise /ai/* once per sweep (costs cents)

Writes soak-<timestamp>.md and .json next to this script's output dir (default ../sessions/).
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import httpx

from obslam import wire

ROOT = Path(__file__).resolve().parents[2]


class WsCounter(threading.Thread):
    def __init__(self, url: str):
        super().__init__(daemon=True)
        self.url = url
        self.kinds = {1: 0, 2: 0, 3: 0, "status": 0, "job": 0, "log": 0, "other": 0}
        self.parse_errors = 0
        self.bytes = 0
        self.reconnects = 0
        self.max_scan_gap = 0.0
        self.last_scan = None
        self.logs: list[dict] = []
        self.stop = threading.Event()

    def run(self) -> None:
        from websockets.sync.client import connect

        while not self.stop.is_set():
            try:
                with connect(self.url, max_size=None) as ws:
                    while not self.stop.is_set():
                        msg = ws.recv(timeout=5)
                        if isinstance(msg, bytes):
                            self.bytes += len(msg)
                            k = msg[0]
                            try:
                                if k == 1:
                                    wire.decode_scan(msg)
                                    now = time.monotonic()
                                    if self.last_scan is not None:
                                        self.max_scan_gap = max(self.max_scan_gap, now - self.last_scan)
                                    self.last_scan = now
                                elif k == 2:
                                    wire.decode_points(msg)
                                elif k == 3:
                                    wire.decode_grid(msg)
                            except Exception:  # noqa: BLE001
                                self.parse_errors += 1
                            self.kinds[k if k in (1, 2, 3) else "other"] += 1
                        else:
                            d = json.loads(msg)
                            t = d.get("type", "other")
                            self.kinds[t if t in self.kinds else "other"] += 1
                            if t == "log" and d.get("level") in ("warn", "error"):
                                self.logs.append(d)
            except TimeoutError:
                continue
            except Exception:  # noqa: BLE001
                if not self.stop.is_set():
                    self.reconnects += 1
                    time.sleep(1)


def server_rss_mb() -> float | None:
    try:
        pids = subprocess.run(["pgrep", "-f", "obslam.server|obslam-server"], capture_output=True, text=True).stdout.split()
        best = 0.0
        for pid in pids:
            rss = subprocess.run(["ps", "-o", "rss=", "-p", pid], capture_output=True, text=True).stdout.strip()
            if rss:
                best = max(best, int(rss) / 1024)
        return round(best, 1) if best else None
    except Exception:  # noqa: BLE001
        return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8000")
    ap.add_argument("--minutes", type=float, default=10)
    ap.add_argument("--ai", action="store_true", help="call /ai/triage, /ai/sitrep and /ai/ask once per sweep")
    ap.add_argument("--out", default=str(ROOT / "sessions"))
    args = ap.parse_args()

    c = httpx.Client(base_url=args.url + "/api", timeout=120)
    st = c.get("/status").json()
    if not st["lidar"]["connected"]:
        c.post("/servo/connect", json={"port": "sim"})
        c.post("/lidar/connect", json={"source": "sim"})
    ws = WsCounter(args.url.replace("http", "ws") + "/ws")
    ws.start()

    presets = [
        {"name": "soak-fine", "phi_start": -60, "phi_end": 60, "phi_step": 1, "rotations": 1, "settle_ms": 50},
        {"name": "soak-std", "phi_start": -60, "phi_end": 60, "phi_step": 2, "rotations": 1, "settle_ms": 50},
        {"name": "soak-double", "phi_start": -45, "phi_end": 45, "phi_step": 3, "rotations": 2, "settle_ms": 100},
        {"name": "soak-wide", "phi_start": -75, "phi_end": 75, "phi_step": 2, "rotations": 1, "settle_ms": 50},
    ]
    rss0 = server_rss_mb()
    t_end = time.monotonic() + args.minutes * 60
    rows = []
    i = 0
    while time.monotonic() < t_end:
        p = presets[i % len(presets)]
        i += 1
        t0 = time.monotonic()
        r = c.post("/job/start", json=p)
        if r.status_code != 200:
            rows.append({"n": i, "preset": p["name"], "error": r.text})
            time.sleep(2)
            continue
        job = None
        while True:
            job = c.get("/status").json()["job"]
            if job and job["state"] != "running":
                break
            time.sleep(0.5)
        sweep_s = time.monotonic() - t0
        ta = time.monotonic()
        m = c.get("/metrics?cell=0.1").json()
        f = c.get("/ops/findings?cell=0.1").json()
        c.get("/ops/layers?cell=0.1").content
        analysis_s = time.monotonic() - ta
        row = {
            "n": i,
            "preset": p["name"],
            "state": job["state"],
            "steps": job["n_steps"],
            "points": job["points"],
            "sweep_s": round(sweep_s, 1),
            "pts_per_s": round(job["points"] / sweep_s) if sweep_s else None,
            "analysis_s": round(analysis_s, 2),
            "coverage_pct": f["coverage"]["coverage_pct"],
            "obstacles": len(f["obstacles"]),
            "shadows": len(f["shadows"]),
            "floor_rms_mm": (m.get("floor") or {}).get("rms_mm"),
            "rss_mb": server_rss_mb(),
            "alarms": c.get("/status").json().get("alarms", []),
        }
        if args.ai:
            tai = time.monotonic()
            try:
                tri = c.post("/ai/triage", json={"cell": 0.1}).json()
                sit = c.post("/ai/sitrep", json={"cell": 0.1}).json()
                ask = c.post("/ai/ask", json={"question": "where should we scan next?", "cell": 0.1}).json()
                row["ai_s"] = round(time.monotonic() - tai, 1)
                row["ai_cost_usd"] = round(sum(float(x.get("cost_usd") or 0) for x in (tri, sit, ask)), 5)
                row["ai_rescan_p"] = (tri.get("rescan") or {}).get("p")
                row["ai_sitrep_model"] = sit.get("model")
            except Exception as e:  # noqa: BLE001
                row["ai_error"] = str(e)[:120]
        rows.append(row)
        print(f"sweep {i:2} {p['name']:12} {job['state']:7} {job['points']:7} pts in {sweep_s:5.1f} s, analysis {analysis_s:.2f} s, rss {row['rss_mb']} MB, alarms {row['alarms']}", flush=True)
        # sessions pile up on disk; keep the last five from this run
        sess = [s for s in c.get("/sessions").json() if s["name"].startswith("soak-")]
        for s in sess[5:]:
            c.delete(f"/sessions/{s['id']}")

    ws.stop.set()
    ws.join(timeout=3)
    good = [r for r in rows if "error" not in r]
    sweep_times = [r["sweep_s"] for r in good]
    summary = {
        "started": datetime.now().isoformat(timespec="seconds"),
        "minutes": args.minutes,
        "sweeps": len(good),
        "failed_starts": len(rows) - len(good),
        "points_total": sum(r["points"] for r in good),
        "sweep_s_min_mean_max": [min(sweep_times), round(statistics.mean(sweep_times), 1), max(sweep_times)] if sweep_times else None,
        "analysis_s_max": max((r["analysis_s"] for r in good), default=None),
        "rss_mb_start_end": [rss0, good[-1]["rss_mb"] if good else None],
        "ws": {"frames": ws.kinds, "bytes_mb": round(ws.bytes / 1e6, 1), "parse_errors": ws.parse_errors, "reconnects": ws.reconnects, "max_scan_gap_s": round(ws.max_scan_gap, 2)},
        "alarms_seen": sorted({a for r in good for a in r["alarms"]}),
        "warn_error_logs": ws.logs[:20],
        "ai_cost_usd_total": round(sum(r.get("ai_cost_usd", 0) for r in good), 4) if args.ai else None,
    }
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    (out / f"soak-{stamp}.json").write_text(json.dumps({"summary": summary, "sweeps": rows}, indent=2))
    md = [f"# Soak run {stamp}: {args.minutes:g} min on synthetic data", ""]
    for k, v in summary.items():
        if k not in ("warn_error_logs",):
            md.append(f"- **{k}**: {v}")
    md += ["", "| # | preset | state | steps | points | sweep s | pts/s | analysis s | cov % | obst | shadows | floor RMS mm | RSS MB | alarms |", "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in good:
        md.append(f"| {r['n']} | {r['preset']} | {r['state']} | {r['steps']} | {r['points']} | {r['sweep_s']} | {r['pts_per_s']} | {r['analysis_s']} | {r['coverage_pct']} | {r['obstacles']} | {r['shadows']} | {r['floor_rms_mm']} | {r['rss_mb']} | {', '.join(r['alarms']) or '-'} |")
    if ws.logs:
        md += ["", "## warn/error log lines seen on the WebSocket", ""] + [f"- `{l['level']}` {l['msg']}" for l in ws.logs[:20]]
    (out / f"soak-{stamp}.md").write_text("\n".join(md) + "\n")
    print("\nreport:", out / f"soak-{stamp}.md")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
