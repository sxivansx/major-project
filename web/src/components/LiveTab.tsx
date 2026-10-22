import { useMemo, useState } from "react";
import { errorText } from "../api/rest";
import { fmt, fmtInt } from "../lib/format";
import { disconnectLidar, disconnectServo, servoHome } from "../store/actions";
import { useThrottled } from "../store/hooks";
import { useStore } from "../store/store";
import { ConnectionDetails } from "./ConnectionPanel";
import { LiveSensor, healthWord } from "./LiveSensor";
import { MountPanel } from "./MountPanel";
import { PolarPlot } from "./PolarPlot";
import { RawValues } from "./RawValues";
import { ServoConsole } from "./ServoPanel";
import { Shell, useAdvancedFlag, type BigNumber } from "./Shell";
import { SnapshotButton } from "./SnapshotButton";

export function LiveTab() {
  const [advanced, setAdvanced] = useAdvancedFlag("live");
  const status = useStore((s) => s.status);
  const scan = useThrottled((s) => s.scan, 500);
  const hzMeasured = useThrottled((s) => s.scanHzMeasured, 1000);
  const [busy, setBusy] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);

  const lidar = status?.lidar;
  const servo = status?.servo;
  const lidarOn = !!lidar?.connected;
  const servoOn = !!servo?.connected;
  const alarms = status?.alarms ?? [];

  const range = useMemo(() => {
    if (!scan) return null;
    let lo = Infinity;
    let hi = -Infinity;
    for (let i = 0; i < scan.n; i++) {
      const r = scan.range[i];
      if (r > 0) {
        if (r < lo) lo = r;
        if (r > hi) hi = r;
      }
    }
    return Number.isFinite(lo) ? ([lo, hi] as [number, number]) : null;
  }, [scan]);

  const hz = lidar?.scan_hz ?? (hzMeasured > 0 ? hzMeasured : null);
  const health = lidarOn ? healthWord(lidar?.health) : "off";
  const numbers: BigNumber[] = [
    {
      label: "Scan rate",
      value: lidarOn ? (
        <>
          {fmt(hz, 1)}
          <small>Hz</small>
        </>
      ) : (
        "–"
      ),
    },
    { label: "Points per scan", value: lidarOn && scan ? fmtInt(scan.n) : "–" },
    {
      label: "Range",
      value:
        lidarOn && range ? (
          <>
            {fmt(range[0], 2)}–{fmt(range[1], 2)}
            <small>m</small>
          </>
        ) : (
          "–"
        ),
    },
    { label: "Health", value: health, className: health === "good" ? "ok" : health === "warning" ? "warn" : health === "error" ? "err" : "" },
  ];

  const run = async (key: string, fn: () => Promise<unknown>) => {
    setErr(null);
    setBusy(key);
    try {
      await fn();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };

  return (
    <Shell
      numbers={numbers}
      main={
        <section className="panel ops-mappanel">
          <div className="panel-head">
            <span>Live 2D scan</span>
            <span className="sub">native angle, clockwise from the cable mark</span>
          </div>
          {alarms.length > 0 && <div className="ops-alarm">{alarms.join(" · ")}</div>}
          <PolarPlot />
        </section>
      }
      side={<LiveSensor />}
      actions={
        <>
          {lidarOn && (
            <button disabled={busy !== null} onClick={() => void run("lidar", disconnectLidar)}>
              Disconnect LiDAR
            </button>
          )}
          {servoOn && (
            <button disabled={busy !== null} onClick={() => void run("servo", disconnectServo)}>
              Disconnect servo
            </button>
          )}
          <button disabled={!servoOn || busy !== null} title="Scan plane horizontal (servo home)" onClick={() => void run("home", servoHome)}>
            Level
          </button>
          <SnapshotButton />
          {err && <span className="err mono">{err}</span>}
        </>
      }
      advanced={
        <div className="ops-advanced-grid cols-3">
          <div className="ops-advanced-col">
            <RawValues />
          </div>
          <div className="ops-advanced-col">
            <ServoConsole />
            <ConnectionDetails />
            <section className="panel">
              <div className="panel-head">
                <span>Alarms</span>
                <span className="sub">{alarms.length}</span>
              </div>
              <div className="panel-body">
                {alarms.length === 0 ? <div className="hint">none</div> : alarms.map((a) => <div key={a} className="err mono">{a}</div>)}
              </div>
            </section>
          </div>
          <div className="ops-advanced-col">
            <MountPanel />
          </div>
        </div>
      }
      advancedOpen={advanced}
      onAdvanced={setAdvanced}
    />
  );
}
