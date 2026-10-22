import { useEffect, useState } from "react";
import { api, errorText } from "../api/rest";
import { clientLog, useStore } from "../store/store";
import { fmt, fmtInt, text } from "../lib/format";

const SIM = "sim";

export function ConnectionPanel() {
  const status = useStore((s) => s.status);
  const ports = useStore((s) => s.ports);
  const setPorts = useStore((s) => s.setPorts);
  const setStatus = useStore((s) => s.setStatus);

  const lidar = status?.lidar;
  const servo = status?.servo;

  const [source, setSource] = useState<"s2" | "sim">("s2");
  const [lidarPort, setLidarPort] = useState("");
  const [baud, setBaud] = useState(1000000);
  const [servoPort, setServoPort] = useState(SIM);
  const [busy, setBusy] = useState<string | null>(null);
  const [lidarErr, setLidarErr] = useState<string | null>(null);
  const [servoErr, setServoErr] = useState<string | null>(null);

  // pick the first port once the list arrives, unless the user already chose
  useEffect(() => {
    if (!lidarPort && ports.length) setLidarPort(ports[0].path);
  }, [ports, lidarPort]);

  const refreshPorts = async () => {
    setBusy("ports");
    try {
      setPorts(await api.ports());
    } catch (e) {
      clientLog("error", `GET /api/ports: ${errorText(e)}`);
    } finally {
      setBusy(null);
    }
  };

  const run = async (
    key: string,
    fn: () => Promise<unknown>,
    setErr: (s: string | null) => void,
  ) => {
    setBusy(key);
    setErr(null);
    try {
      const r = await fn();
      if (r && typeof r === "object" && "lidar" in (r as object)) setStatus(r as never);
    } catch (e) {
      setErr(errorText(e));
      clientLog("error", `${key}: ${errorText(e)}`);
    } finally {
      setBusy(null);
    }
  };

  const lidarConnected = !!lidar?.connected;
  const servoConnected = !!servo?.connected;

  return (
    <section className="panel">
      <div className="panel-head">
        <span>Connection</span>
        <span className="sub">{status ? "" : "no status yet"}</span>
      </div>
      <div className="panel-body">
        <div className="section-title">Lidar</div>
        <div className="field">
          <label>Source</label>
          <select
            value={source}
            disabled={lidarConnected}
            onChange={(e) => setSource(e.target.value as "s2" | "sim")}
          >
            <option value="s2">s2 (RPLIDAR S2M1)</option>
            <option value="sim">sim (simulator)</option>
          </select>
        </div>
        <div className="field">
          <label>Port</label>
          <div className="row">
            <select
              value={lidarPort}
              disabled={lidarConnected || source === "sim"}
              onChange={(e) => setLidarPort(e.target.value)}
            >
              {ports.length === 0 && <option value="">(no ports found)</option>}
              {ports.map((p) => (
                <option key={p.path} value={p.path}>
                  {p.path}
                  {p.description ? ` (${p.description})` : ""}
                </option>
              ))}
            </select>
            <button onClick={refreshPorts} disabled={busy === "ports"} title="Rescan serial ports">
              Refresh
            </button>
          </div>
        </div>
        <div className="field">
          <label>Baud</label>
          <input
            type="number"
            value={baud}
            disabled={lidarConnected || source === "sim"}
            onChange={(e) => setBaud(Number(e.target.value))}
          />
        </div>
        <div className="btn-row">
          {lidarConnected ? (
            <button
              onClick={() => run("lidar/disconnect", () => api.lidarDisconnect(), setLidarErr)}
              disabled={busy !== null}
            >
              Disconnect lidar
            </button>
          ) : (
            <button
              className="primary"
              disabled={busy !== null || (source === "s2" && !lidarPort)}
              onClick={() =>
                run(
                  "lidar/connect",
                  () =>
                    api.lidarConnect({
                      source,
                      port: source === "sim" ? SIM : lidarPort,
                      baud,
                    }),
                  setLidarErr,
                )
              }
            >
              Connect lidar
            </button>
          )}
        </div>
        {lidarErr && <div className="error">{lidarErr}</div>}
        <dl className="kv">
          <dt>State</dt>
          <dd className={lidarConnected ? "ok" : "dim"}>
            {lidarConnected ? `connected (${text(lidar?.source)})` : "disconnected"}
          </dd>
          <dt>Model</dt>
          <dd>{text(lidar?.model)}</dd>
          <dt>Serial</dt>
          <dd title={lidar?.serial ?? ""}>{text(lidar?.serial)}</dd>
          <dt>Firmware</dt>
          <dd>
            {text(lidar?.firmware)}
            {lidar?.hardware != null ? ` / hw ${lidar.hardware}` : ""}
          </dd>
          <dt>Health</dt>
          <dd className={lidar?.health && lidar.health !== "good" ? "warn" : ""}>
            {text(lidar?.health)}
          </dd>
          <dt>Scan rate</dt>
          <dd>{fmt(lidar?.scan_hz, 1, " Hz")}</dd>
          <dt>Points/rev</dt>
          <dd>{fmtInt(lidar?.pts_per_rev)}</dd>
          <dt>Range</dt>
          <dd>
            {fmt(lidar?.range_min_m, 2)} – {fmt(lidar?.range_max_m, 2)} m
          </dd>
          <dt>Port</dt>
          <dd title={lidar?.port ?? ""}>
            {text(lidar?.port)}
            {lidar?.baud ? ` @ ${lidar.baud}` : ""}
          </dd>
        </dl>

        <div className="section-title">Servo (ESP32 tilt)</div>
        <div className="field">
          <label>Port</label>
          <div className="row">
            <select
              value={servoPort}
              disabled={servoConnected}
              onChange={(e) => setServoPort(e.target.value)}
            >
              <option value={SIM}>sim (no hardware)</option>
              {ports.map((p) => (
                <option key={p.path} value={p.path}>
                  {p.path}
                  {p.description ? ` (${p.description})` : ""}
                </option>
              ))}
            </select>
            <button onClick={refreshPorts} disabled={busy === "ports"} title="Rescan serial ports">
              Refresh
            </button>
          </div>
        </div>
        <div className="btn-row">
          {servoConnected ? (
            <button
              onClick={() => run("servo/disconnect", () => api.servoDisconnect(), setServoErr)}
              disabled={busy !== null}
            >
              Disconnect servo
            </button>
          ) : (
            <button
              className="primary"
              disabled={busy !== null}
              onClick={() => run("servo/connect", () => api.servoConnect(servoPort), setServoErr)}
            >
              Connect servo
            </button>
          )}
        </div>
        {servoErr && <div className="error">{servoErr}</div>}
        <dl className="kv">
          <dt>State</dt>
          <dd className={servoConnected ? "ok" : "dim"}>
            {servoConnected ? "connected" : "disconnected"}
          </dd>
          <dt>Port</dt>
          <dd title={servo?.port ?? ""}>{text(servo?.port)}</dd>
          <dt>Limits</dt>
          <dd>
            {fmtInt(servo?.min_deg)} – {fmtInt(servo?.max_deg)} of {fmtInt(servo?.range)}°
          </dd>
        </dl>
      </div>
    </section>
  );
}
