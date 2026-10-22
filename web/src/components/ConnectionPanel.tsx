import { useEffect, useState } from "react";
import { errorText } from "../api/rest";
import { connectLidar, connectServo, refreshPorts } from "../store/actions";
import { useStore } from "../store/store";
import { fmt, fmtInt, text } from "../lib/format";

// Connecting lives in the Live scan tab's Sensor column (LidarConnect,
// ServoConnect); the device detail rows live in its Advanced drawer
// (ConnectionDetails). ConnectionPanel composes them the old sidebar way.

export const SIM = "sim";

function PortSelect({ value, onChange, disabled, sim }: { value: string; onChange: (v: string) => void; disabled?: boolean; sim?: boolean }) {
  const ports = useStore((s) => s.ports);
  const [busy, setBusy] = useState(false);
  return (
    <div className="sensor-row">
      <select value={value} disabled={disabled} onChange={(e) => onChange(e.target.value)}>
        {sim && <option value={SIM}>sim (no hardware)</option>}
        {!sim && ports.length === 0 && <option value="">(no ports found)</option>}
        {ports.map((p) => (
          <option key={p.path} value={p.path}>
            {p.path}
            {p.description ? ` (${p.description})` : ""}
          </option>
        ))}
      </select>
      <button
        className="small"
        disabled={busy}
        title="Rescan serial ports"
        onClick={() => {
          setBusy(true);
          refreshPorts().catch(() => undefined).finally(() => setBusy(false));
        }}
      >
        Refresh
      </button>
    </div>
  );
}

// Source, port, baud and the Connect button for the lidar.
export function LidarConnect() {
  const ports = useStore((s) => s.ports);
  const [source, setSource] = useState<"s2" | "sim">("s2");
  const [port, setPort] = useState("");
  const [baud, setBaud] = useState(1000000);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  // pick the first port once the list arrives, unless the user already chose
  useEffect(() => {
    if (!port && ports.length) setPort(ports[0].path);
  }, [ports, port]);

  const connect = async () => {
    setErr(null);
    setBusy(true);
    try {
      await connectLidar({ source, port: source === "sim" ? SIM : port, baud });
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="sensor-connect">
      <div className="sensor-row">
        <select value={source} onChange={(e) => setSource(e.target.value as "s2" | "sim")} title="Source">
          <option value="s2">s2 (RPLIDAR S2M1)</option>
          <option value="sim">sim (simulator)</option>
        </select>
        <input
          type="number"
          value={baud}
          disabled={source === "sim"}
          title="Baud"
          style={{ width: 96 }}
          onChange={(e) => setBaud(Number(e.target.value))}
        />
      </div>
      <PortSelect value={port} onChange={setPort} disabled={source === "sim"} />
      <div className="sensor-row">
        <button className="primary" disabled={busy || (source === "s2" && !port)} onClick={() => void connect()}>
          Connect LiDAR
        </button>
      </div>
      {err && <div className="error">{err}</div>}
    </div>
  );
}

// Port and the Connect button for the ESP32 tilt servo.
export function ServoConnect() {
  const [port, setPort] = useState(SIM);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const connect = async () => {
    setErr(null);
    setBusy(true);
    try {
      await connectServo(port);
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="sensor-connect">
      <PortSelect value={port} onChange={setPort} sim />
      <div className="sensor-row">
        <button className="primary" disabled={busy} onClick={() => void connect()}>
          Connect servo
        </button>
      </div>
      {err && <div className="error">{err}</div>}
    </div>
  );
}

// The device details from the status object.
export function ConnectionDetails() {
  const status = useStore((s) => s.status);
  const lidar = status?.lidar;
  const servo = status?.servo;
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
        <dl className="kv">
          <dt>State</dt>
          <dd className={lidarConnected ? "ok" : "dim"}>{lidarConnected ? `connected (${text(lidar?.source)})` : "disconnected"}</dd>
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
          <dd className={lidar?.health && lidar.health !== "good" ? "warn" : ""}>{text(lidar?.health)}</dd>
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
        <dl className="kv">
          <dt>State</dt>
          <dd className={servoConnected ? "ok" : "dim"}>{servoConnected ? "connected" : "disconnected"}</dd>
          <dt>Port</dt>
          <dd title={servo?.port ?? ""}>{text(servo?.port)}</dd>
          <dt>Limits</dt>
          <dd>
            {fmtInt(servo?.min_deg)} – {fmtInt(servo?.max_deg)} of {fmtInt(servo?.range)}°
          </dd>
          <dt>servo_deg</dt>
          <dd>{fmt(servo?.servo_deg, 1, "°")}</dd>
          <dt>phi_deg</dt>
          <dd>{fmt(servo?.phi_deg, 1, "°")}</dd>
          <dt>encoder</dt>
          <dd>
            {servo?.encoder ?? "–"}
            {servo?.meas_deg != null ? ` · ${fmt(servo.meas_deg, 1)}° (φ ${fmt(servo.phi_meas_deg, 1)}°)` : ""}
            {servo?.settled === false ? " · moving" : ""}
          </dd>
        </dl>
      </div>
    </section>
  );
}

export function ConnectionPanel() {
  const status = useStore((s) => s.status);
  return (
    <>
      {!status?.lidar.connected && <LidarConnect />}
      {!status?.servo.connected && <ServoConnect />}
      <ConnectionDetails />
    </>
  );
}
