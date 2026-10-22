import { useEffect, useState } from "react";
import { errorText } from "../api/rest";
import { connectLidar, connectServo, refreshPorts } from "../store/actions";
import { useStore } from "../store/store";
import { fmt, fmtInt, text } from "../lib/format";

// Connecting lives in the Live scan tab's Sensor column (LidarConnect,
// ServoConnect); the device detail rows live in its Advanced drawer
// (ConnectionDetails). ConnectionPanel composes them the old sidebar way.

export const SIM = "sim";

// The ESP32-S3 can relay the lidar and the tilt servo over Wi-Fi; the core
// then takes tcp://host:8899 as the lidar port and tcp://host:8900 as the
// servo port, so the pickers accept a typed value as well as a detected port.
export const WIFI_HINT = "Over Wi-Fi: the ESP32 prints its IP on its USB serial; LiDAR on port 8899, tilt on 8900.";
export const isTcpPort = (p: string) => /^tcp:\/\//i.test(p.trim());

const PORT_KEYS = { lidar: "obslam.port.lidar", servo: "obslam.port.servo" };
function loadPort(key: string): string {
  try {
    return localStorage.getItem(key) ?? "";
  } catch {
    return "";
  }
}
function savePort(key: string, v: string): void {
  try {
    localStorage.setItem(key, v);
  } catch {
    // private window or storage blocked: the value just does not persist
  }
}

function PortField({
  id,
  value,
  onChange,
  disabled,
  placeholder,
  sim,
}: {
  id: string;
  value: string;
  onChange: (v: string) => void;
  disabled?: boolean;
  placeholder: string;
  sim?: boolean;
}) {
  const ports = useStore((s) => s.ports);
  const [busy, setBusy] = useState(false);
  return (
    <div className="sensor-row">
      <input
        className="mono"
        list={id}
        value={value}
        disabled={disabled}
        placeholder={placeholder}
        spellCheck={false}
        autoComplete="off"
        onChange={(e) => onChange(e.target.value)}
      />
      <datalist id={id}>
        {sim && <option value={SIM} label="no hardware" />}
        {ports.map((p) => (
          <option key={p.path} value={p.path} label={p.description} />
        ))}
      </datalist>
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

// Source, port, baud and the Connect button for the lidar. The port is a
// detected serial port or a typed tcp://host:8899 (Wi-Fi relay); the last
// value is remembered.
export function LidarConnect() {
  const ports = useStore((s) => s.ports);
  const [source, setSource] = useState<"s2" | "sim">("s2");
  const [port, setPortState] = useState(() => loadPort(PORT_KEYS.lidar));
  const [baud, setBaud] = useState(1000000);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const setPort = (v: string) => {
    setPortState(v);
    savePort(PORT_KEYS.lidar, v);
  };

  // pick the first detected port once the list arrives, unless something was typed or remembered
  useEffect(() => {
    if (!port && ports.length) setPortState(ports[0].path);
  }, [ports, port]);

  const tcp = isTcpPort(port);
  const connect = async () => {
    setErr(null);
    setBusy(true);
    try {
      await connectLidar({ source, port: source === "sim" ? SIM : port.trim(), baud });
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
          <option value="s2">s2 (RPLIDAR S3)</option>
          <option value="sim">sim (simulator)</option>
        </select>
        <input
          type="number"
          value={baud}
          disabled={source === "sim" || tcp}
          title={tcp ? "Baud does not apply over Wi-Fi" : "Baud"}
          style={{ width: 96 }}
          onChange={(e) => setBaud(Number(e.target.value))}
        />
      </div>
      <PortField
        id="lidar-ports"
        value={port}
        onChange={setPort}
        disabled={source === "sim"}
        placeholder="/dev/cu.usbserial-… or tcp://192.168.4.7:8899"
      />
      <div className="sensor-row">
        <button className="primary" disabled={busy || (source === "s2" && !port.trim())} onClick={() => void connect()}>
          Connect LiDAR
        </button>
      </div>
      {err && <div className="error">{err}</div>}
    </div>
  );
}

// Port and the Connect button for the ESP32 tilt servo: a detected serial
// port, "sim", or a typed tcp://host:8900 (Wi-Fi relay); remembered.
export function ServoConnect() {
  const [port, setPortState] = useState(() => loadPort(PORT_KEYS.servo) || SIM);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const setPort = (v: string) => {
    setPortState(v);
    savePort(PORT_KEYS.servo, v);
  };

  const connect = async () => {
    setErr(null);
    setBusy(true);
    try {
      await connectServo(port.trim());
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="sensor-connect">
      <PortField id="servo-ports" value={port} onChange={setPort} placeholder="sim, /dev/cu.usbmodem… or tcp://192.168.4.7:8900" sim />
      <div className="sensor-row">
        <button className="primary" disabled={busy || !port.trim()} onClick={() => void connect()}>
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
