import { fmt, fmtInt } from "../lib/format";
import { useStore } from "../store/store";
import { LidarConnect, ServoConnect, WIFI_HINT } from "./ConnectionPanel";
import { TiltSlider } from "./ServoPanel";

// "Sensor": what the hardware is doing, in sentences; the connect controls
// when something is not connected; and the tilt slider.

export function healthWord(h: string | null | undefined): "good" | "warning" | "error" | string {
  if (!h) return "–";
  const s = h.toLowerCase();
  if (s === "good" || s === "ok") return "good";
  if (s.startsWith("warn")) return "warning";
  if (s.startsWith("err")) return "error";
  return s;
}

export function LiveSensor() {
  const status = useStore((s) => s.status);
  const lidar = status?.lidar;
  const servo = status?.servo;

  let lidarText: string;
  if (lidar?.connected) {
    const name = lidar.model ? (/^S\d/.test(lidar.model) ? `RPLIDAR ${lidar.model}` : lidar.model) : "LiDAR";
    const where = lidar.source === "sim" ? "simulator" : lidar.port ? `on ${lidar.port}` : "connected";
    const health = healthWord(lidar.health);
    const healthText = health === "good" ? "healthy" : health === "–" ? "health unknown" : `health ${health}`;
    lidarText = `${name} ${where}, ${healthText}, ${fmt(lidar.scan_hz, 1)} Hz, ${fmtInt(lidar.pts_per_rev)} points per scan.`;
  } else lidarText = "LiDAR not connected.";

  let servoText: string;
  if (servo?.connected) {
    const phi = servo.phi_deg ?? null;
    const level = phi !== null ? (Math.abs(phi) < 0.5 ? "level" : `tilt φ ${fmt(phi, 1)}°`) : "tilt unknown";
    const enc = servo.encoder && servo.encoder !== "none" ? `encoder ${servo.encoder}` : "no encoder";
    const settled = servo.settled === false ? "moving" : "settled";
    servoText = `Servo at ${fmt(servo.servo_deg, 0)}° (${level}), ${enc}, ${settled}.`;
  } else servoText = "Servo not connected.";

  return (
    <section className="panel ops-right">
      <div className="panel-head">
        <span>Sensor</span>
        <span className="sub">{status ? "" : "no status yet"}</span>
      </div>
      <div className="panel-body ops-summary">
        <div className="sensor-text">{lidarText}</div>
        {!lidar?.connected && <LidarConnect />}
        <div className="sensor-text">{servoText}</div>
        {!servo?.connected && <ServoConnect />}
        {(!lidar?.connected || !servo?.connected) && <div className="hint">{WIFI_HINT}</div>}
        <div className="ops-ask-wrap">
          <TiltSlider />
        </div>
      </div>
    </section>
  );
}
