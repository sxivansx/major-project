import { ws } from "../api/ws";
import { useStore } from "../store/store";
import { useThrottled, useTick } from "../store/hooks";
import { fmt, fmtInt } from "../lib/format";

export function StatusBar() {
  const wsState = useStore((s) => s.wsState);
  const wsAttempt = useStore((s) => s.wsAttempt);
  const wsRetryAt = useStore((s) => s.wsRetryAt);
  const status = useStore((s) => s.status);
  const cloudCount = useStore((s) => s.cloudCount);
  const frames = useThrottled((s) => s.wsFrames, 1000);
  const lastLog = useStore((s) => s.log[s.log.length - 1]);
  useTick(1000);

  let wsText: string;
  let wsClass: string;
  if (wsState === "open") {
    wsText = "ws connected";
    wsClass = "ok";
  } else if (wsState === "connecting") {
    wsText = wsAttempt > 0 ? `ws connecting (attempt ${wsAttempt + 1})` : "ws connecting";
    wsClass = "warn";
  } else {
    const inS = wsRetryAt ? Math.max(0, Math.ceil((wsRetryAt - performance.now()) / 1000)) : null;
    wsText = inS !== null ? `ws disconnected, retry in ${inS}s` : "ws disconnected";
    wsClass = "err";
  }

  const lidar = status?.lidar;
  const servo = status?.servo;
  const job = status?.job;

  return (
    <footer className="statusbar">
      <span className={wsClass}>{wsText}</span>
      {wsState !== "open" && (
        <button onClick={() => ws.reconnectNow()}>Reconnect now</button>
      )}
      <span>
        <span className="dim">lidar </span>
        {lidar?.connected ? `${lidar.source ?? "?"} ${fmt(lidar.scan_hz, 1)} Hz` : "off"}
      </span>
      <span>
        <span className="dim">servo </span>
        {servo?.connected ? `${fmt(servo.servo_deg, 1)}° (φ ${fmt(servo.phi_deg, 1)}°)` : "off"}
      </span>
      <span>
        <span className="dim">job </span>
        {job ? `${job.state} ${job.step}/${job.n_steps}` : "idle"}
      </span>
      <span>
        <span className="dim">cloud </span>
        {fmtInt(cloudCount)} pts
      </span>
      <span className="dim">{fmtInt(frames)} scan frames</span>
      {lastLog && (
        <span
          className={lastLog.level === "error" ? "err" : lastLog.level === "warn" ? "warn" : "dim"}
          style={{ overflow: "hidden", textOverflow: "ellipsis", minWidth: 0 }}
          title={lastLog.msg}
        >
          {lastLog.msg}
        </span>
      )}
    </footer>
  );
}
