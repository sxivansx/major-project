import { useRef, useState } from "react";
import { api, errorText } from "../api/rest";
import { clientLog, useStore } from "../store/store";
import { fmt } from "../lib/format";

const DRAG_SEND_MS = 100;

interface ConsoleLine {
  id: number;
  dir: "in" | "out";
  text: string;
}

export function ServoPanel() {
  const servo = useStore((s) => s.status?.servo);
  const config = useStore((s) => s.config);
  const setStatus = useStore((s) => s.setStatus);

  const connected = !!servo?.connected;
  const min = servo?.min_deg ?? 0;
  const max = servo?.max_deg ?? servo?.range ?? 270;
  const servoDeg = servo?.servo_deg ?? null;
  const phiDeg = servo?.phi_deg ?? null;

  const [dragValue, setDragValue] = useState<number>(servoDeg ?? (min + max) / 2);
  const [dragging, setDragging] = useState(false);
  // while dragging show the drag value, otherwise follow the real servo
  const slider = dragging || servoDeg === null ? dragValue : servoDeg;
  const setSlider = setDragValue;
  const [phiInput, setPhiInput] = useState("");
  const [err, setErr] = useState<string | null>(null);
  const [rawLine, setRawLine] = useState("?");
  const [consoleLines, setConsoleLines] = useState<ConsoleLine[]>([]);
  const consoleId = useRef(0);
  const lastSend = useRef(0);
  const sendTimer = useRef<number | undefined>(undefined);
  const pendingDeg = useRef<number | null>(null);

  const move = async (body: { servo_deg: number } | { phi_deg: number }) => {
    setErr(null);
    try {
      setStatus(await api.servoMove(body));
    } catch (e) {
      setErr(errorText(e));
      clientLog("error", `servo/move: ${errorText(e)}`);
    }
  };

  // throttle sends while dragging; always send the final value on release
  const sendDeg = (deg: number, final: boolean) => {
    const now = performance.now();
    if (final || now - lastSend.current >= DRAG_SEND_MS) {
      window.clearTimeout(sendTimer.current);
      sendTimer.current = undefined;
      pendingDeg.current = null;
      lastSend.current = now;
      void move({ servo_deg: deg });
    } else {
      pendingDeg.current = deg;
      if (sendTimer.current === undefined) {
        sendTimer.current = window.setTimeout(() => {
          sendTimer.current = undefined;
          if (pendingDeg.current !== null) {
            lastSend.current = performance.now();
            void move({ servo_deg: pendingDeg.current });
            pendingDeg.current = null;
          }
        }, DRAG_SEND_MS);
      }
    }
  };

  const home = async () => {
    setErr(null);
    try {
      setStatus(await api.servoHome());
    } catch (e) {
      setErr(errorText(e));
    }
  };

  const sendPhi = () => {
    const v = Number(phiInput);
    if (!Number.isFinite(v)) return;
    void move({ phi_deg: v });
  };

  const sendRaw = async () => {
    const line = rawLine.trim();
    if (!line) return;
    const push = (dir: "in" | "out", t: string) =>
      setConsoleLines((ls) => [...ls.slice(-9), { id: ++consoleId.current, dir, text: t }]);
    push("in", line);
    try {
      const r = await api.servoRaw(line);
      push("out", r.reply);
      // calibration commands change limits/position; refresh the status
      if (/^[?CLHTS]/i.test(line)) {
        try {
          setStatus(await api.status());
        } catch {
          // status refresh is best effort
        }
      }
    } catch (e) {
      push("out", `error: ${errorText(e)}`);
    }
  };

  const mount = config?.mount;

  return (
    <section className="panel">
      <div className="panel-head">
        <span>Servo</span>
        <span className="sub">
          {servoDeg !== null ? `${fmt(servoDeg, 1)}° servo` : ""}
          {phiDeg !== null ? ` · φ ${fmt(phiDeg, 1)}°` : ""}
        </span>
      </div>
      <div className="panel-body">
        <div className="field">
          <label>Servo deg</label>
          <div className="slider-row">
            <input
              type="range"
              min={min}
              max={max}
              step={0.5}
              value={slider}
              disabled={!connected}
              onPointerDown={() => {
                setDragValue(slider);
                setDragging(true);
              }}
              onPointerUp={() => {
                setDragging(false);
                sendDeg(dragValue, true);
              }}
              onKeyUp={() => sendDeg(slider, true)}
              onChange={(e) => {
                const v = Number(e.target.value);
                setSlider(v);
                if (dragging) sendDeg(v, false);
                else sendDeg(v, true); // keyboard / click on the track
              }}
            />
            <span className="val">{fmt(slider, 1)}°</span>
          </div>
        </div>
        <div className="field">
          <label>Tilt φ (deg)</label>
          <div className="row">
            <input
              type="number"
              step={0.5}
              placeholder={phiDeg !== null ? fmt(phiDeg, 1) : "0"}
              value={phiInput}
              disabled={!connected}
              onChange={(e) => setPhiInput(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === "Enter") sendPhi();
              }}
            />
            <button disabled={!connected || phiInput === ""} onClick={sendPhi}>
              Go
            </button>
            <button disabled={!connected} onClick={home} title="H: go to range/2">
              Home
            </button>
          </div>
        </div>
        <dl className="kv">
          <dt>servo_deg</dt>
          <dd>{fmt(servoDeg, 1, "°")}</dd>
          <dt>phi_deg</dt>
          <dd>{fmt(phiDeg, 1, "°")}</dd>
          <dt>encoder</dt>
          <dd>
            {servo?.encoder ?? "–"}
            {servo?.meas_deg != null ? ` · ${fmt(servo.meas_deg, 1)}° (φ ${fmt(servo.phi_meas_deg, 1)}°)` : ""}
            {servo?.settled === false ? " · moving" : ""}
          </dd>
          <dt>Mapping</dt>
          <dd>
            {mount
              ? `φ = ${mount.invert ? "−" : ""}(servo − ${mount.center_deg})`
              : "–"}
          </dd>
        </dl>
        {err && <div className="error">{err}</div>}

        <div className="section-title">Raw console (calibration)</div>
        <div className="row" style={{ display: "flex", gap: 4 }}>
          <input
            className="mono"
            style={{ flex: 1 }}
            value={rawLine}
            disabled={!connected}
            spellCheck={false}
            onChange={(e) => setRawLine(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Enter") void sendRaw();
            }}
            placeholder="? | T 135 | L 20 250 | C 500 2500 270"
          />
          <button disabled={!connected || !rawLine.trim()} onClick={() => void sendRaw()}>
            Send
          </button>
        </div>
        {consoleLines.length > 0 && (
          <div className="console-out">
            {consoleLines.map((l) => (
              <div key={l.id} className={l.dir}>
                {l.dir === "in" ? "> " : "< "}
                {l.text}
              </div>
            ))}
          </div>
        )}
        <div className="hint">
          Commands: ? status, I identify, T deg, S deg ms, H home, L min max, C us_min us_max range
        </div>
      </div>
    </section>
  );
}
