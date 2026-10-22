import { useRef, useState } from "react";
import { api, errorText } from "../api/rest";
import { servoHome, servoMove } from "../store/actions";
import { useStore } from "../store/store";
import { fmt } from "../lib/format";

// The tilt slider lives in the Live scan tab's Sensor column, the raw
// calibration console in its Advanced drawer. ServoPanel composes both the
// old sidebar way.

const DRAG_SEND_MS = 100;

// One slider in servo degrees (the servo's own limits) that shows the tilt φ
// it produces, with Level (home) next to it.
export function TiltSlider() {
  const servo = useStore((s) => s.status?.servo);
  const mount = useStore((s) => s.config?.mount);

  const connected = !!servo?.connected;
  const min = servo?.min_deg ?? 0;
  const max = servo?.max_deg ?? servo?.range ?? 270;
  const servoDeg = servo?.servo_deg ?? null;
  const phiDeg = servo?.phi_deg ?? null;

  const [dragValue, setDragValue] = useState<number>(servoDeg ?? (min + max) / 2);
  const [dragging, setDragging] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const slider = dragging || servoDeg === null ? dragValue : servoDeg;
  const lastSend = useRef(0);
  const sendTimer = useRef<number | undefined>(undefined);
  const pendingDeg = useRef<number | null>(null);

  // φ for a servo angle, from the mount mapping, while dragging
  const phiFor = (deg: number) => (mount ? (mount.invert ? -1 : 1) * (deg - mount.center_deg) : NaN);
  const shownPhi = dragging ? phiFor(slider) : (phiDeg ?? phiFor(slider));

  const move = (deg: number) => servoMove({ servo_deg: deg }).catch((e) => setErr(errorText(e)));

  // throttle sends while dragging; always send the final value on release
  const sendDeg = (deg: number, final: boolean) => {
    const now = performance.now();
    if (final || now - lastSend.current >= DRAG_SEND_MS) {
      window.clearTimeout(sendTimer.current);
      sendTimer.current = undefined;
      pendingDeg.current = null;
      lastSend.current = now;
      void move(deg);
    } else {
      pendingDeg.current = deg;
      if (sendTimer.current === undefined) {
        sendTimer.current = window.setTimeout(() => {
          sendTimer.current = undefined;
          if (pendingDeg.current !== null) {
            lastSend.current = performance.now();
            void move(pendingDeg.current);
            pendingDeg.current = null;
          }
        }, DRAG_SEND_MS);
      }
    }
  };

  return (
    <div className="tilt">
      <div className="slider-row">
        <span className="dim">Tilt</span>
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
            setDragValue(v);
            if (dragging) sendDeg(v, false);
            else sendDeg(v, true); // keyboard / click on the track
          }}
        />
        <span className="val" title={`servo ${fmt(slider, 1)}°`}>
          φ {Number.isFinite(shownPhi) ? fmt(shownPhi, 1) : "–"}°
        </span>
        <button className="small" disabled={!connected} title="Scan plane horizontal (servo home)" onClick={() => servoHome().catch((e) => setErr(errorText(e)))}>
          Level
        </button>
      </div>
      {err && <div className="error">{err}</div>}
    </div>
  );
}

interface ConsoleLine {
  id: number;
  dir: "in" | "out";
  text: string;
}

// Raw line protocol console for calibration (?, T, L, C ...).
export function ServoConsole() {
  const servo = useStore((s) => s.status?.servo);
  const setStatus = useStore((s) => s.setStatus);
  const mount = useStore((s) => s.config?.mount);
  const connected = !!servo?.connected;
  const [rawLine, setRawLine] = useState("?");
  const [phiInput, setPhiInput] = useState("");
  const [err, setErr] = useState<string | null>(null);
  const [consoleLines, setConsoleLines] = useState<ConsoleLine[]>([]);
  const consoleId = useRef(0);

  const sendRaw = async () => {
    const line = rawLine.trim();
    if (!line) return;
    const push = (dir: "in" | "out", t: string) => setConsoleLines((ls) => [...ls.slice(-9), { id: ++consoleId.current, dir, text: t }]);
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

  const sendPhi = () => {
    const v = Number(phiInput);
    if (!Number.isFinite(v)) return;
    setErr(null);
    void servoMove({ phi_deg: v }).catch((e) => setErr(errorText(e)));
  };

  return (
    <section className="panel">
      <div className="panel-head">
        <span>Servo console</span>
        <span className="sub">
          {servo?.servo_deg != null ? `${fmt(servo.servo_deg, 1)}° servo` : ""}
          {servo?.phi_deg != null ? ` · φ ${fmt(servo.phi_deg, 1)}°` : ""}
        </span>
      </div>
      <div className="panel-body">
        <div className="field">
          <label>Tilt φ (deg)</label>
          <div className="row">
            <input
              type="number"
              step={0.5}
              placeholder={servo?.phi_deg != null ? fmt(servo.phi_deg, 1) : "0"}
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
          </div>
        </div>
        <div className="hint">Mapping: {mount ? `φ = ${mount.invert ? "−" : ""}(servo − ${mount.center_deg})` : "–"}</div>
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
        <div className="hint">Commands: ? status, I identify, T deg, S deg ms, H home, L min max, C us_min us_max range</div>
      </div>
    </section>
  );
}

export function ServoPanel() {
  return (
    <>
      <TiltSlider />
      <ServoConsole />
    </>
  );
}
