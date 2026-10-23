import { useEffect, useState } from "react";
import { errorText } from "../api/rest";
import type { SpeechEvent } from "../api/types";
import { fmt } from "../lib/format";
import { agoText, clearSpeech, loadSpeech, setSpeechEnabled, speechBearingText, speechWhy, useAudio, useCoreNow } from "../store/audio";
import { useStore } from "../store/store";

// "Heard": spoken segments transcribed by the core (§5.7), newest first. A
// flagged one is a call for help: exact help words, a sound-alike, or a high
// distress probability from Jev. The direction is from the same array.

function SpeechRow({ ev, now }: { ev: SpeechEvent; now: number }) {
  return (
    <div className={"speech-row" + (ev.flagged ? " flagged" : "")}>
      <div className="speech-head">
        <span className="speech-quote">{ev.transcript ? `"${ev.transcript}"` : "(no words recognised)"}</span>
        <span className="spacer" />
        <span className="mono dim">{agoText(now - ev.t)}</span>
      </div>
      {ev.flagged && <div className="speech-help">call for help: {speechWhy(ev)}</div>}
      <div className="speech-meta">
        {ev.language ?? "language unknown"}
        {" · "}
        {speechBearingText(ev)}
        {!ev.calibrated && ev.bearing_deg !== null ? " (uncalibrated)" : ""}
        {ev.distress_p !== null && ev.distress_p !== undefined ? ` · Jev p ${fmt(ev.distress_p, 2)}` : ev.jev_error ? " · Jev unavailable" : ""}
        {` · ${fmt(ev.duration_s, 1)} s`}
      </div>
    </div>
  );
}

export function SpeechPanel() {
  const st = useStore((s) => s.status?.audio?.speech ?? null);
  const speech = useAudio((s) => s.speech);
  const speechError = useAudio((s) => s.speechError);
  const now = useCoreNow(speech.length > 0, 5000);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const supported = st !== null;

  useEffect(() => {
    if (supported) void loadSpeech();
  }, [supported]);

  if (!supported) return null;

  const toggle = async (on: boolean) => {
    setErr(null);
    setBusy(true);
    try {
      await setSpeechEnabled(on);
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };

  const shown = speech.slice(0, 8);
  return (
    <div className="speech-panel">
      <div className="detections-head">
        <span className="ops-block-title" style={{ margin: 0 }}>
          Heard
        </span>
        <span className="spacer" />
        {speech.length > 0 && (
          <button className="linklike" style={{ marginTop: 0 }} onClick={() => void clearSpeech().catch((e) => setErr(errorText(e)))}>
            Clear
          </button>
        )}
      </div>
      <label className="inline-check" title={st.key ? `speech-to-text: ${st.model ?? "Sarvam"}` : undefined}>
        <input type="checkbox" checked={!!st.enabled} disabled={busy || !st.key} onChange={(e) => void toggle(e.target.checked)} />
        Listen for speech (sends speech to Sarvam)
      </label>
      {!st.key && <div className="hint">no SARVAM_API_KEY: speech is not transcribed.</div>}
      {st.key && st.last_error && <div className="hint err">{st.last_error}</div>}
      {(err || speechError) && <div className="error">{err ?? speechError}</div>}
      {shown.length === 0 ? (
        <div className="hint">{st.enabled ? "Nothing said yet." : "Speech is off."}</div>
      ) : (
        shown.map((ev) => <SpeechRow key={ev.id} ev={ev} now={now} />)
      )}
      {speech.length > shown.length && <div className="hint">and {speech.length - shown.length} earlier</div>}
    </div>
  );
}
