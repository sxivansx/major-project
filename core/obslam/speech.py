"""Spoken calls for help: cut speech out of the mic stream, transcribe it (Sarvam saaras), decide
whether it is a call for rescue (keyword list, then Jev), and give it a bearing from the same
four mics (docs/api.md §5.7).

Segmenting is an energy gate on the pre-emphasised mean of the used mics against a running floor,
so knocks and claps (< 150 ms of signal) are never sent. Transcription and Jev run on one worker
thread; the audio thread only appends 10 ms blocks. Speech goes to Sarvam's cloud API only while
the listener is enabled (default: on when SARVAM_API_KEY is set).
"""

from __future__ import annotations

import io
import json
import logging
import queue
import re
import threading
import time
import wave
from collections import deque
from typing import Callable

import httpx
import numpy as np

from .ai import AIClient, AIError, load_env

log = logging.getLogger(__name__)

SARVAM_STT = "https://api.sarvam.ai/speech-to-text"
STT_MODEL = "saaras:v4"
BLOCK_S = 0.01
START_DB = 9.0  # block energy this far above the floor opens a segment…
STAY_DB = 5.0  # …and it stays open while blocks are this far above it
START_BLOCKS = 4  # consecutive loud blocks before a segment opens (40 ms)
PRE_ROLL_S = 0.25
HANG_S = 0.45  # quiet this long closes the segment
MIN_VOICED_S = 0.18  # less than this above STAY_DB is a knock, not speech
MAX_SEGMENT_S = 6.0
DOA_WINDOW_S = 1.0  # loudest second of the segment goes to GCC-PHAT
MAX_CALLS_PER_MIN = 12
FLOOR_WINDOW_S = 3.0  # noise floor = a low percentile of block energy over this window…
FLOOR_PCT = 20  # …so a steady fan or motor raises it within seconds and pauses in speech keep it honest
FLAG_HOLD_S = 30.0  # the alarm stays up this long after a call for help
DISTRESS_P = 0.6

# Sarvam biases recognition toward these (saaras:v4 keyterms, ≤ 50)
KEYTERMS = ["save me", "help", "help me", "bachao", "बचाओ", "मुझे बचाओ", "ಕಾಪಾಡಿ", "ನನ್ನನ್ನು ಕಾಪಾಡಿ", "ಸಹಾಯ"]
# deterministic match, before and regardless of Jev
PATTERNS = [
    ("save me", re.compile(r"\bsave\s+(me|us)\b", re.I)),
    ("help", re.compile(r"\bhelp\b", re.I)),
    ("bachao", re.compile(r"\bbacha+o\b|बचाओ|बचा\s*लो", re.I)),
    ("madad", re.compile(r"\bmadad\b|मदद", re.I)),
    ("kaapadi", re.compile(r"\bka+pa+di\b|ಕಾಪಾಡಿ", re.I)),
    ("sahaya", re.compile(r"\bsaha+ya\b|ಸಹಾಯ|सहायता", re.I)),
]

DISTRESS_CONTEXT = (
    "A microphone array on a search-and-rescue scanner heard speech in or near a collapsed structure. "
    "The text is an automatic transcript and may be in any Indian language or English, possibly mis-heard."
)


def match_keywords(text: str) -> list[str]:
    return [name for name, rx in PATTERNS if rx.search(text or "")]


# ---- sound-alikes --------------------------------------------------------------------------------
# STT mishears short shouted phrases ("safe me", "shave me", "say me", "bachaav"). Each target and
# each 1–3 word span of the transcript is reduced to a rough phonetic key, then compared by edit
# distance. Latin script only; the native-script forms are covered by PATTERNS.
SOUNDALIKE_TARGETS = ["save me", "save us", "help me", "bachao", "bachao mujhe", "kaapadi", "madad", "sahaya"]
_PHON = [("ph", "f"), ("sh", "s"), ("ck", "k"), ("kh", "k"), ("gh", "g"), ("bh", "b"), ("ch", "c"), ("th", "t"), ("dh", "d"),
         ("f", "v"), ("w", "v"), ("z", "s"), ("q", "k"), ("x", "ks"), ("y", "i"), ("ee", "i"), ("oo", "u"), ("ou", "u"), ("ay", "e"), ("ai", "e")]


def phonetic(s: str) -> str:
    s = re.sub(r"[^a-z]", "", s.lower())
    for a, b in _PHON:
        s = s.replace(a, b)
    return re.sub(r"(.)\1+", r"\1", s)  # doubled letters: "aa", "ll"


def _edit(a: str, b: str) -> int:
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def _allowed(n: int) -> int:
    return 0 if n <= 4 else (1 if n <= 7 else 2)


_TARGET_KEYS = [(t, phonetic(t), len(t.split())) for t in SOUNDALIKE_TARGETS]
SOUNDALIKE_STOP = {"madam", "safe", "saved", "same"}  # common words one edit away from a target


def sound_alikes(text: str, exact: list[str]) -> list[dict]:
    """[{"word": target, "heard": span}] for spans that sound like a target but did not match exactly."""
    words = re.findall(r"[A-Za-z']+", text or "")
    # an exact hit covers its own family: "save me" covers "save us", "help" covers "help me"
    seen = {t for t in SOUNDALIKE_TARGETS if any(t.split()[0] == e.split()[0] for e in exact)}
    out = []
    for n in (1, 2, 3):
        for i in range(len(words) - n + 1):
            span = " ".join(words[i : i + n])
            k = phonetic(span)
            if len(k) < 3 or span.lower() in SOUNDALIKE_STOP:
                continue
            for target, tk, nw in _TARGET_KEYS:
                if target in seen or n != nw:
                    continue
                # the first sound must agree: "have me"/"gave me" are not "save me"
                if k[0] == tk[0] and abs(len(k) - len(tk)) <= 2 and _edit(k, tk) <= _allowed(len(tk)):
                    out.append({"word": target, "heard": span})
                    seen.add(target)
    return out


def wav_bytes(mono: np.ndarray, rate: int) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(np.asarray(mono, np.int16).tobytes())
    return buf.getvalue()


class Segmenter:
    """Energy gate over 10 ms blocks. `feed` returns finished segments as (samples (L, ch) int16, t_start)."""

    def __init__(self, rate: int, channels: int):
        self.rate, self.channels = rate, channels
        self.block = max(1, int(rate * BLOCK_S))
        self._carry = np.zeros((0, channels), np.int16)
        self._pre: deque[np.ndarray] = deque(maxlen=int(PRE_ROLL_S / BLOCK_S))
        self._seg: list[np.ndarray] | None = None
        self._floor: float | None = None
        self._hist: deque[float] = deque(maxlen=int(FLOOR_WINDOW_S / BLOCK_S))
        self._n = 0
        self._loud = 0
        self._quiet = 0
        self._voiced = 0
        self._t0 = 0.0
        self._t = 0.0

    def reset(self) -> None:
        """A gap in the stream: drop the open segment, keep the floor."""
        self._carry = self._carry[:0]
        self._pre.clear()
        self._seg = None
        self._loud = self._quiet = self._voiced = 0

    def _energy(self, b: np.ndarray, use: list[int]) -> float:
        m = b[:, use].astype(np.float64).mean(axis=1)
        d = m[1:] - 0.95 * m[:-1]  # pre-emphasis: room rumble and slow drift do not open the gate
        return float(np.mean(d * d)) + 1e-3

    def feed(self, x: np.ndarray, t: float, use: list[int]) -> list[tuple[np.ndarray, float]]:
        out = []
        data = np.concatenate([self._carry, x])
        n = data.shape[0] // self.block
        self._carry = data[n * self.block :]
        t_first = t - (data.shape[0] - x.shape[0]) / self.rate  # `t` is the time of x's first sample
        for k in range(n):
            b = data[k * self.block : (k + 1) * self.block]
            bt = t_first + k * BLOCK_S
            e = self._energy(b, use)
            self._hist.append(e)
            self._n += 1
            if self._floor is None or self._n % 10 == 0:
                self._floor = float(np.percentile(self._hist, FLOOR_PCT))
            ratio_db = 10.0 * np.log10(e / self._floor)
            if self._n < self._hist.maxlen // 2:
                continue  # half a window to learn the floor first
            if self._seg is None:
                self._loud = self._loud + 1 if ratio_db > START_DB else 0
                self._pre.append(b)
                if self._loud >= START_BLOCKS:
                    self._seg = list(self._pre)
                    self._t0 = bt - (len(self._pre) - 1) * BLOCK_S
                    self._voiced, self._quiet = self._loud, 0
                    self._pre.clear()
                continue
            self._seg.append(b)
            if ratio_db > STAY_DB:
                self._voiced += 1
                self._quiet = 0
            else:
                self._quiet += 1
            long = len(self._seg) * BLOCK_S >= MAX_SEGMENT_S
            if self._quiet * BLOCK_S >= HANG_S or long:
                if self._voiced * BLOCK_S >= MIN_VOICED_S:
                    out.append((np.concatenate(self._seg), self._t0))
                self._seg = None
                self._loud = self._quiet = self._voiced = 0
        return out


def loudest(seg: np.ndarray, rate: int, use: list[int], span_s: float = DOA_WINDOW_S) -> np.ndarray:
    L = seg.shape[0]
    w = int(span_s * rate)
    if L <= w:
        return seg
    e = seg[:, use].astype(np.float64).mean(axis=1) ** 2
    c = np.concatenate([[0.0], np.cumsum(e)])
    i = int(np.argmax(c[w:] - c[:-w]))
    return seg[i : i + w]


class SpeechListener:
    """Owned by the audio pipeline. `locate(window, rate)` is the pipeline's DOA with its current
    geometry and excluded mics; `send` publishes JSON; `log_fn(level, text)` writes the rig log."""

    def __init__(
        self,
        ai: AIClient | None,
        send: Callable[[dict], None],
        log_fn: Callable[[str, str], None],
        locate: Callable[[np.ndarray, int], dict],
        key: str | None = None,
        transport: httpx.BaseTransport | None = None,
        enabled: bool | None = None,
    ):
        self.ai = ai
        self._send = send
        self._log = log_fn
        self._locate = locate
        self._explicit_key = key is not None
        self.key = key if key is not None else load_env("SARVAM_API_KEY")
        self._http = httpx.Client(timeout=15.0, transport=transport)
        self._user_enabled = enabled
        self._seg: Segmenter | None = None
        self._q: queue.Queue = queue.Queue(maxsize=4)
        self._lock = threading.Lock()
        self._events: deque[dict] = deque(maxlen=100)
        self._calls: deque[float] = deque()
        self._id = 0
        self.stats = {"segments": 0, "transcribed": 0, "dropped": 0, "rate_limited": 0, "errors": 0}
        self.last_error: str | None = None
        self._flag: dict | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker, name="speech", daemon=True)
        self._thread.start()

    # ---- state -----------------------------------------------------------------------------------
    @property
    def has_key(self) -> bool:
        if not self.key and not self._explicit_key:
            self.key = load_env("SARVAM_API_KEY")  # pasted into core/.env after startup
        return bool(self.key)

    @property
    def enabled(self) -> bool:
        return self.has_key and (self._user_enabled is not False)

    def set_enabled(self, on: bool) -> None:
        self._user_enabled = bool(on)
        if not on:
            self.reset()

    def reset(self) -> None:
        if self._seg:
            self._seg.reset()

    def close(self) -> None:
        self._stop.set()

    # ---- audio thread ----------------------------------------------------------------------------
    def feed(self, x: np.ndarray, t: float, rate: int, use: list[int]) -> None:
        if not self.enabled or not use:
            return
        if self._seg is None or self._seg.rate != rate or self._seg.channels != x.shape[1]:
            self._seg = Segmenter(rate, x.shape[1])
        for seg, t0 in self._seg.feed(x, t, use):
            self.stats["segments"] += 1
            try:
                self._q.put_nowait((seg, t0, rate, list(use)))
            except queue.Full:
                self.stats["dropped"] += 1

    # ---- worker ----------------------------------------------------------------------------------
    def _worker(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.process(*item)
            except Exception as e:  # noqa: BLE001  (one bad segment must not stop the listener)
                self.stats["errors"] += 1
                self.last_error = f"{type(e).__name__}: {e}"
                log.exception("speech segment failed")

    def _rate_ok(self) -> bool:
        now = time.monotonic()
        while self._calls and now - self._calls[0] > 60.0:
            self._calls.popleft()
        if len(self._calls) >= MAX_CALLS_PER_MIN:
            return False
        self._calls.append(now)
        return True

    def transcribe(self, mono: np.ndarray, rate: int) -> dict:
        r = self._http.post(
            SARVAM_STT,
            headers={"api-subscription-key": self.key or ""},
            files={"file": ("speech.wav", wav_bytes(mono, rate), "audio/wav")},
            data={"model": STT_MODEL, "language_code": "unknown", "keyterms": json.dumps(KEYTERMS)},
        )
        if r.status_code >= 400:
            raise AIError(f"Sarvam {r.status_code}: {r.text[:200]}")
        return r.json()

    def judge(self, text: str, language: str | None) -> tuple[float | None, float]:
        """Jev: probability that the transcript is a call for rescue. (None, 0) without a key."""
        if self.ai is None or not self.ai.decisions_available:
            return None, 0.0
        answers, cost = self.ai.decide(
            {"context": DISTRESS_CONTEXT, "transcript": text, "language": language},
            {
                "distress": {
                    "type": "noul",
                    "instructions": "Is the speaker calling for help or asking to be saved or rescued?",
                    "criteria": {
                        "true": "A plea for help or rescue in any language: 'save me', 'help', 'bachao', 'kaapadi', 'I am trapped', "
                        "'get me out', or a likely mis-transcription of one ('safe me', 'shave me', 'say me', 'bachaav').",
                        "false": "Ordinary talk, a question, a test phrase read out calmly, music or noise.",
                    },
                }
            },
        )
        return float((answers.get("distress") or {}).get("noul", 0.0)), cost

    def process(self, seg: np.ndarray, t0: float, rate: int, use: list[int]) -> dict | None:
        if not self._rate_ok():
            self.stats["rate_limited"] += 1
            return None
        dur = seg.shape[0] / rate
        loc = self._locate(loudest(seg, rate, use), rate)
        mono = seg[:, use].astype(np.float64).mean(axis=1)
        peak = float(np.max(np.abs(mono))) or 1.0
        mono = np.clip(mono * min(8.0, 16000.0 / peak), -32768, 32767)  # quiet far voices: normalise for STT
        t1 = time.perf_counter()
        stt = self.transcribe(mono, rate)
        stt_ms = (time.perf_counter() - t1) * 1000
        self.stats["transcribed"] += 1
        text = (stt.get("transcript") or "").strip()
        lang = stt.get("language_code")
        matched = match_keywords(text)
        alike = sound_alikes(text, matched)
        p, cost, jev_ms, jev_error = None, 0.0, None, None
        if text:
            t2 = time.perf_counter()
            try:
                p, cost = self.judge(text, lang)
            except AIError as e:
                jev_error = str(e)
            jev_ms = (time.perf_counter() - t2) * 1000
        flagged = bool(matched or alike) or (p is not None and p >= DISTRESS_P)
        with self._lock:
            self._id += 1
            ev = {
                "id": self._id,
                "t": round(t0, 3),
                "duration_s": round(dur, 2),
                "transcript": text,
                "language": lang,
                "matched": matched,
                "sounds_like": alike,
                "distress_p": None if p is None else round(p, 3),
                "flagged": flagged,
                "bearing_deg": loc.get("bearing_deg"),
                "bearing_sigma_deg": loc.get("bearing_sigma_deg"),
                "confidence": loc.get("confidence"),
                "calibrated": loc.get("calibrated"),
                "reasons": loc.get("reasons", []),
                "stt_ms": round(stt_ms),
                "jev_ms": None if jev_ms is None else round(jev_ms),
                "jev_error": jev_error,
                "cost_usd": round(cost, 6),
            }
            self._events.append(ev)
            if flagged:
                self._flag = {**ev, "until": time.time() + FLAG_HOLD_S}
        self._send({"type": "speech_event", **ev})
        if flagged:
            self._log("warn", f"call for help heard: \"{text}\" {self.where(ev)}")
        elif text:
            self._log("info", f"speech heard: \"{text}\" {self.where(ev)}")
        return ev

    @staticmethod
    def where(ev: dict) -> str:
        if ev.get("bearing_deg") is None:
            return "(no direction)"
        s = f"from {ev['bearing_deg']:.0f}° ± {ev['bearing_sigma_deg'] or 0:.0f}°"
        if not ev.get("calibrated", True):
            s += " (array uncalibrated)"
        return s

    # ---- read side -------------------------------------------------------------------------------
    def events(self, limit: int = 50) -> list[dict]:
        with self._lock:
            return list(reversed(self._events))[: max(0, limit)]

    def clear(self) -> None:
        with self._lock:
            self._events.clear()
            self._flag = None

    def alarm(self) -> str | None:
        f = self._flag
        if f is None or time.time() > f["until"]:
            return None
        return f"call for help: \"{f['transcript']}\" {self.where(f)}"

    def status(self) -> dict:
        return {
            "enabled": self.enabled,
            "key": self.has_key,
            "model": STT_MODEL,
            "judge": None if self.ai is None else self.ai.decisions_model,
            **self.stats,
            "queued": self._q.qsize(),
            "last_error": self.last_error,
        }
