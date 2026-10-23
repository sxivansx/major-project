import time

import httpx
import numpy as np

from obslam.speech import Segmenter, SpeechListener, match_keywords, sound_alikes

RATE = 16000


def noise(seconds, level, rng):
    return (rng.standard_normal((int(seconds * RATE), 4)) * level).astype(np.int16)


def feed_all(seg, x, block=160):
    out = []
    for i in range(0, x.shape[0], block):
        out += seg.feed(x[i : i + block], i / RATE, [0, 1, 2, 3])
    return out


def test_segmenter_keeps_speech_and_drops_knocks():
    rng = np.random.default_rng(1)
    quiet = noise(1.0, 30, rng)
    talk = noise(0.8, 2000, rng)
    knock = noise(0.05, 8000, rng)
    seg = Segmenter(RATE, 4)
    found = feed_all(seg, np.concatenate([quiet, knock, quiet, talk, quiet]))
    assert len(found) == 1
    samples, t0 = found[0]
    assert 0.8 <= samples.shape[0] / RATE <= 1.6
    assert 1.75 < t0 < 2.05  # the burst at 2.05 s less the pre-roll, not the knock at 1.0 s


def test_keywords_and_sound_alikes():
    assert match_keywords("Please save me!") == ["save me"]
    assert match_keywords("मुझे बचाओ") == ["bachao"]
    assert [a["word"] for a in sound_alikes("safe me", [])] == ["save me"]
    assert [a["word"] for a in sound_alikes("bachaav", [])] == ["bachao"]
    for calm in ("I am safe", "have me over", "madam", "is it safe to come"):
        assert sound_alikes(calm, []) == [], calm


def listener(transcript, sent, logs):
    def stt(req: httpx.Request):
        assert req.headers["api-subscription-key"] == "k"
        assert b"saaras:v4" in req.content
        return httpx.Response(200, json={"transcript": transcript, "language_code": "en-IN"})

    loc = lambda w, r: {"bearing_deg": 197.0, "bearing_sigma_deg": 6.0, "confidence": 0.5, "calibrated": False, "reasons": []}  # noqa: E731
    return SpeechListener(None, sent.append, lambda lv, m: logs.append((lv, m)), loc, key="k", transport=httpx.MockTransport(stt))


def test_flags_a_sound_alike_with_direction():
    sent, logs = [], []
    sp = listener("safe me please", sent, logs)
    ev = sp.process(noise(0.8, 2000, np.random.default_rng(2)), 3.0, RATE, [0, 1, 2, 3])
    assert ev["flagged"] and ev["matched"] == [] and ev["sounds_like"][0]["word"] == "save me"
    assert ev["bearing_deg"] == 197.0 and sent[0]["type"] == "speech_event"
    assert logs[0][0] == "warn" and "197°" in logs[0][1] and "uncalibrated" in logs[0][1]
    assert "safe me" in sp.alarm()
    sp.close()


def test_ordinary_speech_is_not_flagged():
    sent, logs = [], []
    sp = listener("the scan is done", sent, logs)
    ev = sp.process(noise(0.8, 2000, np.random.default_rng(3)), 3.0, RATE, [0, 1, 2, 3])
    assert not ev["flagged"] and sp.alarm() is None and logs[0][0] == "info"
    sp.close()


def test_stream_to_event_through_the_worker():
    sent, logs = [], []
    sp = listener("save me", sent, logs)
    rng = np.random.default_rng(4)
    x = np.concatenate([noise(1.0, 30, rng), noise(0.7, 2000, rng), noise(0.8, 30, rng)])
    for i in range(0, x.shape[0], 160):
        sp.feed(x[i : i + 160], i / RATE, RATE, [0, 1, 2, 3])
    deadline = time.time() + 3
    while not sent and time.time() < deadline:
        time.sleep(0.02)
    assert sent and sent[0]["matched"] == ["save me"]
    assert sp.status()["segments"] == 1 and sp.status()["transcribed"] == 1
    sp.close()


def test_disabled_without_a_key():
    sp = SpeechListener(None, lambda m: None, lambda a, b: None, lambda w, r: {}, key="")
    assert not sp.enabled
    sp.feed(np.zeros((160, 4), np.int16), 0.01, RATE, [0, 1, 2, 3])
    assert sp.status()["segments"] == 0
    sp.close()
