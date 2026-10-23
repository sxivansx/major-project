"""Acoustic layer through the server (docs/api.md §5.3, §5.4) and the rig's alarms."""

import json
import time

import numpy as np
import pytest
from fastapi.testclient import TestClient

from obslam.audio_wire import KIND_AUDIO_PREVIEW, AudioFrame, decode_preview
from obslam.config import Config
from obslam.rig import JobParams, Rig
from obslam.server import create_app
from obslam.session import SessionStore


@pytest.fixture
def client(tmp_path):
    app = create_app(config_path=tmp_path / "config.json", sessions_dir=tmp_path / "sessions", web_dist=tmp_path / "nope", sim_hz=60)
    with TestClient(app) as c:
        c.tmp = tmp_path
        yield c


def angle_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def test_status_and_config_before_connect(client):
    assert client.get("/api/status").json()["audio"] == {"connected": False}
    a = client.get("/api/config").json()["audio"]
    assert a["calibrated"] is False and len(a["mics_mm"]) == 4 and a["trigger_db"] == 12
    assert a["mics_mm"][1] == [-70.0, -19.5, 0.0]
    assert client.post("/api/audio/connect", json={"source": "usb"}).status_code == 422
    assert client.post("/api/audio/sim/emit", json={"bearing_deg": 10}).status_code == 409
    # an old client PUTting a config without the audio block must not reset it
    cfg = client.get("/api/config").json()
    cfg["audio"]["trigger_db"] = 14
    client.put("/api/config", json=cfg)
    cfg.pop("audio")
    client.put("/api/config", json=cfg)
    assert client.get("/api/config").json()["audio"]["trigger_db"] == 14


def test_sim_connect_emit_ws_and_rest(client):
    s = client.post("/api/audio/connect", json={"source": "sim", "port": "sim"}).json()["audio"]
    assert s["connected"] and s["source"] == "sim" and s["calibrated"] is False and "uncalibrated" in s["note"]
    with client.websocket_connect("/ws") as ws:
        assert json.loads(ws.receive_text())["type"] == "status"
        previews, events = [], []
        t0 = time.time()
        while len(previews) < 5 and time.time() - t0 < 5:
            m = ws.receive()
            if m.get("bytes") and m["bytes"][0] == KIND_AUDIO_PREVIEW:
                previews.append(decode_preview(m["bytes"]))
        assert len(previews) == 5
        p = previews[-1]
        assert p["min"].shape == (4, 100) and np.all(p["min"] <= p["max"])
        assert np.all(np.abs(p["levels_dbfs"] + 70) < 2)  # the sim's noise floor
        assert [x["seq"] for x in previews] == sorted(x["seq"] for x in previews)

        st = client.get("/api/status").json()["audio"]
        assert st["rate"] == 48000 and st["frames"] > 20 and st["lost_frames"] == 0
        assert len(st["levels_dbfs"]) == 4 and len(st["noise_dbfs"]) == 4 and st["clipping"] == [False] * 4

        ev = client.post("/api/audio/sim/emit", json={"bearing_deg": 40, "elevation_deg": 0, "snr_db": 22}).json()
        assert ev["id"] >= 1 and ev["label"] == "impulse"
        assert abs(angle_diff(ev["bearing_deg"], 40)) <= max(3.0, 2 * ev["bearing_sigma_deg"])
        assert ev["confidence"] <= 0.5 and any("not calibrated" in r for r in ev["reasons"])  # uncalibrated cap
        assert sorted(ev["tdoa_us"]) == ["12", "13", "14", "23", "24", "34"]
        assert ev["candidates"] == [] and "no analysed map" in ev["reason"]
        assert "person" not in json.dumps(ev).lower()
        t0 = time.time()
        while not events and time.time() - t0 < 3:
            m = ws.receive()
            if m.get("text"):
                d = json.loads(m["text"])
                if d["type"] == "audio_event":
                    events.append(d)
        assert events and events[0]["id"] == ev["id"]

    quiet = client.post("/api/audio/sim/emit", json={"bearing_deg": 40, "snr_db": -5}).json()
    assert quiet["detected"] is False and "trigger" in quiet["reason"]
    client.post("/api/audio/sim/emit", json={"bearing_deg": 200, "snr_db": 22})
    evs = client.get("/api/audio/events?limit=5").json()
    assert len(evs) == 2 and evs[0]["id"] > evs[1]["id"]
    assert client.post("/api/audio/clear").status_code == 204
    assert client.get("/api/audio/events").json() == []
    assert client.post("/api/audio/disconnect").json()["audio"] == {"connected": False}


def test_calibrate_endpoint_with_sim_claps(client):
    client.post("/api/audio/connect", json={"source": "sim"})
    clicks = []
    for b in (0, 50, 100, 150, 200, 250, 300):
        ev = client.post("/api/audio/sim/emit", json={"bearing_deg": b, "snr_db": 25}).json()
        clicks.append({"event_id": ev["id"], "bearing_deg": b})
    r = client.post("/api/audio/calibrate", json={"clicks": clicks, "apply": True})
    assert r.status_code == 200, r.text
    res = r.json()
    # the sim board is exactly the configured one: yaw 0, no skew, order confirmed
    assert abs(angle_diff(res["array_yaw_deg"], 0)) < 1.0
    assert max(abs(v) for v in res["channel_offsets_us"]) < 2.0
    assert res["order"]["confirmed"] and res["ok"] and res["applied"] and res["calibrated"]
    cfg = client.get("/api/config").json()["audio"]
    assert cfg["calibrated"] is True and cfg["channel_offsets_us"] == res["channel_offsets_us"]
    assert json.loads((client.tmp / "config.json").read_text())["audio"]["calibrated"] is True
    ev = client.post("/api/audio/sim/emit", json={"bearing_deg": 75, "snr_db": 25}).json()
    assert ev["confidence"] > 0.5 and ev["calibrated"] is True  # no longer capped
    assert client.post("/api/audio/calibrate", json={"clicks": clicks[:3]}).status_code == 400
    assert client.post("/api/audio/calibrate", json={"clicks": [{"event_id": 999, "bearing_deg": 0}] * 4}).status_code == 400


@pytest.fixture
def rig(tmp_path):
    msgs = []
    r = Rig(Config(), SessionStore(tmp_path / "sessions"), msgs.append, sim_hz=60)
    r.msgs = msgs
    yield r
    r.shutdown()


def test_candidates_come_from_the_swept_map(rig):
    rig.connect_servo("sim")
    rig.connect_lidar("sim")
    rig.start_job(JobParams(phi_start=-60, phi_end=60, phi_step=6, settle_ms=0))
    deadline = time.time() + 20
    while rig.job["state"] == "running" and time.time() < deadline:
        time.sleep(0.05)
    assert rig.job["state"] == "done"
    rig.audio.connect("sim")
    t0 = time.perf_counter()
    rig.audio.refresh_map_now()
    print(f"\nfindings for candidates: {(time.perf_counter() - t0) * 1000:.0f} ms on {rig.live_points().shape[0]} points")
    findings, _ = rig.audio._cached_map()
    target = (findings["shadows"] or findings["obstacles"])[0]
    time.sleep(0.6)  # detector warm-up
    ev = rig.audio.emit_sim(target["bearing_deg"], 0, 25)
    assert ev.get("detected", True), ev
    ids = [c["id"] for c in ev["candidates"]]
    assert target["id"] in ids
    order = [{"void": 0, "shadow": 1, "unknown": 2, "obstacle": 3}[c["kind"]] for c in ev["candidates"]]
    assert order == sorted(order)
    assert ev["map_age_s"] is not None and ev["map_age_s"] < 10
    st = rig.status()["audio"]
    print(f"audio CPU: {st['cpu_us_per_frame']} µs per 10 ms frame, {st['cpu_ms_per_event']} ms per event")
    assert st["cpu_us_per_frame"] < 2000


def test_audio_alarms_are_keyed(rig):
    rig.audio.connect("sim")
    src = rig.audio.source
    src.stop()  # drive frames by hand from here
    rig.audio._connected_at -= 10  # past the grace period
    rng = np.random.default_rng(0)

    def feed(seq, lost=0, peak=False):
        s = rng.normal(0, 10, (480, 4)).astype(np.int16)
        if peak:
            s[100, 2] = 32767
        f = AudioFrame(seq, seq * 10_000, 48000, 14, 0, s, time.time(), lost_before=lost)
        src._deliver(f)

    for i in range(50):
        feed(i)
    assert not [a for a in rig.check_alarms() if "audio" in a]
    feed(60, lost=9)
    feed(61, peak=True)
    al = rig.check_alarms()
    assert any("9 frames lost" in a for a in al)
    assert any("clipping on mic 3" in a and "A shift 15" in a for a in al)
    assert rig.status()["audio"]["clipping"] == [False, False, True, False]
    # nothing for > 2 s: stalled
    src.last_frame_t = time.time() - 5
    assert any("no audio frames" in a for a in rig.check_alarms())
    logs = [json.loads(m)["msg"] for m in rig.msgs if isinstance(m, str) and json.loads(m)["type"] == "log"]
    assert sum("alarm: audio clipping" in m for m in logs) == 1  # raised once, not every second
    rig.audio.disconnect()
    assert not [a for a in rig.check_alarms() if "audio" in a]


def test_sources_over_rest_and_ws(client):
    client.post("/api/audio/connect", json={"source": "sim"})
    assert client.get("/api/audio/sources").json() == {"active": [], "history": []}
    with client.websocket_connect("/ws") as ws:
        ids = []
        for _ in range(3):  # tapping from one place
            ev = client.post("/api/audio/sim/emit", json={"bearing_deg": 130, "snr_db": 25}).json()
            assert ev["noise"] is False and ev["source_id"] == 1 and ev["mics_used"] == [1, 2, 3, 4]
            ids.append(ev["id"])
        other = client.post("/api/audio/sim/emit", json={"bearing_deg": 250, "snr_db": 25}).json()
        assert other["source_id"] == 2
        got = None
        t0 = time.time()
        while time.time() - t0 < 3:
            m = ws.receive()
            if m.get("text"):
                d = json.loads(m["text"])
                if d["type"] == "audio_sources" and len(d["active"]) == 2:
                    got = d
                    break
        assert got is not None
    src = client.get("/api/audio/sources").json()
    a = {s["id"]: s for s in src["active"]}
    assert a[1]["n_events"] == 3 and a[1]["event_ids"] == ids and a[1]["active"] is True
    assert abs(angle_diff(a[1]["bearing_deg"], 130)) < 3 and a[1]["kind"] == "tapping"
    assert a[1]["confidence"] <= 0.5  # uncalibrated array: capped like single events
    assert set(a[1]) >= {"id", "kind", "bearing_deg", "spread_deg", "sigma_deg", "n_events", "first_t", "last_t", "rate_per_min", "confidence", "peak_dbfs", "event_ids", "candidates", "active"}
    assert client.get("/api/status").json()["audio"]["sources_active"] == 2
    # a source quiet for 120 s retires to the history
    client.app.state.rig.audio.tick(time.time() + 121)
    src = client.get("/api/audio/sources").json()
    assert src["active"] == [] and len(src["history"]) == 2 and src["history"][0]["active"] is False
    assert client.post("/api/audio/clear").status_code == 204
    assert client.get("/api/audio/sources").json() == {"active": [], "history": []}


def test_noisy_mic_is_excluded_and_events_use_three(client):
    client.post("/api/audio/connect", json={"source": "sim"})
    rig = client.app.state.rig
    sim = rig.audio.source
    sim.channel_noise_rms = sim.channel_noise_rms * np.array([1.0, 1.0, 10 ** (15 / 20), 1.0])  # ch3 +15 dB
    deadline = time.time() + 6
    st = None
    while time.time() < deadline:
        st = client.get("/api/status").json()["audio"]
        if st["excluded_mics"]:
            break
        time.sleep(0.2)
    assert [m["mic"] for m in st["excluded_mics"]] == [3] and "above the others" in st["excluded_mics"][0]["reason"]
    assert st["noise_dbfs"][2] > max(st["noise_dbfs"][i] for i in (0, 1, 3)) + 12
    time.sleep(0.8)  # the trigger sum restarted its floor without ch3
    ev = client.post("/api/audio/sim/emit", json={"bearing_deg": 75, "snr_db": 22}).json()
    assert ev["mics_used"] == [1, 2, 4] and abs(angle_diff(ev["bearing_deg"], 75)) < 5
    assert any("3 mics" in r for r in ev["reasons"])
    # the hiss stops: after the hysteresis the mic is back
    sim.channel_noise_rms = np.full(4, sim.noise_rms)
    deadline = time.time() + 8
    while time.time() < deadline and client.get("/api/status").json()["audio"]["excluded_mics"]:
        time.sleep(0.2)
    assert client.get("/api/status").json()["audio"]["excluded_mics"] == []


def test_i2s_timeout_frames_are_not_analysed(rig):
    rig.audio.connect("sim")
    src = rig.audio.source
    src.stop()
    rng = np.random.default_rng(1)
    for i in range(60):
        src._deliver(AudioFrame(i, i * 10_000, 48000, 14, 0, rng.normal(0, 10, (480, 4)).astype(np.int16), time.time()))
    loud = (rng.normal(0, 3000, (480, 4))).astype(np.int16)
    src._deliver(AudioFrame(60, 600_000, 48000, 14, 0x1, loud, time.time()))  # timeout frame: maybe padding
    for i in range(61, 80):
        src._deliver(AudioFrame(i, i * 10_000, 48000, 14, 0, rng.normal(0, 10, (480, 4)).astype(np.int16), time.time()))
    st = rig.status()["audio"]
    assert st["events"] == 0 and st["gaps"] >= 1


def test_connect_udp_over_rest(client):
    from test_audio_wire import FakeEspUdp

    from obslam.audio_pipeline import AudioPipeline
    from obslam.audio_wire import encode_aud2

    # port selection: explicit, configured, borrowed host from the other transport, mismatch
    assert AudioPipeline._port_for("udp", None, "udp://10.0.0.5:8902") == "udp://10.0.0.5:8902"
    assert AudioPipeline._port_for("tcp", None, "udp://10.0.0.5:8902") == "tcp://10.0.0.5:8901"
    assert AudioPipeline._port_for("udp", "10.0.0.7", "x") == "udp://10.0.0.7:8902"
    assert client.post("/api/audio/connect", json={"source": "udp", "port": "tcp://10.0.0.5:8901"}).status_code == 400
    assert client.get("/api/config").json()["audio"]["port"].startswith("udp://")

    esp = FakeEspUdp()
    esp.start()
    try:
        st = client.post("/api/audio/connect", json={"source": "udp", "port": f"udp://127.0.0.1:{esp.port}"}).json()["audio"]
        assert st["source"] == "udp" and st["transport_note"] is None
        assert esp.wait_sub()
        for seq in range(80):  # 0.8 s of noise at 16 kHz, the board's rate
            n = 160
            s = np.random.default_rng(seq).normal(0, 20, (n, 4)).astype(np.int16)
            esp.send(encode_aud2(seq, seq * 10_000, 16000, s))
            time.sleep(0.002)
        t0 = time.time()
        while time.time() - t0 < 3 and client.get("/api/status").json()["audio"]["frames"] < 80:
            time.sleep(0.05)
        st = client.get("/api/status").json()["audio"]
        assert st["link"] == "up" and st["frames"] == 80 and st["rate"] == 16000 and st["lost_frames"] == 0
        assert client.post("/api/audio/disconnect").json()["audio"] == {"connected": False}
        t0 = time.time()
        while time.time() - t0 < 2 and esp.audx == 0:
            time.sleep(0.02)
        assert esp.audx == 1
    finally:
        esp.close()
    st = client.post("/api/audio/connect", json={"source": "tcp", "port": "tcp://127.0.0.1:9"}).json()["audio"]
    assert "udp://" in st["transport_note"]
    client.post("/api/audio/disconnect")
