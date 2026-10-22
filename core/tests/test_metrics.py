import time

import pytest
from fastapi.testclient import TestClient

from obslam.config import Config
from obslam.rig import Rig
from obslam.server import create_app
from obslam.session import SessionStore


@pytest.fixture
def rig(tmp_path):
    msgs = []
    r = Rig(Config(), SessionStore(tmp_path / "sessions"), msgs.append, sim_hz=60)
    r.msgs = msgs
    yield r
    r.shutdown()


def test_alarm_raised_when_the_source_dies(rig):
    rig.connect_servo("sim")
    rig.connect_lidar("sim")
    time.sleep(0.1)
    rig._scan_connected_at -= 10  # skip the warm-up grace
    assert rig.check_alarms() == []

    def boom():
        raise RuntimeError("boom")

    rig.scan.phi_provider = boom  # next frame raises inside the source thread
    deadline = time.time() + 3
    while rig.scan is not None and time.time() < deadline:
        time.sleep(0.02)
    assert rig.scan is None
    alarms = rig.check_alarms()
    assert alarms == ["lidar stream died: boom"]
    assert rig.status()["alarms"] == alarms
    assert any("alarm: lidar stream died" in m for m in rig.msgs if isinstance(m, str))
    # reconnecting clears it (after the grace period)
    rig.connect_lidar("sim")
    rig._scan_connected_at -= 10
    time.sleep(0.1)
    assert rig.check_alarms() == []
    assert any("cleared: lidar stream died" in m for m in rig.msgs if isinstance(m, str))


def test_metrics_empty(rig):
    m = rig.metrics()
    assert m["points"] == 0 and m["grid"]["filled"] == 0 and m["floor"] is None


def test_snapshot_and_metrics_over_http(tmp_path):
    app = create_app(config_path=tmp_path / "c.json", sessions_dir=tmp_path / "s", web_dist=tmp_path / "nope", sim_hz=60)
    with TestClient(app) as c:
        assert c.get("/api/scan/snapshot.csv").status_code == 400  # lidar not connected
        c.post("/api/servo/connect", json={"port": "sim"})
        c.post("/api/lidar/connect", json={"source": "sim"})
        assert c.get("/api/scan/snapshot.csv?rotations=0").status_code == 400
        csv = c.get("/api/scan/snapshot.csv?rotations=2").text.splitlines()
        assert csv[0] == "rotation,t,phi,angle,range,quality"
        assert len(csv) > 6000 and csv[1].startswith("0,") and csv[-1].startswith("1,")
        assert c.get("/api/status").json()["alarms"] == []

        c.post("/api/job/start", json={"name": "m", "phi_start": -30, "phi_end": 30, "phi_step": 10, "settle_ms": 0})
        deadline = time.time() + 10
        while time.time() < deadline:
            job = c.get("/api/status").json()["job"]
            if job["state"] != "running":
                break
            time.sleep(0.05)
        assert job["state"] == "done"
        m = c.get("/api/metrics?cell=0.1").json()
        assert m["steps"] == 7 and m["points"] > 10_000 and m["points_per_step"] > 1000
        assert m["duration_s"] > 0
        assert m["grid"]["cell_m"] == pytest.approx(0.1) and 0 < m["grid"]["coverage_pct"] < 100
        assert m["floor"]["points"] > 500
        assert m["floor"]["rms_mm"] < 40  # sim noise is 12 mm, 1-sigma
        assert abs(m["floor"]["tilt_deg"]) < 1.0
