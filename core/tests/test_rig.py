import json
import time

import numpy as np
import pytest

from obslam import wire
from obslam.config import Config
from obslam.rig import JobParams, Rig, RigError
from obslam.session import SessionStore


@pytest.fixture
def rig(tmp_path):
    msgs = []
    r = Rig(Config(), SessionStore(tmp_path / "sessions"), msgs.append, sim_hz=60)
    r.msgs = msgs
    yield r
    r.shutdown()


def kinds(msgs):
    out = {"json": [], 1: [], 2: [], 3: []}
    for m in msgs:
        if isinstance(m, bytes):
            out[m[0]].append(m)
        else:
            out["json"].append(json.loads(m))
    return out


def test_phis():
    assert list(JobParams(phi_start=-10, phi_end=10, phi_step=10).phis()) == [-10, 0, 10]
    assert list(JobParams(phi_start=10, phi_end=-10, phi_step=10).phis()) == [10, 0, -10]
    assert list(JobParams(phi_start=5, phi_end=5, phi_step=1).phis()) == [5]


def test_connect_and_status(rig):
    assert rig.status()["lidar"]["connected"] is False
    with pytest.raises(RigError):
        rig.connect_lidar("s2")  # no port
    rig.connect_servo("sim")
    rig.connect_lidar("sim")
    time.sleep(0.2)
    s = rig.status()
    assert s["lidar"]["connected"] and s["lidar"]["model"] == "SIM"
    assert s["lidar"]["pts_per_rev"] > 3000 and s["lidar"]["scan_hz"] > 20
    assert s["servo"]["servo_deg"] == 90.0 and s["servo"]["phi_deg"] == 0.0
    assert s["servo"]["meas_deg"] == 90.0 and s["servo"]["phi_meas_deg"] == 0.0 and s["servo"]["encoder"] == "sim"
    assert any(isinstance(m, bytes) and m[0] == wire.KIND_SCAN for m in rig.msgs)
    with pytest.raises(RigError):
        rig.connect_lidar("sim")
    rig.disconnect_lidar()
    assert rig.status()["lidar"]["connected"] is False


def test_servo_moves_and_limits(rig):
    with pytest.raises(RigError):
        rig.move_servo(servo_deg=10)
    rig.connect_servo("sim")
    assert rig.move_servo(phi_deg=30) == 120.0
    assert rig.status()["servo"]["phi_deg"] == 30.0
    with pytest.raises(RigError):
        rig.move_servo(servo_deg=300)
    assert rig.home_servo() == 90.0
    assert rig.servo_raw("?").startswith("STATUS")


def test_job_end_to_end(rig):
    rig.connect_servo("sim")
    rig.connect_lidar("sim")
    with pytest.raises(RigError):
        rig.start_job(JobParams(phi_start=-200, phi_end=0, phi_step=10))
    job_id = rig.start_job(JobParams(name="t", phi_start=-30, phi_end=30, phi_step=15, rotations=1, settle_ms=0))
    assert job_id == 1
    deadline = time.time() + 10
    while rig.job["state"] == "running" and time.time() < deadline:
        time.sleep(0.05)
    assert rig.job["state"] == "done", rig.job
    assert rig.job["step"] == 5 and rig.job["points"] > 10_000
    k = kinds(rig.msgs)
    assert len(k[2]) == 5
    assert len(k[3]) >= 1
    jobs = [m for m in k["json"] if m["type"] == "job"]
    assert jobs[-1]["state"] == "done" and jobs[-1]["session_id"]
    sid = jobs[-1]["session_id"]
    meta = rig.sessions.get(sid)
    assert meta["n_points"] == rig.job["points"] and meta["n_steps"] == 5
    # the saved sweep reproduces the live cloud
    np.testing.assert_allclose(rig.sessions.points(sid), rig.live_points(), atol=1e-6)
    # the sim encoder agrees with the command, and both are stored
    a = rig.sessions.arrays(sid)
    np.testing.assert_allclose(a["phi_meas"], a["phi"])
    assert meta["encoder"] == "sim"
    g = rig.grid(cell=0.1)
    assert g.w > 10 and np.isfinite(g.data).sum() > 100
    # floor cells sit at ~0, walls near 3 m across the x extent
    assert abs(np.nanmin(g.data)) < 0.05
    # load back
    rig.clear_live()
    assert rig.live_points().shape[0] == 0
    rig.load_session(sid)
    assert rig.live_points().shape[0] == meta["n_points"] and rig.session_loaded == sid


def test_job_stop(rig):
    rig.connect_servo("sim")
    rig.connect_lidar("sim")
    rig.start_job(JobParams(phi_start=-60, phi_end=60, phi_step=1, settle_ms=50))
    time.sleep(0.3)
    rig.stop_job()
    assert rig.job["state"] == "stopped"
    assert 0 < rig.job["step"] < 121
