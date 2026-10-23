import json
import time

import pytest
from fastapi.testclient import TestClient

from obslam import wire
from obslam.server import create_app


@pytest.fixture
def client(tmp_path):
    app = create_app(
        config_path=tmp_path / "config.json",
        sessions_dir=tmp_path / "sessions",
        web_dist=tmp_path / "nope",
        sim_hz=60,
    )
    with TestClient(app) as c:
        yield c


def test_status_ports_config(client):
    s = client.get("/api/status").json()
    assert s["lidar"]["connected"] is False and s["job"] is None
    assert isinstance(client.get("/api/ports").json(), list)
    cfg = client.get("/api/config").json()
    cfg["mount"]["h_mm"] = 12.5
    assert client.put("/api/config", json=cfg).json()["mount"]["h_mm"] == 12.5
    assert client.get("/api/config").json()["mount"]["h_mm"] == 12.5
    assert client.get("/").json()["name"] == "onboard-slam"


def test_rig_errors_are_400(client):
    r = client.post("/api/servo/move", json={"phi_deg": 10})
    assert r.status_code == 400 and "not connected" in r.json()["error"]
    assert client.get("/api/grid?layer=median").status_code == 400
    assert client.get("/api/sessions/../etc").status_code in (400, 404)
    assert client.get("/api/sessions/nope").status_code == 404


def test_sim_job_over_http_and_ws(client):
    assert client.post("/api/servo/connect", json={"port": "sim"}).json()["servo"]["connected"]
    assert client.post("/api/lidar/connect", json={"source": "sim"}).json()["lidar"]["connected"]
    with client.websocket_connect("/ws") as ws:
        first = json.loads(ws.receive_text())
        assert first["type"] == "status"
        job_id = client.post("/api/job/start", json={"name": "api", "phi_start": -20, "phi_end": 20, "phi_step": 20, "settle_ms": 0}).json()["job_id"]
        seen = set()
        deadline = time.time() + 10
        done = None
        while time.time() < deadline and done is None:
            m = ws.receive()
            if "bytes" in m and m["bytes"] is not None:
                seen.add(m["bytes"][0])
            elif "text" in m and m["text"] is not None:
                d = json.loads(m["text"])
                seen.add(d["type"])
                if d["type"] == "job" and d["state"] != "running":
                    done = d
        assert done and done["state"] == "done" and done["job_id"] == job_id
        assert {wire.KIND_SCAN, wire.KIND_POINTS, wire.KIND_GRID, "job"} <= seen
    sessions = client.get("/api/sessions").json()
    assert len(sessions) == 1 and sessions[0]["n_points"] > 1000
    sid = sessions[0]["id"]
    pts = client.get(f"/api/sessions/{sid}/points").content
    assert len(pts) == 4 + sessions[0]["n_points"] * 12
    g = wire.decode_grid(client.get(f"/api/sessions/{sid}/grid?cell=0.2&layer=count").content)
    assert g.layer == "count" and g.w > 5
    g2 = wire.decode_grid(client.get("/api/grid?cell=0.2").content)
    assert g2.w == g.w and g2.h == g.h
    assert client.get(f"/api/sessions/{sid}/export.ply").text.startswith("ply\n")
    assert client.get(f"/api/sessions/{sid}/export.csv").text.startswith("phi,phi_meas,theta")
    assert client.post(f"/api/sessions/{sid}/load").json()["session_loaded"] == sid
    assert client.delete(f"/api/sessions/{sid}").status_code == 204
    assert client.get("/api/sessions").json() == []
    assert client.post("/api/lidar/disconnect").json()["lidar"]["connected"] is False
