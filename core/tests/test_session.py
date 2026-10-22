import numpy as np
import pytest

from obslam.config import MountConfig
from obslam.session import SessionStore, valid_id


def test_round_trip_and_exports(tmp_path):
    s = SessionStore(tmp_path)
    sid = s.new_id("Bench Test 1!")
    assert sid.endswith("-bench-test-1") and valid_id(sid)
    n = 10
    arrays = {
        "step": np.zeros(n, np.uint16),
        "phi": np.full(n, -30.0, np.float32),
        "theta": np.linspace(0, 90, n, dtype=np.float32),
        "range": np.linspace(1, 2, n, dtype=np.float32),
        "quality": np.full(n, 50, np.uint8),
        "t": np.full(n, 1.7e9),
    }
    meta = {"id": sid, "name": "bench", "created": 1.7e9, "n_points": n, "mount": MountConfig(h_mm=10).model_dump()}
    s.save(meta, arrays)
    assert [m["id"] for m in s.list()] == [sid]
    assert s.get(sid)["n_points"] == n
    a = s.arrays(sid)
    np.testing.assert_array_equal(a["theta"], arrays["theta"])
    xyz = s.points(sid)
    assert xyz.shape == (n, 3)
    ply = s.ply(xyz).decode()
    assert ply.startswith("ply\n") and f"element vertex {n}" in ply and ply.count("\n") == 7 + n
    csv = s.csv(sid).decode().splitlines()
    assert csv[0] == "phi,theta,range,quality,x,y,z" and len(csv) == n + 1
    s.delete(sid)
    assert s.list() == []
    with pytest.raises(KeyError):
        s.get(sid)


def test_bad_ids_rejected(tmp_path):
    s = SessionStore(tmp_path)
    for bad in ("../x", "a/b", "", "x" * 200):
        assert not valid_id(bad)
        with pytest.raises(KeyError):
            s.get(bad)
