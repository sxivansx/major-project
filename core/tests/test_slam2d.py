import math
import time

import numpy as np

from obslam.slam2d import RoomMapper, scan_to_points

# an L-shaped room with a cupboard: segments (x1, y1, x2, y2) in metres
ROOM = [
    (-1.0, -1.5, 4.0, -1.5), (4.0, -1.5, 4.0, 1.0), (4.0, 1.0, 2.0, 1.0), (2.0, 1.0, 2.0, 3.0),
    (2.0, 3.0, -1.0, 3.0), (-1.0, 3.0, -1.0, -1.5),
    (0.5, 1.8, 1.2, 1.8), (1.2, 1.8, 1.2, 2.4), (1.2, 2.4, 0.5, 2.4), (0.5, 2.4, 0.5, 1.8),
]


def lidar(pose, rng, n=2800, noise=0.01):
    """Ray-cast the room from `pose`; return LiDAR-native angles (clockwise deg) and ranges."""
    x, y, th = pose
    a = np.linspace(0, 360, n, endpoint=False)
    world = th - np.radians(a)  # clockwise lidar angle → CCW world heading
    dx, dy = np.cos(world), np.sin(world)
    best = np.full(n, np.inf)
    for x1, y1, x2, y2 in ROOM:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x1 - x) * ey - (y1 - y) * ex) / den
            u = ((x1 - x) * dy - (y1 - y) * dx) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    r = best + rng.normal(0, noise, n)
    r[~np.isfinite(best)] = 0
    return a, r.astype(np.float32), np.full(n, 40, np.uint8)


def test_scan_points_follow_the_clockwise_convention():
    p = scan_to_points(np.array([0.0, 90.0]), np.array([1.0, 2.0]), np.array([10, 10]))
    np.testing.assert_allclose(p, [[1, 0], [0, -2]], atol=1e-9)


def test_handheld_walk_tracks_and_maps_the_room():
    rng = np.random.default_rng(0)
    m = RoomMapper()
    # walk 2.5 m, turning 90° on the way, at walking-with-a-scanner speed (0.3 m/s, 10 Hz)
    poses = []
    x, y, th = 0.0, 0.0, 0.0
    for k in range(120):
        if k < 60:
            x += 0.03
        elif k < 90:
            th += math.radians(3)
        else:
            y += 0.025
        poses.append((x, y, th))
    t0 = time.perf_counter()
    for p in [(0.0, 0.0, 0.0)] + poses:
        a, r, q = lidar(p, rng)
        m.add_scan(scan_to_points(a, r, q))
    per_scan_ms = (time.perf_counter() - t0) / (len(poses) + 1) * 1000
    ex, ey, eth = m.pose
    tx, ty, tth = poses[-1]
    assert math.hypot(ex - tx, ey - ty) < 0.05, (m.pose, poses[-1])
    assert abs(math.degrees(eth - tth)) < 1.5
    assert not m.lost and m.last.ok and m.last.inliers > 0.8
    assert m.keyframes > 20
    assert 15.0 < m.area_m2() < 18.1  # 12.5 + 6.0 m² of L-shaped room, less the 0.42 m² cupboard
    assert per_scan_ms < 100, per_scan_ms
    cls, x0, y0 = m.classes()
    assert (cls == 2).sum() > 200 and x0 < -1.0 and y0 < -1.5
    st = m.state()
    assert st["path"][-1] == [round(ex, 3), round(ey, 3)] and st["match"]["rms_mm"] < 20


def test_a_jump_is_reported_lost_not_mapped():
    rng = np.random.default_rng(1)
    m = RoomMapper()
    m.add_scan(scan_to_points(*lidar((0, 0, 0), rng)))
    before = m.L.copy()
    m.add_scan(scan_to_points(*lidar((0, 0, 0), rng)))
    # the scanner is carried into a different room in one step: nothing in the map matches
    far = scan_to_points(np.linspace(0, 360, 2800, endpoint=False), np.full(2800, 0.4, np.float32), np.full(2800, 40))
    res = m.add_scan(far)
    assert res is not None and not res.ok and m.lost
    assert np.array_equal(m.L > 0.6, before > 0.6)  # the map was not corrupted


def test_room_api_with_the_sim_lidar(tmp_path):
    from fastapi.testclient import TestClient

    from obslam import wire
    from obslam.server import create_app

    app = create_app(config_path=tmp_path / "c.json", sessions_dir=tmp_path / "s", web_dist=tmp_path / "nope", sim_hz=20)
    with TestClient(app) as c:
        assert c.post("/api/room/start").status_code == 400  # no lidar yet
        c.post("/api/lidar/connect", json={"source": "sim"})
        st = c.post("/api/room/start", json={"cell_m": 0.05}).json()
        assert st["state"] == "mapping"
        deadline = time.time() + 5
        while c.get("/api/room").json()["keyframes"] < 1 and time.time() < deadline:
            time.sleep(0.05)
        info = c.get("/api/room").json()
        assert info["keyframes"] >= 1 and info["grid"]["w"] > 10
        g = wire.decode_room(c.get("/api/room/grid").content)
        assert g["w"] == info["grid"]["w"] and (g["classes"] == 2).any()
        assert c.post("/api/room/stop").json()["state"] == "idle"


def test_next_station_points_into_the_unseen_wing():
    rng = np.random.default_rng(2)
    m = RoomMapper()
    # from the far right of the main room the upper-left wing (x < 2, y > 1) is mostly hidden
    for _ in range(3):
        m.add_scan(scan_to_points(*lidar((3.5, -1.0, 0.0), rng)))
    nx = m.state()["next"]
    st = nx["station"]
    assert st is not None and nx["open_edges"] >= 1 and not nx["complete"]
    assert st["distance_m"] > 0.4
    cls, _, _ = m.classes()
    assert (cls == 3).any()
    # walking the whole room closes it
    m2 = RoomMapper()
    path = [(0.0, 0.0, 0.0)] + [(0.04 * k, 0.0, 0.0) for k in range(1, 70)]
    path += [(2.76 - 0.04 * k, 0.0, 0.0) for k in range(1, 45)]  # back to x ≈ 1.0
    path += [(1.0, 0.04 * k, 0.0) for k in range(1, 50)]  # up into the wing
    for p in path:
        m2.add_scan(scan_to_points(*lidar(p, rng)))
    assert not m2.lost
    assert m2.state()["next"]["open_edges"] <= nx["open_edges"]
