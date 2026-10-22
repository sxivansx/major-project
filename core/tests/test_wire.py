import numpy as np
import pytest

from obslam import wire
from obslam.grid import build_grid


def test_scan_round_trip():
    a = np.linspace(0, 359, 50, dtype=np.float32)
    r = np.random.default_rng(1).random(50).astype(np.float32)
    q = np.full(50, 47, np.uint8)
    buf = wire.encode_scan(7, 1700000000.5, -12.5, a, r, q)
    assert len(buf) == 24 + 50 * 9
    d = wire.decode_scan(buf)
    assert d["seq"] == 7 and d["t"] == 1700000000.5 and abs(d["phi"] + 12.5) < 1e-6
    np.testing.assert_array_equal(d["angle"], a)
    np.testing.assert_array_equal(d["range"], r)
    np.testing.assert_array_equal(d["quality"], q)


def test_points_round_trip():
    xyz = np.arange(30, dtype=np.float32).reshape(10, 3)
    buf = wire.encode_points(3, 4, 5.5, xyz)
    assert len(buf) == 20 + 10 * 12
    d = wire.decode_points(buf)
    assert (d["job_id"], d["step"]) == (3, 4)
    np.testing.assert_array_equal(d["xyz"], xyz)


def test_grid_round_trip():
    p = np.array([[0.05, 0.05, 1.0], [0.15, 0.25, 2.0]], np.float32)
    g = build_grid(p, 0.1, -1, 3, "max")
    buf = wire.encode_grid(g)
    assert len(buf) == 32 + g.w * g.h * 4
    d = wire.decode_grid(buf)
    assert (d.w, d.h, d.layer) == (g.w, g.h, "max")
    assert d.cell == pytest.approx(g.cell) and d.x0 == pytest.approx(g.x0) and d.y0 == pytest.approx(g.y0)
    np.testing.assert_array_equal(np.isnan(d.data), np.isnan(g.data))
    np.testing.assert_array_equal(d.data[~np.isnan(d.data)], g.data[~np.isnan(g.data)])
