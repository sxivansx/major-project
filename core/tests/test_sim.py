import numpy as np
import pytest

from obslam.config import MountConfig
from obslam.geometry import lidar_to_base
from obslam.sim import SimScanSource


def test_level_scan_sees_walls_only():
    m = MountConfig(axis_z_mm=80, h_mm=0)
    sim = SimScanSource(lambda: 0.0, m, noise_m=0.0, dropout=0.0)
    f = sim.scan_at(0.0)
    assert 3000 < f.n <= 3200
    p = lidar_to_base(f.angle, f.range, 0.0, m)
    np.testing.assert_allclose(p[:, 2], 0.08, atol=1e-4)
    # nothing beyond the room, everything inside or on the walls
    assert p[:, 0].max() <= 3.0 + 1e-3 and p[:, 0].min() >= -3.0 - 1e-3
    assert p[:, 1].max() <= 2.0 + 1e-3 and p[:, 1].min() >= -2.0 - 1e-3


def test_tilted_scan_hits_floor_at_zero_height():
    m = MountConfig(axis_z_mm=80, h_mm=0)
    sim = SimScanSource(lambda: 0.0, m, noise_m=0.0, dropout=0.0)
    f = sim.scan_at(-40.0)
    p = lidar_to_base(f.angle, f.range, -40.0, m)
    floor = p[np.abs(p[:, 2]) < 1e-3]
    assert floor.shape[0] > 500
    # the raised half of the plane meets the y=+2 wall at 2·tan40° above the axis
    assert p[:, 2].max() == pytest.approx(2.0 * np.tan(np.deg2rad(40)) + 0.08, abs=1e-3)
    # at 70° the plane clears the walls and reaches the 3 m ceiling
    f70 = sim.scan_at(-70.0)
    p70 = lidar_to_base(f70.angle, f70.range, -70.0, m)
    assert (np.abs(p70[:, 2] - 3.0) < 1e-3).sum() > 100
    # every point is on a known surface: floor, ceiling, a wall, or a box top/side
    on_surface = (
        (np.abs(p[:, 2]) < 1e-3)
        | (np.abs(p[:, 2] - 3.0) < 1e-3)
        | (np.abs(np.abs(p[:, 0]) - 3.0) < 1e-3)
        | (np.abs(np.abs(p[:, 1]) - 2.0) < 1e-3)
    )
    assert on_surface.mean() > 0.9


def test_threaded_source_delivers_frames():
    m = MountConfig()
    sim = SimScanSource(lambda: 10.0, m, hz=100)
    got = []
    sim.start(got.append)
    import time

    time.sleep(0.15)
    sim.stop()
    assert len(got) >= 5
    assert all(g.angle.shape == g.range.shape == g.quality.shape for g in got)
