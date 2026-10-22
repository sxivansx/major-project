import numpy as np
import pytest

from obslam.grid import build_grid


def pts():
    return np.array(
        [
            [0.01, 0.01, 0.1],
            [0.02, 0.02, 0.3],  # same cell as above
            [0.12, 0.01, 0.5],
            [0.01, 0.12, 5.0],  # clipped by z_max
        ],
        np.float32,
    )


def test_layers():
    g = build_grid(pts(), 0.1, -1, 1, "max", bounds=(0, 0, 0.2, 0.2))
    assert (g.w, g.h) == (2, 2)
    assert g.data[0, 0] == pytest.approx(0.3)
    assert g.data[0, 1] == pytest.approx(0.5)
    assert np.isnan(g.data[1, 0]) and np.isnan(g.data[1, 1])
    assert build_grid(pts(), 0.1, -1, 1, "min", bounds=(0, 0, 0.2, 0.2)).data[0, 0] == pytest.approx(0.1)
    assert build_grid(pts(), 0.1, -1, 1, "mean", bounds=(0, 0, 0.2, 0.2)).data[0, 0] == pytest.approx(0.2)
    assert build_grid(pts(), 0.1, -1, 1, "count", bounds=(0, 0, 0.2, 0.2)).data[0, 0] == 2


def test_auto_bounds_pad_one_cell():
    g = build_grid(pts(), 0.1, -1, 1)
    assert g.x0 == pytest.approx(-0.1) and g.y0 == pytest.approx(-0.1)
    assert g.w >= 3 and g.h >= 3
    assert np.nanmax(g.data) == pytest.approx(0.5)


def test_empty_and_errors():
    g = build_grid(np.zeros((0, 3)), 0.1, -1, 1)
    assert g.data.shape == (1, 1) and np.isnan(g.data[0, 0])
    with pytest.raises(ValueError):
        build_grid(pts(), 0.1, -1, 1, "median")


def test_coarsens_when_too_big():
    p = np.array([[0, 0, 0], [100, 100, 0]], np.float32)
    g = build_grid(p, 0.01, -1, 1, max_cells=10_000)
    assert g.w * g.h <= 10_000 * 1.1
    assert g.cell > 0.01
