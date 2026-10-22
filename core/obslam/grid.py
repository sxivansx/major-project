"""2.5D elevation grid from base-frame points. Layout matches the kind-3 frame in docs/api.md §3."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

LAYERS = ("max", "min", "mean", "count")
LAYER_ID = {name: i for i, name in enumerate(LAYERS)}


@dataclass
class Grid:
    layer: str
    cell: float
    x0: float
    y0: float
    w: int
    h: int
    z_min: float
    z_max: float
    data: np.ndarray  # (h, w) float32, NaN = empty

    def cell_center(self, ix: int, iy: int) -> tuple[float, float]:
        return self.x0 + (ix + 0.5) * self.cell, self.y0 + (iy + 0.5) * self.cell


def _empty(layer, cell, z_min, z_max) -> Grid:
    return Grid(layer, cell, 0.0, 0.0, 1, 1, z_min, z_max, np.full((1, 1), np.nan, np.float32))


def build_grid(
    xyz: np.ndarray,
    cell: float,
    z_min: float,
    z_max: float,
    layer: str = "max",
    bounds: tuple[float, float, float, float] | None = None,
    max_cells: int = 4_000_000,
) -> Grid:
    """Rasterise points into a height grid.

    bounds = (x0, y0, x1, y1) in metres; auto-fitted to the points with one cell of padding when None.
    If the auto grid would exceed max_cells the cell size is coarsened.
    """
    if layer not in LAYERS:
        raise ValueError(f"unknown layer {layer!r}")
    cell = float(cell)
    if cell <= 0:
        raise ValueError("cell must be > 0")
    p = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    if p.shape[0] == 0:
        return _empty(layer, cell, z_min, z_max)
    p = p[(p[:, 2] >= z_min) & (p[:, 2] <= z_max)]
    if p.shape[0] == 0:
        return _empty(layer, cell, z_min, z_max)

    if bounds is None:
        x0 = float(np.floor(p[:, 0].min() / cell) * cell - cell)
        y0 = float(np.floor(p[:, 1].min() / cell) * cell - cell)
        x1 = float(np.ceil(p[:, 0].max() / cell) * cell + cell)
        y1 = float(np.ceil(p[:, 1].max() / cell) * cell + cell)
        w = max(1, int(round((x1 - x0) / cell)))
        h = max(1, int(round((y1 - y0) / cell)))
        if w * h > max_cells:
            cell *= float(np.sqrt(w * h / max_cells))
            return build_grid(p, cell, z_min, z_max, layer, None, max_cells)
    else:
        x0, y0, x1, y1 = map(float, bounds)
        w = max(1, int(round((x1 - x0) / cell)))
        h = max(1, int(round((y1 - y0) / cell)))
        if w * h > max_cells:
            raise ValueError("grid too large for these bounds")

    ix = np.floor((p[:, 0] - x0) / cell).astype(np.int64)
    iy = np.floor((p[:, 1] - y0) / cell).astype(np.int64)
    inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    ix, iy, pz = ix[inside], iy[inside], p[inside, 2].astype(np.float64)
    flat = iy * w + ix
    n = w * h
    count = np.bincount(flat, minlength=n)
    data = np.full(n, np.nan, np.float32)

    if layer == "count":
        data[count > 0] = count[count > 0]
    elif layer == "mean":
        s = np.bincount(flat, weights=pz, minlength=n)
        nz = count > 0
        data[nz] = s[nz] / count[nz]
    else:
        order = np.lexsort((pz, flat))
        fs, zs = flat[order], pz[order]
        if layer == "max":
            pick = np.r_[fs[1:] != fs[:-1], True]  # last of each run = max
        else:
            pick = np.r_[True, fs[1:] != fs[:-1]]  # first of each run = min
        data[fs[pick]] = zs[pick]

    return Grid(layer, cell, x0, y0, w, h, float(z_min), float(z_max), data.reshape(h, w))
