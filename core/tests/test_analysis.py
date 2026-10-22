import numpy as np
import pytest

from obslam import wire
from obslam.analysis import BLIND, BLOCKED, MARGINAL, PASSABLE, SHADOW, UNKNOWN, OpsParams, analyze, bearing_deg, label_regions, plan_route
from obslam.config import MountConfig
from obslam.geometry import lidar_to_base
from obslam.grid import Grid
from obslam.sim import ROOM, SimScanSource


def sweep(axis_z_mm: float) -> np.ndarray:
    """A clean sweep of the simulated room: -60..60 in 2° steps, no noise, no dropouts."""
    m = MountConfig(axis_z_mm=axis_z_mm, h_mm=0)
    sim = SimScanSource(lambda: 0.0, m, noise_m=0.0, dropout=0.0)
    parts = []
    for phi in np.arange(-60, 61, 2):
        f = sim.scan_at(float(phi))
        parts.append(lidar_to_base(f.angle, f.range, float(phi), m))
    return np.concatenate(parts)


@pytest.fixture(scope="module")
def room_cloud():
    return sweep(80)  # the bench rig: scanner 8 cm above the floor


@pytest.fixture(scope="module")
def tripod_cloud():
    return sweep(1000)  # deployment: scanner on a tripod, 1 m up


def test_low_scanner_barely_samples_the_floor(room_cloud, tripod_cloud):
    low = analyze(room_cloud, OpsParams(cell_m=0.1), scanner_z_m=0.08).findings["coverage"]
    high = analyze(tripod_cloud, OpsParams(cell_m=0.1), scanner_z_m=1.0).findings["coverage"]
    # at 8 cm the floor is a few lines at y = 0.08/tan φ with wide gaps; at 1 m the whole room floor is seen
    assert low["unknown_cells"] > 2 * high["unknown_cells"]
    # from 8 cm every box is taller than the scanner and shadows everything behind it
    assert low["shadow_cells"] > 2 * high["shadow_cells"]
    assert high["floor_reach_m"] >= 2.9
    # the blind stripe scales with height: ±0.05 m at 8 cm, ±0.58 m on the tripod
    assert low["blind_stripe"]["half_width_m"] == pytest.approx(0.08 / np.tan(np.radians(60)), abs=0.01)
    assert high["blind_stripe"]["half_width_m"] == pytest.approx(0.577, abs=0.01)


def test_bearing_convention():
    assert bearing_deg(1, 0) == 0
    assert bearing_deg(0, -1) == 90  # clockwise from +x, like the lidar's native angle
    assert bearing_deg(-1, 0) == 180
    assert bearing_deg(0, 1) == 270


def test_label_regions():
    m = np.zeros((5, 5), bool)
    m[0, 0] = m[1, 1] = True  # diagonal → one 8-connected region
    m[4, 4] = True
    labels, n = label_regions(m)
    assert n == 2 and labels[0, 0] == labels[1, 1] != labels[4, 4]


def test_room_findings(room_cloud):
    res = analyze(room_cloud, OpsParams(cell_m=0.1))
    f = res.findings
    assert f["floor"]["fitted"] and abs(f["floor"]["z_at_origin_m"]) < 0.02 and f["floor"]["tilt_deg"] < 0.5
    # the four rubble boxes come out as obstacles. A box taller than the scanner only shows its near
    # face, so test by footprint overlap, not by centroid.
    boxes = [b for b in ROOM["boxes"] if b[5] < 2.0]
    found = f["obstacles"]
    for bx in boxes:
        hit = [o for o in found if bx[0] - 0.2 < o["x_m"] < bx[3] + 0.2 and bx[1] - 0.2 < o["y_m"] < bx[4] + 0.2]
        assert hit, f"box {bx} not detected"
        assert abs(max(o["height_m"] for o in hit) - bx[5]) < 0.1
    # traversability: walls and box faces blocked, the sampled floor passable
    cls = res.classes
    assert (cls == PASSABLE).sum() > 100
    assert (cls == BLOCKED).sum() > 50
    # cells behind boxes are shadows; coverage is not total
    assert f["coverage"]["coverage_pct"] < 100 and f["coverage"]["shadow_cells"] > 10
    assert f["shadows"] and f["next_station"] is not None
    # a level room has no covered voids or depressions
    assert f["voids"] == [] and f["depressions"] == []


def test_void_detected_under_a_slab(room_cloud):
    # synthesize a slab 0.6 m above the floor over 1 m² with floor returns beneath it
    xs, ys = np.meshgrid(np.arange(-0.5, 0.5, 0.05), np.arange(-0.5, 0.5, 0.05))
    slab = np.c_[xs.ravel() + 1.0, ys.ravel() - 1.0, np.full(xs.size, 0.6)]
    under = np.c_[xs.ravel() + 1.0, ys.ravel() - 1.0, np.full(xs.size, 0.0)]
    cloud = np.concatenate([room_cloud, slab, under]).astype(np.float32)
    f = analyze(cloud, OpsParams(cell_m=0.1)).findings
    assert f["voids"], "slab with open space beneath should be a covered void"
    v = f["voids"][0]
    assert abs(v["x_m"] - 1.0) < 0.2 and abs(v["y_m"] + 1.0) < 0.2 and 0.5 < v["clearance_m"] < 0.7
    assert v["area_m2"] == pytest.approx(1.0, abs=0.25)


def test_axle_stripe_is_blind(tripod_cloud):
    """Every scan plane contains the tilt axle, so the floor along it (|y| < h/tan φmax) is never seen."""
    res = analyze(tripod_cloud, OpsParams(cell_m=0.1), scanner_z_m=1.0, phi_max_deg=60)
    g = res.grid
    iy0 = int((0.0 - g.y0) / g.cell)
    row = res.classes[iy0]  # the row of cells along y = 0
    assert (row == BLIND).sum() > 0.5 * row.size
    assert (row == SHADOW).sum() == 0
    iy = int((1.0 - g.y0) / g.cell)
    assert (res.classes[iy] == PASSABLE).sum() > 0.5 * row.size


def synthetic_grid():
    """20×20 cells of 0.1 m: walls round the edge, a 4×4 block in the middle."""
    w = h = 20
    g = Grid("max", 0.1, 0.0, 0.0, w, h, -1, 3, np.zeros((h, w), np.float32))
    cls = np.full((h, w), PASSABLE, np.uint8)
    cls[0, :] = cls[-1, :] = cls[:, 0] = cls[:, -1] = BLOCKED
    cls[8:12, 8:12] = BLOCKED
    cls[8:12, 12:14] = MARGINAL
    rel = np.zeros((h, w), np.float32)
    rel[8:12, 8:12] = 0.8
    return cls, g, rel


def test_route_on_synthetic_grid():
    cls, g, rel = synthetic_grid()
    r = plan_route(cls, g, (0.15, 0.15), (1.85, 1.85), rel)
    assert r and r["found"] and r["start_snapped_m"] == 0
    for x, y in r["path"]:
        assert not (0.8 <= x < 1.2 and 0.8 <= y < 1.2), "route crosses the block"
    assert r["length_m"] == pytest.approx(np.hypot(1.7, 1.7), abs=0.4)
    # the marginal strip is used only if cheaper overall: a goal on its far side still has a free way round
    assert plan_route(cls, g, (0.15, 0.15), (1.0, 1.0), rel) is None  # goal inside the block
    assert plan_route(cls, g, (0.15, 0.15), (5.0, 5.0), rel) is None  # off grid
    assert plan_route(cls, g, (0.15, 0.15), (0.05, 1.0), rel) is None  # goal in the wall
    # start in the wall snaps to the nearest open cell
    r2 = plan_route(cls, g, (0.05, 0.05), (1.85, 0.15), rel)
    assert r2 and 0 < r2["start_snapped_m"] < 0.2
    # start too far from anything open fails
    assert plan_route(cls, g, (-5.0, -5.0), (1.85, 0.15), rel, snap_m=1.0) is None


def test_room_route_smoke(tripod_cloud):
    """From beside the scanner, at least the far seen floor cells are reachable, and no route steps on a box."""
    res = analyze(tripod_cloud, OpsParams(cell_m=0.1), scanner_z_m=1.0, phi_max_deg=60)
    g = res.grid
    # far floor-level passable cells (the flat top of a wall is "passable" too, but unreachable by design)
    with np.errstate(invalid="ignore"):
        low = (res.classes == PASSABLE) & (res.rel_max < 0.25)
    ys, xs = np.nonzero(low)
    cx = g.x0 + (xs + 0.5) * g.cell
    cy = g.y0 + (ys + 0.5) * g.cell
    far = np.argsort(-np.hypot(cx, cy))[:40]
    found = 0
    for k in far:
        r = plan_route(res.classes, g, (0.0, 0.7), (float(cx[k]), float(cy[k])), res.rel_max)
        if r:
            found += 1
            for x, y in r["path"]:
                for bx in ROOM["boxes"][4:]:
                    assert not (bx[0] < x < bx[3] and bx[1] < y < bx[4]), f"route crosses box {bx}"
    assert found >= 5
    assert plan_route(res.classes, g, (0.0, 0.7), (0.0, 0.0), res.rel_max) is None  # the blind stripe


def test_ops_layers_round_trip(room_cloud):
    res = analyze(room_cloud, OpsParams(cell_m=0.1))
    g = res.grid
    buf = wire.encode_ops_layers(g.w, g.h, g.cell, g.x0, g.y0, res.classes, res.rel_max)
    assert len(buf) == 24 + g.w * g.h * 5
    d = wire.decode_ops_layers(buf)
    np.testing.assert_array_equal(d["classes"], res.classes)
    assert set(np.unique(d["classes"])) <= {UNKNOWN, PASSABLE, MARGINAL, BLOCKED, SHADOW, BLIND}


def test_empty_cloud():
    res = analyze(np.zeros((0, 3), np.float32))
    assert res.findings["points"] == 0 and res.findings["obstacles"] == [] and res.findings["floor"]["fitted"] is False
