"""Binary encodings for the WebSocket and session endpoints. docs/api.md §3 is the contract."""

from __future__ import annotations

import struct

import numpy as np

from .grid import LAYER_ID, LAYERS, Grid

KIND_SCAN, KIND_POINTS, KIND_GRID, KIND_OPS = 1, 2, 3, 4
KIND_ROOM = 6  # 5 is the audio preview (audio_wire.py)

_SCAN_HDR = struct.Struct("<BxxxIdfI")  # kind, seq, t, phi, n  → 24 bytes
_POINTS_HDR = struct.Struct("<BxxxIIfI")  # kind, job_id, step, phi, n → 20 bytes
_GRID_HDR = struct.Struct("<BBHIIfffff")  # kind, layer, pad, w, h, cell, x0, y0, z_min, z_max → 32 bytes


def encode_scan(seq: int, t: float, phi: float, angle, rng, quality) -> bytes:
    a = np.ascontiguousarray(angle, dtype="<f4")
    r = np.ascontiguousarray(rng, dtype="<f4")
    q = np.ascontiguousarray(quality, dtype="u1")
    n = a.shape[0]
    return _SCAN_HDR.pack(KIND_SCAN, seq, t, phi, n) + a.tobytes() + r.tobytes() + q.tobytes()


def decode_scan(buf: bytes) -> dict:
    kind, seq, t, phi, n = _SCAN_HDR.unpack_from(buf)
    assert kind == KIND_SCAN
    o = _SCAN_HDR.size
    a = np.frombuffer(buf, "<f4", n, o)
    r = np.frombuffer(buf, "<f4", n, o + 4 * n)
    q = np.frombuffer(buf, "u1", n, o + 8 * n)
    return {"seq": seq, "t": t, "phi": phi, "angle": a, "range": r, "quality": q}


def encode_points(job_id: int, step: int, phi: float, xyz) -> bytes:
    p = np.ascontiguousarray(xyz, dtype="<f4").reshape(-1, 3)
    return _POINTS_HDR.pack(KIND_POINTS, job_id, step, phi, p.shape[0]) + p.tobytes()


def decode_points(buf: bytes) -> dict:
    kind, job_id, step, phi, n = _POINTS_HDR.unpack_from(buf)
    assert kind == KIND_POINTS
    xyz = np.frombuffer(buf, "<f4", 3 * n, _POINTS_HDR.size).reshape(n, 3)
    return {"job_id": job_id, "step": step, "phi": phi, "xyz": xyz}


def encode_grid(g: Grid) -> bytes:
    data = np.ascontiguousarray(g.data, dtype="<f4")
    hdr = _GRID_HDR.pack(KIND_GRID, LAYER_ID[g.layer], 0, g.w, g.h, g.cell, g.x0, g.y0, g.z_min, g.z_max)
    return hdr + data.tobytes()


def decode_grid(buf: bytes) -> Grid:
    kind, layer, _, w, h, cell, x0, y0, z_min, z_max = _GRID_HDR.unpack_from(buf)
    assert kind == KIND_GRID
    data = np.frombuffer(buf, "<f4", w * h, _GRID_HDR.size).reshape(h, w)
    return Grid(LAYERS[layer], cell, x0, y0, w, h, z_min, z_max, data)


_OPS_HDR = struct.Struct("<BBHIIfff")  # kind=4, pad, pad, w, h, cell, x0, y0 → 24 bytes


def encode_ops_layers(w: int, h: int, cell: float, x0: float, y0: float, classes, rel_height) -> bytes:
    """Operator layers: u8 class per cell (0 unknown, 1 passable, 2 marginal, 3 blocked, 4 shadow, 5 blind)
    followed by f32 height above the fitted floor (NaN empty). Both row-major, row = y."""
    c = np.ascontiguousarray(classes, dtype="u1").reshape(h * w)
    r = np.ascontiguousarray(rel_height, dtype="<f4").reshape(h * w)
    return _OPS_HDR.pack(KIND_OPS, 0, 0, w, h, cell, x0, y0) + c.tobytes() + r.tobytes()


_ROOM_HDR = struct.Struct("<BBHIIfff")  # kind=6, pad, pad, w, h, cell, x0, y0 → 24 bytes


def encode_room(classes, cell: float, x0: float, y0: float) -> bytes:
    """Room map (§3.5): u8 per cell, 0 unknown, 1 free, 2 occupied; row-major, row = y."""
    c = np.ascontiguousarray(classes, dtype="u1")
    h, w = c.shape
    return _ROOM_HDR.pack(KIND_ROOM, 0, 0, w, h, cell, x0, y0) + c.tobytes()


def decode_room(buf: bytes) -> dict:
    kind, _, _, w, h, cell, x0, y0 = _ROOM_HDR.unpack_from(buf)
    assert kind == KIND_ROOM
    cls = np.frombuffer(buf, "u1", w * h, _ROOM_HDR.size).reshape(h, w)
    return {"w": w, "h": h, "cell": cell, "x0": x0, "y0": y0, "classes": cls}


def decode_ops_layers(buf: bytes) -> dict:
    kind, _, _, w, h, cell, x0, y0 = _OPS_HDR.unpack_from(buf)
    assert kind == KIND_OPS
    o = _OPS_HDR.size
    classes = np.frombuffer(buf, "u1", w * h, o).reshape(h, w)
    rel = np.frombuffer(buf, "<f4", w * h, o + w * h).reshape(h, w)
    return {"w": w, "h": h, "cell": cell, "x0": x0, "y0": y0, "classes": classes, "rel": rel}


def encode_session_points(xyz) -> bytes:
    p = np.ascontiguousarray(xyz, dtype="<f4").reshape(-1, 3)
    return struct.pack("<I", p.shape[0]) + p.tobytes()
