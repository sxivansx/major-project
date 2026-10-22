"""FastAPI server: REST + WebSocket per docs/api.md §3, serves web/dist when built."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from serial.tools import list_ports

from . import wire
from .config import Config, load_config, save_config
from .grid import LAYERS
from .rig import JobParams, Rig, RigError
from .session import SessionStore, valid_id

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "core" / "config.json"
SESSIONS_DIR = ROOT / "sessions"
WEB_DIST = ROOT / "web" / "dist"
BRIDGE = ROOT / "driver" / "build" / "s2bridge"


class Broadcaster:
    """Thread-safe fan-out to WebSocket clients. Scan frames are dropped for slow clients."""

    def __init__(self):
        self.loop: asyncio.AbstractEventLoop | None = None
        self.clients: set[asyncio.Queue] = set()

    def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue()
        self.clients.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        self.clients.discard(q)

    def publish(self, msg: str | bytes) -> None:
        if self.loop is None or self.loop.is_closed():
            return
        self.loop.call_soon_threadsafe(self._fanout, msg)

    def _fanout(self, msg: str | bytes) -> None:
        droppable = isinstance(msg, bytes) and msg[:1] == bytes([wire.KIND_SCAN])
        for q in list(self.clients):
            if droppable and q.qsize() > 8:
                continue
            q.put_nowait(msg)


def list_serial_ports() -> list[dict]:
    seen = {}
    for p in list_ports.comports():
        path = p.device
        if sys.platform == "darwin" and path.startswith("/dev/tty."):
            path = "/dev/cu." + path[len("/dev/tty.") :]
        seen[path] = {"path": path, "description": p.description or "", "hwid": p.hwid or ""}
    return [seen[k] for k in sorted(seen)]


class LidarConnect(BaseModel):
    source: Literal["s2", "sim"] = "s2"
    port: str | None = None
    baud: int = 1_000_000


class ServoConnect(BaseModel):
    port: str


class ServoMove(BaseModel):
    servo_deg: float | None = None
    phi_deg: float | None = None


class RawLine(BaseModel):
    line: str


def create_app(
    config_path: Path = CONFIG_PATH,
    sessions_dir: Path = SESSIONS_DIR,
    web_dist: Path = WEB_DIST,
    bridge: Path = BRIDGE,
    autostart_sim: bool = False,
    sim_hz: float = 10.0,
) -> FastAPI:
    bc = Broadcaster()
    config = load_config(config_path)
    rig = Rig(config, SessionStore(sessions_dir), bc.publish, bridge_path=bridge, sim_hz=sim_hz)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        bc.loop = asyncio.get_running_loop()
        if autostart_sim:
            rig.connect_servo("sim")
            rig.connect_lidar("sim")

        async def ticker():
            while True:
                await asyncio.sleep(1.0)
                try:
                    rig.check_alarms()
                except Exception:  # noqa: BLE001
                    log.exception("alarm check failed")
                rig.publish_status()

        task = asyncio.create_task(ticker())
        try:
            yield
        finally:
            task.cancel()
            await asyncio.to_thread(rig.shutdown)

    app = FastAPI(title="major-project", lifespan=lifespan)
    app.state.rig = rig
    app.state.broadcaster = bc

    @app.exception_handler(RigError)
    async def _rig_error(_: Request, e: RigError):
        return JSONResponse({"error": str(e)}, status_code=400)

    api = "/api"

    @app.get(f"{api}/status")
    def status():
        return rig.status()

    @app.get(f"{api}/ports")
    def ports():
        return list_serial_ports()

    @app.post(f"{api}/lidar/connect")
    async def lidar_connect(body: LidarConnect):
        await asyncio.to_thread(rig.connect_lidar, body.source, body.port, body.baud)
        return rig.status()

    @app.post(f"{api}/lidar/disconnect")
    async def lidar_disconnect():
        await asyncio.to_thread(rig.disconnect_lidar)
        return rig.status()

    @app.post(f"{api}/servo/connect")
    async def servo_connect(body: ServoConnect):
        await asyncio.to_thread(rig.connect_servo, body.port)
        return rig.status()

    @app.post(f"{api}/servo/disconnect")
    async def servo_disconnect():
        await asyncio.to_thread(rig.disconnect_servo)
        return rig.status()

    @app.post(f"{api}/servo/move")
    async def servo_move(body: ServoMove):
        await asyncio.to_thread(rig.move_servo, body.servo_deg, body.phi_deg)
        return rig.status()

    @app.post(f"{api}/servo/home")
    async def servo_home():
        await asyncio.to_thread(rig.home_servo)
        return rig.status()

    @app.post(f"{api}/servo/raw")
    async def servo_raw(body: RawLine):
        reply = await asyncio.to_thread(rig.servo_raw, body.line)
        return {"reply": reply}

    @app.post(f"{api}/job/start")
    def job_start(params: JobParams):
        return {"job_id": rig.start_job(params)}

    @app.post(f"{api}/job/stop")
    async def job_stop():
        await asyncio.to_thread(rig.stop_job)
        return rig.status()

    @app.post(f"{api}/live/clear")
    def live_clear():
        rig.clear_live()
        return rig.status()

    @app.get(f"{api}/scan/snapshot.csv")
    async def scan_snapshot(rotations: int = 1):
        if not 1 <= rotations <= 20:
            raise HTTPException(400, "rotations must be 1..20")
        body = await asyncio.to_thread(rig.snapshot_csv, rotations)
        name = f"snapshot-{rotations}rot.csv"
        return Response(body, media_type="text/csv", headers={"Content-Disposition": f'attachment; filename="{name}"'})

    @app.get(f"{api}/metrics")
    async def metrics(cell: float | None = None, z_min: float | None = None, z_max: float | None = None):
        if cell is not None and cell <= 0:
            raise HTTPException(400, "cell must be > 0")
        return await asyncio.to_thread(rig.metrics, cell, z_min, z_max)

    def grid_args(cell, layer, z_min, z_max):
        if layer is not None and layer not in LAYERS:
            raise HTTPException(400, f"layer must be one of {LAYERS}")
        if cell is not None and cell <= 0:
            raise HTTPException(400, "cell must be > 0")
        return cell, layer, z_min, z_max

    @app.get(f"{api}/grid")
    async def grid(cell: float | None = None, layer: str | None = None, z_min: float | None = None, z_max: float | None = None):
        g = await asyncio.to_thread(rig.grid, *grid_args(cell, layer, z_min, z_max))
        return Response(wire.encode_grid(g), media_type="application/octet-stream")

    @app.get(f"{api}/sessions")
    def sessions():
        return rig.sessions.list()

    def _sid(sid: str) -> str:
        if not valid_id(sid):
            raise HTTPException(400, "bad session id")
        return sid

    @app.get(f"{api}/sessions/{{sid}}")
    def session_meta(sid: str):
        try:
            return rig.sessions.get(_sid(sid))
        except KeyError:
            raise HTTPException(404, "no such session") from None

    @app.get(f"{api}/sessions/{{sid}}/points")
    async def session_points(sid: str):
        try:
            xyz = await asyncio.to_thread(rig.sessions.points, _sid(sid))
        except KeyError:
            raise HTTPException(404, "no such session") from None
        return Response(wire.encode_session_points(xyz), media_type="application/octet-stream")

    @app.get(f"{api}/sessions/{{sid}}/grid")
    async def session_grid(sid: str, cell: float | None = None, layer: str | None = None, z_min: float | None = None, z_max: float | None = None):
        cell, layer, z_min, z_max = grid_args(cell, layer, z_min, z_max)
        g = config.grid
        try:
            xyz = await asyncio.to_thread(rig.sessions.points, _sid(sid))
        except KeyError:
            raise HTTPException(404, "no such session") from None
        from .grid import build_grid

        gr = await asyncio.to_thread(
            build_grid, xyz, cell or g.cell_m, g.z_min_m if z_min is None else z_min,
            g.z_max_m if z_max is None else z_max, layer or g.layer,
        )
        return Response(wire.encode_grid(gr), media_type="application/octet-stream")

    @app.get(f"{api}/sessions/{{sid}}/export.ply")
    async def session_ply(sid: str):
        try:
            xyz = await asyncio.to_thread(rig.sessions.points, _sid(sid))
        except KeyError:
            raise HTTPException(404, "no such session") from None
        body = await asyncio.to_thread(rig.sessions.ply, xyz)
        return Response(body, media_type="application/octet-stream", headers={"Content-Disposition": f'attachment; filename="{sid}.ply"'})

    @app.get(f"{api}/sessions/{{sid}}/export.csv")
    async def session_csv(sid: str):
        try:
            body = await asyncio.to_thread(rig.sessions.csv, _sid(sid))
        except KeyError:
            raise HTTPException(404, "no such session") from None
        return Response(body, media_type="text/csv", headers={"Content-Disposition": f'attachment; filename="{sid}.csv"'})

    @app.delete(f"{api}/sessions/{{sid}}", status_code=204)
    def session_delete(sid: str):
        try:
            rig.sessions.delete(_sid(sid))
        except KeyError:
            raise HTTPException(404, "no such session") from None
        if rig.session_loaded == sid:
            rig.clear_live()
        return Response(status_code=204)

    @app.post(f"{api}/sessions/{{sid}}/load")
    async def session_load(sid: str):
        await asyncio.to_thread(rig.load_session, _sid(sid))
        return rig.status()

    @app.get(f"{api}/config")
    def config_get():
        return config.model_dump()

    @app.put(f"{api}/config")
    def config_put(body: Config):
        config.mount, config.grid, config.lidar = body.mount, body.grid, body.lidar
        save_config(config, config_path)
        rig.publish_status()
        return config.model_dump()

    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        q = bc.subscribe()
        closed = asyncio.Event()

        async def reader():
            try:
                while True:
                    await websocket.receive_text()
            except (WebSocketDisconnect, RuntimeError):
                closed.set()

        reader_task = asyncio.create_task(reader())
        try:
            await websocket.send_text(json.dumps({"type": "status", **rig.status()}))
            while not closed.is_set():
                getter = asyncio.create_task(q.get())
                waiter = asyncio.create_task(closed.wait())
                done, _ = await asyncio.wait({getter, waiter}, return_when=asyncio.FIRST_COMPLETED)
                if getter in done:
                    waiter.cancel()
                    msg = getter.result()
                    if isinstance(msg, bytes):
                        await websocket.send_bytes(msg)
                    else:
                        await websocket.send_text(msg)
                else:
                    getter.cancel()
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            reader_task.cancel()
            bc.unsubscribe(q)

    if web_dist.exists():
        app.mount("/", StaticFiles(directory=str(web_dist), html=True), name="web")
    else:

        @app.get("/")
        def index():
            return {"name": "major-project", "docs": "/docs", "note": "build web/ for the dashboard"}

    return app


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(prog="obslam-server")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--sim", action="store_true", help="connect the simulated lidar and servo at start")
    ap.add_argument("--log", default="info")
    args = ap.parse_args(argv)
    logging.basicConfig(level=args.log.upper(), format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    import uvicorn

    uvicorn.run(create_app(autostart_sim=args.sim), host=args.host, port=args.port, log_level=args.log)


if __name__ == "__main__":
    main()
