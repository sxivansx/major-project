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
from .acoustic import calibrate as calibrate_array
from .audio_pipeline import AudioError
from .audio_wire import KIND_AUDIO_PREVIEW
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
        # live previews are superseded by the next one within 50–100 ms: a slow client skips them
        droppable = isinstance(msg, bytes) and msg[:1] in (bytes([wire.KIND_SCAN]), bytes([KIND_AUDIO_PREVIEW]))
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
        if any(x in path for x in ("Bluetooth-Incoming-Port", "debug-console")):
            continue  # Mac-internal ports, never a LiDAR or servo
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


class RouteRequest(BaseModel):
    from_xy: list[float]
    to_xy: list[float]
    cell: float | None = None
    max_step: float | None = None
    max_slope: float | None = None


class AudioConnect(BaseModel):
    source: Literal["udp", "tcp", "sim"] = "udp"
    port: str | None = None


class AudioEmit(BaseModel):
    bearing_deg: float
    elevation_deg: float = 0.0
    snr_db: float = 20.0
    kind: Literal["knock", "voice"] = "knock"


class CalibrationClick(BaseModel):
    bearing_deg: float  # where the click really was: base bearing, clockwise from base +x
    elevation_deg: float = 0.0
    event_id: int | None = None  # take the raw tdoas from a stored event…
    tdoa_us: dict[str, float] | None = None  # …or give them directly


class AudioCalibrate(BaseModel):
    clicks: list[CalibrationClick]
    apply: bool = False  # write yaw + offsets to the config (and calibrated=true if the order is confirmed)
    refine_positions: bool = False  # ≥ 6 claps: also fit the mic positions (applied too when apply=true)


class SpeechToggle(BaseModel):
    enabled: bool


class AIRequest(BaseModel):
    cell: float | None = None


class AskRequest(BaseModel):
    question: str
    cell: float | None = None


def create_app(
    config_path: Path = CONFIG_PATH,
    sessions_dir: Path = SESSIONS_DIR,
    web_dist: Path = WEB_DIST,
    bridge: Path = BRIDGE,
    autostart_sim: bool = False,
    sim_hz: float = 10.0,
    ai_client=None,
    speech_key: str | None = None,
) -> FastAPI:
    from .ai import AIClient

    bc = Broadcaster()
    config = load_config(config_path)
    rig = Rig(config, SessionStore(sessions_dir), bc.publish, bridge_path=bridge, sim_hz=sim_hz)
    ai = ai_client or AIClient()
    from .speech import SpeechListener

    rig.audio.speech = SpeechListener(ai, rig.audio._send, rig.log, rig.audio.locate_window, key=speech_key)

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
            rig.audio.speech.close()
            await asyncio.to_thread(rig.shutdown)

    app = FastAPI(title="major-project", lifespan=lifespan)
    app.state.rig = rig
    app.state.broadcaster = bc

    @app.exception_handler(RigError)
    async def _rig_error(_: Request, e: RigError):
        return JSONResponse({"error": str(e)}, status_code=400)

    @app.exception_handler(AudioError)
    async def _audio_error(_: Request, e: AudioError):
        return JSONResponse({"error": str(e)}, status_code=e.status_code)

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

    # ---- acoustic layer (docs/api.md §5.3) ---------------------------------------------------------
    @app.post(f"{api}/audio/connect")
    def audio_connect(body: AudioConnect):
        rig.audio.connect(body.source, body.port)  # never blocks: the TCP link comes up in its own thread
        rig.publish_status()
        return rig.status()

    @app.post(f"{api}/audio/disconnect")
    async def audio_disconnect():
        await asyncio.to_thread(rig.audio.disconnect)
        rig.publish_status()
        return rig.status()

    @app.get(f"{api}/audio/events")
    def audio_events(limit: int = Query(50, ge=0, le=200)):
        return rig.audio.events(limit)

    @app.get(f"{api}/audio/sources")
    def audio_sources():
        return rig.audio.sources()

    @app.post(f"{api}/audio/clear", status_code=204)
    def audio_clear():
        rig.audio.clear()
        return Response(status_code=204)

    # ---- spoken calls for help (§5.7) --------------------------------------------------------------
    @app.get(f"{api}/audio/speech")
    def speech_events(limit: int = Query(50, ge=0, le=100)):
        return {"status": rig.audio.speech.status(), "events": rig.audio.speech.events(limit)}

    @app.post(f"{api}/audio/speech")
    def speech_toggle(body: SpeechToggle):
        rig.audio.speech.set_enabled(body.enabled)
        rig.publish_status()
        return rig.audio.speech.status()

    @app.post(f"{api}/audio/speech/clear", status_code=204)
    def speech_clear():
        rig.audio.speech.clear()
        return Response(status_code=204)

    @app.post(f"{api}/audio/sim/emit")
    async def audio_sim_emit(body: AudioEmit):
        return await asyncio.to_thread(rig.audio.emit_sim, body.bearing_deg, body.elevation_deg, body.snr_db, body.kind)

    @app.post(f"{api}/audio/calibrate")
    def audio_calibrate(body: AudioCalibrate):
        """Clicks from known bearings → array yaw, per-channel capture offsets, channel-order check."""
        by_id = {e["id"]: e for e in rig.audio.events(200)}
        clicks = []
        for k in body.clicks:
            if k.tdoa_us is not None:
                t = k.tdoa_us
            elif k.event_id is not None:
                ev = by_id.get(k.event_id)
                if ev is None or len(ev.get("tdoa_us") or {}) != 6:
                    raise HTTPException(400, f"no stored event {k.event_id} with six tdoas")
                t = ev["tdoa_us"]
            else:
                raise HTTPException(400, "each click needs event_id or tdoa_us")
            clicks.append({"bearing_deg": k.bearing_deg, "elevation_deg": k.elevation_deg, "tdoa_us": t})
        a = config.audio
        try:
            res = calibrate_array(clicks, a.mics_mm, a.speed_of_sound, rig.audio.rate or 48000, refine_positions=body.refine_positions)
        except (ValueError, KeyError) as e:
            raise HTTPException(400, str(e)) from None
        res["applied"] = False
        if body.apply:
            a.array_yaw_deg = res["array_yaw_deg"]
            a.channel_offsets_us = res["channel_offsets_us"]
            if "refined" in res:
                a.mics_mm = res["refined"]["mics_mm"]
            # yaw and offsets are worth keeping either way; "calibrated" is only earned when the
            # click test also confirmed the channel order with a small misfit
            a.calibrated = bool(res["ok"])
            save_config(config, config_path)
            rig.audio.config_changed()
            rig.publish_status()
            res["applied"] = True
            res["calibrated"] = a.calibrated
        return res

    # ---- operator products -------------------------------------------------------------------
    from .analysis import OpsParams, analyze, plan_route

    def ops_params(cell: float | None, max_step: float | None, max_slope: float | None) -> OpsParams:
        p = OpsParams()
        if cell is not None:
            if cell <= 0:
                raise HTTPException(400, "cell must be > 0")
            p.cell_m = cell
        if max_step is not None:
            p.max_step_m = max_step
        if max_slope is not None:
            p.max_slope_deg = max_slope
        return p

    def run_analysis(p: OpsParams):
        return analyze(rig.live_points(), p, scanner_z_m=rig.scanner_z_m(), phi_max_deg=rig.phi_max_deg)

    @app.get(f"{api}/ops/findings")
    async def ops_findings(cell: float | None = None, max_step: float | None = None, max_slope: float | None = None):
        res = await asyncio.to_thread(run_analysis, ops_params(cell, max_step, max_slope))
        job = rig.status()["job"]
        res.findings["scan"] = {
            "in_progress": bool(job and job["state"] == "running"),
            "step": job["step"] if job else 0,
            "n_steps": job["n_steps"] if job else 0,
            "session": rig.session_loaded,
        }
        return res.findings

    @app.get(f"{api}/ops/layers")
    async def ops_layers(cell: float | None = None, max_step: float | None = None, max_slope: float | None = None):
        res = await asyncio.to_thread(run_analysis, ops_params(cell, max_step, max_slope))
        g = res.grid
        return Response(wire.encode_ops_layers(g.w, g.h, g.cell, g.x0, g.y0, res.classes, res.rel_max), media_type="application/octet-stream")

    @app.post(f"{api}/ops/route")
    async def ops_route(body: RouteRequest):
        if len(body.from_xy) != 2 or len(body.to_xy) != 2:
            raise HTTPException(400, "from_xy and to_xy are [x, y] in metres")
        res = await asyncio.to_thread(run_analysis, ops_params(body.cell, body.max_step, body.max_slope))
        p = OpsParams()
        route = plan_route(res.classes, res.grid, tuple(body.from_xy), tuple(body.to_xy), res.rel_max, snap_m=p.snap_m, snap_goal_m=p.snap_goal_m)
        return route or {"found": False, "reason": "no passable route (goal blocked, unknown, or unreachable)"}

    # ---- AI layer -----------------------------------------------------------------------------
    from . import ai as ai_mod

    @app.exception_handler(ai_mod.AIError)
    async def _ai_error(_: Request, e: ai_mod.AIError):
        return JSONResponse({"error": str(e)}, status_code=503 if "unavailable" in str(e) else 502)

    @app.get(f"{api}/ai/status")
    def ai_status():
        return ai.status()

    @app.post(f"{api}/ai/triage")
    async def ai_triage(body: AIRequest):
        res = await asyncio.to_thread(run_analysis, ops_params(body.cell, None, None))
        return await asyncio.to_thread(ai_mod.triage, ai, res.findings)

    @app.post(f"{api}/ai/sitrep")
    async def ai_sitrep(body: AIRequest):
        res = await asyncio.to_thread(run_analysis, ops_params(body.cell, None, None))
        tri = None
        if ai.decisions_available:
            try:
                tri = await asyncio.to_thread(ai_mod.triage, ai, res.findings)
            except ai_mod.AIError as e:
                log.warning("triage before sitrep failed: %s", e)
        return await asyncio.to_thread(ai_mod.sitrep, ai, res.findings, tri)

    @app.post(f"{api}/ai/ask")
    async def ai_ask(body: AskRequest):
        if not body.question.strip():
            raise HTTPException(400, "question is empty")
        p = ops_params(body.cell, None, None)
        res = await asyncio.to_thread(run_analysis, p)

        def route_to(x: float, y: float):
            return plan_route(res.classes, res.grid, (0.0, 0.0), (x, y), res.rel_max, snap_m=p.snap_m, snap_goal_m=max(p.snap_goal_m, 1.0))

        return await asyncio.to_thread(ai_mod.ask, ai, body.question, res.findings, route_to)

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
        # a client that predates the audio block must not reset the measured geometry to defaults
        if "audio" in body.model_fields_set:
            config.audio = body.audio
            rig.audio.config_changed()
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
