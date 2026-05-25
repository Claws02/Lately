"""
NextGen Swarm Simulator - Ground Control Server
Async FastAPI backend handling fleet management, telemetry routing, and show execution.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import sys
import tempfile
import time
from typing import Any, AsyncGenerator, Optional

# ---------------------------------------------------------------------------
# Path bootstrap – allow imports from backend/ and project root
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_BACKEND_DIR = os.path.abspath(os.path.join(_HERE, ".."))      # backend/
_PROJECT_DIR = os.path.abspath(os.path.join(_HERE, "..", "..")) # Lately/
for _p in (_BACKEND_DIR, _PROJECT_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fastapi import (
    FastAPI,
    File,
    HTTPException,
    UploadFile,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

# Internal imports
from src.telemetry.websocket_hub import WebSocketHub
from src.choreo_parser.skyc_reader import (
    ColorKeyframe,
    DroneTrajectory,
    LightCue,
    ShowFile,
    SkycReader,
    Waypoint,
)
from src.choreo_parser.csv_ingest import CSVIngest

from mds_logging.formatter import LogEntry, LogLevel
from mds_logging.session import LogSession
from mds_logging.watcher import LogWatcher

# Simulation engine
from simulation.fast_tensor_sim.sim_core import DroneSwarmSimulator

# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s - %(message)s",
)
log = logging.getLogger("swarm.server")

# ---------------------------------------------------------------------------
# Application constants
# ---------------------------------------------------------------------------

VERSION = "1.0.0"
TELEMETRY_HZ = 60
TELEMETRY_INTERVAL = 1.0 / TELEMETRY_HZ

# ---------------------------------------------------------------------------
# Global singletons
# ---------------------------------------------------------------------------

app = FastAPI(
    title="NextGen Swarm Simulator API",
    version=VERSION,
    description="Ground control server for the drone show simulator.",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173", "*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ws_hub = WebSocketHub()
log_watcher = LogWatcher(buffer_size=2000)
log_session = LogSession()

# Shared state
_simulator: Optional[DroneSwarmSimulator] = None
_sim_task: Optional[asyncio.Task] = None
_tel_task: Optional[asyncio.Task] = None
_loaded_show: Optional[ShowFile] = None
_sim_running: bool = False


# ---------------------------------------------------------------------------
# Pydantic request models
# ---------------------------------------------------------------------------


class DroneCountRequest(BaseModel):
    count: int


# ---------------------------------------------------------------------------
# Demo show generator
# ---------------------------------------------------------------------------


class DemoShowGenerator:
    """
    Generates a built-in demo show:
        Phase 1 (0-5 s)   : 100 drones at takeoff positions (ground)
        Phase 2 (5-15 s)  : ascend and form a circle at 15 m
        Phase 3 (15-25 s) : transition to a Fibonacci sphere
        Phase 4 (25-35 s) : spiral ascent with RGB cycling
        Phase 5 (35-40 s) : descend and land
    """

    N_DRONES = 100
    DURATION = 40.0

    def generate(self) -> ShowFile:
        trajectories: list[DroneTrajectory] = []
        light_cues: list[LightCue] = []

        for i in range(self.N_DRONES):
            traj, lc = self._drone_show(i)
            trajectories.append(traj)
            light_cues.append(lc)

        show = ShowFile(
            version="demo-1.0",
            title="Demo Show - Circle to Sphere to Spiral",
            duration=self.DURATION,
            drone_count=self.N_DRONES,
            settings={"safe_altitude_m": 40.0, "speed_ms": 3.0},
            trajectories=trajectories,
            light_cues=light_cues,
        )
        log.info("Demo show generated: %d drones, %.0fs", self.N_DRONES, self.DURATION)
        return show

    def _drone_show(self, i: int) -> tuple[DroneTrajectory, LightCue]:
        n = self.N_DRONES
        angle = (2.0 * math.pi * i) / n
        hue_base = angle / (2.0 * math.pi)  # 0..1

        # Ground start position (circle of radius 30 m)
        gx = 30.0 * math.cos(angle)
        gy = 30.0 * math.sin(angle)

        # Circle at 15 m altitude
        cx, cy, cz = gx, gy, 15.0

        # Sphere position (Fibonacci sphere)
        phi = math.acos(1.0 - 2.0 * (i + 0.5) / n)
        theta = math.pi * (1.0 + math.sqrt(5.0)) * i
        sx = 20.0 * math.sin(phi) * math.cos(theta)
        sy = 20.0 * math.sin(phi) * math.sin(theta)
        sz = 20.0 * math.cos(phi) + 20.0  # centre sphere at 20 m

        # Spiral top
        sp_radius = 25.0 + 8.0 * math.sin(angle * 3)
        sp_angle = angle + math.pi
        sptx = sp_radius * math.cos(sp_angle)
        spty = sp_radius * math.sin(sp_angle)
        sptz = sz + 10.0

        waypoints: list[Waypoint] = [
            Waypoint(t=0.0,  x=gx,   y=gy,   z=0.0),
            Waypoint(t=5.0,  x=cx,   y=cy,   z=cz),
            Waypoint(t=15.0, x=sx,   y=sy,   z=sz),
            Waypoint(t=25.0, x=sptx, y=spty, z=sptz),
            Waypoint(t=35.0, x=gx,   y=gy,   z=5.0),
            Waypoint(t=40.0, x=gx,   y=gy,   z=0.0),
        ]

        # RGB colour cycling: each drone starts at a different hue
        def hue_to_rgb(h: float) -> tuple[int, int, int]:
            h = h % 1.0
            r, g, b = 0.0, 0.0, 0.0
            i6 = int(h * 6)
            f = h * 6 - i6
            p, q, tv = 0.0, 1.0 - f, f
            sector = i6 % 6
            if sector == 0:   r, g, b = 1.0, tv, p
            elif sector == 1: r, g, b = q,  1.0, p
            elif sector == 2: r, g, b = p,  1.0, tv
            elif sector == 3: r, g, b = p,  q,   1.0
            elif sector == 4: r, g, b = tv, p,   1.0
            elif sector == 5: r, g, b = 1.0, p,  q
            return int(r * 255), int(g * 255), int(b * 255)

        keyframes: list[ColorKeyframe] = []
        for t_step in range(0, 41, 2):
            t = float(t_step)
            hue = (hue_base + t / 40.0) % 1.0
            rr, gg, bb = hue_to_rgb(hue)
            keyframes.append(ColorKeyframe(t=t, r=rr, g=gg, b=bb))

        return (
            DroneTrajectory(drone_id=i, waypoints=waypoints),
            LightCue(drone_id=i, keyframes=keyframes),
        )


# ---------------------------------------------------------------------------
# Background tasks
# ---------------------------------------------------------------------------


async def _sim_runner() -> None:
    """Run the simulator physics loop as a background asyncio task."""
    if _simulator is not None:
        try:
            await _simulator.run()
        except asyncio.CancelledError:
            _simulator.stop()
            raise


async def _telemetry_loop() -> None:
    """Broadcast telemetry at TELEMETRY_HZ to all WebSocket clients."""
    while True:
        try:
            if _simulator is not None and ws_hub.get_client_count() > 0:
                states = _simulator.get_telemetry()
                await ws_hub.broadcast_telemetry(states)
            await asyncio.sleep(TELEMETRY_INTERVAL)
        except asyncio.CancelledError:
            break
        except Exception as exc:
            log.warning("Telemetry loop error: %s", exc)
            await asyncio.sleep(0.1)


async def _publish_log(
    level: LogLevel,
    message: str,
    subsystem: str = "SERVER",
    drone_id: Optional[int] = None,
) -> None:
    """Publish to both the log watcher (SSE) and the active log session."""
    entry = LogEntry(level=level, message=message, subsystem=subsystem, drone_id=drone_id)
    await log_watcher.publish(entry)
    if log_session.is_active:
        await log_session.log(level, message, subsystem=subsystem, drone_id=drone_id)


# ---------------------------------------------------------------------------
# Application lifecycle
# ---------------------------------------------------------------------------


@app.on_event("startup")
async def _on_startup() -> None:
    global _simulator, _tel_task, _sim_task

    log.info("NextGen Swarm Simulator v%s starting up", VERSION)

    # Create the simulator with a default drone count
    _simulator = DroneSwarmSimulator(drone_count=100)

    # Start physics loop
    _sim_task = asyncio.create_task(_sim_runner())

    # Start telemetry broadcast loop
    _tel_task = asyncio.create_task(_telemetry_loop())

    # Open log session
    log_dir = os.path.join(os.path.dirname(__file__), "..", "..", "logs")
    session_id = f"session_{int(time.time())}"
    await log_session.start(session_id, log_dir, drone_count=100, show_name="default")

    await _publish_log(LogLevel.INFO, f"Server started - version {VERSION}")

    # Auto-generate demo show if DEMO_MODE env is set
    if os.environ.get("DEMO_MODE"):
        log.info("DEMO_MODE detected - generating demo show.")
        await _load_demo_show()


@app.on_event("shutdown")
async def _on_shutdown() -> None:
    global _sim_task, _tel_task

    log.info("Shutting down...")

    for task in (_sim_task, _tel_task):
        if task and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    await log_session.stop()
    await log_watcher.shutdown()
    log.info("Shutdown complete.")


# ---------------------------------------------------------------------------
# Helper – load demo show
# ---------------------------------------------------------------------------


async def _load_demo_show() -> ShowFile:
    global _loaded_show
    gen = DemoShowGenerator()
    show = gen.generate()
    _loaded_show = show
    if _simulator is not None:
        _simulator.load_show(show)
    await _publish_log(
        LogLevel.INFO,
        f"Demo show loaded: '{show.title}', {show.drone_count} drones, {show.duration:.0f}s",
    )
    return show


# ---------------------------------------------------------------------------
# REST endpoints
# ---------------------------------------------------------------------------


@app.get("/")
async def health_check() -> dict[str, Any]:
    """Health check - returns version and basic status."""
    return {
        "status": "ok",
        "version": VERSION,
        "service": "nextgen-swarm-simulator",
        "timestamp": time.time(),
    }


@app.get("/api/status")
async def get_status() -> dict[str, Any]:
    """Fleet status: drone count, simulation state, show loaded."""
    sim_state = None
    drone_count = 0
    if _simulator is not None:
        sim_state_obj = _simulator.get_simulator_state()
        drone_count = sim_state_obj.drone_count
        sim_state = {
            "running": sim_state_obj.running,
            "show_loaded": sim_state_obj.show_loaded,
            "show_time": sim_state_obj.show_time,
            "show_duration": sim_state_obj.show_duration,
            "active_drones": sim_state_obj.active_drones,
            "failsafe_drones": sim_state_obj.failsafe_drones,
            "sim_hz": sim_state_obj.sim_hz,
            "real_time_factor": sim_state_obj.real_time_factor,
        }

    return {
        "drone_count": drone_count,
        "sim_running": _sim_running,
        "show_loaded": _loaded_show is not None,
        "ws_clients": ws_hub.get_client_count(),
        "sim_state": sim_state,
        "timestamp": time.time(),
    }


@app.post("/api/upload_show")
async def upload_show(file: UploadFile = File(...)) -> dict[str, Any]:
    """
    Accept a multipart file upload (.skyc or .csv), parse it, and load it
    into the simulator.
    """
    global _loaded_show

    filename = file.filename or "upload"
    suffix = os.path.splitext(filename)[1].lower()

    if suffix not in (".skyc", ".csv"):
        raise HTTPException(
            status_code=400,
            detail=f"Unsupported file type '{suffix}'. Use .skyc or .csv.",
        )

    with tempfile.NamedTemporaryFile(delete=False, suffix=suffix) as tmp:
        contents = await file.read()
        tmp.write(contents)
        tmp_path = tmp.name

    try:
        if suffix == ".skyc":
            reader = SkycReader()
            show = reader.load(tmp_path)
            warnings = reader.validate(show)
        else:
            ingest = CSVIngest()
            show = ingest.load(tmp_path)
            warnings = []

        _loaded_show = show
        if _simulator is not None:
            _simulator.load_show(show)

        await _publish_log(
            LogLevel.INFO,
            f"Show uploaded: '{show.title}' ({show.drone_count} drones, {show.duration:.1f}s)",
        )
        return {
            "success": True,
            "title": show.title,
            "drone_count": show.drone_count,
            "duration": show.duration,
            "warnings": warnings,
        }
    except Exception as exc:
        log.exception("Show upload failed")
        raise HTTPException(status_code=422, detail=str(exc))
    finally:
        os.unlink(tmp_path)


@app.post("/api/generate_demo")
async def generate_demo() -> dict[str, Any]:
    """Generate a built-in demo show and load it into the simulator."""
    show = await _load_demo_show()
    return {
        "success": True,
        "title": show.title,
        "drone_count": show.drone_count,
        "duration": show.duration,
    }


@app.post("/api/start_show")
async def start_show() -> dict[str, Any]:
    """Begin simulation execution / show playback."""
    global _sim_running
    if _simulator is None:
        raise HTTPException(status_code=503, detail="Simulator not initialised.")
    if _loaded_show is None:
        raise HTTPException(
            status_code=400,
            detail="No show loaded. Upload or generate a show first.",
        )
    if _sim_running:
        return {"success": True, "message": "Simulation already running."}

    _sim_running = True
    started = _simulator.start_show()
    await _publish_log(LogLevel.INFO, "Simulation started." if started else "Simulation started (drones not fully ready).")
    return {"success": True, "message": "Simulation started.", "show_started": started}


@app.post("/api/stop_show")
async def stop_show() -> dict[str, Any]:
    """Stop show playback; drones hover in place."""
    global _sim_running
    if _simulator is None:
        raise HTTPException(status_code=503, detail="Simulator not initialised.")
    _sim_running = False
    _simulator.stop_show()
    await _publish_log(LogLevel.INFO, "Simulation stopped.")
    return {"success": True, "message": "Simulation stopped."}


@app.post("/api/arm_all")
async def arm_all() -> dict[str, Any]:
    """Arm all virtual drones (sets IDLE/GROUNDED -> ARMING -> TAKEOFF)."""
    if _simulator is None:
        raise HTTPException(status_code=503, detail="Simulator not initialised.")
    _simulator.arm_all()
    count = _simulator._drone_count
    await _publish_log(LogLevel.INFO, f"Armed {count} drones.")
    return {"success": True, "armed": count}


@app.post("/api/land_all")
async def land_all() -> dict[str, Any]:
    """Command all airborne drones to land."""
    if _simulator is None:
        raise HTTPException(status_code=503, detail="Simulator not initialised.")
    _simulator.land_all()
    count = _simulator._drone_count
    await _publish_log(LogLevel.INFO, f"Landing command sent to {count} drones.")
    return {"success": True, "landing": count}


@app.post("/api/set_drone_count")
async def set_drone_count(request: DroneCountRequest) -> dict[str, Any]:
    """Resize the simulated fleet."""
    if request.count < 1 or request.count > 5000:
        raise HTTPException(status_code=400, detail="count must be between 1 and 5000.")
    if _simulator is None:
        raise HTTPException(status_code=503, detail="Simulator not initialised.")
    _simulator.set_drone_count(request.count)
    await _publish_log(LogLevel.INFO, f"Drone count set to {request.count}.")
    return {"success": True, "drone_count": request.count}


@app.get("/api/show_info")
async def show_info() -> dict[str, Any]:
    """Return metadata about the currently loaded show."""
    if _loaded_show is None:
        return {"loaded": False}
    show = _loaded_show
    return {
        "loaded": True,
        "title": show.title,
        "version": show.version,
        "duration": show.duration,
        "drone_count": show.drone_count,
        "settings": show.settings,
    }


# ---------------------------------------------------------------------------
# SSE log streaming
# ---------------------------------------------------------------------------


@app.get("/api/logs/stream")
async def stream_logs() -> StreamingResponse:
    """
    Server-Sent Events endpoint that streams JSONL log entries in real time.
    """

    async def _generator() -> AsyncGenerator[str, None]:
        # First send recent history
        for entry in log_watcher.get_recent(50):
            yield f"data: {json.dumps(entry.to_dict(), default=str)}\n\n"
        # Then stream live
        async for entry in log_watcher.stream():
            yield f"data: {json.dumps(entry.to_dict(), default=str)}\n\n"

    return StreamingResponse(
        _generator(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "X-Accel-Buffering": "no",
        },
    )


# ---------------------------------------------------------------------------
# WebSocket endpoint
# ---------------------------------------------------------------------------


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    """
    Main telemetry WebSocket.

    Accepts incoming JSON command messages from the frontend.
    Receives telemetry broadcasts from the WebSocketHub at 60 Hz.
    """
    await ws.accept()
    await ws_hub.register(ws)

    # Send welcome
    await ws.send_json(
        {
            "type": "WELCOME",
            "version": VERSION,
            "drone_count": _simulator._drone_count if _simulator else 0,
            "timestamp": time.time(),
        }
    )

    try:
        while True:
            try:
                raw = await asyncio.wait_for(ws.receive_text(), timeout=0.1)
            except asyncio.TimeoutError:
                continue
            except WebSocketDisconnect:
                break

            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                await ws.send_json({"type": "ERROR", "message": "Invalid JSON"})
                continue

            cmd = (msg.get("type") or msg.get("action") or "").upper()
            await _handle_ws_command(ws, cmd, msg)

    except WebSocketDisconnect:
        pass
    except Exception as exc:
        log.warning("WebSocket error: %s", exc)
    finally:
        await ws_hub.unregister(ws)


async def _handle_ws_command(ws: WebSocket, cmd: str, msg: dict) -> None:
    """Dispatch incoming WebSocket command messages."""
    global _sim_running

    if cmd == "START":
        if _simulator and _loaded_show:
            _sim_running = True
            started = _simulator.start_show()
            await ws.send_json({"type": "ACK", "action": "START", "show_started": started})
            await _publish_log(LogLevel.INFO, "Show started via WebSocket.")
        else:
            await ws.send_json({"type": "ERROR", "message": "No show loaded."})

    elif cmd == "STOP":
        if _simulator:
            _sim_running = False
            _simulator.stop_show()
            await ws.send_json({"type": "ACK", "action": "STOP"})
            await _publish_log(LogLevel.INFO, "Show stopped via WebSocket.")

    elif cmd == "ARM":
        if _simulator:
            _simulator.arm_all()
            await ws.send_json({"type": "ACK", "action": "ARM"})

    elif cmd == "LAND":
        if _simulator:
            _simulator.land_all()
            await ws.send_json({"type": "ACK", "action": "LAND"})

    elif cmd == "SET_DRONE_COUNT":
        count = int(msg.get("count", 100))
        if _simulator and 1 <= count <= 5000:
            _simulator.set_drone_count(count)
            await ws.send_json({"type": "ACK", "action": "SET_DRONE_COUNT", "count": count})
        else:
            await ws.send_json({"type": "ERROR", "message": "Invalid count."})

    elif cmd == "STATUS":
        sim_state = _simulator.get_simulator_state() if _simulator else None
        await ws.send_json({
            "type": "STATUS",
            "sim_running": _sim_running,
            "drone_count": sim_state.drone_count if sim_state else 0,
            "show_loaded": _loaded_show is not None,
            "ws_clients": ws_hub.get_client_count(),
        })

    elif cmd == "PING":
        await ws.send_json({"type": "PONG", "timestamp": time.time()})

    else:
        await ws.send_json({"type": "ERROR", "message": f"Unknown command: {cmd}"})
