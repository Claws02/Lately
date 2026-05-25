"""
FlockWave-inspired swarm protocol message definitions.
Provides Pydantic models for all inter-system communication.
"""

from __future__ import annotations

import time
import uuid
from enum import Enum
from typing import Any, Optional

from pydantic import BaseModel, Field


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------


class DroneStatus(str, Enum):
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    FLYING = "FLYING"
    LANDING = "LANDING"
    GROUNDED = "GROUNDED"
    FAILSAFE = "FAILSAFE"
    ERROR = "ERROR"


class LogLevel(str, Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


# ---------------------------------------------------------------------------
# Base message
# ---------------------------------------------------------------------------


class FlockwaveMessage(BaseModel):
    """Base class for all FlockWave protocol messages."""

    msgId: str = Field(default_factory=lambda: str(uuid.uuid4()))
    type: str
    timestamp: float = Field(default_factory=time.time)

    model_config = {"populate_by_name": True}

    def to_json(self) -> dict[str, Any]:
        return self.model_dump()


# ---------------------------------------------------------------------------
# Sub-models
# ---------------------------------------------------------------------------


class Position(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Velocity(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


class Attitude(BaseModel):
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


class Color(BaseModel):
    r: int = Field(default=255, ge=0, le=255)
    g: int = Field(default=255, ge=0, le=255)
    b: int = Field(default=255, ge=0, le=255)


class DroneInfo(BaseModel):
    """Complete state snapshot for a single drone."""

    id: int
    position: Position = Field(default_factory=Position)
    velocity: Velocity = Field(default_factory=Velocity)
    attitude: Attitude = Field(default_factory=Attitude)
    color: Color = Field(default_factory=Color)
    battery: float = Field(default=1.0, ge=0.0, le=1.0)  # 0.0–1.0
    status: DroneStatus = DroneStatus.IDLE


# ---------------------------------------------------------------------------
# Concrete message types
# ---------------------------------------------------------------------------


class TelemetryBroadcast(FlockwaveMessage):
    """TEL – bulk telemetry snapshot sent to all connected clients."""

    type: str = "TEL"
    drones: list[DroneInfo] = Field(default_factory=list)


class CommandMessage(FlockwaveMessage):
    """CMD – ground-control command directed at the fleet or a subset."""

    type: str = "CMD"
    action: str
    params: dict[str, Any] = Field(default_factory=dict)


class ShowUploadMessage(FlockwaveMessage):
    """SHOW_UPLOAD – full show data sent from the uploader to the simulator."""

    type: str = "SHOW_UPLOAD"
    # Each element is a list of {t, x, y, z} dicts
    trajectories: list[list[dict[str, float]]] = Field(default_factory=list)
    # Each element is a list of {t, r, g, b} dicts
    colors: list[list[dict[str, Any]]] = Field(default_factory=list)
    duration: float = 0.0


class FleetStatusMessage(FlockwaveMessage):
    """FLEET_STATUS – high-level fleet health summary."""

    type: str = "FLEET_STATUS"
    total: int = 0
    active: int = 0
    failsafe: int = 0
    grounded: int = 0
    flying: int = 0


class LogMessage(FlockwaveMessage):
    """LOG – structured log entry relayed to connected clients."""

    type: str = "LOG"
    level: LogLevel = LogLevel.INFO
    drone_id: Optional[int] = None
    subsystem: str = "SYSTEM"
    message: str = ""
    log_timestamp: float = Field(default_factory=time.time)


# ---------------------------------------------------------------------------
# Factory helpers
# ---------------------------------------------------------------------------


def make_telemetry(drone_infos: list[DroneInfo]) -> TelemetryBroadcast:
    """Build a telemetry broadcast message from a list of DroneInfo objects."""
    return TelemetryBroadcast(drones=drone_infos)


def make_fleet_status(
    drones: list[DroneInfo],
) -> FleetStatusMessage:
    """Compute fleet status counts from current drone states."""
    total = len(drones)
    active = sum(
        1 for d in drones if d.status not in (DroneStatus.GROUNDED, DroneStatus.IDLE)
    )
    failsafe = sum(1 for d in drones if d.status == DroneStatus.FAILSAFE)
    grounded = sum(1 for d in drones if d.status == DroneStatus.GROUNDED)
    flying = sum(1 for d in drones if d.status == DroneStatus.FLYING)
    return FleetStatusMessage(
        total=total,
        active=active,
        failsafe=failsafe,
        grounded=grounded,
        flying=flying,
    )


def make_log(
    level: LogLevel,
    message: str,
    *,
    drone_id: Optional[int] = None,
    subsystem: str = "SYSTEM",
) -> LogMessage:
    """Create a structured log message."""
    return LogMessage(
        level=level,
        message=message,
        drone_id=drone_id,
        subsystem=subsystem,
    )
