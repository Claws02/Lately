"""
MAVLink routing layer for communicating with SITL drones via UDP.
Wraps pymavlink's mavutil with async-friendly helpers.
"""

from __future__ import annotations

import asyncio
import hashlib
import hmac
import logging
import struct
import time
from typing import Any, Callable, Optional

log = logging.getLogger(__name__)

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mavdialect

    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    log.warning("pymavlink not installed – MAVLinkRouter will run in stub mode.")


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MAV_FRAME_LOCAL_NED = 1
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_CMD_DO_SET_MODE = 176

TYPE_MASK_POSITION_ONLY = 0b0000_1111_1100_0111  # ignore velocity, accel, yaw

SIGNING_KEY_BYTES = 32  # 256-bit key for MAVLink2 signing


# ---------------------------------------------------------------------------
# MAVLink2 signing helper
# ---------------------------------------------------------------------------


def _build_signing_key(passphrase: str) -> bytes:
    """Derive a 32-byte MAVLink2 signing key from a passphrase."""
    return hashlib.sha256(passphrase.encode()).digest()


# ---------------------------------------------------------------------------
# Main router class
# ---------------------------------------------------------------------------


class MAVLinkRouter:
    """Async-friendly MAVLink router for swarm SITL communication."""

    def __init__(self, signing_passphrase: Optional[str] = None) -> None:
        self._conn: Any = None  # mavutil connection
        self._host: str = "127.0.0.1"
        self._port: int = 14550
        self._signing_key: Optional[bytes] = (
            _build_signing_key(signing_passphrase) if signing_passphrase else None
        )
        self._connected: bool = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._heartbeat_callbacks: list[Callable] = []

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    async def connect(self, host: str = "127.0.0.1", port: int = 14550) -> bool:
        """
        Connect to a SITL instance via UDP.
        Returns True on success, False on failure.
        """
        self._host = host
        self._port = port
        self._loop = asyncio.get_event_loop()

        if not MAVLINK_AVAILABLE:
            log.warning("pymavlink unavailable – MAVLinkRouter is in stub mode.")
            self._connected = True
            return True

        try:
            connection_string = f"udpin:{host}:{port}"
            self._conn = mavutil.mavlink_connection(
                connection_string,
                source_system=255,
                source_component=0,
            )

            # Enable MAVLink2 signing if a key was provided
            if self._signing_key:
                self._conn.signing.secret_key = self._signing_key
                self._conn.signing.sign_outgoing = True
                self._conn.signing.allow_unsigned = True
                log.info("MAVLink2 signing enabled.")

            # Wait for heartbeat (non-blocking with timeout)
            await asyncio.wait_for(
                asyncio.get_event_loop().run_in_executor(
                    None, self._conn.wait_heartbeat
                ),
                timeout=10.0,
            )
            self._connected = True
            log.info("MAVLink connected to %s:%d", host, port)
            return True

        except asyncio.TimeoutError:
            log.error("MAVLink heartbeat timeout connecting to %s:%d", host, port)
            return False
        except Exception as exc:
            log.error("MAVLink connection error: %s", exc)
            return False

    def is_connected(self) -> bool:
        return self._connected

    async def disconnect(self) -> None:
        if self._conn and MAVLINK_AVAILABLE:
            try:
                self._conn.close()
            except Exception:
                pass
        self._connected = False
        log.info("MAVLink disconnected.")

    # ------------------------------------------------------------------
    # Command sending
    # ------------------------------------------------------------------

    async def send_command_long(
        self,
        target_system: int,
        command: int,
        params: list[float],
        target_component: int = 1,
        confirmation: int = 0,
    ) -> bool:
        """
        Send MAV_CMD_LONG to a target system.
        params: list of up to 7 float parameters.
        """
        if not self._connected:
            log.warning("Cannot send command – not connected.")
            return False

        p = (params + [0.0] * 7)[:7]

        if not MAVLINK_AVAILABLE or self._conn is None:
            log.debug(
                "STUB send_command_long sys=%d cmd=%d params=%s",
                target_system,
                command,
                p,
            )
            return True

        try:
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self._conn.mav.command_long_send(
                    target_system,
                    target_component,
                    command,
                    confirmation,
                    p[0],
                    p[1],
                    p[2],
                    p[3],
                    p[4],
                    p[5],
                    p[6],
                ),
            )
            return True
        except Exception as exc:
            log.error("send_command_long error: %s", exc)
            return False

    async def inject_rtcm(self, data: bytes) -> bool:
        """
        Inject RTCM3 GPS correction data via GPS_RTCM_DATA messages.
        Splits data into 180-byte chunks as per the MAVLink spec.
        """
        if not self._connected:
            return False

        CHUNK_SIZE = 180
        chunks = [data[i : i + CHUNK_SIZE] for i in range(0, len(data), CHUNK_SIZE)]
        sequence = int(time.time() * 10) & 0xFF

        for idx, chunk in enumerate(chunks):
            flags = (len(chunks) > 1) << 0  # fragmented flag
            padded = chunk + b"\x00" * (CHUNK_SIZE - len(chunk))

            if not MAVLINK_AVAILABLE or self._conn is None:
                log.debug("STUB inject_rtcm chunk %d/%d len=%d", idx + 1, len(chunks), len(chunk))
                continue

            try:
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: self._conn.mav.gps_rtcm_data_send(
                        flags, len(chunk), padded
                    ),
                )
            except Exception as exc:
                log.error("inject_rtcm error on chunk %d: %s", idx, exc)
                return False

        return True

    async def set_target_position(
        self,
        drone_id: int,
        x: float,
        y: float,
        z: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
    ) -> bool:
        """
        Send SET_POSITION_TARGET_LOCAL_NED to command a drone to a position.
        x, y, z in meters (NED frame). vx, vy, vz in m/s.
        """
        if not self._connected:
            return False

        # type_mask: ignore acceleration and yaw, use position + velocity
        type_mask = 0b0000_1111_1000_0000  # use pos + vel, ignore accel/yaw

        if not MAVLINK_AVAILABLE or self._conn is None:
            log.debug(
                "STUB set_target_position drone=%d pos=(%.2f,%.2f,%.2f)",
                drone_id,
                x,
                y,
                z,
            )
            return True

        try:
            time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self._conn.mav.set_position_target_local_ned_send(
                    time_boot_ms,
                    drone_id,  # target_system
                    1,          # target_component (autopilot)
                    MAV_FRAME_LOCAL_NED,
                    type_mask,
                    x, y, z,   # position
                    vx, vy, vz, # velocity
                    0, 0, 0,   # acceleration (ignored)
                    0, 0,       # yaw, yaw_rate (ignored)
                ),
            )
            return True
        except Exception as exc:
            log.error("set_target_position error: %s", exc)
            return False

    async def arm(self, drone_id: int) -> bool:
        """Arm motors on target drone."""
        log.info("Arming drone %d", drone_id)
        return await self.send_command_long(
            target_system=drone_id,
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            params=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    async def disarm(self, drone_id: int) -> bool:
        """Disarm motors on target drone."""
        log.info("Disarming drone %d", drone_id)
        return await self.send_command_long(
            target_system=drone_id,
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            params=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    async def set_mode(self, drone_id: int, mode: int) -> bool:
        """
        Set flight mode. Mode values depend on autopilot firmware.
        For ArduCopter: 0=STABILIZE, 4=GUIDED, 5=LOITER, 6=RTL, 9=LAND.
        """
        log.info("Setting drone %d mode to %d", drone_id, mode)
        return await self.send_command_long(
            target_system=drone_id,
            command=MAV_CMD_DO_SET_MODE,
            params=[1.0, float(mode), 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    async def takeoff(self, drone_id: int, altitude_m: float = 10.0) -> bool:
        """Command a drone to take off to a given altitude."""
        log.info("Commanding drone %d to take off to %.1f m", drone_id, altitude_m)
        return await self.send_command_long(
            target_system=drone_id,
            command=MAV_CMD_NAV_TAKEOFF,
            params=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, altitude_m],
        )

    async def land(self, drone_id: int) -> bool:
        """Command a drone to land at current position."""
        log.info("Commanding drone %d to land", drone_id)
        return await self.send_command_long(
            target_system=drone_id,
            command=MAV_CMD_NAV_LAND,
            params=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    # ------------------------------------------------------------------
    # Heartbeat listener
    # ------------------------------------------------------------------

    async def listen_heartbeats(self, callback: Callable[[dict], None]) -> None:
        """
        Async generator loop that listens for MAVLink HEARTBEAT messages
        and invokes the callback with parsed packet data.
        Runs until the connection is closed.
        """
        if not MAVLINK_AVAILABLE or self._conn is None:
            log.debug("STUB listen_heartbeats – yielding synthetic heartbeats.")
            while self._connected:
                await asyncio.sleep(1.0)
                await callback(
                    {
                        "type": "HEARTBEAT",
                        "system_id": 1,
                        "component_id": 1,
                        "autopilot": 3,
                        "base_mode": 0,
                        "custom_mode": 0,
                        "system_status": 4,
                        "mavlink_version": 3,
                    }
                )
            return

        loop = asyncio.get_event_loop()
        while self._connected:
            try:
                msg = await loop.run_in_executor(
                    None,
                    lambda: self._conn.recv_match(
                        type="HEARTBEAT", blocking=True, timeout=2.0
                    ),
                )
                if msg is not None:
                    packet = {
                        "type": "HEARTBEAT",
                        "system_id": msg.get_srcSystem(),
                        "component_id": msg.get_srcComponent(),
                        "autopilot": msg.autopilot,
                        "base_mode": msg.base_mode,
                        "custom_mode": msg.custom_mode,
                        "system_status": msg.system_status,
                        "mavlink_version": msg.mavlink_version,
                    }
                    if asyncio.iscoroutinefunction(callback):
                        await callback(packet)
                    else:
                        callback(packet)
            except asyncio.CancelledError:
                break
            except Exception as exc:
                log.error("Heartbeat listener error: %s", exc)
                await asyncio.sleep(0.5)
