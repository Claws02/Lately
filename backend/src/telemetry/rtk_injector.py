"""
RTK GPS simulation injector.
Simulates a base station and computes synthetic RTCM corrections for
all virtual drones in the swarm.
"""

from __future__ import annotations

import asyncio
import logging
import math
import random
import struct
import time
from enum import Enum
from typing import Any

import numpy as np

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# RTK state enumeration
# ---------------------------------------------------------------------------


class RTKState(str, Enum):
    NO_FIX = "NO_FIX"
    FIX_3D = "3D_FIX"
    RTK_FLOAT = "RTK_FLOAT"
    RTK_FIXED = "RTK_FIXED"


# Accuracy in meters for each RTK state
_RTK_ACCURACY: dict[RTKState, float] = {
    RTKState.NO_FIX: 15.0,
    RTKState.FIX_3D: 3.0,
    RTKState.RTK_FLOAT: 0.30,
    RTKState.RTK_FIXED: 0.02,
}

# Noise multiplier per state (applied on top of base accuracy)
_RTK_NOISE: dict[RTKState, float] = {
    RTKState.NO_FIX: 5.0,
    RTKState.FIX_3D: 1.0,
    RTKState.RTK_FLOAT: 0.15,
    RTKState.RTK_FIXED: 0.01,
}


# ---------------------------------------------------------------------------
# Base-station position (survey point in ENU meters, origin = launch site)
# ---------------------------------------------------------------------------

DEFAULT_BASE_POS = np.array([0.0, 0.0, 0.0], dtype=np.float64)


# ---------------------------------------------------------------------------
# RTCM3 message helpers
# ---------------------------------------------------------------------------


def _rtcm3_wrap(payload: bytes, msg_type: int) -> bytes:
    """
    Wrap a raw payload in a minimal RTCM3 frame:
        0xD3  | 6-bit reserved (0) + 10-bit length | payload | 24-bit CRC
    """
    length = len(payload)
    header = struct.pack(">H", length & 0x03FF) + payload
    # Preamble byte
    frame = bytes([0xD3]) + header
    crc = _crc24q(frame)
    crc_bytes = struct.pack(">I", crc)[1:]  # 3 bytes big-endian
    return frame + crc_bytes


def _crc24q(data: bytes) -> int:
    """CRC-24Q algorithm used by RTCM3."""
    POLY = 0x1864CFB
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= POLY
    return crc & 0xFFFFFF


def _encode_rtcm1001(
    drone_id: int,
    ref_station_id: int,
    tow_ms: int,
    correction_m: float,
) -> bytes:
    """
    Produce a minimal synthetic RTCM 1001 (GPS L1-only RTK) message body.
    Real RTCM 1001 is bit-packed; here we produce a simplified stand-in
    that carries the correction magnitude and drone identifier for the
    simulation's internal consumption.
    """
    # Message type 1001 (10 bits), ref station id (12 bits),
    # GPS epoch time (30 bits), sync flag (1 bit), N sat (5 bits)
    # For simulation we pack: msg_type, ref_id, tow, drone_id, correction
    payload = struct.pack(
        ">HHIHf",
        1001,              # message type
        ref_station_id,
        tow_ms & 0xFFFFFFFF,
        drone_id & 0xFFFF,
        float(correction_m),
    )
    return _rtcm3_wrap(payload, 1001)


# ---------------------------------------------------------------------------
# RTKInjector
# ---------------------------------------------------------------------------


class RTKInjector:
    """
    Simulates an RTK base station that computes and injects GPS corrections
    for all drones in the swarm.

    The base station is located at a known survey point (base_pos).
    For each drone we compute:
        - The geometric distance from base to drone
        - A synthetic RTCM correction representing the ionospheric /
          tropospheric error the rover should subtract
        - The correction is injected via the MAVLink router
    """

    def __init__(
        self,
        base_pos: np.ndarray | None = None,
        ref_station_id: int = 1,
        initial_state: RTKState = RTKState.RTK_FIXED,
    ) -> None:
        self._base_pos: np.ndarray = (
            base_pos.copy() if base_pos is not None else DEFAULT_BASE_POS.copy()
        )
        self._ref_station_id = ref_station_id
        self._state: RTKState = initial_state
        self._signal_quality: float = 1.0  # 0.0 – 1.0
        self._injection_count: int = 0
        self._last_inject_time: float = 0.0

    # ------------------------------------------------------------------
    # State management
    # ------------------------------------------------------------------

    def set_signal_quality(self, quality: float) -> None:
        """
        Set simulated signal quality (0.0 = no signal, 1.0 = perfect).
        This degrades RTK state automatically.
        """
        self._signal_quality = max(0.0, min(1.0, quality))
        if quality >= 0.95:
            self._state = RTKState.RTK_FIXED
        elif quality >= 0.70:
            self._state = RTKState.RTK_FLOAT
        elif quality >= 0.30:
            self._state = RTKState.FIX_3D
        else:
            self._state = RTKState.NO_FIX
        log.debug("RTK state -> %s (quality=%.2f)", self._state, quality)

    @property
    def state(self) -> RTKState:
        return self._state

    @property
    def base_pos(self) -> np.ndarray:
        return self._base_pos.copy()

    # ------------------------------------------------------------------
    # Core computation
    # ------------------------------------------------------------------

    def generate_rtcm_correction(
        self,
        drone_pos: np.ndarray,
        base_pos: np.ndarray | None = None,
    ) -> bytes:
        """
        Compute and return a synthetic RTCM3 correction message for one drone.

        The simulated error is proportional to the baseline length
        (longer baseline → more atmospheric divergence).

        Parameters
        ----------
        drone_pos : np.ndarray
            Drone position in ENU meters.
        base_pos : np.ndarray or None
            Base station position; defaults to self._base_pos.

        Returns
        -------
        bytes
            Raw RTCM3-framed correction data.
        """
        bp = base_pos if base_pos is not None else self._base_pos
        baseline_m = float(np.linalg.norm(drone_pos - bp))

        # Ionospheric error scales at ~3 ppm of baseline for RTK_FIXED
        noise = _RTK_NOISE[self._state]
        iono_error_m = (baseline_m * 3e-6 + _RTK_ACCURACY[self._state]) * (
            1.0 + noise * random.gauss(0, 0.1)
        )

        tow_ms = int((time.time() % 604800) * 1000)  # GPS time of week

        # Drone ID is encoded symbolically (not a real RTCM concept)
        drone_id_sym = int(np.sum(np.abs(drone_pos))) & 0xFFFF

        return _encode_rtcm1001(
            drone_id=drone_id_sym,
            ref_station_id=self._ref_station_id,
            tow_ms=tow_ms,
            correction_m=iono_error_m,
        )

    def get_rtk_accuracy(self, noise_level: float = 0.0) -> float:
        """
        Return the expected position accuracy in meters for the current
        RTK state, optionally degraded by additional noise.

        Parameters
        ----------
        noise_level : float
            Extra noise multiplier (0.0 = nominal, 1.0 = doubles the error).

        Returns
        -------
        float
            Accuracy in meters.
        """
        base_acc = _RTK_ACCURACY[self._state]
        degraded = base_acc * (1.0 + max(0.0, noise_level))
        return round(degraded, 4)

    # ------------------------------------------------------------------
    # Bulk injection
    # ------------------------------------------------------------------

    async def inject_to_all(
        self,
        drones: list[dict[str, Any]],
        mavlink_router: Any | None = None,
    ) -> int:
        """
        Broadcast RTK corrections to all drones.

        Parameters
        ----------
        drones : list of dict
            Each dict must have keys 'id', 'x', 'y', 'z' (ENU meters).
        mavlink_router : MAVLinkRouter or None
            If provided, corrections are injected via MAVLink GPS_RTCM_DATA.
            If None, correction data is computed but not actually sent
            (useful for testing).

        Returns
        -------
        int
            Number of drones successfully served.
        """
        served = 0
        for drone in drones:
            try:
                pos = np.array(
                    [drone.get("x", 0.0), drone.get("y", 0.0), drone.get("z", 0.0)],
                    dtype=np.float64,
                )
                rtcm_data = self.generate_rtcm_correction(pos)
                if mavlink_router is not None and mavlink_router.is_connected():
                    success = await mavlink_router.inject_rtcm(rtcm_data)
                    if success:
                        served += 1
                else:
                    served += 1  # count as served in stub mode
            except Exception as exc:
                log.warning("RTK injection failed for drone %s: %s", drone.get("id"), exc)

        self._injection_count += served
        self._last_inject_time = time.time()
        return served

    # ------------------------------------------------------------------
    # Stats
    # ------------------------------------------------------------------

    def stats(self) -> dict[str, Any]:
        """Return current RTK injector statistics."""
        return {
            "state": self._state.value,
            "accuracy_m": self.get_rtk_accuracy(),
            "signal_quality": round(self._signal_quality, 3),
            "total_injections": self._injection_count,
            "last_inject_time": self._last_inject_time,
            "base_pos": self._base_pos.tolist(),
            "ref_station_id": self._ref_station_id,
        }
