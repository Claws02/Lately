"""
3D geofencing engine for drone swarm operations.

Supports cylindrical and spherical zones with SAFE / WARN / HARD classification.
Provides per-drone and vectorised swarm checks, violation tracking, and
repulsion vectors to push drones back inside safe zones.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional

import numpy as np

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------


class ZoneType(str, Enum):
    SAFE = "SAFE"    # Normal operating area
    WARN = "WARN"    # Caution zone – alert but allow flight
    HARD = "HARD"    # Absolute boundary – triggers failsafe


class ZoneShape(str, Enum):
    SPHERE = "SPHERE"
    CYLINDER = "CYLINDER"


class GeofenceStatus(str, Enum):
    SAFE = "SAFE"
    WARNING = "WARNING"
    VIOLATION = "VIOLATION"


# ---------------------------------------------------------------------------
# Data-classes
# ---------------------------------------------------------------------------


@dataclass
class GeofenceZone:
    """
    Definition of a single geofence zone.

    For SPHERE:   enforced as a sphere of `radius_m` around `center`.
    For CYLINDER: enforced as an infinite-height cylinder of `radius_m`
                  around the (center.x, center.y) axis, with altitude
                  limits [min_alt, max_alt].
    """

    zone_id: int
    center_x: float
    center_y: float
    center_z: float
    radius_m: float
    min_alt: float = 0.0
    max_alt: float = 400.0
    zone_type: ZoneType = ZoneType.SAFE
    shape: ZoneShape = ZoneShape.CYLINDER
    label: str = ""

    @property
    def center(self) -> np.ndarray:
        return np.array([self.center_x, self.center_y, self.center_z])


@dataclass
class DroneGeofenceResult:
    drone_id: int
    status: GeofenceStatus
    violating_zone_ids: list[int] = field(default_factory=list)
    warning_zone_ids: list[int] = field(default_factory=list)
    nearest_boundary_m: float = float("inf")


# ---------------------------------------------------------------------------
# GeofenceEngine
# ---------------------------------------------------------------------------


class GeofenceEngine:
    """
    Evaluates geofence constraints for individual drones and the whole swarm.
    """

    def __init__(self) -> None:
        self._zones: dict[int, GeofenceZone] = {}
        self._violations: dict[int, set[int]] = {}       # drone_id -> zone_ids
        self._violation_callbacks: list[Callable] = []
        self._next_zone_id: int = 1

    # ------------------------------------------------------------------
    # Zone management
    # ------------------------------------------------------------------

    def add_zone(self, zone: GeofenceZone) -> int:
        """
        Register a geofence zone.  If zone_id is 0 a new ID is assigned.

        Returns the zone_id.
        """
        if zone.zone_id == 0:
            zone.zone_id = self._next_zone_id
            self._next_zone_id += 1
        self._zones[zone.zone_id] = zone
        log.info(
            "Added geofence zone %d (%s %s) r=%.1fm",
            zone.zone_id,
            zone.zone_type.value,
            zone.shape.value,
            zone.radius_m,
        )
        return zone.zone_id

    def remove_zone(self, zone_id: int) -> bool:
        """Remove a zone by ID.  Returns True if it existed."""
        if zone_id in self._zones:
            del self._zones[zone_id]
            log.info("Removed geofence zone %d", zone_id)
            return True
        return False

    def get_zone(self, zone_id: int) -> Optional[GeofenceZone]:
        return self._zones.get(zone_id)

    def list_zones(self) -> list[GeofenceZone]:
        return list(self._zones.values())

    # ------------------------------------------------------------------
    # Violation callbacks
    # ------------------------------------------------------------------

    def on_violation(self, callback: Callable[[int, list[int]], None]) -> None:
        """
        Register a callback invoked on geofence violation.

        Signature: callback(drone_id: int, violating_zone_ids: list[int])
        """
        self._violation_callbacks.append(callback)

    # ------------------------------------------------------------------
    # Per-drone check
    # ------------------------------------------------------------------

    def check_drone(
        self,
        drone_id: int,
        x: float,
        y: float,
        z: float,
    ) -> GeofenceStatus:
        """
        Check a single drone against all registered zones.

        Returns the most severe status (VIOLATION > WARNING > SAFE).
        """
        result = self._evaluate(drone_id, x, y, z)

        # Track violations and fire callbacks
        if result.violating_zone_ids:
            prev_violations = self._violations.get(drone_id, set())
            new_violations = set(result.violating_zone_ids)
            if new_violations != prev_violations:
                self._violations[drone_id] = new_violations
                for cb in self._violation_callbacks:
                    try:
                        cb(drone_id, list(new_violations))
                    except Exception as exc:
                        log.warning("Geofence callback error: %s", exc)
        else:
            self._violations.pop(drone_id, None)

        return result.status

    def check_drone_detailed(
        self,
        drone_id: int,
        x: float,
        y: float,
        z: float,
    ) -> DroneGeofenceResult:
        """Like check_drone but returns the full DroneGeofenceResult."""
        return self._evaluate(drone_id, x, y, z)

    # ------------------------------------------------------------------
    # Swarm check (vectorised)
    # ------------------------------------------------------------------

    def check_swarm(
        self,
        positions: np.ndarray,
    ) -> list[GeofenceStatus]:
        """
        Vectorised geofence check for all drones.

        Parameters
        ----------
        positions : np.ndarray  shape (N, 3)
            Drone positions as rows [x, y, z].

        Returns
        -------
        list[GeofenceStatus]  length N
        """
        positions = np.asarray(positions, dtype=np.float64)
        statuses: list[GeofenceStatus] = []
        for i, pos in enumerate(positions):
            s = self.check_drone(i, float(pos[0]), float(pos[1]), float(pos[2]))
            statuses.append(s)
        return statuses

    # ------------------------------------------------------------------
    # Violation query
    # ------------------------------------------------------------------

    def get_violations(self) -> list[int]:
        """Return list of drone IDs currently in violation."""
        return [
            did for did, zones in self._violations.items() if zones
        ]

    def get_all_drone_statuses(self) -> dict[int, set[int]]:
        """Return raw violations dict (drone_id -> violated zone IDs)."""
        return dict(self._violations)

    # ------------------------------------------------------------------
    # Repulsion vector
    # ------------------------------------------------------------------

    def get_repulsion_vector(
        self,
        x: float,
        y: float,
        z: float,
        strength: float = 1.0,
    ) -> np.ndarray:
        """
        Compute a repulsion force vector that pushes a drone away from
        boundary violations and back toward the safe interior.

        For SAFE zones: pushes inward if the drone is outside the boundary.
        For HARD/WARN zones (exclusion zones): pushes outward if inside.

        Parameters
        ----------
        x, y, z : float
            Drone position in meters.
        strength : float
            Force magnitude scaling factor.

        Returns
        -------
        np.ndarray  shape (3,)
            Repulsion direction and magnitude (m/s² equivalent).
        """
        pos = np.array([x, y, z], dtype=np.float64)
        force = np.zeros(3, dtype=np.float64)

        for zone in self._zones.values():
            center = zone.center
            to_center = center - pos
            dist = np.linalg.norm(to_center)

            if dist < 1e-6:
                # At zone center; push upward by default
                force += np.array([0.0, 0.0, 0.1]) * strength
                continue

            unit = to_center / dist

            if zone.zone_type == ZoneType.SAFE:
                # SAFE zone: repel toward center if drone is outside
                if zone.shape == ZoneShape.SPHERE:
                    if dist > zone.radius_m:
                        overshoot = dist - zone.radius_m
                        force += unit * overshoot * strength
                elif zone.shape == ZoneShape.CYLINDER:
                    horiz = np.array([to_center[0], to_center[1], 0.0])
                    horiz_dist = np.linalg.norm(horiz)
                    if horiz_dist > zone.radius_m:
                        overshoot = horiz_dist - zone.radius_m
                        horiz_unit = horiz / horiz_dist if horiz_dist > 1e-6 else horiz
                        force += horiz_unit * overshoot * strength
                    # Altitude bounds
                    if z < zone.min_alt:
                        force[2] += (zone.min_alt - z) * strength
                    elif z > zone.max_alt:
                        force[2] -= (z - zone.max_alt) * strength

            else:
                # HARD / WARN exclusion zone: repel away from center if inside
                if zone.shape == ZoneShape.SPHERE:
                    if dist < zone.radius_m:
                        penetration = zone.radius_m - dist
                        force -= unit * penetration * strength
                elif zone.shape == ZoneShape.CYLINDER:
                    horiz = np.array([to_center[0], to_center[1], 0.0])
                    horiz_dist = np.linalg.norm(horiz)
                    if horiz_dist < zone.radius_m and zone.min_alt <= z <= zone.max_alt:
                        penetration = zone.radius_m - horiz_dist
                        if horiz_dist > 1e-6:
                            horiz_unit = horiz / horiz_dist
                        else:
                            horiz_unit = np.array([1.0, 0.0, 0.0])
                        force -= horiz_unit * penetration * strength

        return force

    # ------------------------------------------------------------------
    # Internal evaluation
    # ------------------------------------------------------------------

    def _evaluate(
        self,
        drone_id: int,
        x: float,
        y: float,
        z: float,
    ) -> DroneGeofenceResult:
        """Core evaluation logic used by both public check methods."""
        result = DroneGeofenceResult(drone_id=drone_id, status=GeofenceStatus.SAFE)

        for zone in self._zones.values():
            distance_to_boundary = self._distance_to_boundary(x, y, z, zone)

            # Negative distance = inside boundary (for exclusion), outside safe
            if zone.zone_type == ZoneType.SAFE:
                # Inside safe zone is good; outside is a violation
                outside = self._is_outside_safe(x, y, z, zone)
                if outside:
                    if zone.zone_type == ZoneType.HARD:
                        result.violating_zone_ids.append(zone.zone_id)
                    else:
                        result.warning_zone_ids.append(zone.zone_id)
            elif zone.zone_type in (ZoneType.HARD, ZoneType.WARN):
                # Inside exclusion zone is a violation
                inside = self._is_inside(x, y, z, zone)
                if inside:
                    if zone.zone_type == ZoneType.HARD:
                        result.violating_zone_ids.append(zone.zone_id)
                    else:
                        result.warning_zone_ids.append(zone.zone_id)

            # Track nearest boundary
            if abs(distance_to_boundary) < result.nearest_boundary_m:
                result.nearest_boundary_m = abs(distance_to_boundary)

        if result.violating_zone_ids:
            result.status = GeofenceStatus.VIOLATION
        elif result.warning_zone_ids:
            result.status = GeofenceStatus.WARNING
        else:
            result.status = GeofenceStatus.SAFE

        return result

    @staticmethod
    def _is_inside(x: float, y: float, z: float, zone: GeofenceZone) -> bool:
        """Return True if (x,y,z) is inside the zone volume."""
        if zone.shape == ZoneShape.SPHERE:
            dx = x - zone.center_x
            dy = y - zone.center_y
            dz = z - zone.center_z
            return (dx * dx + dy * dy + dz * dz) <= zone.radius_m ** 2
        else:  # CYLINDER
            dx = x - zone.center_x
            dy = y - zone.center_y
            horiz_sq = dx * dx + dy * dy
            return horiz_sq <= zone.radius_m ** 2 and zone.min_alt <= z <= zone.max_alt

    @staticmethod
    def _is_outside_safe(x: float, y: float, z: float, zone: GeofenceZone) -> bool:
        """Return True if (x,y,z) is outside the safe zone volume."""
        return not GeofenceEngine._is_inside(x, y, z, zone)

    @staticmethod
    def _distance_to_boundary(
        x: float, y: float, z: float, zone: GeofenceZone
    ) -> float:
        """
        Signed distance to zone boundary.
        Positive = outside sphere/cylinder, negative = inside.
        """
        if zone.shape == ZoneShape.SPHERE:
            dx = x - zone.center_x
            dy = y - zone.center_y
            dz = z - zone.center_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            return dist - zone.radius_m
        else:  # CYLINDER
            dx = x - zone.center_x
            dy = y - zone.center_y
            horiz = math.sqrt(dx * dx + dy * dy)
            return horiz - zone.radius_m
