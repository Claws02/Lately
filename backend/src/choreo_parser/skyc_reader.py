"""
Parser for the .skyc (Skybrush Compiled) show format.

A .skyc file is a ZIP archive containing:
  show.json                   – show-level metadata
  trajectories/drone_XXX.json – per-drone {t, x, y, z} waypoint lists
  lights/drone_XXX.json       – per-drone {t, r, g, b} colour keyframe lists

Trajectories are interpolated with Catmull-Rom splines for smooth motion.
"""

from __future__ import annotations

import json
import logging
import math
import zipfile
from dataclasses import dataclass, field
from typing import Any

import numpy as np

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data-classes
# ---------------------------------------------------------------------------


@dataclass
class Waypoint:
    t: float
    x: float
    y: float
    z: float


@dataclass
class ColorKeyframe:
    t: float
    r: int
    g: int
    b: int


@dataclass
class DroneTrajectory:
    drone_id: int
    waypoints: list[Waypoint] = field(default_factory=list)

    def duration(self) -> float:
        if not self.waypoints:
            return 0.0
        return self.waypoints[-1].t


@dataclass
class LightCue:
    drone_id: int
    keyframes: list[ColorKeyframe] = field(default_factory=list)


@dataclass
class ShowFile:
    """Top-level container for a parsed drone show."""

    # Metadata from show.json
    version: str = "1.0"
    title: str = "Untitled Show"
    duration: float = 0.0
    drone_count: int = 0
    settings: dict[str, Any] = field(default_factory=dict)

    # Per-drone data
    trajectories: list[DroneTrajectory] = field(default_factory=list)
    light_cues: list[LightCue] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Catmull-Rom spline helpers
# ---------------------------------------------------------------------------


def _catmull_rom_point(
    p0: np.ndarray,
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
    t: float,
    alpha: float = 0.5,
) -> np.ndarray:
    """
    Evaluate a single point on a centripetal Catmull-Rom spline.

    Parameters
    ----------
    p0, p1, p2, p3 : np.ndarray  shape (3,)
        Four consecutive control points.
    t : float
        Parameter in [0, 1] between p1 and p2.
    alpha : float
        0 = uniform, 0.5 = centripetal (default), 1 = chordal.

    Returns
    -------
    np.ndarray  shape (3,)
    """

    def _knot(pi: np.ndarray, pj: np.ndarray) -> float:
        d = np.linalg.norm(pj - pi)
        return d ** alpha

    t0 = 0.0
    t1 = t0 + _knot(p0, p1)
    t2 = t1 + _knot(p1, p2)
    t3 = t2 + _knot(p2, p3)

    # Guard against degenerate knot intervals
    dt1 = t1 - t0 if abs(t1 - t0) > 1e-10 else 1e-10
    dt2 = t2 - t1 if abs(t2 - t1) > 1e-10 else 1e-10
    dt3 = t3 - t2 if abs(t3 - t2) > 1e-10 else 1e-10

    tc = t1 + t * dt2  # global param in [t1, t2]

    A1 = (t1 - tc) / dt1 * p0 + (tc - t0) / dt1 * p1
    A2 = (t2 - tc) / dt2 * p1 + (tc - t1) / dt2 * p2
    A3 = (t3 - tc) / dt3 * p2 + (tc - t2) / dt3 * p3

    dt12 = t2 - t0 if abs(t2 - t0) > 1e-10 else 1e-10
    dt23 = t3 - t1 if abs(t3 - t1) > 1e-10 else 1e-10

    B1 = (t2 - tc) / dt12 * A1 + (tc - t0) / dt12 * A2
    B2 = (t3 - tc) / dt23 * A2 + (tc - t1) / dt23 * A3

    C = (t2 - tc) / dt2 * B1 + (tc - t1) / dt2 * B2
    return C


# ---------------------------------------------------------------------------
# SkycReader
# ---------------------------------------------------------------------------


class SkycReader:
    """
    Loads and parses .skyc show files.  Also provides trajectory and colour
    interpolation utilities for the playback engine.
    """

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------

    def load(self, path: str) -> ShowFile:
        """
        Load a .skyc ZIP file and return a populated ShowFile.

        Parameters
        ----------
        path : str
            File system path to the .skyc file.

        Returns
        -------
        ShowFile
        """
        show = ShowFile()

        with zipfile.ZipFile(path, "r") as zf:
            names = zf.namelist()

            # --- show.json ---
            if "show.json" in names:
                with zf.open("show.json") as fh:
                    meta = json.load(fh)
                show.version = str(meta.get("version", "1.0"))
                show.title = str(meta.get("title", "Untitled Show"))
                show.duration = float(meta.get("duration", 0.0))
                show.drone_count = int(meta.get("drone_count", 0))
                show.settings = meta.get("settings", {})

            # --- trajectories/drone_XXX.json ---
            traj_names = sorted(
                n for n in names if n.startswith("trajectories/drone_") and n.endswith(".json")
            )
            for traj_name in traj_names:
                drone_id = self._extract_id(traj_name)
                with zf.open(traj_name) as fh:
                    raw = json.load(fh)
                waypoints = [
                    Waypoint(
                        t=float(wp["t"]),
                        x=float(wp["x"]),
                        y=float(wp["y"]),
                        z=float(wp["z"]),
                    )
                    for wp in raw
                ]
                show.trajectories.append(DroneTrajectory(drone_id=drone_id, waypoints=waypoints))

            # --- lights/drone_XXX.json ---
            light_names = sorted(
                n for n in names if n.startswith("lights/drone_") and n.endswith(".json")
            )
            for light_name in light_names:
                drone_id = self._extract_id(light_name)
                with zf.open(light_name) as fh:
                    raw = json.load(fh)
                keyframes = [
                    ColorKeyframe(
                        t=float(kf["t"]),
                        r=int(kf["r"]),
                        g=int(kf["g"]),
                        b=int(kf["b"]),
                    )
                    for kf in raw
                ]
                show.light_cues.append(LightCue(drone_id=drone_id, keyframes=keyframes))

        # Infer drone_count if not set in metadata
        if show.drone_count == 0:
            show.drone_count = max(len(show.trajectories), len(show.light_cues))

        # Infer duration if not set
        if show.duration == 0.0 and show.trajectories:
            show.duration = max(t.duration() for t in show.trajectories)

        log.info(
            "Loaded show '%s': %d drones, %.1fs duration",
            show.title,
            show.drone_count,
            show.duration,
        )
        return show

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def validate(self, show: ShowFile) -> list[str]:
        """
        Validate a ShowFile and return a list of warning strings.
        An empty list means the show is valid.
        """
        warnings: list[str] = []

        if show.drone_count <= 0:
            warnings.append("drone_count is 0 or negative.")

        if show.duration <= 0.0:
            warnings.append("Show duration is 0 or negative.")

        if not show.trajectories:
            warnings.append("No trajectories found.")

        traj_ids = {t.drone_id for t in show.trajectories}
        light_ids = {lc.drone_id for lc in show.light_cues}

        for tid in traj_ids:
            if tid not in light_ids:
                warnings.append(f"Drone {tid} has trajectory but no light cues.")

        for lid in light_ids:
            if lid not in traj_ids:
                warnings.append(f"Drone {lid} has light cues but no trajectory.")

        for traj in show.trajectories:
            if len(traj.waypoints) < 2:
                warnings.append(
                    f"Drone {traj.drone_id} trajectory has fewer than 2 waypoints."
                )
            else:
                times = [wp.t for wp in traj.waypoints]
                if times != sorted(times):
                    warnings.append(
                        f"Drone {traj.drone_id} waypoints are not in time order."
                    )

        return warnings

    # ------------------------------------------------------------------
    # Interpolation
    # ------------------------------------------------------------------

    def get_position_at(
        self,
        trajectory: DroneTrajectory,
        t: float,
    ) -> tuple[float, float, float]:
        """
        Interpolate position at time t using Catmull-Rom splines.

        Parameters
        ----------
        trajectory : DroneTrajectory
        t : float
            Query time in seconds.

        Returns
        -------
        (x, y, z) tuple of floats
        """
        wps = trajectory.waypoints
        if not wps:
            return (0.0, 0.0, 0.0)
        if t <= wps[0].t:
            return (wps[0].x, wps[0].y, wps[0].z)
        if t >= wps[-1].t:
            return (wps[-1].x, wps[-1].y, wps[-1].z)

        # Find the segment [i, i+1] that brackets t
        idx = 0
        for i in range(len(wps) - 1):
            if wps[i].t <= t <= wps[i + 1].t:
                idx = i
                break

        # Gather the four control points for Catmull-Rom
        i0 = max(idx - 1, 0)
        i1 = idx
        i2 = min(idx + 1, len(wps) - 1)
        i3 = min(idx + 2, len(wps) - 1)

        p0 = np.array([wps[i0].x, wps[i0].y, wps[i0].z])
        p1 = np.array([wps[i1].x, wps[i1].y, wps[i1].z])
        p2 = np.array([wps[i2].x, wps[i2].y, wps[i2].z])
        p3 = np.array([wps[i3].x, wps[i3].y, wps[i3].z])

        # Local parameter in [0, 1] across [i1, i2]
        dt = wps[i2].t - wps[i1].t
        local_t = (t - wps[i1].t) / dt if dt > 1e-10 else 0.0

        pt = _catmull_rom_point(p0, p1, p2, p3, local_t)
        return (float(pt[0]), float(pt[1]), float(pt[2]))

    def get_color_at(
        self,
        light_cue: LightCue,
        t: float,
    ) -> tuple[int, int, int]:
        """
        Interpolate LED colour at time t using linear interpolation between
        adjacent keyframes.

        Parameters
        ----------
        light_cue : LightCue
        t : float
            Query time in seconds.

        Returns
        -------
        (r, g, b) tuple of ints in [0, 255]
        """
        kfs = light_cue.keyframes
        if not kfs:
            return (255, 255, 255)
        if t <= kfs[0].t:
            return (kfs[0].r, kfs[0].g, kfs[0].b)
        if t >= kfs[-1].t:
            return (kfs[-1].r, kfs[-1].g, kfs[-1].b)

        # Find bracketing keyframes
        for i in range(len(kfs) - 1):
            if kfs[i].t <= t <= kfs[i + 1].t:
                k0, k1 = kfs[i], kfs[i + 1]
                dt = k1.t - k0.t
                alpha = (t - k0.t) / dt if dt > 1e-10 else 0.0
                r = int(round(k0.r + alpha * (k1.r - k0.r)))
                g = int(round(k0.g + alpha * (k1.g - k0.g)))
                b = int(round(k0.b + alpha * (k1.b - k0.b)))
                return (
                    max(0, min(255, r)),
                    max(0, min(255, g)),
                    max(0, min(255, b)),
                )

        return (kfs[-1].r, kfs[-1].g, kfs[-1].b)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _extract_id(path: str) -> int:
        """
        Extract the numeric drone ID from a path like
        ``trajectories/drone_042.json``.
        """
        name = path.split("/")[-1]          # e.g. "drone_042.json"
        stem = name.replace(".json", "")    # e.g. "drone_042"
        parts = stem.split("_")
        try:
            return int(parts[-1])
        except (ValueError, IndexError):
            return 0
