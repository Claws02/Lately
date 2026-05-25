"""
CSV trajectory importer.

Expected CSV columns:
    drone_id, t, x, y, z, r, g, b

drone_id separates individual drones.  All other columns are numeric.
Returns the same ShowFile / DroneTrajectory / LightCue types used by SkycReader.
"""

from __future__ import annotations

import csv
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .skyc_reader import (
    ColorKeyframe,
    DroneTrajectory,
    LightCue,
    ShowFile,
    Waypoint,
)

log = logging.getLogger(__name__)

REQUIRED_COLUMNS = {"drone_id", "t", "x", "y", "z", "r", "g", "b"}


class CSVValidationError(ValueError):
    """Raised when required CSV columns are missing or data is malformed."""


class CSVIngest:
    """
    Parses a CSV file with swarm trajectory + light data into a ShowFile.

    Each row describes one sample for one drone at one timestamp.  Multiple
    rows with the same drone_id are aggregated into that drone's trajectory
    and light cues.
    """

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def load(self, path: str) -> ShowFile:
        """
        Parse a CSV file and return a ShowFile.

        Parameters
        ----------
        path : str
            File system path to the CSV file.

        Returns
        -------
        ShowFile
        """
        rows = self._read_csv(path)
        if not rows:
            raise CSVValidationError(f"CSV file is empty: {path}")

        self.validate_columns(rows[0])

        # Group rows by drone_id
        drone_rows: dict[int, list[dict[str, str]]] = {}
        for row in rows:
            did = int(float(row["drone_id"]))
            drone_rows.setdefault(did, []).append(row)

        trajectories: list[DroneTrajectory] = []
        light_cues: list[LightCue] = []
        max_t = 0.0

        for drone_id in sorted(drone_rows.keys()):
            samples = sorted(drone_rows[drone_id], key=lambda r: float(r["t"]))

            waypoints: list[Waypoint] = []
            keyframes: list[ColorKeyframe] = []

            for row in samples:
                t = float(row["t"])
                x = float(row["x"])
                y = float(row["y"])
                z = float(row["z"])
                r = max(0, min(255, int(float(row["r"]))))
                g = max(0, min(255, int(float(row["g"]))))
                b = max(0, min(255, int(float(row["b"]))))

                waypoints.append(Waypoint(t=t, x=x, y=y, z=z))
                keyframes.append(ColorKeyframe(t=t, r=r, g=g, b=b))

                if t > max_t:
                    max_t = t

            trajectories.append(DroneTrajectory(drone_id=drone_id, waypoints=waypoints))
            light_cues.append(LightCue(drone_id=drone_id, keyframes=keyframes))

        stem = Path(path).stem
        show = ShowFile(
            version="csv-1.0",
            title=stem,
            duration=max_t,
            drone_count=len(trajectories),
            trajectories=trajectories,
            light_cues=light_cues,
        )

        log.info(
            "CSV ingest: loaded '%s' – %d drones, %.1f s",
            stem,
            show.drone_count,
            show.duration,
        )
        return show

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def validate_columns(self, row: dict[str, Any]) -> None:
        """
        Verify that all required column names are present.

        Parameters
        ----------
        row : dict
            A single parsed CSV row (keys are column headers).

        Raises
        ------
        CSVValidationError
            If any required column is absent.
        """
        present = {k.strip().lower() for k in row.keys()}
        missing = REQUIRED_COLUMNS - present
        if missing:
            raise CSVValidationError(
                f"CSV is missing required columns: {sorted(missing)}"
            )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _read_csv(path: str) -> list[dict[str, str]]:
        """
        Read a CSV file and return a list of row dicts.
        Column headers are stripped and lowercased.
        """
        rows: list[dict[str, str]] = []
        with open(path, newline="", encoding="utf-8-sig") as fh:
            reader = csv.DictReader(fh)
            for raw_row in reader:
                # Normalise key names
                normalised = {k.strip().lower(): v.strip() for k, v in raw_row.items()}
                rows.append(normalised)
        return rows
