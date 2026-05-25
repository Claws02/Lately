"""
Log session management.

A LogSession represents a single simulation run's log file.
It writes JSONL entries to disk and is safe for concurrent async use.
"""

from __future__ import annotations

import asyncio
import json
import os
import time
from pathlib import Path
from typing import Any, Optional

from .formatter import JSONLFormatter, LogEntry, LogLevel


class LogSession:
    """
    Manages the lifecycle of a single logging session.

    Usage
    -----
    session = LogSession()
    await session.start("run_001", "/var/log/swarm")
    await session.log(LogLevel.INFO, "Simulation started", subsystem="SIM")
    await session.stop()
    """

    def __init__(self) -> None:
        self._formatter = JSONLFormatter()
        self._lock = asyncio.Lock()
        self._file: Optional[Any] = None   # open file handle
        self._session_id: str = ""
        self._output_dir: str = ""
        self._log_path: str = ""
        self._start_time: float = 0.0
        self._entry_count: int = 0
        self._drone_count: int = 0
        self._show_name: str = ""
        self._active: bool = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def start(
        self,
        session_id: str,
        output_dir: str,
        drone_count: int = 0,
        show_name: str = "",
    ) -> str:
        """
        Open a new JSONL log file and record session metadata.

        Parameters
        ----------
        session_id : str
            Unique identifier for this session (e.g. "run_20240101_120000").
        output_dir : str
            Directory where the JSONL file will be written.
        drone_count : int
            Number of drones in the simulation (metadata only).
        show_name : str
            Human-readable show name (metadata only).

        Returns
        -------
        str
            Absolute path to the opened log file.
        """
        async with self._lock:
            self._session_id = session_id
            self._output_dir = output_dir
            self._drone_count = drone_count
            self._show_name = show_name
            self._start_time = time.time()
            self._entry_count = 0
            self._active = True

            Path(output_dir).mkdir(parents=True, exist_ok=True)
            self._log_path = os.path.join(output_dir, f"{session_id}.jsonl")

            self._file = open(self._log_path, "w", encoding="utf-8", buffering=1)

            # Write session-open metadata as the first line
            meta = {
                "event": "session_start",
                "session_id": session_id,
                "start_time": self._start_time,
                "drone_count": drone_count,
                "show_name": show_name,
            }
            self._file.write(json.dumps(meta, default=str) + "\n")

        return self._log_path

    async def stop(self) -> dict[str, Any]:
        """
        Flush and close the log file, writing a summary as the final entry.

        Returns
        -------
        dict
            Session summary metadata.
        """
        async with self._lock:
            if not self._active:
                return {}

            stop_time = time.time()
            summary = {
                "event": "session_stop",
                "session_id": self._session_id,
                "start_time": self._start_time,
                "stop_time": stop_time,
                "duration_s": round(stop_time - self._start_time, 3),
                "total_entries": self._entry_count,
                "drone_count": self._drone_count,
                "show_name": self._show_name,
                "log_path": self._log_path,
            }

            if self._file:
                self._file.write(json.dumps(summary, default=str) + "\n")
                self._file.flush()
                self._file.close()
                self._file = None

            self._active = False
            return summary

    @property
    def is_active(self) -> bool:
        return self._active

    @property
    def log_path(self) -> str:
        return self._log_path

    @property
    def session_id(self) -> str:
        return self._session_id

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    async def log(
        self,
        level: LogLevel,
        message: str,
        *,
        drone_id: Optional[int] = None,
        subsystem: str = "SYSTEM",
        **extra: Any,
    ) -> None:
        """
        Write a structured log entry to the JSONL file.

        Parameters
        ----------
        level : LogLevel
        message : str
        drone_id : int or None
        subsystem : str
        **extra : Any
            Additional key/value pairs included in the 'extra' dict.
        """
        entry = LogEntry(
            level=level,
            message=message,
            subsystem=subsystem,
            drone_id=drone_id,
            extra=dict(extra) if extra else {},
        )

        async with self._lock:
            if not self._active or self._file is None:
                return
            line = self._formatter.format(entry)
            self._file.write(line + "\n")
            self._entry_count += 1

    async def log_info(self, message: str, **kwargs: Any) -> None:
        await self.log(LogLevel.INFO, message, **kwargs)

    async def log_warning(self, message: str, **kwargs: Any) -> None:
        await self.log(LogLevel.WARNING, message, **kwargs)

    async def log_error(self, message: str, **kwargs: Any) -> None:
        await self.log(LogLevel.ERROR, message, **kwargs)

    async def log_debug(self, message: str, **kwargs: Any) -> None:
        await self.log(LogLevel.DEBUG, message, **kwargs)

    # ------------------------------------------------------------------
    # Stats
    # ------------------------------------------------------------------

    def stats(self) -> dict[str, Any]:
        """Return current session stats without acquiring the lock."""
        return {
            "session_id": self._session_id,
            "active": self._active,
            "log_path": self._log_path,
            "entry_count": self._entry_count,
            "start_time": self._start_time,
            "drone_count": self._drone_count,
            "show_name": self._show_name,
        }
