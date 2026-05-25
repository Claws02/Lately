"""
JSONL logging formatter with optional coloured console output.

LogLevel enum  – DEBUG / INFO / WARNING / ERROR / CRITICAL
LogEntry       – immutable structured log record
JSONLFormatter – serialises entries to JSONL strings and coloured console lines
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Optional


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------


class LogLevel(str, Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


# ---------------------------------------------------------------------------
# ANSI colour codes
# ---------------------------------------------------------------------------

_ANSI_RESET = "\033[0m"

_LEVEL_COLORS: dict[LogLevel, str] = {
    LogLevel.DEBUG:    "\033[90m",   # dark grey
    LogLevel.INFO:     "\033[96m",   # cyan
    LogLevel.WARNING:  "\033[93m",   # yellow
    LogLevel.ERROR:    "\033[91m",   # red
    LogLevel.CRITICAL: "\033[95m",   # magenta
}

_LEVEL_LABELS: dict[LogLevel, str] = {
    LogLevel.DEBUG:    "DEBUG   ",
    LogLevel.INFO:     "INFO    ",
    LogLevel.WARNING:  "WARNING ",
    LogLevel.ERROR:    "ERROR   ",
    LogLevel.CRITICAL: "CRITICAL",
}


# ---------------------------------------------------------------------------
# LogEntry
# ---------------------------------------------------------------------------


@dataclass
class LogEntry:
    """A single structured log record."""

    level: LogLevel
    message: str
    subsystem: str = "SYSTEM"
    drone_id: Optional[int] = None
    timestamp: float = field(default_factory=time.time)
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Return a plain dict representation (JSON-serialisable)."""
        d: dict[str, Any] = {
            "timestamp": self.timestamp,
            "level": self.level.value,
            "subsystem": self.subsystem,
            "message": self.message,
        }
        if self.drone_id is not None:
            d["drone_id"] = self.drone_id
        if self.extra:
            d["extra"] = self.extra
        return d


# ---------------------------------------------------------------------------
# JSONLFormatter
# ---------------------------------------------------------------------------


class JSONLFormatter:
    """
    Converts LogEntry objects to serialised strings.

    format()         → compact JSON line (for file writing / network)
    format_console() → human-readable, colour-coded terminal line
    """

    def format(self, entry: LogEntry) -> str:
        """
        Serialise entry to a single JSONL (JSON Lines) string.
        The string does NOT include a trailing newline.

        Parameters
        ----------
        entry : LogEntry

        Returns
        -------
        str
        """
        return json.dumps(entry.to_dict(), separators=(",", ":"), default=str)

    def format_console(self, entry: LogEntry) -> str:
        """
        Format entry as a coloured, human-readable console line.

        Format:
            [HH:MM:SS.mmm] LEVEL    SUBSYSTEM  [drone=N]  message

        Parameters
        ----------
        entry : LogEntry

        Returns
        -------
        str  (includes ANSI colour codes, no trailing newline)
        """
        color = _LEVEL_COLORS.get(entry.level, "")
        label = _LEVEL_LABELS.get(entry.level, entry.level.value.ljust(8))

        # Human-readable timestamp (local time)
        ts = time.strftime("%H:%M:%S", time.localtime(entry.timestamp))
        ms = int((entry.timestamp % 1) * 1000)
        ts_str = f"{ts}.{ms:03d}"

        # Optional drone tag
        drone_tag = f" [drone={entry.drone_id}]" if entry.drone_id is not None else ""

        # Optional extra fields inline
        extra_str = ""
        if entry.extra:
            parts = [f"{k}={v}" for k, v in entry.extra.items()]
            extra_str = "  {" + ", ".join(parts) + "}"

        return (
            f"{color}"
            f"[{ts_str}] {label} {entry.subsystem:<12}{drone_tag}"
            f"  {entry.message}{extra_str}"
            f"{_ANSI_RESET}"
        )
