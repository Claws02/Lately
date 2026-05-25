"""
WebSocket broadcast hub for streaming telemetry to all connected browser clients.
Thread-safe, handles graceful client disconnection.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Any

from fastapi import WebSocket

log = logging.getLogger(__name__)


class WebSocketHub:
    """
    Manages all active WebSocket connections and provides broadcast primitives.

    Architecture:
    - Each client gets its own asyncio.Queue so the broadcaster never blocks
      on a slow receiver.
    - A single background worker drains each queue and actually sends data.
    """

    def __init__(self) -> None:
        # Maps WebSocket -> its outbound queue
        self._clients: dict[WebSocket, asyncio.Queue] = {}
        self._lock = asyncio.Lock()
        self._worker_tasks: dict[WebSocket, asyncio.Task] = {}

    # ------------------------------------------------------------------
    # Registration
    # ------------------------------------------------------------------

    async def register(self, ws: WebSocket) -> None:
        """Add a new WebSocket client."""
        queue: asyncio.Queue = asyncio.Queue(maxsize=120)
        async with self._lock:
            self._clients[ws] = queue
            task = asyncio.create_task(self._sender_worker(ws, queue))
            self._worker_tasks[ws] = task
        log.info("WebSocket client registered – total: %d", len(self._clients))

    async def unregister(self, ws: WebSocket) -> None:
        """Remove a WebSocket client and cancel its worker."""
        async with self._lock:
            task = self._worker_tasks.pop(ws, None)
            self._clients.pop(ws, None)

        if task and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

        log.info("WebSocket client unregistered – total: %d", len(self._clients))

    def get_client_count(self) -> int:
        """Return number of currently connected clients."""
        return len(self._clients)

    # ------------------------------------------------------------------
    # Broadcasting
    # ------------------------------------------------------------------

    async def broadcast(self, message: dict[str, Any]) -> None:
        """
        Enqueue a JSON-serialisable message for all connected clients.
        Drops the message for a client whose queue is full (back-pressure protection).
        """
        payload = json.dumps(message, default=str)
        dead: list[WebSocket] = []

        async with self._lock:
            clients_snapshot = list(self._clients.items())

        for ws, queue in clients_snapshot:
            try:
                queue.put_nowait(payload)
            except asyncio.QueueFull:
                log.debug("Queue full for client – dropping message.")
            except Exception:
                dead.append(ws)

        for ws in dead:
            await self.unregister(ws)

    async def broadcast_telemetry(self, drone_states: list[dict]) -> None:
        """
        Format and broadcast a telemetry snapshot.
        Intended to be called at 60 Hz from the simulation loop.
        """
        message = {
            "type": "TEL",
            "timestamp": time.time(),
            "drones": drone_states,
        }
        await self.broadcast(message)

    # ------------------------------------------------------------------
    # Internal sender worker
    # ------------------------------------------------------------------

    async def _sender_worker(self, ws: WebSocket, queue: asyncio.Queue) -> None:
        """
        Per-client coroutine: pulls messages from the queue and sends them
        over the WebSocket.  On any send error the client is unregistered.
        """
        try:
            while True:
                payload = await queue.get()
                try:
                    await ws.send_text(payload)
                except Exception as exc:
                    log.debug("WebSocket send error (%s) – dropping client.", exc)
                    break
        except asyncio.CancelledError:
            pass
        finally:
            # Ensure cleanup even if the worker exits abnormally
            async with self._lock:
                self._clients.pop(ws, None)
                self._worker_tasks.pop(ws, None)
