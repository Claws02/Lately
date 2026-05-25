"""
In-memory pub/sub log watcher for SSE (Server-Sent Events) streaming.

LogWatcher maintains a circular buffer of recent entries and a set of
subscriber queues.  Any coroutine can subscribe to receive live log
entries as they are published.
"""

from __future__ import annotations

import asyncio
import collections
from typing import AsyncGenerator

from .formatter import LogEntry


class LogWatcher:
    """
    Pub/sub hub for live log streaming.

    Typical use
    -----------
    watcher = LogWatcher()

    # Producer side (e.g. simulation loop):
    await watcher.publish(entry)

    # Consumer side (e.g. SSE endpoint):
    async for entry in watcher.stream():
        yield f"data: {entry.to_dict()}\\n\\n"
    """

    def __init__(self, buffer_size: int = 1000) -> None:
        self._buffer: collections.deque[LogEntry] = collections.deque(
            maxlen=buffer_size
        )
        self._subscribers: list[asyncio.Queue[LogEntry | None]] = []
        self._lock = asyncio.Lock()

    # ------------------------------------------------------------------
    # Subscription management
    # ------------------------------------------------------------------

    async def subscribe(self) -> asyncio.Queue[LogEntry | None]:
        """
        Register a new subscriber.

        Returns
        -------
        asyncio.Queue
            The caller reads LogEntry objects from this queue.
            A ``None`` sentinel is put when the watcher is shut down.
        """
        queue: asyncio.Queue[LogEntry | None] = asyncio.Queue(maxsize=500)
        async with self._lock:
            self._subscribers.append(queue)
        return queue

    async def unsubscribe(self, queue: asyncio.Queue) -> None:
        """
        Remove a subscriber queue.

        Parameters
        ----------
        queue : asyncio.Queue
            The queue previously returned by subscribe().
        """
        async with self._lock:
            try:
                self._subscribers.remove(queue)
            except ValueError:
                pass

    def subscriber_count(self) -> int:
        """Return current number of active subscribers."""
        return len(self._subscribers)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    async def publish(self, entry: LogEntry) -> None:
        """
        Push a log entry to the circular buffer and all subscriber queues.

        Parameters
        ----------
        entry : LogEntry
        """
        async with self._lock:
            self._buffer.append(entry)
            dead: list[asyncio.Queue] = []

            for q in self._subscribers:
                try:
                    q.put_nowait(entry)
                except asyncio.QueueFull:
                    # Slow consumer – drop message (non-blocking)
                    dead.append(q)

            for q in dead:
                try:
                    self._subscribers.remove(q)
                except ValueError:
                    pass

    async def shutdown(self) -> None:
        """
        Signal all subscribers that no more entries will arrive by pushing
        None sentinels, then clear the subscriber list.
        """
        async with self._lock:
            for q in self._subscribers:
                try:
                    q.put_nowait(None)
                except asyncio.QueueFull:
                    pass
            self._subscribers.clear()

    # ------------------------------------------------------------------
    # Recent entries
    # ------------------------------------------------------------------

    def get_recent(self, count: int = 100) -> list[LogEntry]:
        """
        Return the last `count` log entries from the circular buffer.

        Parameters
        ----------
        count : int
            Maximum number of entries to return.

        Returns
        -------
        list[LogEntry]  (oldest-first)
        """
        items = list(self._buffer)
        return items[-count:] if len(items) > count else items

    # ------------------------------------------------------------------
    # Async generator for SSE
    # ------------------------------------------------------------------

    async def stream(self) -> AsyncGenerator[LogEntry, None]:
        """
        Async generator that yields LogEntry objects as they arrive.

        Suitable for use in FastAPI SSE (Server-Sent Events) endpoints.
        The generator exits when a None sentinel is received from the queue.

        Example
        -------
        async def sse_endpoint(request: Request):
            async def generate():
                async for entry in watcher.stream():
                    yield f"data: {json.dumps(entry.to_dict())}\\n\\n"
            return StreamingResponse(generate(), media_type="text/event-stream")
        """
        queue = await self.subscribe()
        try:
            while True:
                entry = await queue.get()
                if entry is None:
                    break
                yield entry
        finally:
            await self.unsubscribe(queue)
