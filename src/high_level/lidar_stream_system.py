#!/usr/bin/env python3
# lidar_stream_system.py
# High-level helper to expose LiDAR point clouds via HTTP.
# Author: Daniel Würmli

"""High-level LiDAR stream manager."""

import threading
from typing import Optional

from ..low_level.lidar_stream import create_app


class LidarStreamSystem:
    """Manage LiDAR streaming server lifecycle."""

    def __init__(self, lidar, port: int = 5051, host: str = "0.0.0.0", hz: float = 12.0):
        self._lidar = lidar
        self.port = int(port)
        self.host = host
        self.hz = float(hz)

        self._app = None
        self._store = None
        self._thread: Optional[threading.Thread] = None

    def _ensure_app(self):
        if self._app is None:
            self._app, self._store = create_app(self._lidar.get_points, hz=self.hz)

    def start(self, background: bool = True):
        """Start LiDAR stream server."""
        self._ensure_app()
        if self._store:
            self._store.start()

        if not background:
            self._app.run(host=self.host, port=self.port, threaded=True)
            return None

        if self._thread and self._thread.is_alive():
            return self._thread

        self._thread = threading.Thread(
            target=self._app.run,
            kwargs={"host": self.host, "port": self.port, "threaded": True},
            daemon=True,
        )
        self._thread.start()
        return self._thread

    def stop(self):
        """Stop background polling thread."""
        if self._store:
            self._store.stop()


__all__ = ["LidarStreamSystem"]
