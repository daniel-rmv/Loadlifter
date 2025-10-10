#!/usr/bin/env python3
# camera_system.py
# High-level camera system built on top of the low-level camera bridge.
# Author: Daniel Würmli

"""High-level camera system built on top of the low-level camera bridge."""

import threading
from typing import Optional

from ..low_level.camera_bridge import create_app, run_stream_server


class CameraSystem:
    """Manage the camera stream with optional background execution."""

    def __init__(
        self,
        source: str = "0",
        port: int = 5000,
        quality: int = 70,
        outw: int = 960,
        maxfps: int = 30,
        host: str = "0.0.0.0",
    ):
        self.source = source
        self.port = int(port)
        self.quality = int(quality)
        self.outw = int(outw)
        self.maxfps = int(maxfps)
        self.host = host

        self._app = None
        self._camera = None
        self._store = None
        self._thread: Optional[threading.Thread] = None

    def _init_app(self) -> None:
        if self._app is None:
            src = 0 if str(self.source) == "0" else self.source
            self._app, self._camera, self._store = create_app(
                src,
                quality=self.quality,
                outw=self.outw,
                maxfps=self.maxfps,
            )

    def _run_blocking(self) -> None:
        if self._app is None:
            raise RuntimeError("CameraSystem was not initialized.")
        try:
            self._app.run(host=self.host, port=self.port, threaded=True)
        finally:
            if self._camera is not None:
                self._camera.close()

    def start(self, background: bool = True) -> Optional[threading.Thread]:
        """
        Start the streaming server. Runs in a background thread by default and returns the thread (or None when blocking).
        """
        self._init_app()
        if not background:
            self._run_blocking()
            return None

        if self._thread and self._thread.is_alive():
            return self._thread

        self._thread = threading.Thread(target=self._run_blocking, daemon=True)
        self._thread.start()
        return self._thread

    def stop(self) -> None:
        """Close the camera resource only; Flask must be stopped externally."""
        if self._camera is not None:
            self._camera.close()

    def run_cli(self) -> None:
        """CLI helper for backwards-compatible invocation."""
        run_stream_server(
            source=self.source,
            port=self.port,
            quality=self.quality,
            outw=self.outw,
            maxfps=self.maxfps,
            host=self.host,
        )


__all__ = ["CameraSystem"]
