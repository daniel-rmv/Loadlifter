#!/usr/bin/env python3
# lidar_stream.py
# Lightweight Flask/SSE server to stream LiDAR point clouds.
# Author: Daniel Würmli

"""Utility helpers to expose LiDAR point clouds over HTTP."""

import threading, time, json
from typing import Callable, List, Tuple

from flask import Flask, Response, jsonify


Point = Tuple[float, float, float]


class _PointStore:
    """Threaded helper that keeps the latest LiDAR points."""

    def __init__(self, getter: Callable[[], List[Point]], hz: float = 12.0):
        self._getter = getter
        self._interval = 1.0 / max(1.0, float(hz))
        self._lock = threading.Lock()
        self._points: List[Point] = []
        self._stop = threading.Event()
        self._thread: threading.Thread = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return

        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def snapshot(self) -> List[Point]:
        with self._lock:
            return list(self._points)

    def _run(self) -> None:
        while not self._stop.is_set():
            pts = self._getter()
            if not isinstance(pts, (list, tuple)):
                pts = []
            with self._lock:
                self._points = pts
            self._stop.wait(self._interval)


def create_app(get_points: Callable[[], List[Point]], hz: float = 12.0):
    """Create a Flask app plus backing store for LiDAR streaming."""

    store = _PointStore(get_points, hz=hz)
    app = Flask(__name__)

    @app.route("/")
    def index():
        return Response(
            _INDEX_HTML,
            mimetype="text/html",
        )

    @app.route("/points")
    def points():
        pts = _convert_points(store.snapshot())
        return jsonify({"points": pts, "hz": hz})

    @app.route("/stream")
    def stream():
        def _gen():
            while True:
                pts = _convert_points(store.snapshot())
                payload = json.dumps({"points": pts})
                yield f"data:{payload}\n\n"
                time.sleep(1.0 / max(1.0, hz))

        return Response(_gen(), mimetype="text/event-stream")

    return app, store


def _convert_points(raw: List[Point]) -> List[dict]:
    out = []
    if not raw:
        return out
    for entry in raw:
        try:
            angle, dist, intensity = entry
        except Exception:
            continue
        out.append(
            {
                "angle": float(angle),
                "distance": float(dist),
                "intensity": float(intensity),
            }
        )
    return out


_INDEX_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8" />
    <title>LiDAR Stream</title>
    <style>
        body { background:#111; color:#eee; font-family:Arial, sans-serif; margin:0; }
        #canvas-container { display:flex; justify-content:center; align-items:center; height:100vh; }
        canvas { background:#000; border:1px solid #444; }
        #legend { position:absolute; top:15px; left:15px; background:rgba(0,0,0,0.6); padding:8px 12px; border-radius:6px; font-size:14px; }
    </style>
</head>
<body>
    <div id="legend">LiDAR points (0° right, 180° left, 270° front)</div>
    <div id="canvas-container">
        <canvas id="lidar" width="640" height="640"></canvas>
    </div>
    <script>
        const canvas = document.getElementById("lidar");
        const ctx = canvas.getContext("2d");
        const cx = canvas.width / 2;
        const cy = canvas.height / 2;
        const scale = 0.2;  // pixels per mm
        const maxRadius = Math.min(cx, cy) - 20;

        function draw(points) {
            ctx.fillStyle = "#000";
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            ctx.strokeStyle = "#222";
            ctx.beginPath();
            for (let r = 200; r <= 2000; r += 200) {
                const rad = r * scale;
                ctx.beginPath();
                ctx.arc(cx, cy, Math.min(rad, maxRadius), 0, Math.PI * 2);
                ctx.stroke();
            }

            ctx.fillStyle = "#ff5252";
            for (const p of points) {
                const ang = (p.angle * Math.PI) / 180.0;
                const dist = p.distance;
                const px = cx + Math.cos(ang) * dist * scale;
                const py = cy - Math.sin(ang) * dist * scale;
                ctx.fillRect(px - 2, py - 2, 4, 4);
            }

            ctx.strokeStyle = "#2ecc71";
            ctx.beginPath();
            ctx.moveTo(cx, cy - maxRadius);
            ctx.lineTo(cx, cy + maxRadius);
            ctx.stroke();

            ctx.strokeStyle = "#3498db";
            ctx.beginPath();
            ctx.moveTo(cx - maxRadius, cy);
            ctx.lineTo(cx + maxRadius, cy);
            ctx.stroke();
        }

        const source = new EventSource("stream");
        source.onmessage = (event) => {
            try {
                const payload = JSON.parse(event.data);
                draw(payload.points || []);
            } catch (err) {
                console.error(err);
            }
        };
    </script>
</body>
</html>
"""


__all__ = ["create_app"]
