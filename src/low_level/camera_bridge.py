#!/usr/bin/env python3
# camera_bridge.py
# Camera streaming bridge with overlay support.
# Author: Daniel Würmli

"""Camera streaming bridge with overlay support."""

import argparse
import os
import signal
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
from flask import Flask, Response, render_template_string, request

cv2.setNumThreads(1)

HTML = """<!doctype html><html><head><meta charset="utf-8"><title>Pi Stream Bridge</title>
<style>body{background:#111;color:#eee;font-family:system-ui;margin:0}.wrap{display:flex;flex-direction:column;align-items:center;gap:12px;padding:16px}img{max-width:98vw;border-radius:8px}</style>
</head><body><div class="wrap"><h2>Pi Stream Bridge</h2>
<img src="/video"/></div></body></html>"""


class Camera:
    """Threaded camera reader."""

    def __init__(self, source=0):
        self.cap = cv2.VideoCapture(source)  # Do not tweak CAP_PROP settings; leave defaults
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera could not be opened: {source}")
        self.lock = threading.Lock()
        self.frame: Optional[np.ndarray] = None
        self.run = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.run:
            ok, frm = self.cap.read()
            if ok and frm is not None:
                with self.lock:
                    self.frame = frm
            else:
                time.sleep(0.01)

    def get(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self):
        self.run = False
        try:
            self.cap.release()
        except Exception:
            pass


class Store:
    """Overlay store that accepts JPEG payloads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.overlay_jpg: Optional[bytes] = None
        self.overlay_ts: float = 0.0

    def push(self, jpg_bytes: bytes):
        with self.lock:
            self.overlay_jpg = jpg_bytes
            self.overlay_ts = time.time()

    def get_overlay(self, max_age=0.7) -> Optional[bytes]:
        with self.lock:
            if self.overlay_jpg and (time.time() - self.overlay_ts) <= max_age:
                return self.overlay_jpg
            return None


def encode_jpeg(img, quality=70):
    ok, jpg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return jpg.tobytes() if ok else None


def gen_raw(cam: Camera, quality=70, maxfps=30):
    min_dt = 1.0 / max(1, int(maxfps))
    last = 0
    while True:
        now = time.time()
        if now - last < min_dt:
            time.sleep(0.001)
            continue
        last = now
        frm = cam.get()
        if frm is None:
            time.sleep(0.01)
            continue
        b = encode_jpeg(frm, quality)
        if not b:
            continue
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + b + b"\r\n"


def gen_video(cam: Camera, store: Store, quality=70, outw=960, maxfps=30):
    min_dt = 1.0 / max(1, int(maxfps))
    last = 0
    while True:
        now = time.time()
        if now - last < min_dt:
            time.sleep(0.001)
            continue
        last = now
        ov = store.get_overlay()
        if ov is not None:
            b = ov  # direkt durchreichen -> Pi spart CPU
        else:
            frm = cam.get()
            if frm is None:
                time.sleep(0.01)
                continue
            if outw and frm.shape[1] > outw:
                h, w = frm.shape[:2]
                nh = int(h * (outw / w))
                frm = cv2.resize(frm, (outw, nh), interpolation=cv2.INTER_AREA)
            b = encode_jpeg(frm, quality)
            if not b:
                continue
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + b + b"\r\n"


def create_app(source=0, quality=70, outw=960, maxfps=30) -> Tuple[Flask, Camera, Store]:
    """Create the Flask app along with the camera and overlay store."""
    app = Flask(__name__)
    cam = Camera(source)
    store = Store()

    @app.route("/")
    def index():
        return render_template_string(HTML)

    @app.route("/raw")
    def raw():
        return Response(
            gen_raw(cam, quality, maxfps),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.route("/video")
    def video():
        return Response(
            gen_video(cam, store, quality, outw, maxfps),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.route("/push", methods=["POST"])
    def push():
        # erwartet image/jpeg im Body (annotiertes Frame vom Mac)
        if not request.data:
            return "no data", 400
        store.push(request.data)
        return "ok", 200

    @app.route("/healthz")
    def health():
        return "ok"

    return app, cam, store


def run_stream_server(
    source="0",
    port=5000,
    quality=70,
    outw=960,
    maxfps=30,
    host="0.0.0.0",
):
    """Start the streaming server (blocking)."""
    os.environ.setdefault("OMP_NUM_THREADS", "2")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "2")
    src = 0 if str(source) == "0" else source

    app, cam, _ = create_app(src, quality=quality, outw=outw, maxfps=maxfps)

    def _sigint(sig, frm):  # pragma: no cover - runtime helper
        cam.close()
        os._exit(0)

    signal.signal(signal.SIGINT, _sigint)

    try:
        app.run(host=host, port=port, threaded=True)
    finally:
        cam.close()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default="0", help="0=USB camera, otherwise path/URL")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--quality", type=int, default=70)
    p.add_argument("--outw", type=int, default=960)
    p.add_argument("--maxfps", type=int, default=30)
    return p.parse_args()


def main():
    args = parse_args()
    run_stream_server(
        source=args.source,
        port=args.port,
        quality=args.quality,
        outw=args.outw,
        maxfps=args.maxfps,
    )


if __name__ == "__main__":
    main()
