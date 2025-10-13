#!/usr/bin/env python3
# overlay_client.py
# YOLO overlay client for the Pi video stream.
# Author: Daniel Würmli

import argparse, time, requests, cv2, os
from ultralytics import YOLO

def main():
    ap = argparse.ArgumentParser(description="Run YOLO overlay on MJPEG stream from Pi")
    default_model = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "models", "best.pt"))
    ap.add_argument("--model", default=default_model)
    pi_default = os.getenv("PI_CAMERA_BASE_URL")
    ap.add_argument(
        "--pi",
        default=pi_default,
        required=pi_default is None,
        help="Base URL of the Pi MJPEG server (defaults to PI_CAMERA_BASE_URL)",
    )
    ap.add_argument("--source", type=str, help="Optional explicit MJPEG source URL")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--fps", type=int, default=7) 
    ap.add_argument("--q", type=int, default=80)
    args = ap.parse_args()

    model_path = args.model
    if not os.path.exists(model_path):
        raise SystemExit(f"Model file not found: {model_path}")
    model = YOLO(model_path)
    source_url = args.source if args.source else args.pi.rstrip("/") + "/raw"
    cap = cv2.VideoCapture(source_url)
    if not cap.isOpened(): raise SystemExit(f"Could not open stream: {source_url}")

    min_dt = 1.0/max(1,args.fps); last=0
    push_url = args.pi.rstrip("/") + "/push"
    while True:
        ok, frame = cap.read()
        if not ok: time.sleep(0.01); continue
        now = time.time()
        if now - last < min_dt:
            time.sleep(0.001); continue
        last = now
        r = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
        ann = r.plot()
        ok2, jpg = cv2.imencode(".jpg", ann, [int(cv2.IMWRITE_JPEG_QUALITY), int(args.q)])
        if not ok2: continue
        try:
            requests.post(push_url, data=jpg.tobytes(), headers={"Content-Type":"image/jpeg"}, timeout=0.5)
        except Exception:
            pass  # Pi temporarily unreachable; skip this frame

if __name__ == "__main__":
    main()
