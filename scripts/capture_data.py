#!/usr/bin/env python3
# capture_data.py
# Capture frames from MJPEG stream and save for ML.
# Author: Daniel Würmli

"""Capture frames from the running HTTP/MJPEG stream and save numbered images when ENTER is pressed.
- Default names: 1.jpg, 2.jpg, 3.jpg, ...
- With --background flag: 1_bg.jpg, 2_bg.jpg, ...

Numbering resets to 1 on every start.
"""

import argparse
import os
import sys
import threading
import time
from pathlib import Path
import cv2
from src.utils.env import load_dotenv

# Ensure local .env is loaded when the script runs standalone.
load_dotenv()


def parse_args():
    ap = argparse.ArgumentParser(description="Press ENTER to save 1.jpg / 1_bg.jpg from HTTP stream")
    ap.add_argument(
        "--url",
        type=str,
        default=os.getenv("CAPTURE_STREAM_URL", "http://127.0.0.1:5000/video"),
        help="HTTP/MJPEG stream URL (defaults to CAPTURE_STREAM_URL env variable)",
    )
    ap.add_argument("--out", type=str, default="dataset", help="Output directory (default: dataset)")
    ap.add_argument("--show", action="store_true", help="Optionally show a preview window")
    ap.add_argument("--background", action="store_true",
                    help="Background mode: save files as 1_bg.jpg, 2_bg.jpg, ...")
    return ap.parse_args()


class InputThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.save_flag = False
        self.quit_flag = False

    def run(self):
        print("[INFO] ENTER = Save | q = Quit")
        while not self.quit_flag:
            ch = sys.stdin.read(1)
            if not ch:
                time.sleep(0.01)
                continue
            if ch == "\n":
                self.save_flag = True
            elif ch.strip().lower() == "q":
                self.quit_flag = True


def open_stream(url: str):
    cap = None
    while True:
        cap = cv2.VideoCapture(url)
        if cap.isOpened():
            return cap
        print(f"[WARN] Could not open stream: {url}. Retrying in 1s ...")
        time.sleep(1)


def main():
    args = parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = open_stream(args.url)

    it = InputThread()
    it.start()

    counter = 1
    last_frame = None

    print(f"[INFO] Source: {args.url}")
    print(f"[INFO] Output: {out_dir.resolve()}")
    print("[INFO] ENTER -> Save | q -> Quit")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[WARN] No frame received. Reconnecting ...")
            cap.release()
            time.sleep(0.5)
            cap = open_stream(args.url)
            continue

        last_frame = frame

        if args.show:
            cv2.imshow("Preview (HTTP stream)", frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break

        if it.save_flag and last_frame is not None:
            it.save_flag = False
            name = f"{counter}_bg.jpg" if args.background else f"{counter}.jpg"
            out_path = out_dir / name
            if cv2.imwrite(str(out_path), last_frame):
                print(f"[SAVE] {out_path.name}")
                counter += 1
            else:
                print(f"[ERROR] Saving failed: {out_path}")

        if it.quit_flag:
            break

    cap.release()
    if args.show:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
