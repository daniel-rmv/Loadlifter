#!/usr/bin/env python3
# test_camera.py
# Display a simple camera livestream for manual verification.
# Author: Daniel Würmli

import cv2

def main():
    cap = cv2.VideoCapture(0)  # 0 = default camera index
    if not cap.isOpened():
        print("Failed to open camera.")
        return

    print("[INFO] Livestream running - press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Failed to read frame...")
            continue

        cv2.imshow("Camera Livestream", frame)

        # Check for keypress
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
