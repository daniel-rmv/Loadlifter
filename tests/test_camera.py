#!/usr/bin/env python3
# camera_stream.py
# Zeigt einfach nur den Livestream der Kamera.

import cv2

def main():
    cap = cv2.VideoCapture(0)  # 0 = erste Kamera
    if not cap.isOpened():
        print("Konnte Kamera nicht öffnen!")
        return

    print("[INFO] Livestream läuft – drück 'q' zum Beenden.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Kein Frame gelesen...")
            continue

        cv2.imshow("Kamera Livestream", frame)

        # Taste prüfen
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()