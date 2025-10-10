# detect_live.py
# Run:  python3 detect_live.py
# Optional: python3 detect_live.py --source 0        # Webcam
#           python3 detect_live.py --source path/zu/video.mp4
#           python3 detect_live.py --source path/zu/ordner_mit_bildern
#           python3 detect_live.py --conf 0.25

import argparse, cv2, time
from ultralytics import YOLO

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="runs/detect/train_fresh/weights/best.pt")
    ap.add_argument("--source", default="0", help="0=Webcam, Pfad zu Video/Bild/Ordner")
    ap.add_argument("--conf", type=float, default=0.25)
    args = ap.parse_args()

    model = YOLO(args.model)
    names = model.names  # nimmt Klassennamen direkt aus dem Modell -> kein YAML-Mismatch

    # Quelle öffnen
    src = 0 if args.source == "0" else args.source
    is_cam = str(src).isdigit()
    if is_cam or (isinstance(src, str) and (src.startswith("http://") or src.startswith("rtsp://"))):
        cap = cv2.VideoCapture(0 if is_cam else src)
        if not cap.isOpened():
            raise SystemExit("Kamera/Stream lässt sich nid öffne.")

        fps_t0, fps_n = time.time(), 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            results = model.predict(frame, conf=args.conf, verbose=False)
            r = results[0]
            # zeichnen
            for box, cls, conf in zip(r.boxes.xyxy.cpu().numpy(),
                                      r.boxes.cls.cpu().numpy().astype(int),
                                      r.boxes.conf.cpu().numpy()):
                x1, y1, x2, y2 = box.astype(int)
                label = f"{names[cls]} {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, label, (x1, max(20, y1-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            # FPS klein anzeigen
            fps_n += 1
            dt = time.time() - fps_t0
            if dt >= 1:
                cv2.putText(frame, f"FPS ~ {fps_n/dt:.1f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                fps_t0, fps_n = time.time(), 0

            cv2.imshow("YOLOv8n live (q zum Beenden)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
    else:
        # Ordner/Bild/Video Datei – nutzt die eingebaute Pipeline
        model.predict(source=src, conf=args.conf, save=True, stream=False)  # Outputs -> runs/detect/predict
        print("Fertig. Resultate unter runs/detect/predict/")

if __name__ == "__main__":
    main()