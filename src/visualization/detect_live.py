# detect_live.py
# Run:  python3 detect_live.py
# Optional: python3 detect_live.py --source 0        # Webcam
#           python3 detect_live.py --source path/to/video.mp4
#           python3 detect_live.py --source path/to/image_folder
#           python3 detect_live.py --conf 0.25

import argparse, cv2, time
from ultralytics import YOLO

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="runs/detect/train_fresh/weights/best.pt")
    ap.add_argument("--source", default="0", help="0=webcam, or path/URL to video/image/folder")
    ap.add_argument("--conf", type=float, default=0.25)
    args = ap.parse_args()

    model = YOLO(args.model)
    names = model.names  # pull class names directly from the model -> no YAML mismatch

    # Open source stream
    src = 0 if args.source == "0" else args.source
    is_cam = str(src).isdigit()
    if is_cam or (isinstance(src, str) and (src.startswith("http://") or src.startswith("rtsp://"))):
        cap = cv2.VideoCapture(0 if is_cam else src)
        if not cap.isOpened():
            raise SystemExit("Could not open camera/stream.")

        fps_t0, fps_n = time.time(), 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            results = model.predict(frame, conf=args.conf, verbose=False)
            r = results[0]
            # draw detections
            for box, cls, conf in zip(r.boxes.xyxy.cpu().numpy(),
                                      r.boxes.cls.cpu().numpy().astype(int),
                                      r.boxes.conf.cpu().numpy()):
                x1, y1, x2, y2 = box.astype(int)
                label = f"{names[cls]} {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, label, (x1, max(20, y1-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            # Display approximate FPS
            fps_n += 1
            dt = time.time() - fps_t0
            if dt >= 1:
                cv2.putText(frame, f"FPS ~ {fps_n/dt:.1f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                fps_t0, fps_n = time.time(), 0

            cv2.imshow("YOLOv8n live (press q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
    else:
        # Folder/image/video file – rely on the built-in pipeline
        model.predict(source=src, conf=args.conf, save=True, stream=False)  # Outputs -> runs/detect/predict
        print("Done. Results available under runs/detect/predict/")

if __name__ == "__main__":
    main()