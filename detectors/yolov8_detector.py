import time
import cv2
from ultralytics import YOLO

class YOLOv8Detector:
    def __init__(self, model_path="models/yolov8n.pt"):
        self.model = YOLO(model_path)

    def detect(self, frame):
        start = time.time()
        results = self.model.predict(source=frame, conf=0.3, classes=[0], verbose=False)
        end = time.time()

        for box in results[0].boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = map(int, box[:4])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
        return frame, round((end - start) * 1000, 2)  # ms
