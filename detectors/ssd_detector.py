import cv2
import time

class SSDDetector:
    def __init__(self, prototxt, model):
        self.net = cv2.dnn.readNetFromCaffe(prototxt, model)
        self.classes = {15: 'person', 7: 'car'}

    def detect(self, frame):
        start = time.time()
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()
        h, w = frame.shape[:2]

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            class_id = int(detections[0, 0, i, 1])
            if confidence > 0.4 and class_id in self.classes:
                box = detections[0, 0, i, 3:7] * [w, h, w, h]
                x1, y1, x2, y2 = box.astype("int")
                label = self.classes[class_id]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        end = time.time()
        return frame, round((end - start) * 1000, 2)
