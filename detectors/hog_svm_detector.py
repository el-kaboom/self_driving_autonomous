import cv2
import time

class HOGSVMDetector:
    def __init__(self):
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(self, frame):
        start = time.time()
        boxes, _ = self.hog.detectMultiScale(frame, winStride=(8,8))
        end = time.time()

        for (x, y, w, h) in boxes:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        return frame, round((end - start) * 1000, 2)
