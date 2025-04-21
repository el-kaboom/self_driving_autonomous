import matplotlib.pyplot as plt
from collections import deque
import threading
import time

class LivePlotter:
    def __init__(self, maxlen=30):
        self.yolo_times = deque(maxlen=maxlen)
        self.ssd_times = deque(maxlen=maxlen)
        self.hog_times = deque(maxlen=maxlen)

    def update(self, model, value):
        if model == "YOLOv8":
            self.yolo_times.append(value)
        elif model == "SSD":
            self.ssd_times.append(value)
        elif model == "HOG+SVM":
            self.hog_times.append(value)

    def run(self):
        plt.ion()
        fig, ax = plt.subplots()
        while True:
            ax.clear()
            ax.plot(self.yolo_times, label="YOLOv8", color='green')
            ax.plot(self.ssd_times, label="SSD", color='orange')
            ax.plot(self.hog_times, label="HOG+SVM", color='blue')
            ax.set_title("Model Inference Time (ms)")
            ax.legend()
            ax.set_ylim(0, 200)
            plt.pause(0.1)
