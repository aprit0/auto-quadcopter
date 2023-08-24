
import imageio as iio
import numpy as np
import time

import cv2 as cv


class Video:
    def __init__(self, save=False, id='0', fps=30):
        self.save = save
        self.id = id
        self.stream = iio.get_reader("<video0>")
        # self.cap = cv.VideoCapture(0)
        # self.cap.set(cv.CAP_PROP_BUFFERSIZE, 1)
        self.last_read = time.time()
        self.loop_time = 1 / fps
        self.raw = None
        self.processed = None
        self.count = 0

    def read(self):
        if time.time() - self.last_read > self.loop_time:
            self.raw = self.stream.get_next_data()
            # _, self.raw = self.cap.read()
            self.last_read = time.time()
            self.process()
            return 1
        else:
            return 0
    
    def process(self):
        self.processed = cv.cvtColor(self.raw, cv.COLOR_BGR2GRAY)
        if self.save:
            self.show(self.processed)
    
    def show(self, im):
        cv.imwrite(f'camera_{self.count}.jpg', im)
        self.count += 1
        print(self.count)


if __name__ == '__main__':
    cam0 = Video(save=True)
    while True:
        cam0.read()
