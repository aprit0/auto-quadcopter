
import imageio as iio
import numpy as np
import time, os

import cv2 as cv


class Video:
    ip_local ='192.168.8.113' 
    ip_nord = '100.90.67.157'
    ip_tail = '100.109.35.134'
    def __init__(self, save=False, id=ip_tail, fps=30):
        self.save = save
        self.id = id
        self.cap = None
        self.last_read = time.time()
        self.loop_time = 1 / fps
        self.raw = None
        self.processed = None
        self.count = 0
        cwd = os.getcwd()
        filepath = cwd + "/devices/failed.jpg" if "devices" not in cwd else cwd + "/failed.jpg"
        self.failed_image = cv.imread(filepath)
        print(self.failed_image)
    def init(self):
        self.cap = cv.VideoCapture(f'http://{self.id}:8080/video')
        self.cap.set(cv.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        # if time.time() - self.last_read > self.loop_time:
        if not self.cap:
            self.init()
        ret, self.raw = self.cap.read() # 1080p
        if type(self.raw) == type(None):
            self.cap = None
            print('failed')
            return self.failed_image
        self.last_read = time.time()
        self.process()
        print(self.processed.shape, self.failed_image.shape)
        return self.processed

    
    def process(self):
        self.processed = self.raw#cv.cvtColor(self.raw, cv.COLOR_BGR2GRAY)
        if self.save:
            self.show(self.processed)
    
    def show(self, im):
        cv.imwrite('camera.jpg', im)
        self.count += 1
        print(self.count)


if __name__ == '__main__':
    cam0 = Video(save=False)
    while True:
        _ = cam0.read()
