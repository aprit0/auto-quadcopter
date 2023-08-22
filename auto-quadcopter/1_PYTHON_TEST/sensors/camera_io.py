import imageio as iio
import numpy as np
import time

import cv2 as cv

camera = iio.get_reader("<video0>")
idx = 0
while True:
    t_0 = time.time()
    for i in range(50):
        screenshot = camera.get_data(0)
    print('loop:', (time.time() - t_0 )/50, screenshot.shape)
    grey = cv.cvtColor(screenshot, cv.COLOR_BGR2GRAY)
    cv.imwrite('test_grey.jpg', grey)