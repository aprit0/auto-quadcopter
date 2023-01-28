# Inspired by https://github.com/simondlevy/OpenCV-Python-Hacks 
import time
import math
import cv2 as cv
import numpy as np

from devices.utils import quat_2_euler

class OpticalFlow:
    image_shape = [288,352]
    def __init__(self, camera_idx=0, show=False):
        self.pose = [0, 0, 0]  # X, Y, Z
        self.vel = [0, 0, 0]
        self.euler = None
        self.height = None

        # Camera
        self.show = show
        self.t_0 = None
        self.last_frame = None
        scale_percent = 100 # percent of original size
        self.new_width = int(self.image_shape[1] * scale_percent / 100)
        self.new_height = int(self.image_shape[0] * scale_percent / 100)
        # fourcc = cv.VideoWriter_fourcc('F', 'M', 'P', '4')
        # cv.VideoWriter_fourcc(*'XVID'),
        # self.video_grey = cv.VideoWriter('out_grey.avi', fourcc, 20.0, (self.image_shape[1], self.image_shape[0]))


    def get_velocities(self, current_frame):
        flow = cv.calcOpticalFlowFarneback(self.last_frame, current_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0) # 0.05ms
        magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])
        FPS = time.time() - self.t_0
        mag, ang = np.median(magnitude) * FPS, np.median(angle)
        # mag = mag/FPS if mag > 0.5 else 0 # Holding steady
        vel_x = mag * np.cos(ang)
        vel_y = mag * np.sin(ang)

        pixels_per_metre = lambda height, img_dim: (img_dim * 0.5) / height
        pp_x = pixels_per_metre(self.height, self.last_frame.shape[1])
        pp_y = pixels_per_metre(self.height, self.last_frame.shape[0])
        fx = vel_x / pp_x
        fy = vel_y / pp_y
        self.vel = [fx, fy, 0]
        

    def set_image(self, img, raw):
        new_frame = cv.resize(img, (self.new_width, self.new_height), interpolation=cv.INTER_AREA)
        # print(np.all(new_frame == self.last_frame)) # New Frame
        # print(img.shape)
        if self.last_frame is not None:
            self.get_velocities(new_frame)
        self.last_frame = new_frame
        self.t_0 = time.time()
        if self.show:
            cv.imwrite(f'camera.jpg', img)
            # self.video_grey.write(raw)
    
    def set_height(self, height):
        self.height = height

    def set_angle(self, quat):
        self.euler = [math.radians(i) for i in quat_2_euler(*quat)]