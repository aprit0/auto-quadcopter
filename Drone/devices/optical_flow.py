# Inspired by https://github.com/simondlevy/OpenCV-Python-Hacks 
import board
import busio
import adafruit_vl53l0x
import time
import math
import cv2 as cv
import numpy as np

from devices.utils import quat_2_euler

class OpticalFlow:
    def __init__(self, camera_idx=0):
        self.pose = [0, 0, 0]  # X, Y, Z
        # Distance sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.tof = adafruit_vl53l0x.VL53L0X(i2c)
        self.euler = None
        self.height = None

        # Camera
        self.camera = cv.VideoCapture(camera_idx)  # cv.CAP_DSHOW
        # self.camera.set(cv.CAP_PROP_BUFFERSIZE, 2)
        # self.camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
        self.last_frame, self.t_0 = self.get_image()
        ret, img = self.camera.read()
        self.mask = np.zeros_like(img)
        self.mask[..., 1] = 255
        self.video = cv.VideoWriter('output_video.mp4', cv.VideoWriter_fourcc(*'MP4V'), 30,(self.mask.shape[0], self.mask.shape[1]))

    def get_image(self, show=False):
        ret, img = self.camera.read()
        # img = img[55:, :] # Remove bodywork from image
        scale_percent = 100 # percent of original size
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        new_frame = cv.resize(img, (width, height), interpolation=cv.INTER_AREA)
        grey = cv.cvtColor(new_frame, cv.COLOR_BGR2GRAY)
        t_dot = time.time()
        if show:
            cv.imshow("input", img)
            cv.imwrite('test.jpg', img)
        return grey, t_dot

    def get_pose(self):
        self.get_height() # 0.035ms
        if self.euler is not None and self.height is not None:
            current_frame, t_1 = self.get_image() # 0.016ms with compression
            flow = cv.calcOpticalFlowFarneback(self.last_frame, current_frame,
                                               None, 0.5, 3, 15, 3, 5, 1.2, 0) # 0.05ms
            # flow_xy = np.sum(flow, axis=(0, 1))


            magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])
            self.visualise(magnitude, angle, current_frame)
            mag = np.median(magnitude) # pixels / period
            ang = np.median(angle)
            print(mag, math.degrees(ang))
            FPS = t_1 - self.t_0
            vel_pixels = mag * FPS
            pixels_per_metre = lambda height, img_dim: (img_dim * 0.5) / height
            pp_x = pixels_per_metre(self.height, self.last_frame.shape[1])
            pp_y = pixels_per_metre(self.height, self.last_frame.shape[0])

            # count = (flow.shape[0] * flow.shape[1])
            # vel = self._velocity_meters_per_second(average_velocity_pixels_per_second, flow.shape[:-1],
                                                #    self.height, self.euler[0])
            self.pose[0] += 0.# * FPS
            self.pose[1] += 0.# * FPS
            self.pose[2] = self.height

            self.t_0 = t_1
            self.last_frame = current_frame
            # print(
            #     f'{[str(round(i, 5)) for i in self.pose]} '#{[str(round(i, 5)) for i in self.euler]}'
            #     )
            return self.pose
        else:
            print(self.euler, self.height)
            return None
    def visualise(self, magnitude, angle, grey):
        self.mask[..., 0] = angle * 180 / np.pi / 2
        # Sets image value according to the optical flow
        # magnitude (normalized)
        self.mask[..., 2] = cv.normalize(magnitude, None, 0, 255, cv.NORM_MINMAX)
        # Converts HSV to RGB (BGR) color representation
        rgb = cv.cvtColor(self.mask, cv.COLOR_HSV2BGR)
        # Opens a new window and displays the output frame
        cv.imwrite("test_flow.jpg", rgb)
        # cv.imwrite("test_grey.jpg", grey)


    def get_height(self):
        self.height = self.tof.range * 0.001  # In metres

    def set_angle(self, quat):
        self.euler = [math.radians(i) for i in quat_2_euler(*quat)]

    @staticmethod
    def _velocity_meters_per_second(velocity_pixels_per_second, dimsize_pixels, height_metres, perspective_angle):
        perspective_angle = 1e-15 if perspective_angle == 0 else perspective_angle
        pixels_per_meter = [i / 2 / (math.tan(perspective_angle / 2)) / height_metres for i in dimsize_pixels]
        return np.divide(velocity_pixels_per_second, pixels_per_meter) 


if __name__ == '__main__':
    OF = OpticalFlow()
    while True:
        OF.set_angle(0)
        OF.get_pose()
        print(OF.pose)
