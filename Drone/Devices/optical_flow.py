import board
import busio
import adafruit_vl53l0x
import time
import math
import cv2 as cv
import numpy as np


class OpticalFlow:
    def __init__(self, camera_idx=0):
        self.pose = [0, 0, 0]  # X, Y, Z
        # Distance sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.tof = adafruit_vl53l0x.VL53L0X(i2c)
        self.angle = None
        self.height = None

        # Camera
        self.camera = cv.VideoCapture(camera_idx)  # cv.CAP_DSHOW
        self.camera.set(cv.CAP_PROP_BUFFERSIZE, 2)
        self.last_frame, self.t_0 = self.get_image()

    def get_image(self, show=False):
        ret, new_frame = self.camera.read()
        grey = cv.cvtColor(new_frame, cv.COLOR_BGR2GRAY)
        t_dot = time.time()
        if show:
            cv.imshow("input", grey)
        return grey, t_dot

    def get_pose(self):
        if self.angle is not None and self.height is not None:
            current_frame, t_1 = self.get_image()
            flow = cv.calcOpticalFlowFarneback(self.last_frame, current_frame,
                                               None, 0.5, 3, 15, 3, 5, 1.2, 0)
            flow_xy = np.sum(flow, axis=(0, 1))
            FPS = t_1 - self.t_0
            count = (flow.shape[0] * flow.shape[1])
            average_velocity_pixels_per_second = (flow_xy / count / FPS)
            vel = self._velocity_meters_per_second(average_velocity_pixels_per_second, flow.shape,
                                                   self.height, self.angle)
            self.pose[0] += vel[1] * FPS
            self.pose[1] += vel[0] * FPS
            self.pose[2] = self.height

            self.t_0 = t_1
            self.last_frame = current_frame

            return self.pose

    def get_height(self):
        self.height = self.tof.range * 0.001  # In metres

    def set_angle(self, angle):
        self.angle = angle

    @staticmethod
    def _velocity_meters_per_second(velocity_pixels_per_second, dimsize_pixels, height_metres, perspective_angle):
        distance_pixels = ((dimsize_pixels / 2) /
                           math.tan(perspective_angle / 2))
        pixels_per_meter = distance_pixels / height_metres
        return velocity_pixels_per_second / pixels_per_meter


if __name__ == '__main__':
    OF = OpticalFlow()
    while True:
        OF.get_height()
        OF.get_angle()
        OF.get_pose()
        print(OF.pose)
