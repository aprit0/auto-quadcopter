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
    feature_params = dict( maxCorners = 100,
                qualityLevel = 0.3,
                minDistance = 7,
                blockSize = 7 )
    
    lk_params = dict( winSize = (15, 15),
            maxLevel = 2,
            criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT,
                        10, 0.03))
    def __init__(self, camera_idx=0):
        self.pose = [0, 0, 0]  # X, Y, Z
        # Distance sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.tof = adafruit_vl53l0x.VL53L0X(i2c)
        self.tof.measurement_timing_budget = 20000
        self.euler = None
        self.height = None

        # Camera
        self.camera = cv.VideoCapture(camera_idx)  # cv.CAP_DSHOW
        self.camera.set(cv.CAP_PROP_BUFFERSIZE, 2)
        self.camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
        self.last_frame, self.t_0, frame = self.get_image()
        self.mask = np.zeros_like(frame)
        self.mask[..., 1] = 255
        fourcc = cv.VideoWriter_fourcc(*'XVID')
        self.video = cv.VideoWriter('output.avi', fourcc, 20.0, (frame.shape[1], frame.shape[0]))
        self.p0 = cv.goodFeaturesToTrack(self.last_frame, mask = None,
                            **self.feature_params)

    def get_image(self, show=False):
        ret, img = self.camera.read()
        # img = img[55:, :] # Remove bodywork from image
        scale_percent = 10 # percent of original size
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        new_frame = cv.resize(img, (width, height), interpolation=cv.INTER_AREA)
        grey = cv.cvtColor(new_frame, cv.COLOR_BGR2GRAY)
        t_dot = time.time()
        if show:
            cv.imshow("input", img)
            cv.imwrite('test.jpg', img)
        return grey, t_dot, new_frame

    def get_pose(self):
        time_0 = time.time()
        self.get_height() # 0.035ms
        if self.euler is not None and self.height is not None:
            time_1 = time.time()
            current_frame, t_1, _ = self.get_image() # 0.016ms with compression
            time_2 = time.time()
            # flow = cv.calcOpticalFlowFarneback(self.last_frame, current_frame,
                                            #    None, 0.5, 3, 15, 3, 5, 1.2, 0) # 0.05ms
            p1, st, err = cv.calcOpticalFlowPyrLK(self.last_frame, current_frame,
                                           self.p0, None,
                                           **self.lk_params)
            magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])
            time_3 = time.time()
            self.visualise(magnitude, angle, current_frame)
            print(f'Height: {time_1 - time_0}, image: {time_2 - time_1}, flow: {time_3 - time_2}, viz: {time.time() - time_3}')
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
        # cv.imwrite("test_flow.jpg", rgb)
        # cv.imwrite("test_grey.jpg", grey)
        self.video.write(rgb)


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
    for i in range(10):
        OF.set_angle([0, 0, 0, 0])
        OF.get_pose()
        print(OF.pose, i)
    
