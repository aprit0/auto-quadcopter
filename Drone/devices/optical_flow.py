# Inspired by https://github.com/simondlevy/OpenCV-Python-Hacks 
import time
import math
import cv2 as cv
import numpy as np
import imageio as iio

from devices.utils import quat_2_euler

class OpticalFlow:
    def __init__(self, camera_idx=0):
        self.pose = [0, 0, 0]  # X, Y, Z
        self.euler = None
        self.height = None

        # Camera
        self.camera = iio.get_reader("<video0>")
        self.last_frame, self.t_0, frame = self.get_image()
        self.mask = np.zeros_like(frame)
        self.mask[..., 1] = 255
        self.video_grey = cv.VideoWriter('output_grey.avi', cv.VideoWriter_fourcc(*'XVID'), 20.0, (frame.shape[1], frame.shape[0]))
        self.video_flow = cv.VideoWriter('output_flow.avi', cv.VideoWriter_fourcc(*'XVID'), 20.0, (frame.shape[1], frame.shape[0]))

    def get_image(self, show=False):
        img = None
        img = self.camera.get_next_data()
        # img = img[55:, :] # Remove bodywork from image
        scale_percent = 40 # percent of original size
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
        if self.euler is not None and self.height is not None:
            # time_1 = time.time()
            current_frame, t_1, img = self.get_image() # 0.016ms with compression
            # time_2 = time.time()

            flow = cv.calcOpticalFlowFarneback(self.last_frame, current_frame,
                                               None, 0.5, 3, 15, 3, 5, 1.2, 0) # 0.05ms
            magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])
            # print(magnitude, angle)

            time_3 = time.time()
            self.visualise(magnitude, angle, img)
            # print(f'Height: {time_1 - time_0}, image: {time_2 - time_1}, flow: {time_3 - time_2}, viz: {time.time() - time_3}')
            mag, ang = np.median(magnitude), np.median(angle)
            if mag > 1e-3:
                FPS = t_1 - self.t_0
                # print('MAG: ', mag,ang, FPS)
                mag = mag/FPS if mag > 0.5 else 0 # Holding steady
                vel_x = mag * np.cos(ang)
                vel_y = mag * np.sin(ang)
                pixels_per_metre = lambda height, img_dim: (img_dim * 0.5) / height
                pp_x = pixels_per_metre(self.height, self.last_frame.shape[1])
                pp_y = pixels_per_metre(self.height, self.last_frame.shape[0])
                fx = vel_x / pp_x
                fy = vel_y / pp_y
                # count = (flow.shape[0] * flow.shape[1])
                # vel = self._velocity_meters_per_second(average_velocity_pixels_per_second, flow.shape[:-1],
                                                    #    self.height, self.euler[0])
                self.pose[0] += fx * FPS
                self.pose[1] += fy * FPS
                self.pose[2] = self.height
                print(self.pose)

                self.t_0 = t_1
                self.last_frame = current_frame
                # print(
                #     f'{[str(round(i, 5)) for i in self.pose]} '#{[str(round(i, 5)) for i in self.euler]}'
                #     )
                return self.pose
        # print(self.euler, self.height)
        return None
    def visualise(self, magnitude, angle, img):
        self.mask[..., 0] = angle * 180 / np.pi / 2
        # Sets image value according to the optical flow
        # magnitude (normalized)
        self.mask[..., 2] = cv.normalize(magnitude, None, 0, 255, cv.NORM_MINMAX)
        # Converts HSV to RGB (BGR) color representation
        rgb = cv.cvtColor(self.mask, cv.COLOR_HSV2BGR)
        # Opens a new window and displays the output frame
        # cv.imwrite("test_flow.jpg", rgb)
        # cv.imwrite("test_grey.jpg", grey)
        self.video_flow.write(rgb)
        self.video_grey.write(img)


    def set_height(self, height):
        self.height = height

    def set_angle(self, quat):
        self.euler = [math.radians(i) for i in quat_2_euler(*quat)]

    @staticmethod
    def _velocity_meters_per_second(velocity_pixels_per_second, dimsize_pixels, height_metres, perspective_angle):
        perspective_angle = 1e-15 if perspective_angle == 0 else perspective_angle
        pixels_per_meter = [i / 2 / (math.tan(perspective_angle / 2)) / height_metres for i in dimsize_pixels]
        return np.divide(velocity_pixels_per_second, pixels_per_meter) 


if __name__ == '__main__':
    OF = OpticalFlow()
    # for i in range(150):
    t_0 = 0
    try:
        while True:
            if time.time() - t_0 > 0.01:
                OF.set_height(0.2)
                OF.set_angle([0, 0, 0, 0])
                OF.get_pose()
                # print(time.time() - t_0)
                t_0 = time.time()
    except Exception as e:
        print(e)
        OF.camera.close()
    
    
