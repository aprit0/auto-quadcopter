# Inspired by https://github.com/simondlevy/OpenCV-Python-Hacks 
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
  
    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize = (15, 15),
                    maxLevel = 2,
                    criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT,
                                10, 0.03))
    # image_shape = [480,640]
    def __init__(self, camera_idx=0, show=False):
        self.pose = [0, 0, 0]  # X, Y, Z
        self.vel = [0, 0, 0]
        self.euler = None
        self.height = None
        self.count = 0

        # Camera
        self.show = show
        self.t_0 = None
        self.last_frame = None
        # scale_percent = 100 # percent of original size
        # self.new_width = int(self.image_shape[1] * scale_percent / 100)
        # self.new_height = int(self.image_shape[0] * scale_percent / 100)
        # fourcc = cv.VideoWriter_fourcc('F', 'M', 'P', '4')
        # cv.VideoWriter_fourcc(*'XVID'),
        # self.video_grey = cv.VideoWriter('out_grey.avi', fourcc, 20.0, (self.image_shape[1], self.image_shape[0]))
        self.p0 = None


    def dense_velocities(self, current_frame):

        flow = cv.calcOpticalFlowFarneback(self.last_frame, current_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0) # 0.05ms
        magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])
        FPS = 1# time.time() - self.t_0
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
        if self.show:
            vis = self.draw_flow(self.img, flow)
            cv.imwrite(f'camera{self.count}.jpg', vis)
            self.count += 1
            # self.video_grey.write(raw)
    
    def sparse_velocities(self, current_frame):
        p1, st, err = cv.calcOpticalFlowPyrLK(self.last_frame,current_frame,self.p0, None,**self.lk_params)
        good_new = p1[st == 1]
        good_old = self.p0[st == 1]
        #vis
        mask = np.zeros_like(current_frame)
        color = np.random.randint(0, 255, (100, 3))
        frame = self.img.copy()
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            [a, b, c, d] = [int(i) for i in [a, b, c, d]]
            mask = cv.line(mask, (a, b), (c, d),color[i].tolist(), 2)
            
            frame = cv.circle(frame, (a, b), 5,color[i].tolist(), -1)

        d_good = np.subtract(good_new, good_old)
        magnitude = np.linalg.norm(d_good, axis=1)
        angle = np.arctan2(d_good[:, 1], d_good[:, 0])
        mag, ang = np.median(magnitude), np.median(angle)
        print(mag, math.degrees(ang))
        vel_x = mag * np.cos(ang) # pixels / frame
        vel_y = mag * np.sin(ang)
        pixels_per_metre = lambda height, img_dim: (img_dim * 0.5) / height
        pp_x = pixels_per_metre(self.height, self.last_frame.shape[1])
        pp_y = pixels_per_metre(self.height, self.last_frame.shape[0])
        fx = vel_x / pp_x
        fy = vel_y / pp_y
        self.vel = [fx, fy, 0]
        


        img = cv.add(frame, mask)
        cv.imwrite(f'camera{self.count}.jpg', img)
        self.count += 1

        self.p0 = good_new.reshape(-1, 1, 2)


    @staticmethod
    def draw_flow(img, flow, step=16):
        h, w = img.shape[:2]
        y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
        fx, fy = flow[y,x].T
        lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)
        vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
        cv.polylines(vis, lines, 0, (0, 255, 0))
        for (x1, y1), (x2, y2) in lines:
            cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
        return vis 

    def set_image(self, img, raw):
        self.img = img
        print(img.shape)
        self.raw = raw
        new_frame = img # cv.resize(img, (self.new_width, self.new_height), interpolation=cv.INTER_AREA)
        # print(np.all(new_frame == self.last_frame)) # New Frame
        # print(img.shape)
        if self.last_frame is not None:
            self.dense_velocities(new_frame)
            # self.sparse_velocities(new_frame)
        else:
            self.p0 = cv.goodFeaturesToTrack(new_frame, mask = None,
                                **self.feature_params)

        self.last_frame = new_frame.copy()
        self.t_0 = time.time()
    
    def set_height(self, height):
        self.height = height

    def set_angle(self, quat):
        self.euler = [math.radians(i) for i in quat_2_euler(*quat)]