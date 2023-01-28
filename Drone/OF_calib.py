import time
import cv2 as cv
import pandas as pd
import numpy as np

from devices.bno085 import INERTIAL
from control.optical_flow import OpticalFlow
from devices.camera import Video
from devices.tof import VL53
'''
IMU Y+ Forwards
IMU X+ Right


'''

if __name__ == '__main__':

    CAM = Video()
    OF = OpticalFlow(show=True)
    IMU = INERTIAL()
    TOF = VL53()
    t_imu = time.time()
    t_of = time.time()
    t_start = time.time()
    t_loop = time.time()
    OF_vel = None
    IMU_vel = None
    OF_disp = [0, 0]
    IMU_disp = [0, 0]
    df_vel = {'OFx':[], 'OFy':[], 'IMUx':[], 'IMUy':[], 'dt':[]}
    while time.time() - t_start < 15 + 3:
    # while True:
        if time.time() - t_of > 0.05 or OF_vel is None:
            df_vel['dt'].append(time.time() - t_loop)
            t_loop = time.time()

            height = TOF.read()
            OF.set_height(height)
            _ = CAM.read()
            dt = time.time() - t_of
            t_of = time.time() 
            image_np = CAM.processed
            OF.set_image(image_np, CAM.raw)
            OF_vel = [i * dt for i in OF.vel[:-1]]
            # OF_vel = [i if abs(i) > 0.3 else 0 for i in OF_vel ]
            OF_disp = [i + j for (i,j) in zip(OF_disp, OF_vel)]
            df_vel['OFx'].append(OF_vel[0])
            df_vel['OFy'].append(OF_vel[1])
            
            IMU.read()
            dt = time.time() - t_imu
            t_imu = time.time() 
            OF_vel = [i * dt for i in OF.vel[:-1]]
            IMU_vel = [i * dt  for i in IMU.linear_accel[:-1]]
            IMU_vel = [i if abs(i) > 0.2 else 0 for i in IMU_vel ]
            df_vel['IMUx'].append(IMU_vel[0])
            df_vel['IMUy'].append(IMU_vel[1])
            IMU_disp = [i + j for (i,j) in zip(IMU_disp, IMU_vel)]
            # print(i, [ f'OF | I: {i} | {j}' for (i,j) in zip(OF_disp, IMU_disp)], dt)
            print([ f'OF | I: {i} | {j}' for (i,j) in zip(OF_vel, IMU_vel)], dt)
            t_0 = time.time()
    df_out = pd.DataFrame(df_vel)
    df_out.to_csv('OF_Calib.csv')
    print('done')