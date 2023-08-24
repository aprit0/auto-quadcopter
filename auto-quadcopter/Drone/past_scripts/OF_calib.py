import time
import cv2 as cv
import pandas as pd
import numpy as np

from devices.bno085 import INERTIAL
from control.optical_flow import OpticalFlow
from devices.camera import Video
from devices.tof import ZAXIS
'''
IMU Y+ Forwards
IMU X+ Right


'''

if __name__ == '__main__':

    CAM = Video(save=True)
    OF = OpticalFlow(show=True)
    TOF = ZAXIS()
    OF_vel = [0, 0] 
    OF_disp = [0, 0]
    df_vel = {'OFx':[], 'OFy':[], 'OFdx':[], 'OFdy':[], 'dt':[]}
    # while time.time() - t_start < 15:
    # while True:
    for i in range(4):
        input('Photo?')
        TOF.read()
        height = TOF.height
        OF.set_height(height)
        _ = CAM.read()
        dt = 1
        df_vel['dt'].append(dt)
        t_of = time.time() 

        image_np = CAM.processed
        OF.set_image(image_np, CAM.raw)
        OF_vel = [i * dt for i in OF.vel[:-1]]
        # OF_vel = [i if abs(i) > 0.3 else 0 for i in OF_vel ]
        OF_disp = [i + j for (i,j) in zip(OF_disp, OF_vel)]
        df_vel['OFx'].append(OF_disp[0])
        df_vel['OFy'].append(OF_disp[1])
        df_vel['OFdx'].append(OF_vel[0])
        df_vel['OFdy'].append(OF_vel[1])
        
    df_out = pd.DataFrame(df_vel)
    print(df_out)
    df_out.to_csv('OF_Calib.csv')
    print('done')