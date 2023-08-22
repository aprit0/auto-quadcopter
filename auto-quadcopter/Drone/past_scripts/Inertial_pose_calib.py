import time
import cv2 as cv
import pandas as pd
import numpy as np

from devices.bno085 import INERTIAL
'''
IMU Y+ Forwards
IMU X+ Right


'''

if __name__ == '__main__':

    IMU = INERTIAL()
    t_imu = time.time()
    t_start = time.time()
    t_loop = time.time()
    IMU_vel = [0, 0]
    IMU_disp = [0, 0]
    df_vel = {'IMUx':[], 'IMUy':[],'IMUvx':[], 'IMUvy':[], 'dt':[]}
    calib = []
    loop_period = 0.01
    # calibrate velocity
    while time.time() - t_start < 3:
        if time.time() - t_imu > loop_period:
            IMU.read()
            dt = (time.time() - t_imu) 
            calib.append([i * dt for i in IMU.linear_accel])
            t_imu = time.time()
    calib_array = np.array(calib)
    offset_vel = np.median(calib_array, axis=0)
    print('vel_off',offset_vel)
    print(calib[:10])
    # time.sleep(2)

    t_start = time.time()
    while time.time() - t_start < 15:
        if time.time() - t_imu > loop_period:
            IMU.read()
            dt = time.time() - t_imu
            t_imu = time.time() 
            ddisp = [IMU.linear_accel[i] * dt for i in range(2)]
            IMU_vel = [IMU_vel[i] + ddisp[i] for i in range(2)]
            IMU_vel = [IMU_vel[i] if abs(ddisp[i]) > 1e-3 else 0 for i in range(len(ddisp))]


            IMU_disp = [IMU_disp[i] + IMU_vel[i] * dt for i in range(2)]
            # print(IMU_vel[0])
            # IMU_vel = [i if abs(i) > 1e-5 else 0 for i in IMU_vel ]
            if dt < 3 * loop_period:
                print('yep', IMU_vel, IMU_disp)
                df_vel['dt'].append(time.time() - t_imu)
                df_vel['IMUvx'].append(IMU_vel[0])
                df_vel['IMUvy'].append(IMU_vel[1])
                df_vel['IMUx'].append(IMU_disp[0])
                df_vel['IMUy'].append(IMU_disp[1])
            else:
                print('nope ', dt)
            # print(i, [ f'OF | I: {i} | {j}' for (i,j) in zip(OF_disp, IMU_disp)], dt)
            # print(IMU_disp, dt)
            t_0 = time.time()
    df_out = pd.DataFrame(df_vel)
    df_out.to_csv('OF_Calib.csv')
    print('done')