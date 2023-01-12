import time, datetime
import math
import RTIMU
import numpy as np

class INERTIAL:
    SETTINGS_FILE = "RTIMULib_2"
    def __init__(self, zero=True):

        # Init
        self.s = RTIMU.Settings(self.SETTINGS_FILE)
        self.imu = RTIMU.RTIMU(self.s)
        self.name = self.imu.IMUName()
        if (not self.imu.IMUInit()):
            print("IMU Init Failed")
            sys.exit(1)
        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)
        self.poll_interval = self.imu.IMUGetPollInterval() # 0.004

        # params
        self.zero = zero # Remove initial rotations
        self.last_read = time.time()
        self.quat = [-1, -1, -1, -1] # x, y, z, w
        self.euler = [-1, -1, -1] # roll, pitch, yaw
        self.loop_window, self.loop_count, self.loop_mean = [], 10, 0
    
    def read(self):
        '''
        Returns: bool successful read
        '''
        if self.imu.IMURead():
            # Time loop
            self.loop_window.append(time.time() - self.last_read)
            if len(self.loop_window) > self.loop_count:
                self.loop_window.pop(0)
                self.loop_mean = np.mean(self.loop_window)
            # Get Data 
            data = self.imu.getIMUData()
            self.quat = data['fusionQPose']
            fusionPose = data['fusionPose']
            r, p, y = math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])
            self.euler = [r, p, y] 
            print(data['fusionPoseValid'], [f'{i:.2f}' for i in self.euler])
            self.last_read = time.time()
            if data['fusionQPoseValid']:
                return 1
        return 0

            
        
if __name__ == '__main__':
    mpu = INERTIAL()
    while True:
        mpu.read()
        # time.sleep(0.1)
        # print(mpu.quat)


