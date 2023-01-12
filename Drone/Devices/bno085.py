import time
import numpy as np
from board import SCL, SDA
from busio import I2C
from adafruit_bno08x import (
    BNO_REPORT_STEP_COUNTER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

class INERTIAL:
    SETTINGS_FILE = "RTIMULib_2"
    def __init__(self, zero=True):

        # Init
        i2c = I2C(SCL, SDA, frequency=800000)
        self.imu = BNO08X_I2C(i2c)
        self.imu.enable_feature(BNO_REPORT_STEP_COUNTER)
        self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.imu.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)

        # params
        self.zero = zero # Remove initial rotations
        self.quat = [-1, -1, -1, -1] # x, y, z, w
        self.euler = [-1, -1, -1] # roll, pitch, yaw
    
    def read(self):
        '''
        Returns: bool successful read
        Current loop speed of max: 0.0022, min: 0.00067
        '''
        # Get Data 
        i, j, k, w = self.imu.quaternion
        self.quat = [i, j, k, w]
        r, p, y = self.quat_2_euler(i, j, k, w)
        self.euler = [r, p, y] 
        return 1

    @staticmethod
    def quat_2_euler(x, y, z, w):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)

        t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
        Y = np.degrees(np.arcsin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))

        return X, Y, Z

            
        
if __name__ == '__main__':
    mpu = INERTIAL()
    while True:
        mpu.read()
        # time.sleep(0.1)
        # print(mpu.quat)


