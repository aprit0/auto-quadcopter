import time
from board import SCL, SDA
from busio import I2C
from adafruit_bno08x import (
    BNO_REPORT_STEP_COUNTER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from devices.utils import quat_2_euler

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
        self.quat = [] # x, y, z, w
        self.euler = [] # roll, pitch, yaw
    
    def read(self):
        '''
        Returns: bool successful read
        Current loop speed of max: 0.0022, min: 0.00067
        '''
        # Get Data 
        i, j, k, w = self.imu.quaternion
        self.quat = [i, j, k, w]
        r, p, y = quat_2_euler(i, j, k, w)
        self.euler = [r, p, y] 
        return 1

        
if __name__ == '__main__':
    mpu = INERTIAL()
    while True:
        mpu.read()
        # time.sleep(0.1)
        print(mpu.euler)


