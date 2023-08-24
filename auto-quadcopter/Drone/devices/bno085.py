import time
import math
from board import SCL, SDA
from busio import I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_LINEAR_ACCELERATION
)
from adafruit_bno08x.i2c import BNO08X_I2C
# from utils import quat_2_euler
from devices.utils import quat_2_euler

class INERTIAL:
    SETTINGS_FILE = "RTIMULib_2"
    def __init__(self, zero=True):

        # Init
        i2c = I2C(SCL, SDA, frequency=800000)
        self.imu = BNO08X_I2C(i2c)
        self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.imu.enable_feature(BNO_REPORT_GYROSCOPE)
        self.imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

        # params
        self.quat = [] # x, y, z, w
        self.euler = [] # roll, pitch, yaw
        self.euler_last = []
        self.linear_accel = [] # x, y, z
        self.linear_vel = [0] * 3 # x, y, z
        self.angular_vel = []
        self.t_0 = None
        self.status = 0

        self.read()
    
    def read(self):
        '''
        Returns: bool successful read
        Current loop speed of max: 0.0022, min: 0.00067
        '''
        # Get Data 
        ax, ay, az = self.imu.linear_acceleration
        self.linear_accel = [ax, ay, az]
        i, j, k, w = self.imu.quaternion
        self.quat = [i, j, k, w]
        gyro_x, gyro_y, gyro_z = self.imu.gyro
        self.angular_vel = [gyro_x, gyro_y, gyro_z]
        r, p, y = quat_2_euler(i, j, k, w)
        self.euler = [r, p, y] 
        if self.t_0 is not None:
            dt = time.time() - self.t_0
            vel = [i * dt for i in self.linear_accel]
            self.linear_vel[0] = math.cos(math.radians(p)) * vel[0]
            self.linear_vel[1] = math.cos(math.radians(r)) * vel[1]
            self.linear_vel[2] = math.cos(math.radians(r)) * math.cos(math.radians(p)) * vel[2]
        self.t_0 = time.time()
        self.euler_last = self.euler
        self.status = 1 # No health check implemented
        return self.status

        
if __name__ == '__main__':
    mpu = INERTIAL()
    t_0 = time.time()
    while True:
        if time.time() - t_0 > 0.01:
            mpu.read()
            t_0 = time.time()
            # time.sleep(0.1)
            print([round(i, 4) for i in mpu.linear_vel])


