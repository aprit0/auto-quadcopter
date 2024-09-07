from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_LINEAR_ACCELERATION
)
from utils import quat_2_euler
import busio, board


class INERTIAL(BNO08X_I2C):
    def __init__(self) -> None:
        self.i2c = busio.I2C(board.GP3, board.GP2,frequency=400000)
        super().__init__(self.i2c)
        self.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.enable_feature(BNO_REPORT_GYROSCOPE)
        self.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

        self.quat = None
        self.euler = None
        pass


    def read(self):
        i, j, k, w = self.quaternion
        self.quat = [i, j, k, w]
        r, p, y = quat_2_euler(i, j, k, w)
        self.euler = [r, p, y] 
        # print(self.euler)
        return self.euler
        # return [0,0,1]
                 