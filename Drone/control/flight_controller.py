
import math
import numpy as np

from Control.balance import ControlSystem


'''
Receives:
- Quaternions
- Joystick CMDS
On a timer as well:
- PID updates

Outputs:
- Motor CMDS
'''

class FC:
    def __init__(self, calib_imu=True):
        self.CS = ControlSystem()

        # Calib
        self.calib_imu = calib_imu
        self.euler_offsets = None
        # Roll, Pitch, Yaw
        self.euler_current = [0, 0, 0]
        self.euler_setpoints = [0, 0, 0]
        self.throttle_pid = [0, 0, 0]
        self.throttle_base = 1000

    def run(self):
        self.update_pid()
        # Roll axis facing forwards, ie Negative AC from front
        # Pitch axis facing right from front, ie Negative Facing up
        fl = self.throttle_base + self.throttle_pid[0] - self.throttle_pid[1] - self.throttle_pid[2] 
        fr = self.throttle_base - self.throttle_pid[0] - self.throttle_pid[1] + self.throttle_pid[2]
        bl = self.throttle_base + self.throttle_pid[0] + self.throttle_pid[1] + self.throttle_pid[2]
        br = self.throttle_base - self.throttle_pid[0] + self.throttle_pid[1] - self.throttle_pid[2]
        print(f'{[round(i) for i in self.throttle_pid]}')
        print(f'PID0: {self.throttle_pid[2]:.2f} Ang0: {self.euler_current[2]:.2f}, Set0: {self.euler_setpoints[2]}' )
        print(self.euler_offsets)
        return [fl, fr, bl, br]
        
    def update_pid(self):
        self.throttle_pid = self.CS.run(self.euler_current, self.euler_setpoints)

    def update_pose(self, euler):
        if self.calib_imu and type(self.euler_offsets) == type(None):
            self.euler_offsets = [-1 * i for i in euler]
            # Only apply offsets to yaw
            self.euler_offsets[0] = 0
            self.euler_offsets[1] = 0
        elif not self.calib_imu and type(self.euler_offsets) == type(None):
            self.euler_offsets = [0, 0, 0]
        self.euler_current = [i + j for (i, j) in zip(euler, self.euler_offsets)]


    @staticmethod
    def euler_from_quaternion(x, y, z, w):
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return [math.degrees(i) for i in [pitch_y, roll_x, yaw_z]] # in radians 
