
import math
import numpy as np

from Control.balance import ControlSystem
from Control.planner import PLAN

'''
Receives:
- Quaternions
- Joystick CMDS
On a timer as well:
- PID updates

Outputs:
- Motor CMDS
'''

class FC(PLAN):
    LIM_PITCH = [-10, 10]
    LIM_ROLL = [-10, 10]
    LIM_YAW = [0, 360]
    LIM_HEIGHT = [0, -1]

    def __init__(self):
        self.CS = ControlSystem()

        # Roll, Pitch, Yaw
        self.cartesian_current = [0, 0, 0] # [x, y, z]
        self.cartesian_setpoints = [0, 0, 0] # [x, y, z]
        self.twist_current = [0, 0, 0, 0] # [dx, dy, dz, dth]
        self.euler_current = [0, 0, 0] # [roll, pitch, yaw]
        self.euler_setpoints = [0, 0, 0] # [roll, pitch, yee-haw]
        self.throttle_pid = [0, 0, 0, 0] # [roll, pitch, yaw, height]
        self.throttle_base = 1000

    def run(self):
        self.update_pid()
        # Roll axis facing forwards, ie Negative AC from front
        # Pitch axis facing right from front, ies Negative Facing up
        fl = self.throttle_base + self.throttle_pid[0] - self.throttle_pid[1] - self.throttle_pid[2] + self.throttle_pid[3] 
        fr = self.throttle_base - self.throttle_pid[0] - self.throttle_pid[1] + self.throttle_pid[2] + self.throttle_pid[3] 
        bl = self.throttle_base + self.throttle_pid[0] + self.throttle_pid[1] + self.throttle_pid[2] + self.throttle_pid[3] 
        br = self.throttle_base - self.throttle_pid[0] + self.throttle_pid[1] - self.throttle_pid[2] + self.throttle_pid[3] 
        # print(f'{[round(i) for i in self.throttle_pid]}')
        # print(f'PID0: {self.throttle_pid[2]:.2f} Ang0: {self.euler_current[2]:.2f}, Set0: {self.euler_setpoints[2]}' )
        # print(self.euler_offsets)
        return [fl, fr, bl, br]
        
    def update_pid(self):
        current = self.euler_current.append(self.cartesian_current[-1])
        setpoint = self.euler_setpoints.append(self.cartesian_setpoints[-1])
        self.throttle_pid = self.CS.run(current, setpoint)

    def update_setpoints(self, input, mode='angle'):
        if mode == 'twist':
            # input: [linear, angular]
            setpoints = self.twist_2_euler(input) 
        elif mode == 'pose':
            # input: 
            setpoints = self.pose_2_euler(input)
        else:
            # input: [euler]
            setpoints = input.append([0, 0, self.cartesian_current[-1]])

        # Apply limits
        
        [self.euler_setpoints, self.cartesian_setpoints] = setpoints

    def twist_2_euler(self, input, STEP=0.01):
        [linear, angular] = input
        twist_setpoint = linear.append(angular[-1]) # [dx, dy, dz, dth]
        error = [(i - j) / abs(max(i, j)) for (i, j) in zip(twist_setpoint, self.twist_current)] 
        roll = error[0] * STEP + self.euler_current[0] 
        pitch = error[1] * STEP + self.euler_current[1]
        yaw = error[2] * STEP + self.euler_current[2]
        height = error[3] * STEP + self.cartesian_current[2]
        return [[roll, pitch, yaw], [0, 0, height]]

    def pose_2_euler(self, input):
        return [[], []]


    
    def update_pose(self, pose, twist):
        # pose: [cartesian, euler]
        # twist: [linear, angular]
        [self.cartesian_current, self.euler_current] = pose
        self.twist_current = twist[0].append(twist[1][-1])




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
