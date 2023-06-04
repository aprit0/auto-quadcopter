
import math
import numpy as np

try:
    from control.balance import ControlSystem
    from control.planner import PLAN
except:
    from balance import ControlSystem
    from planner import PLAN


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
        self.twist_setpoints = [0, 0, 0, 0] # [dx, dy, dz, dth]
        self.euler_current = [0, 0, 0] # [roll, pitch, yaw]
        self.euler_setpoints = [0, 0, 0] # [roll, pitch, yee-haw]
        self.throttle_pid = [0, 0, 0, 0] # [roll, pitch, yaw, height]
        self.throttle_base = 1000
        self.bools = {
            "arm": bool(0),
            "hold": bool(0)
            }

    def run(self):
        self.update_pid('twist')
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
        
    def update_pid(self, mode: str='twist'):
        '''
        REQ: Current and Setpoint transposed from twist to euler
        input: [0, 5, 0] m/s && [1, 4, 1, 1] -> 
        '''
        # Modify to match [roll, pitch, yaw, height]
        # current = self.twist_current[:2] + [self.twist_current[3], self.twist_current[2]] 
        self.update_setpoints()
        current = self.euler_current + [self.cartesian_current[-1]]
        setpoint = self.euler_setpoints + [self.cartesian_setpoints[-1]]

        self.throttle_pid = self.CS.run(current, setpoint)

    def update_setpoints(self, mode: str='twist'):
        if mode == 'twist':
            # input: [linear, angular]
            setpoints = self.twist_2_euler() 
        elif mode == 'pose':
            # input: 
            setpoints = self.pose_2_euler()
        else:
            # input: [euler]
            # setpoints = input.append([0, 0, self.cartesian_current[-1]])
            raise

        # Apply limits 
        [self.euler_setpoints, self.cartesian_setpoints] = setpoints
        rnd = lambda x: [round(i, 0) for i in x]
        # print(f'Update: {rnd(self.euler_setpoints)} : {rnd(self.euler_current)} : {rnd(self.twist_current)} : {rnd(self.twist_setpoints)} ')

    def twist_2_euler(self, STEP=10, MAX_ANG=10):
        error = [(i - j) if i != 0.0 or j != 0.0 else 0.0 for (i, j) in zip(self.twist_setpoints, self.twist_current)] 
        print(error)
        # Convert error to angle/height with STEP multiplier
        roll = np.clip(error[0] * STEP + self.euler_current[0], -MAX_ANG, MAX_ANG) 
        pitch = np.clip(error[1] * STEP + self.euler_current[1], -MAX_ANG, MAX_ANG)
        # height = error[2] * STEP + self.cartesian_current[2] # for position hold
        height = self.cartesian_current[2]

        yaw = error[3] * STEP + self.euler_current[2]
        if yaw > 360:
            yaw -= 360
        elif yaw < 0:
            yaw += 360
        else:
            pass
        return [[roll, pitch, yaw], [0, 0, height]]

    def pose_2_euler(self, input):
        return [[], []]


    def update_twist_setpoints(self, input):
        [linear, angular] = input
        self.twist_setpoints = linear[0] + [angular[1][-1]]
    
    def update_pose(self, pose, twist):
        # pose: [cartesian, euler]
        # twist: [linear, angular]
        [self.cartesian_current, self.euler_current] = pose
        twist_new = twist
        self.twist_current = twist_new[0] + [twist[1][-1]]


    def update_bools(self, name: str, value: bool = False, get: bool = True):
        if get:
            return self.bools[name]
        else:
            self.bools[name] = value




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
