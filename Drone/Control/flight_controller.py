
import math

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
    def __init__(self):
        self.CS = ControlSystem()
        self.ARM = 0
        self.MODE = 0

        # Roll, Pitch, Yaw
        self.euler_current = [0, 0, 0]
        self.euler_setpoints = [0, 0, 0]
        self.throttle_base = 1000
        self.throttle_pid = [0, 0, 0]
        self.joy_angle_range = [[-15, 15], [-15, 15], [0, 360]]
        self.joy_input_range = [1000, 2000]
        self.button_output_range = [[0,1], [0, 10]]

    def run(self):
        self.update_pid()
        # Roll axis facing forwards, ie Negative AC from front
        # Pitch axis facing right from front, ie Negative Facing up
        fl = self.throttle_base + self.throttle_pid[0] - self.throttle_pid[1] 
        fr = self.throttle_base - self.throttle_pid[0] - self.throttle_pid[1] 
        bl = self.throttle_base + self.throttle_pid[0] + self.throttle_pid[1] 
        br = self.throttle_base - self.throttle_pid[0] + self.throttle_pid[1] 
        return [fl, fr, bl, br]
        
    def read_joystick(self, axes, buttons):
        throttle, r, p, y, _ = axes
        self.throttle_base = throttle
        self.euler_setpoints = np.interp([r, p, y], self.joy_input_range, self.joy_angle_range)
        print(self.euler_setpoints)

        self.ARM, self.MODE = bnp.interp(buttons[:2], self.joy_input_range, self.button_output_range)
        self.update_buttons()

    def update_buttons():
        self.ARM = 0 if self.ARM < 0.5 else 1

    def update_pid(self):
        self.throttle_pid = self.CS.run(self.euler_current, self.euler_setpoints)

    def update_pose(self, q):
        self.euler_current = self.euler_from_quaternion(q[0], q[1], q[2], q[3])

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
        
            return [roll_x, pitch_y, yaw_z] # in radians 