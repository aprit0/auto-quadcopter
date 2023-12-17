import math, os
from simple_pid import PID
import numpy as np
try:
    from control.flight_controller import FC
except:
    from flight_controller import FC

class FCI:
    def __init__(self) -> None:
        self.FC = FC()
        self.original_odom = None
        self.cartesian_current = [0, 0, 0] # [x, y, z]
        self.cartesian_setpoints = [0, 0, 0] # [x, y, z]
        self.twist_current = [0, 0, 0, 0] # [dx, dy, dz, dth]
        self.twist_setpoints = [0, 0, 0, 0] # [dx, dy, dz, dth]
        self.euler_current = [0, 0, 0] # [roll, pitch, yaw]
        self.euler_setpoints = [0, 0, 0] # [roll, pitch, yee-haw]
        self.throttle_base = 1000
        self.flight_mode = 0
        self.bools = {
            "arm": bool(0),
            "hold": bool(0),
            "mode":bool(0),
            }
        
        lim_o = 100
        # Yaw
        self.yaw = PID(5, 0, 0)
        self.yaw.output_limits = (-lim_o, lim_o)

        # Hover XY
        kp = 1.2
        ki = 0.1
        kd = 0.55
        self.roll, self.pitch = PID(kp, ki, kd), PID(-kp, -ki, -kd)
        self.roll.output_limits = (-lim_o, lim_o)
        self.pitch.output_limits = (-lim_o, lim_o)

        # Hover Z
        kp_h = 40#2
        ki_h = 0#0.2
        kd_h = 0#1
        self.height = PID(kp_h, ki_h, kd_h)
        self.height.output_limits = (-lim_o, lim_o)

        self.rnd = lambda x: [round(float(i), 1) for i in x]
    
    def run(self) -> list:
        # Returns cmd[4]: [1000, 2000]
        if not self.bools['arm'] or not self.original_odom:
            os.system("clear")
            print(f"FCI[disarmed]][Twist:{self.bools['mode']}]: Arm:{self.bools['arm']}, Odom:{bool(self.original_odom)}")
            return [self.throttle_base]*4
        current = self.euler_current + [self.cartesian_current[-1]]
        setpoint_euler = self.euler_setpoints + [self.cartesian_current[-1]]
        setpoint_twist = self.calc_twist()
        setpoint_pose = [] # self.calc_pose()
        if self.bools["mode"] == True:
            # velocity control
            setpoint = setpoint_twist + self.twist_setpoints[2:][::-1] # [r:0, p:1, y:2, h:3]
            pass
        elif self.bools["mode"] == 2:
            # position control
            print("Position Control")
            # https://github.com/alduxvm/DronePilot/blob/master/mw-hover-controller.py
            setpoint = setpoint_pose
        else:
            print("Euler Control")
            setpoint = setpoint_euler
        # Yaw calcs
        self.original_odom[0][1][-1] += setpoint[2] * 0.1
        yaw_setpoint = self.original_odom[0][1][-1]
        yaw_setpoint = yaw_setpoint if abs(yaw_setpoint) < 180 else yaw_setpoint + (-1 * np.sign(yaw_setpoint) * 360) 
        yaw_current = current[2]
        y_new = yaw_setpoint - yaw_current
        yaw_dist = y_new if abs(y_new) < 180 else y_new + (-1 * np.sign(y_new) * 360) 
        self.yaw.setpoint = 0
        setpoint[2] = yaw_setpoint
        pid_yaw = self.yaw(yaw_dist) 
        # print(yaw_dist, yaw_current, yaw_setpoint, pid_yaw)
        
        # Height calcs
        # setpoint[3] = self.original_odom[0][0][-1]
        # self.height.setpoint = setpoint[3]
        # pid_height = self.height(current[3])
        # TMP: Until Z control
        pid_height = self.twist_setpoints[2] - 1000
        current[3] = pid_height
        setpoint[3] = pid_height
        self.pid_rp = self.FC.run(current, setpoint)

        # Roll axis facing forwards, ie Negative AC from front
        # Pitch axis facing right from front, ies Negative Facing up
        # Motors are as facing front. Ie from the persons perspective looking at them
        fl = self.throttle_base + self.pid_rp[0] + self.pid_rp[1] + pid_yaw + pid_height # type: ignore
        fr = self.throttle_base + self.pid_rp[0] - self.pid_rp[1] - pid_yaw + pid_height # type: ignore
        br = self.throttle_base - self.pid_rp[0] - self.pid_rp[1] + pid_yaw + pid_height # type: ignore
        bl = self.throttle_base - self.pid_rp[0] + self.pid_rp[1] - pid_yaw + pid_height # type: ignore #-pid_yaw#
        # print(f"PID: {self.rnd(self.pid_rp + [pid_yaw, pid_height])}")
        # print(f'PID0: {self.throttle_pid[0]:.2f} Ang0: {self.euler_current[0]:.2f}, Set0: {self.euler_setpoints[0]}' )
        # print(f"FCI[Update]: {[f'E: {j}/{i} T: {k}/{l}' for [i, j, k, l] in zip(self.rnd(self.euler_setpoints), self.rnd(self.euler_current), self.rnd(self.twist_current), self.rnd(self.twist_setpoints))]}")
        print(f"FCI[Update][{self.bools['mode']}]: {[f'{k.upper()}: {i}/{j}' for [i, j, k] in zip(self.rnd(current), self.rnd(setpoint), ['r', 'p', 'y', 'h'])]}")
        return [fr, fl, bl, br]
    

    def calc_twist(self, rp_gain: int = 7) -> list:
        # print(f"FCI[twist]: {[f'{i}/{j}' for (i, j) in zip(self.rnd(self.twist_current), self.rnd(self.twist_setpoints))]}")
        error = [np.sign(j) * (i - j)/j if j!= 0 else i for (i, j) in zip(self.twist_current[:2], self.twist_setpoints[:2])]
        # print(f"FCI[error]: {self.rnd(error)}")
        setpoint = [i + j*rp_gain for (i, j) in zip(self.euler_setpoints[:2], error)]
        # Enforce range limits
        setpoint[:2] = list(np.clip(setpoint, -10, 10))
        # setpoint[2] = setpoint[2] if setpoint[2] < 180 or setpoint[2] > -180 else setpoint[2]  % (np.sign(setpoint[2] ) * 180) - (np.sign(setpoint[2] ) * 180)

        return setpoint[:2]
    
    # Update variables
    def update_euler_setpoints(self, input) -> None:
        # [x, y, z, w]
        self.euler_setpoints = self.euler_from_quaternion(*input) 

    def update_twist_setpoints(self, input) -> None:
        [l, a] = input # [linear, angular]
        self.twist_setpoints = [l.x, l.y, l.z, a.z]

    def update_pose(self, pose, twist) -> None:
        # pose: [cartesian, euler]
        # twist: [linear, angular]
        if not self.original_odom:
            self.original_odom = [pose, twist]
        [self.cartesian_current, self.euler_current] = pose
        twist_new = twist
        self.twist_current = twist_new[0] + [twist[1][-1]]

    def update_bools(self, name: str, value: bool = False) -> None:
        self.bools[name] = value
        if self.bools['arm']:
            self.FC.pid_enable()
            self.yaw.auto_mode = True
            self.height.auto_mode = True
        else:
            self.FC.pid_disable()
            self.yaw.auto_mode = False 
            self.height.auto_mode = False 



    
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
