import math
from simple_pid import PID
import numpy as np
try:
    from control.flight_controller import FC
except:
    from flight_controller import FC

class FCI:
    def __init__(self) -> None:
        self.FC = FC()
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
            }
        
        lim_o = 100
        # Yaw
        self.yaw = PID(1, 0, 0)
        self.yaw.output_limits = (-lim_o, lim_o)

        # Hover XY
        kp = 1.2
        ki = 0.1
        kd = 0.55
        self.roll, self.pitch = PID(kp, ki, kd), PID(-kp, -ki, -kd)
        self.roll.output_limits = (-lim_o, lim_o)
        self.pitch.output_limits = (-lim_o, lim_o)

        # Hover Z
        kp_h = 2
        ki_h = 0.2
        kd_h = 1
        self.height = PID(kp_h, ki_h, kd_h)
        self.height.output_limits = (-lim_o, lim_o)

        self.rnd = lambda x: [round(float(i), 1) for i in x]
    
    def run(self) -> list:
        # Returns cmd[4]: [1000, 2000]
        if not self.bools['arm']:
            print("FCI[disarmed]")
            return [self.throttle_base]*4
        current = self.euler_current + [self.cartesian_current[-1]]
        setpoint_euler = self.euler_setpoints + [self.cartesian_setpoints[-1]]
        setpoint_twist = self.calc_twist()
        setpoint_pose = [] # self.calc_pose()
        if self.flight_mode == 1:
            # velocity control
            setpoint = setpoint_twist
            pass
        elif self.flight_mode == 2:
            # position control
            # https://github.com/alduxvm/DronePilot/blob/master/mw-hover-controller.py
            setpoint = setpoint_pose
        else:
            setpoint = setpoint_euler

        setpoint[2] = current[2]
        self.yaw.setpoint = current[2]
        pid_yaw = self.yaw(setpoint[2]) 
        self.height.setpoint = setpoint[3]
        pid_height = self.height(current[3])
        self.pid_rp = self.FC.run(current, setpoint)
        # Roll axis facing forwards, ie Negative AC from front
        # Pitch axis facing right from front, ies Negative Facing up
        fl = self.throttle_base + self.pid_rp[0] - self.pid_rp[1] #- pid_yaw + pid_height # type: ignore
        fr = self.throttle_base + self.pid_rp[0] + self.pid_rp[1] #+ pid_yaw + pid_height # type: ignore
        bl = self.throttle_base - self.pid_rp[0] + self.pid_rp[1] #+ pid_yaw + pid_height # type: ignore
        br = self.throttle_base - self.pid_rp[0] - self.pid_rp[1] #- pid_yaw + pid_height # type: ignore
        # print(f"PID: {self.pid_rp + [pid_yaw, pid_height]}")
        # print(f'PID0: {self.throttle_pid[0]:.2f} Ang0: {self.euler_current[0]:.2f}, Set0: {self.euler_setpoints[0]}' )
        # print(f"FCI[Update]: {[f'E: {j}/{i} T: {k}/{l}' for [i, j, k, l] in zip(rnd(self.euler_setpoints), rnd(self.euler_current), rnd(self.twist_current), rnd(self.twist_setpoints))]}")
        print(f"FCI[Update][{self.flight_mode}]: \
              {[f'{k.upper()}: {i}/{j}' for [i, j, k] in zip(self.rnd(current), self.rnd(setpoint), ['r', 'p', 'y', 'h'])]}")
        return [fl, fr, bl, br]
    

    def calc_twist(self, gain: int=1) -> list:
        # print(f"FCI[twist]: {self.rnd(self.twist_current)}")
        error = [(i - j) * gain for (i, j) in zip(self.twist_current, self.twist_setpoints)]
        # print(f"FCI[error]: {error}")
        setpoint = [i + j for (i, j) in zip(self.euler_setpoints + [self.cartesian_setpoints[-1]], error)]
        # Enforce range limits
        setpoint[:2] = list(np.clip(setpoint[:2], -10, 10))
        setpoint[2] = setpoint[2] if setpoint[2] < 180 or setpoint[2] > -180 else setpoint[2]  % (np.sign(setpoint[2] ) * 180) - (np.sign(setpoint[2] ) * 180)
        setpoint[3] = max(0, setpoint[3])

        return setpoint
    
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
