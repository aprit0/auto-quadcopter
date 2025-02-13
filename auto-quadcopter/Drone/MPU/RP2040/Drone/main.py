import asyncio
import json
import math as m
import time
from communication import PICO2PI
from motors import MOTORS
from pid import FC_PID
from sensors import INERTIAL




class FlightController(PICO2PI):
    PID_FILENAME = "PID_CALIB.json"
    THROTTLE_BASE = 1000
    TEST_LIMIT = 1200
    def __init__(self, test_mode=False):
        self.euler_setpoints = [0] * 3
        self.throttle_setpoint = 1000
        self.euler_current = [None] * 3
        self.ARM = False
        self.test_mode = test_mode
        super().__init__()

        # PID
        self.P, self.I, self.D = None, None, None
        self.pid = FC_PID()
        self.load_pid()

        # Sensors
        self.imu = INERTIAL()

        # Motors
        self.motors = MOTORS()
        
    async def main(self):
        if self.test_mode:
            self.set_arm({"any_value": True})
        while True:
            self.euler_current = self.imu.read()
            y_new = self.euler_setpoints[2] - self.euler_current[2]
            yaw_dist = y_new if abs(y_new) < 180 else y_new + (-1 * m.copysign(1, y_new) * 360)
            pid_out = self.pid.run(self.euler_current[:2]+[yaw_dist])
            # print("main euler: ", self.euler_current[:2], yaw_dist)
            if self.pid.enabled or self.test_mode:
                # print("main pid: ", self.throttle_setpoint, pid_out)
                '''
                self.euler_current = [roll, pitch] + [yaw]
                roll: +ive euler, LHS Up, RHS Down, LHS less speed, RHS more speed, pid more negative
                pitch: +ve euler, Back Up, Front Down, Back less speed, Front more speed, pid more negative 
                '''
                fl = self.throttle_setpoint + pid_out[0] - pid_out[1] #+ pid_out[2]
                fr = self.throttle_setpoint - pid_out[0] - pid_out[1] #- pid_out[2]
                br = self.throttle_setpoint - pid_out[0] + pid_out[1] #+ pid_out[2]
                bl = self.throttle_setpoint + pid_out[0] + pid_out[1] #- pid_out[2]
                if self.test_mode:
                    [fl, fr, br, bl] = [min(self.TEST_LIMIT, i) for i in [fl, fr, br, bl]]
            else:
                fl, fr, bl, br = self.THROTTLE_BASE, self.THROTTLE_BASE, self.THROTTLE_BASE, self.THROTTLE_BASE 
            self.motors.set_speed(fl, fr, bl, br, self.ARM)
            await asyncio.sleep(0)
        
    def set_setpoints(self, msg):
        self.throttle_setpoint = int(msg["T"])
        self.euler_setpoints = [float(msg[key]) for key in ["R", "P", "Y"]]
        self.euler_setpoints[2] = self.euler_setpoints[2] if abs(self.euler_setpoints[2]) < 180 else self.euler_setpoints[2] + (-1 * m.copysign(1, self.euler_setpoints[2]) * 360) 
        self.pid.set_euler_setpoints([i for i in self.euler_setpoints])

    def set_arm(self, msg):
        self.ARM = bool(int(list(msg.values())[0]))
        if self.ARM:
            self.pid.pid_enable()
        else:
            self.pid.pid_disable()
        
    def write_config(self, _msg):
        self.write_pid()

    def get_pid_setpoints(self, _msg):
        out = self.pid.get_params()
        # self.set_log(out)
        self.write_msg("get_pid_setpoints", {i:j for i, j in zip(["P", "I", "D"], out)})

        
    def get_pose(self, _msg):
        # print("pose")
        out_dict = {key: round(value, 1) for key, value in zip(["R", "P", "Y"], self.euler_current)}
        self.write_msg("get_pose", out_dict)

    def load_pid(self):
        f = open(self.PID_FILENAME)
        out = json.load(f)
        for key in out:
            setattr(self, key, out[key])
        self.pid.set_params(self.P, self.I, self.D)

    def set_pid_setpoints(self, msg):
        out = list(msg.items())
        # self.set_log(out)
        for (key, value) in out:
            setattr(self, key, float(value))
        self.pid.set_params(self.P, self.I, self.D)
        # print(self.pid.kp)
        
    def write_pid(self):
        pid_out = {key: value for key, value in zip(["P", "I", "D"], self.pid.get_params())} 
        # print(pid_out)
        with open(self.PID_FILENAME, "w") as f:
            json.dump(pid_out, f)


def custom_exception_handler(loop, context):
    print(context)
    loop.default_exception_handler(context)
    loop.stop()

if __name__ == "__main__":
    FC = FlightController(False)
    loop = asyncio.get_event_loop()
    # loop.set_exception_handler(custom_exception_handler)
    loop.create_task(FC.main())
    # loop.create_task(FC.read_serial())
    loop.create_task(FC.update_state())
    loop.create_task(FC.sender())
    loop.create_task(FC.receiver())
    loop.run_forever()
