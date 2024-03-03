from communication import PICO2PI
import asyncio
import json
from pid import FC_PID
from sensors import INERTIAL




class FlightController(PICO2PI):
    PID_FILENAME = "PID_CALIB.json"
    def __init__(self, baudrate=115200):
        self.euler_setpoint = [0] * 3
        self.euler_current = [None] * 3
        self.ARM = False

        super(PICO2PI, self).__init__(baudrate)

        # PID
        self.P, self.I, self.D = None, None, None
        self.pid = FC_PID()
        self.load_pid()

        # Sensors
        self.imu = INERTIAL()
        
    async def main(self):
        self.euler_current = self.imu.read()

    def set_setpoints(self, msg):
        self.pid.set_euler_setpoints([i for i in msg.values()])

    def set_pid_setpoints(self, msg):
        self.pid.set_params(*msg.values())
            
    def set_arm(self, msg):
        self.ARM = bool(msg.values()[0])
        if self.ARM:
            self.pid.pid_enable()
        else:
            self.pid.pid_disable()
        

    def write_config(self, msg):
        self.write_pid()

    def get_pid_setpoints(self):
        self.pid.get_params()
        
    def get_pose(self):
        out_dict = {key: round(value, ndigits=1) for key, value in zip(["R", "P", "Y"], self.euler_current)}
        self.write_msg("get_pose", out_dict)      

    def load_pid(self):
        out = json.loads(self.PID_FILENAME)
        for key in out:
            setattr(self, key, out[key])
        self.pid.set_params(self.P, self.I, self.D)

    def set_pid(self, key, value):
        setattr(self, key, value)
        self.pid.set_params(self.P, self.I, self.D)
        
    def write_pid(self):
        pid_out = {key: value for key, value in zip(["P", "I", "D"], self.pid.get_params())} 
        with open(self.PID_FILENAME, "w") as f:
            json.dump(pid_out, f)



if __name__ == "__main__":
    FC = FlightController()
    loop = asyncio.get_event_loop()
    loop.create_task(FC.main())
    loop.create_task(FC.read_serial())
    loop.run_forever()