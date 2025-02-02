import serial
import time
from yamspy import MSPy
import numpy as np

# <TYPE,KEY:VALUE,KEY:VALUE> ... <TYPE,DATA,DATA>
message_mapping = {
    # Setters
    "A": "set_setpoints",
    "B": "set_pid_setpoints",
    "C": "set_arm",
    "D": "write_config",
    # Getters
    "Z": "get_pid_setpoints",
    "Y": "get_pose",
    # Debug
    "0x0": "get_log"
}
inv_message_mapping = {value: key for [key, value] in message_mapping.items()}


class PI2MSP(object):
    PWM_ARM = 1800
    PWM_DISARM = 1000
    CMDS_init = {
    'roll':     1500,
    'pitch':    1500,
    'throttle': 900,
    'yaw':      1500,
    'aux1':     1000, # DISARMED (1000) / ARMED (1800)
    'aux2':     1000, # ANGLE (1000) / HORIZON (1500) / FLIP (1800)
    'aux3':     1000, # FAILSAFE (1800)
    'aux4':     1000  # HEADFREE (1800)
    }
    def __init__(self, port="/dev/ttyUSB0") -> None:
        self.port = port
        self.board = None
        self.shutdown = False
        self.CMDS = self.CMDS_init.copy()

        self.buffer = ""
        self.t_0 = 0
        self.euler_pose = []
        self.cmd_euler = []


    def set_setpoints(self):
        if self.cmd_euler:
            # _msg = ["T", "R", "P", "Y"]
            roll_pwm = np.interp(np.clip(self.cmd_euler[0], -10, 10), [-10, 10], [1300, 1700])
            pitch_pwm = np.interp(np.clip(self.cmd_euler[1], -10, 10), [-10, 10], [1000, 2000])
            yaw_pwm = np.interp(self.cmd_euler[2], [-10, 10], [1000, 2000])
            self.CMDS["roll"] = roll_pwm
            self.CMDS["pitch"] = pitch_pwm
            self.CMDS["yaw"] = yaw_pwm
            self.CMDS["throttle"] = self.throttle
            self.cmd_euler = []

    def clip_z(self, z):
        while not 360 > z >= 0:
            if z > 360:
                z -= 360
            elif z < 0:
                z += 360
        return z
    
    def set_pid_setpoints(self, _msg):
        # out_dict = {key: round(value, 1) for key, value in zip(["P", "I", "D"], _msg)}
        # self.write_msg("set_pid_setpoints", out_dict)
        pass

    def set_arm(self):
        self.CMDS["aux1"] = self.PWM_ARM if self.ARM else self.PWM_DISARM

    def get_pid_setpoints(self, _msg=""):
        if self.board.send_RAW_msg(MSPy.MSPCodes['MSP_PID'], data=[]):
            dataHandler = self.board.receive_msg()
            self.board.process_recv_data(dataHandler)
            print("get_pid_setpoints", self.board.PIDs)

    def get_pose(self, _msg=""):
        # Kinematics:, "YAW" ["ROLL RIGHT DOWN +", "PITCH FRONT DOWN +", "YAW CLOCKWISE + [0,360)""]
        self.euler_pose = [round(float(i),1) for i in self.board.SENSOR_DATA['kinematics']]

    def main(self):
        while not self.shutdown:
            try:
                with MSPy(device=self.port, loglevel='WARNING', baudrate=115200) as self.board:
                    if self.board == 1:
                        print("board state: ", self.board)
                        time.sleep(0.1)
                        continue
                    while self.board and not self.shutdown:
                        # Setup
                        self.set_setpoints()
                        # MSP Commands
                        # print(self.CMDS)
                        self.board.fast_msp_rc_cmd([self.CMDS[i] for i in self.CMDS])
                        self.board.fast_read_attitude()
                        # Internal Commands
                        self.get_pose()
                        print(f"[{True if self.CMDS['aux1'] == self.PWM_ARM else False}]: ", {i:round(j,0) for (i,j) in self.CMDS.items()}, self.euler_pose)
            except Exception as e:
                print(e)

if __name__ == "__main__":
    comms = PI2MSP()
    comms.main()