from YAMSPy.yamspy import MSPy
import time


class MW:
    def __init__(self, serial_port="/dev/ttyUSB0"):
        super().__init__(device=serial_port, loglevel='WARNING', baudrate=115200) #as board:
        self.connect()
        self.CMDS = {
            'roll': 1500,
            'pitch': 1500,
            'throttle': 900,
            'yaw': 1500,
            'aux1': 1000,  # DISARMED (1000) / ARMED (1800)
            'aux2': 1000,  # ANGLE (1000) / HORIZON (1500) / FLIP (1800)
            'aux3': 1000,  # FAILSAFE (1800)
            'aux4': 1000  # HEADFREE (1800)
        }
        self.CMD_key = list(self.CMDS.keys())
        self.state = {
            'kinematics': [0., 0., 0.],
            'altitude': 0,
            'sonar': 0
        }

    def update_state(self, axes):
        print('Arm: ', self.bit_check(self.CONFIG['mode'], 0))
        self.CMDS = {self.CMD_key[i]: axes[i] for i in range(axes)}
        self.fast_read_attitude()
        raw_cmd = [self.CMDS[key] for key in list(self.CMDS.keys())]
        self.send_RAW_RC(raw_cmd)
        self.state['kinematics'] = self.SENSOR_DATA['kinematics']
        self.state['altitude'] = self.SENSOR_DATA['altitude']
        self.state['sonar'] = self.SENSOR_DATA['sonar']
