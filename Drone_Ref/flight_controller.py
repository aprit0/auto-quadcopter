import time
import numpy as np

from i2c_comms import I2C_MSG

class NANO():
    def __init__():
        self.COMS = I2C_MSG()

        self.CMD = {
            'height': 1000, # delta metres from previous timestep 
            'roll': 1500, # (n, 1000, 2000, a, b)
            'pitch': 1500, # (n, 1000, 2000, a, b)
            'yaw': 1500, # (n, 1000, 2000, a, b)
        }
        self.CMD_key = list(self.CMD.keys())
        self.modes = {
            'aux1': 1000,  # DISARMED (1000) / ARMED (2000)
            'aux2': 1000,  
            'aux3': 1000,  
            'aux4': 1000  
        }
        self.mode_key = list(self.mode.keys())
        self.pose = {
            'kinematics': [0., 0., 0.], # Roll, Pitch, Yaw
            'altitude': 0,
            'sonar': 0
        }
        self.range_joy = [1000, 2000] # joysticks expected range
        self.range_int = [0, 254] # Int range for transmission


    def update_CMD(axes):
        # Receives joystick angles
        self.CMD = {self.CMD_key[i]: axes[i] for i in range(len(axes))}
        CMD = [np.interp(axes[i], self.range_joy, self.range_int) for i in self.CMD.values()]
        self.COMS.send_block(0, CMD)

    def update_modes(buttons):
        new_mode = {self.mode_key[i]: buttons[i] for i in range(len(self.mode_key))}
        if self.mode != new_mode:
            print(f'Not equal: {self.mode}, {new_mode}')
            self.mode = new_mode
            mode = [np.interp(axes[i], self.range_joy, self.range_int) for i in self.mode.values()]
            self.COMS.send_block(1, mode)

    def update_pose(self):
        raw_pose = self.COMS.read_block(0, 4)
        self.pose['kinematics'] = raw_pose[1:3]
        self.pose['altitude'] = raw_pose[0]
        # self.pose['sonar'] = raw_pose[4] 