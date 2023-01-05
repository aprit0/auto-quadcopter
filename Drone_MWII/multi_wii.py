from YAMSPy.yamspy import MSPy
import serial
import numpy as np
import time


class MW(MSPy):
    def __init__(self, serial_port="/dev/ttyUSB1", feedback=False):
        super().__init__(device=serial_port, loglevel='WARNING', baudrate=115200) #as board:
        self.feedback = feedback
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
        self.ser.reset_input_buffer()
        if feedback:
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
        self.CMD_order = ['throttle', 'yaw', 'roll', 'pitch', 'aux1', 'aux2']
        self.state = {
            'kinematics': [0., 0., 0.],
            'altitude': 0,
            'sonar': 0
        }

    def exit(self):
        pass

    def send_PWM(self):
        CMD_out = [str(int(self.CMDS[key])) for key in self.CMD_order]
        print('CMD_IN', CMD_out)
        t_0 = time.time()
        send_string = ','.join(CMD_out)
        send_string += "\n"
        self.ser.write(send_string.encode('utf-8'))
        line = self.ser.readline().decode('utf-8').rstrip()
        print('CMD_OUT', line, time.time() - t_0)

        

    def update_state(self):
        if self.feedback:
            self.fast_read_attitude()
            self.state['kinematics'] = self.SENSOR_DATA['kinematics']
            self.state['altitude'] = self.SENSOR_DATA['altitude']
            self.state['sonar'] = self.SENSOR_DATA['sonar']
        
    def update_cmds(self, axes):
        t_0 = time.time()
        # Update CMDS
        ax = axes[:6]#0: LX,1: LY,2: LT,3: RX,4: RY,5: RT
        bt = axes[6:]
        self.CMDS['roll'] = ax[0] 
        self.CMDS['pitch'] = ax[4] 
        self.CMDS['throttle'] = ax[1] 
        self.CMDS['yaw'] = ax[3] 

        self.CMDS['aux1'] = bt[0] 
        self.CMDS['aux2'] = bt[1] 
        self.CMDS['aux3'] = bt[2] 
        self.CMDS['aux4'] = bt[3] 
        t_1 = time.time()
        #print('Arm: ', self.bit_check(self.CONFIG['mode'], 0))
        self.send_PWM()
        # print('update ', t_1-t_0, time.time() - t_1)

    def main(self, axes):
        t_0 = time.time()
        self.update_state()
        t_1 = time.time()
        self.update_cmds(axes)
        # print('main ', t_1-t_0, time.time() - t_1)
        
if __name__ == '__main__':
    FC = MW()
    while True:
        a = [1000]*8
        FC.update_state(a)
        print(a)
