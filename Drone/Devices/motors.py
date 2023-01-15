from adafruit_servokit import ServoKit
import numpy as np
import time


class ESC:
    channels = 16
    motor_channels = { # EDIT
        'fl':12,
        'fr':13,
        'bl':15,
        'br':14
        # 'test': 0
    }
    arm_val = 1050
    disarm_val = 1000
    max_val = 2000
    def __init__(self):
        self.pca = ServoKit(channels=self.channels)
        self.motor_count = len(list(self.motor_channels.keys()))
        self.motor_id = list(self.motor_channels.values())
        for i in self.motor_id: # may be 0-4 or 11-15, unsure
            self.pca.servo[i].set_pulse_width_range(self.disarm_val, self.max_val)
        self.map_val = lambda a: np.interp(a, [self.disarm_val, self.max_val], [0, 180])
        self.armed = False
        self.last_cmd = [0]*self.motor_count
        self.disarm()

    def arm(self):
        self.drive([self.arm_val]*self.motor_count, force=1)
        self.armed = True 

    def disarm(self):
        self.drive([self.disarm_val]*self.motor_count, force=1)
        self.armed = False 


    def drive(self, cmd, force=0):
        # cmd: [fl, fr, bl, br] where fl:br ∈ [1000, 2000]
        if self.armed or force: 
            cmd = np.clip(np.array(cmd), self.disarm_val, self.max_val)
            mapped_cmd = self.map_val(cmd)
            index = 0
            for key in self.motor_channels.keys():
                servo_id = self.motor_channels[key]
                cmd_id = self.motor_id.index(self.motor_channels[key])

                self.pca.servo[servo_id].angle = mapped_cmd[cmd_id]
                index += 1
            self.last_cmd = cmd

    def test(self):
        self.arm()
        for i in range(self.motor_count):
            cmd = [self.disarm_val] * self.motor_count
            self.drive(cmd)
            cmd[i] = 1100
            input(f'Test motor {self.motor_id[i]}?')
            self.drive(cmd)
            time.sleep(1.5)
        self.disarm()



if __name__ == '__main__':
    motor = ESC()
    while True:
        x = int(input('Motor began \n0: Disarm \n1: Test \n2: Drive'))
        if not x:
            motor.disarm()
        elif x == 1:
            print('Begin Test')
            motor.test()
        elif x == 2:
            x = int(input('Motor speed: '))
            motor.arm()
            cmd = [x]*4
            motor.drive(cmd)
            time.sleep(1)



    
        
