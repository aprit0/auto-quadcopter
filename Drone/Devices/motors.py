from adafruit_servokit import ServoKit
import numpy as np
import time


class ESC:
    channels = 16
    motors = 4
    def __init__(self):
        self.pca = ServoKit(channels=self.channels)
        for i in range(self.motors): # may be 0-4 or 11-15, unsure
            self.pca.servo[i].set_pulse_width_range(1000, 2000)
        self.map_val = lambda a: np.interp(a, [1000, 2000], [0, 180])
        self.motor_id = { # EDIT
            'fl':0,
            'fr':1,
            'bl':2,
            'br':3
        }
        self.armed = False

    def arm(self, arm_val=1050):
        self.drive([arm_val]*4)
        self.armed = True

    def disarm(self, disarm_val=900):
        self.drive([disarm_val]*4)
        self.armed = False 


    def drive(self, cmd):
        # cmd: [fl, fr, bl, br] where fl:br ∈ [1000, 2000]
        if self.armed: 
            mapped_cmd = self.map_val(cmd)
            index = 0
            for key in self.motor_id.keys():
                id = self.motor_id[key]
                self.pca.servo[id].angle = mapped_cmd[index]
                index += 1

    def test(self):
        self.arm()
        for i in range(self.motors):
            cmd = [1000] * 4
            self.drive(cmd)
            cmd[i] = 1100
            input(f'Test motor {i}?')
            self.drive(cmd)
            time.sleep(1)

if __name__ == '__main__':
    motor = ESC()
    print('Motor began')
    motor.test()


    
        
