try:
    from sensor_lib.arduino import NANO
except:
    from devices.sensor_lib.arduino import NANO
import numpy as np
import time


class ESC:
    channels = 16
    motor_count = 4
    arm_val = 1050
    disarm_val = 1000
    max_val = 2000
    def __init__(self):
        self.nano = NANO() 
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
            cmd = [int(i) for i in np.clip(np.array(cmd), self.disarm_val, self.max_val)]
            self.nano.write_servo(cmd)
            self.last_cmd = cmd

    def test(self):
        self.arm()
        for i in range(self.motor_count):
            cmd = [self.disarm_val] * self.motor_count
            self.drive(cmd)
            cmd[i] = 1100
            input(f'Test motor {i}?')
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



    
        
