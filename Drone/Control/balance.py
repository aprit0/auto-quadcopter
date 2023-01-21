from simple_pid import PID

'''
Inputs:
- Euler 

Outputs:
- Throttle adjustments

Functions:
- Balance roll and pitch angles given by joysticks using PID
    Future:
    - Height mode given by barometer
    - Hover mode given by optical flow
'''

class ControlSystem:
    def __init__(self):
        self.kp =1.2
        '''
        7: takeoff then small oscillation
        '''
        self.ki = 0.1#.01#0.001# 0.01
        self.kd = 0.55# 1
        '''
        0.01:
        '''
        self.setpoint = 0
        
        self.roll, self.pitch = PID(self.kp, self.ki, self.kd), PID(-self.kp, -self.ki, -self.kd)
        self.yaw = PID(-2, 0, 0)
        # self.height = PID()
        self.init()

    def init(self, lim_o=100, proportional=False):
        self.roll.output_limits = (-lim_o, lim_o)
        self.pitch.output_limits = (-lim_o, lim_o)
        self.yaw.output_limits = (-lim_o, lim_o)
        self.roll.proportional_on_measurement = proportional
        self.pitch.proportional_on_measurement = proportional
        self.yaw.proportional_on_measurement = proportional

    def run(self, euler_input, euler_setpoint):
        self.roll.setpoint = euler_setpoint[0]
        self.pitch.setpoint = euler_setpoint[1]
        self.pitch.setpoint = euler_setpoint[2]
        output = [self.roll(euler_input[0]), self.pitch(euler_input[1]),  self.yaw(euler_input[2])]

        return [float(i) for i in output]

    def get_params(self):
        self.kp, self.ki, self.kd = self.roll.components

    def update_params(self, kp, ki, kd):
        self.roll.tunings(kp, ki, kd)
        self.pitch.tunings(kp, ki, kd)
        self.yaw.tunings(kp, ki, kd)

import random
import time
class Drone:
    # Simulation drone one axis
    def __init__(self):
        self.roll = 0

    def update(self, response, dt):
        if response > 0:
            self.roll += 1 * response * dt
        
        self.roll += random.randint(-2, 2) * dt
        return self.roll

if __name__ == '__main__':
    control_system = ControlSystem()
    dd = Drone()
    roll = dd.roll
    setpoint = 0
    t_0 = time.time()
    counter = 0
    for i in range(100):
        throttle = control_system.run([roll, 0, 0], [setpoint, 0, 0])
        roll = dd.update(throttle[0], time.time() - t_0)
        t_0 = time.time()
        if i > 10:
            setpoint = 10
            counter += 1
        time.sleep(0.01)
        print(counter, roll, setpoint, throttle[0])



    


    




        
