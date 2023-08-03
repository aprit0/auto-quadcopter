from simple_pid import PID


class FC:
    '''
    FC: Designed as a barebones implementation for roll and pitch
    '''
    def __init__(self):
        self.kp =1.2
        self.ki = 0.1
        self.kd = 0.55
        self.setpoint = 0
        self.roll, self.pitch = PID(self.kp, self.ki, self.kd), PID(-self.kp, -self.ki, -self.kd)
        self.init()

    def init(self, lim_o=100, proportional=False):
        self.roll.output_limits = (-lim_o, lim_o)
        self.pitch.output_limits = (-lim_o, lim_o)
        self.roll.proportional_on_measurement = proportional
        self.pitch.proportional_on_measurement = proportional
        self.pid_disable()

    def run(self, euler_input, euler_setpoint):
        # [roll, pitch]
        self.roll.setpoint = euler_setpoint[0]
        self.pitch.setpoint = euler_setpoint[1]
        output = [self.roll(euler_input[0]), self.pitch(euler_input[1])]
        rnd = lambda x: [round(i, 0) for i in x]
        # print('FC[balance]: ', rnd(euler_input) , rnd(euler_setpoint), rnd(output))

        return [float(i) for i in output] # type: ignore

    def get_params(self):
        self.kp, self.ki, self.kd = self.roll.components

    def update_params(self, kp, ki, kd):
        # self.roll.tunings(kp, ki, kd)
        # self.pitch.tunings(kp, ki, kd)
        pass

    def pid_disable(self) -> None:
        self.roll.auto_mode = False
        self.pitch.auto_mode = False
    
    def pid_enable(self) -> None:
        self.roll.auto_mode = True 
        self.pitch.auto_mode = True



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
    control_system = FC()
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
