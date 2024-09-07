from lib.simple_pid import PID


class FC_PID:
    '''
    FC: Designed as a barebones implementation for roll and pitch
    '''
    kp = 5
    ki = 0#0.1
    kd = 0#0.55
    def __init__(self):
        self.roll, self.pitch, self.yaw = PID(self.kp, self.ki, self.kd), PID(-self.kp, -self.ki, -self.kd), PID(self.kp, 0, 0)
        self.init()
        self.enabled = False

    def init(self, lim_o=100, proportional=False):
        self.roll.output_limits = (-lim_o, lim_o)
        self.pitch.output_limits = (-lim_o, lim_o)
        self.yaw.output_limits = (-lim_o, lim_o)
        self.roll.proportional_on_measurement = proportional
        self.pitch.proportional_on_measurement = proportional
        self.yaw.proportional_on_measurement = proportional
        self.pid_disable()
        self.set_euler_setpoints([0,0,0])

    def set_euler_setpoints(self, euler_setpoints):
        self.roll.setpoint = euler_setpoints[0]
        self.pitch.setpoint = euler_setpoints[1]
        self.yaw.setpoint = 0

    def run(self, euler_input):
        # [roll, pitch]
        yaw_pid = [self.yaw(euler_input[2])]
        output = [self.roll(euler_input[0]), self.pitch(euler_input[1])] + yaw_pid
        # rnd = lambda x: [round(i, 0) for i in x]
        # print('FC[balance]: ', rnd(euler_input) , rnd(euler_setpoint), rnd(output))

        return [i for i in output] # type: ignore

    def set_params(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.roll.set_tunings((kp, ki, kd))
        self.pitch.set_tunings((kp, ki, kd))
        self.yaw.set_tunings((kp, 0, 0))
    
    def get_params(self):
        return self.kp, self.ki, self.kd 

    def pid_disable(self) -> None:
        self.roll.set_auto_mode(False)
        self.pitch.set_auto_mode(False)
        self.yaw.set_auto_mode(False)
        self.enabled = False
    
    def pid_enable(self) -> None:
        self.roll.set_auto_mode(True)
        self.pitch.set_auto_mode(True)
        self.yaw.set_auto_mode(True)
        self.enabled = True



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
    

# import matplotlib.pyplot as plt
# if __name__ == '__main__':
#     vals = []
#     control_system = FC_PID()
#     dd = Drone()
#     roll = dd.roll
#     setpoint = 0
#     t_0 = time.monotonic()
#     counter = 0
#     control_system.pid_enable()
#     for i in range(1000):
#         throttle = control_system.run([roll, 0, 0], [setpoint, 0, 0])
#         roll = dd.update(throttle[0], time.monotonic() - t_0)
#         t_0 = time.monotonic()
#         if i > 10:
#             setpoint = 10
#             counter += 1
#         time.sleep(0.005)
#         print(counter, roll, setpoint, throttle[0])
#         vals.append(roll)
#     plt.plot(vals)
#     plt.show()
