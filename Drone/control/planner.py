import numpy as np

'''
Functions
1. Read and convert joystick values
    - Determine mode due to buttons
    - Determine throttle output range due to mode
2. Joy ranges
    - Convert Joystick to dx or x
        - (process x to path of dx)
    - Convert dx to angle

'''

class PLAN:
    def __init__(self):
        self.ARM = 0
        self.MODE = 0
        self.joy_angle_range = [[-5, 5], [-180, 180]]
        self.joy_input_range = [1000, 2000]
        self.button_output_range = [0, 1]
        self.buttons_old = [0] * 11
        self.velocities = None


    def read_joystick(self, axes, buttons):
        self.update_buttons(buttons)
        [throttle, r, p, y, _, _] = list(axes)
        self.throttle_base = throttle
        self.euler_setpoints[:2] = np.interp([r, p], self.joy_input_range, self.joy_angle_range[0])
        self.euler_setpoints[2] = np.interp([y], self.joy_input_range, self.joy_angle_range[1])
        # print(f'{throttle:.2f}, {self.euler_setpoints}')

    def update_buttons(self, buttons):
        buttons_new = np.interp(buttons, self.joy_input_range, self.button_output_range)
        self.ARM = self.latch_button(self.ARM, buttons_new[0], self.buttons_old[0])
        self.MODE = self.latch_button(self.MODE, buttons_new[1], self.buttons_old[1])
        self.buttons_old = buttons_new

    @staticmethod
    def latch_button(current, new, old):
        if old == 0 and new != 0 and current == 0:
            return 1
        elif old == 0 and new != 0 and current == 1:
            return 0
        else:
            return current