import os
import time
import pygame
import numpy as np
from copy import deepcopy

from controllers.xbox_wireless import CONTROLLER as XBOX
from controllers.ps5_wireless import CONTROLLER as PS5

'''
Aim: General controller class for ROS
- Not ROS connected
Functions
- .get_event()
- .update_state()

See update_state for XBOX one Controller keymaps
'''


class Joystick:
    STEP = 0.05  # Increment for increase in axes
    DISC_HOME = 5  # Timeout for controller to reconnect before returning home

    def __init__(self):
        self.reset = {'axes': [-1] + [0.] * 5,  # + [-1.] + [0.] * 2 + [-1.],
                      'buttons': [0.] * 11}
        self.state = deepcopy(self.reset) 
        self.state_old = deepcopy(self.reset) 
        self.state_debounce = deepcopy(self.reset) 
        self.time_disconnect = None
        self.status = 0
        self.joy = None
        
        self.joy_init()
        self.controller = XBOX()
        if "DualSense" in self.joy.get_name():
            self.controller = PS5()


    def joy_init(self):
        while pygame.joystick.get_count() == 0:
            event = pygame.event.get()  # Required to update the joystick values
            print('No Controller detected')
        pygame.joystick.init()
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.joy.rumble(0.5, 1, 200)
        # print(self.joy.get_power_level())
        # print(self.joy.get_name())


    def joy_heartbeat(self):
        old_status = self.status
        self.status = bool(pygame.joystick.get_count())
        if old_status == 0 and self.status == 1:
            self.joy_init()

    def update_state(self):
        event = pygame.event.get()  # Required to update the joystick values
        # Main Control Loop
        self.joy_heartbeat()
        # Control gate 0: Check if connected to Joystick
        if self.status:
            self.get_joystick()
            # Control gate 1: Reset Throttle/Angle set points on command
            if self.state['buttons'][8]:
                # Reset angles on RB
                self.state['axes'][1:] = self.reset['axes'][1:]
            if self.state['buttons'][7]:
                # Reset throttle on LB
                self.state['axes'][0] = self.reset['axes'][0]
        else:
            print('disc')
            # Joystick disconnected
            self.state['axes'][1:] = self.reset['axes'][1:]  # Set drone to balance upright
            self.state['buttons'][1] = 1  # Set drone to hold altitude
        return self.input_to_pwm()

    def input_to_pwm(self):
        for key in list(self.state.keys()):
            self.state[key] = np.clip(self.state[key], -1, 1)
        state = deepcopy(self.reset)
        state['axes'] = np.interp(list(self.state['axes']), (-1, 1), (1000, 2000))
        state['buttons'] = np.interp(list(self.state['buttons']), (0, 1), (1000, 2000))
        return state

    def get_joystick(self):
        axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
        hats = [self.joy.get_hat(i) for i in
                range(self.joy.get_numhats())] if self.joy.get_numhats() > 1 else self.joy.get_hat(0)
        self.controller.map(axes, buttons, hats)

        for i in range(len(self.state['buttons'])):
            debounce_cutoff = 0 # Not valid given 'action on release' 
            val = self.controller.buttons[i]
            if i == 7 or i == 8:
                self.state['buttons'][i] = val
            else:
                if not val and self.state_old['buttons'][i]:
                    self.state_debounce['buttons'][i] += 1
                if self.state['buttons'][i] and self.state_debounce['buttons'][i] > debounce_cutoff:
                    self.state['buttons'][i] = 0
                    self.state_debounce['buttons'][i] = 0
                elif not self.state['buttons'][i] and self.state_debounce['buttons'][i] > debounce_cutoff:
                    self.state['buttons'][i] = 1
                    self.state_debounce['buttons'][i] = 0
            self.state_old['buttons'][i] = val
        # 0: Arm, 1: ALT Hold
        self.state['axes'][0] += self.controller.cartesian_axes[0] * self.STEP  # Throttle : LY
        self.state['axes'][1] = self.controller.cartesian_axes[3]#+= self.controller.cartesian_axes[3] * self.STEP  # Roll : RX
        self.state['axes'][2] = self.controller.cartesian_axes[2]#+= self.controller.cartesian_axes[2] * self.STEP  # Pitch : RY
        self.state['axes'][3] = self.controller.cartesian_axes[1]#+= self.controller.cartesian_axes[1] * self.STEP  # Yaw : LX


if __name__ == '__main__':
    pygame.init()
    joy = Joystick()
    while True:
        state = joy.update_state()
        print(state)
