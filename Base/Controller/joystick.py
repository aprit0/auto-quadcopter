import pygame
import numpy as np
from copy import deepcopy
'''
Aim: General controller class for ROS
- Not ROS connected
Functions
- .get_event()
- .update_state()

See update_state for XBOX one Controller keymaps
'''


class Joystick:
    def __init__(self, Keyboard=True, joystick=None, MVA=False):
        self.reset = {'axes': [-1] + [0.] * 5, # + [-1.] + [0.] * 2 + [-1.],
                      'buttons': [0.] * 11}
        self.MVA = MVA
        self.Key = Keyboard
        self.state = deepcopy(self.reset)
        self.joy = joystick

    def filter(self, old_axes):
        new_axes = self.state['axes']
        split = [0.8, 0.2]  # moving average
        return [split[1] * new_axes[i] + split[0] * old_axes[i] for i in range(len(old_axes))]

    def update_state(self, events):
        old_state = deepcopy(self.state)
        for event in events:
            if self.Key:
                self.get_keyboard(event)
            else:
                self.get_joystick(event)
        if self.MVA:
            self.state['axes'] = self.filter(old_state['axes'])
        if self.state['buttons'][4] or self.state['buttons'][5]:
            # Reset angles on RB or LB
            self.state['axes'][1:] = [0.]*5

        return self.input_to_pwm()

    def input_to_pwm(self):
        for key in list(self.state.keys()):
            self.state[key] = np.clip(self.state[key], -1, 1)
        state = deepcopy(self.reset)
        state['axes'] = np.interp(list(self.state['axes']), (-1, 1), (1000, 2000))
        state['buttons'] = np.interp(list(self.state['buttons']), (0, 1), (1000, 2000))
        return state

    def get_keyboard(self, event):
        # Limited from -1 to 1
        STEP = 0.2
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            self.state['axes'][0] += STEP
        elif keys[pygame.K_DOWN]:
            self.state['axes'][0] -= STEP
        elif keys[pygame.K_1]:
            self.state['buttons'][0] = 1
        elif keys[pygame.K_2]:
            self.state['buttons'][1] = 1
        elif keys[pygame.K_0]:
            self.state = deepcopy(self.reset)
        else:
            pass

    def get_joystick(self, event):
        # Limited from -1 to 1
        new_buttons = []
        for num in range(self.joy.get_numbuttons()):
            '''
            BUTTON
            Number: Key
            0: A
            1: B
            2: X
            3: Y
            4: LB
            5: Rb
            6: 2Squares
            7: Hamburger
            8: Xbox
            9: LeftJoy
            10: RightJoy
            '''
            pressed = self.joy.get_button(num)
            new_buttons.append(pressed)
        self.state['buttons'] = new_buttons # Seperated to allow for remap
        new_axes = []
        for axes in range(self.joy.get_numaxes()):
            # Joystick: [-1, 1] maps to [Left, Right] and [Up, Down]
            # Trigger: [-1, 1] maps to [0, depressed]
            '''
            0: LX
            1: LY
            2: LT
            3: RX
            4: RY
            5: RT
            '''
            pose = self.joy.get_axis(axes)
            # if axes == 2 or axes == 5:
            #     # Triggers
            #     pass
            # else:
            # Joysticks
            # direction = -1 if pose < -0.5 else 1 if pose > 0.5 else 0
            STEP = 0.03
            pose = 0 if 0.10 > pose > -0.10 else pose
            new_axes.append(pose * STEP)
            
        # print(self.state, new_axes)
        self.state['axes'][0] -= new_axes[1] # Throttle : LY
        self.state['axes'][1] += new_axes[3] # Roll : RX
        self.state['axes'][2] += new_axes[4] # Pitch : RY
        self.state['axes'][3] += new_axes[0] # Yaw : LX
