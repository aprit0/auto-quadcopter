import pygame
'''
Aim: General controller class for ROS
- Not ROS connected
Functions
- .get_event()
- .update_state()

See update_state for XBOX one Controller keymaps
'''

class Joystick():
    def __init__(self):
        self.state = {'axes': [0.]*2 + [-1.] + [0.]*2 + [-1.],
                      'buttons': [0]*11}

    def update_state(self, events):
        for event in events:
            if event.type == pygame.JOYBUTTONDOWN:
                # Button
                num = event.button
                self.state['buttons'][num] = 0 if self.state['buttons'][num] != 0 else 1
                if num == 0:
                    # A pressed
                    pass
                elif num == 1:
                    # B pressed
                    pass
                elif num == 2:
                    # X pressed
                    pass
                elif num == 3:
                    # Y pressed
                    pass
                elif num == 4:
                    # LB pressed
                    pass
                elif num == 5:
                    # RB pressed
                    pass
                elif num == 6:
                    # 2 Squares pressed
                    pass
                elif num == 7:
                    # Hamburger pressed
                    pass
                elif num == 8:
                    # XBOX pressed
                    pass
                elif num == 9:
                    # Left Joy pressed
                    pass
                elif num == 10:
                    # Right Joy pressed
                    pass
                else:
                    pass
            elif event.type == pygame.JOYAXISMOTION:
                # Joystick: [-1, 1] maps to [Left, Right] and [Up, Down]
                # Trigger: [-1, 1] maps to [0, depressed]
                axes = event.axis
                self.state['axes'][axes] = event.value
                if axes == 0:
                    # LX pressed
                    pass
                elif axes == 1:
                    # LY pressed
                    pass
                elif axes == 2:
                    # LT pressed
                    pass
                elif axes == 3:
                    # LX pressed
                    pass
                elif axes == 4:
                    # LY pressed
                    pass
                elif axes == 5:
                    # RT pressed
                    pass
            elif event.type == pygame.JOYHATMOTION:
                # xy buttons
                pass
            else:
                pass