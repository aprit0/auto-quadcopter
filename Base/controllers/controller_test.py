import pygame
import time

from xbox_wireless import CONTROLLER

def test(joy, cont):
    event_new = pygame.event.get()
    # joysticks
    axes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
    # cont.map(axes, None, None)
    # print({i: axes[i] for i in range(len(axes))})
    # print(cont.cartesian_axes, cont.trigger_axes)

    # Buttons
    buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
    hats = [joy.get_hat(i) for i in range(joy.get_numhats())]
    # print({i: buttons[i] for i in range(len(buttons))}, hats)
    cont.map(None, buttons, hats[0])
    print(cont.buttons)

if __name__ == '__main__':
    cont = CONTROLLER()
    pygame.init()
    pygame.joystick.init()
    while pygame.joystick.get_count() == 0:
        print('No Controller detected')
    joy = pygame.joystick.Joystick(0)
    joy.init()
    while True:
        test(joy, cont)
        time.sleep(0.1)



