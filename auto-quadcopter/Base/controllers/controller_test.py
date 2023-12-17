import pygame
import time

# from xbox_wireless import CONTROLLER
from ps5_wireless import CONTROLLER

def test(joy, cont):
    event_new = pygame.event.get()
    # joysticks
    axes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
    # cont.map(axes, None, None)
    # print("axes: ", {i: axes[i] for i in range(len(axes))})
    # print(cont.cartesian_axes, cont.trigger_axes)

    # Buttons
    buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
    hats = [joy.get_hat(i) for i in range(joy.get_numhats())]
    # print("buttons: ", {i: buttons[i] for i in range(len(buttons))}, hats)

    cont.map(axes, buttons, hats[0])
    # print(cont.cartesian_axes)
    # print(cont.trigger_axes)
    # print(cont.buttons)

def stats(joy):
    print("name: ",joy.get_name())
    print("power: ",joy.get_power_level())
    print("buttons: ",joy.get_numbuttons())
    print("hats: ",joy.get_numhats())
    print("axes: ",joy.get_numaxes())

if __name__ == '__main__':
    cont = CONTROLLER()
    pygame.init()
    pygame.joystick.init()
    while pygame.joystick.get_count() == 0:
        print('No Controller detected')
    joy = pygame.joystick.Joystick(0)
    joy.init()
    stats(joy)
    while True:
        test(joy, cont)
        time.sleep(0.1)



