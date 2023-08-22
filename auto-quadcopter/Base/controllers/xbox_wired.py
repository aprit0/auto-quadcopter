import numpy as np


class CONTROLLER:
    # Cartesian joysticks mapped to range [-1,1] where Q1: [1,1] and Q3: [-1,-1] in [x,y]
    # Trigger joysticks mapped to range [0, 1] where uncompressed is [0]
    raw_cartesian_range = [-1, 1]
    raw_trigger_range = [0, 1]
    raw_button_range = [0, 1]

    def __init__(self):
        self.cartesian_axes = None
        self.trigger_axes = None
        self.buttons = None

    def map(self, axes, buttons, hat):
        if axes is not None:
            # axes
            ly = -axes[1]
            lx = axes[0]
            ry = -axes[4]
            rx = axes[3]
            lt = np.interp(axes[2], (-1, 1), (0, 1))
            rt = np.interp(axes[5], (-1, 1), (0, 1))
            self.cartesian_axes = [ly, lx, ry, rx]
            self.trigger_axes = [lt, rt]

        if buttons is not None:
            # buttons
            a = buttons[0]
            b = buttons[1]
            y = buttons[3]
            x = buttons[2]
            main = buttons[8]
            menu = buttons[7]
            select = buttons[6]
            lb = buttons[4]
            rb = buttons[5]
            lj = buttons[9]
            rj = buttons[10]
            up = 1 if hat[1] == 1 else 0
            down = 1 if hat[1] == -1 else 0
            left = 1 if hat[0] == -1 else 0
            right = 1 if hat[0] == 1 else 0
            self.buttons = [a, b, y, x, main, menu, select, lb, rb, lj, rj, up, down, left, right]
