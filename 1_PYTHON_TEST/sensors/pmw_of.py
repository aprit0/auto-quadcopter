#!/usr/bin/env python
import time
import argparse
from pmw3901 import PMW3901, PAA5100, BG_CS_FRONT_BCM, BG_CS_BACK_BCM


# Pick the right class for the specified breakout
SensorClass = PMW3901

# flo = SensorClass(spi_port=0, spi_cs_gpio=BG_CS_FRONT_BCM) #if args.spi_slot == 'front' else BG_CS_BACK_BCM)
flo = SensorClass(spi_port=0, spi_cs_gpio=BG_CS_BACK_BCM)

flo.set_rotation(0)

tx = 0
ty = 0

try:
    while True:
        t_0 = time.time()
        try:
            x, y = flo.get_motion()
        except RuntimeError:
            continue
        tx += x
        ty += y
        print("{:05f} Relative: x {:03d} y {:03d} | Absolute: x {:03d} y {:03d}".format(time.time() - t_0,x, y, tx, ty))
except KeyboardInterrupt:
    pass



