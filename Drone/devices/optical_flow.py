import time
import argparse
# from sensor_lib.pmw_0 import PMW3901, BG_CS_FRONT_BCM, BG_CS_BACK_BCM
from devices.sensor_lib.pmw_0 import PMW3901, BG_CS_FRONT_BCM, BG_CS_BACK_BCM


class FLOW:
    def __init__(self):
        self.flo = PMW3901(spi_port=0, spi_cs_gpio=BG_CS_BACK_BCM)
        self.flo.set_rotation(0)

        self.pose = [0, 0] # x, y
        self.twist = [0, 0] # dx, dy
        self.last_time = None

    def read(self):
        x, y = self.flo.get_motion()
        if self.last_time is not None:
            dt = time.time() - self.last_time
            self.twist = [x / dt, y / dt] if dt != 0. else [0., 0.]
            self.pose = [i + j for (i, j) in zip(self.pose, [x, y])]
        self.last_time = time.time()


if __name__ == '__main__':
    OF = FLOW()
    while True:
        OF.read()
        print(OF.pose, [round(i, 0) for i in OF.twist]) 


