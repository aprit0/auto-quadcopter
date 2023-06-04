import time
import argparse
try:
    from devices.sensor_lib.pmw_0 import PMW3901, BG_CS_FRONT_BCM, BG_CS_BACK_BCM
except:
    from sensor_lib.pmw_0 import PMW3901, BG_CS_FRONT_BCM, BG_CS_BACK_BCM


"""
Y+ Forwards, X+ Right
"""

class FLOW:
    def __init__(self):
        self.flo = PMW3901(spi_port=0, spi_cs_gpio=BG_CS_BACK_BCM)
        self.flo.set_rotation(0)

        self.pose = [0, 0] # x, y
        self.twist = [0, 0] # dx, dy
        self.last_time = None
        self.status = 0

    def read(self):
        y, x = self.flo.get_motion()
        # Orientation
        x = x * -1
        # Filter
        # x = x if abs(x) > 10 else 0.0
        # y = y if abs(y) > 10 else 0.0
                   
        if self.last_time is not None:
            # dt = time.time() - self.last_time
            dt = 1
            self.twist = [x * dt, y * dt] if dt != 0. else [0., 0.]
            self.pose = [i + j for (i, j) in zip(self.pose, [x, y])]
        self.last_time = time.time()
        self.status = 1 # No health check implemented


if __name__ == '__main__':
    OF = FLOW()
    time.sleep(2)
    while True:
        OF.read()
        print([round(i, 5) for i in OF.twist]) 
        # print([round(i, 1) for i in OF.pose]) 
        time.sleep(0.01)


