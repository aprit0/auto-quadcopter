import smbus2 as smbus
import time
# from utils import map
from devices.utils import map
class PICO:
    _DISTANCE = 0x01
    def __init__(self, addr=0x40):
        self.addr = 0x40
        self.bus = smbus.SMBus(1)
        self.height = None
        self.d_height = None
        self.last_height = None
        self.last_time = None

        self.read()

    def read(self):
        self.read_height()
    
    def read_height(self):
        self.last_height = self.height
        out = self.bus.read_i2c_block_data(self.addr, self._DISTANCE, 1)
        height_mm = map(out[0]**2, 0, 255**2, 0, 2000)
        self.height = height_mm / 1000
        if self.last_time is not None:
            dt = time.time() - self.last_time
            self.d_height = (self.height - self.last_height) / dt
        self.last_time = time.time()

if __name__ == '__main__':
    dev = PICO()
    for i in range(10):
        t_0 = time.time()
        dev.read()
        print(round(dev.height, 4), dev.d_height, time.time() - t_0)


