import time
import numpy as np
import board
import busio
import adafruit_vl53l0x
from devices.mpl_0 import MPL3115A2
# from mpl_0 import MPL3115A2
import VL53L0X


class ZAXIS:
    def __init__(self):
        self.tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
        self.tof.open()
        self.tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.HIGH_SPEED)
        self.altimeter = MPL3115A2()
        self.height = None
        self.status = 0
        self.alt_offset = None
        self.calibrate_alt()
        self.read()

    def calibrate_alt(self):
        t_0 = time.time()
        calib_list = []
        while len(calib_list) < 5:
            alt_status = self.altimeter.read()
            alt = self.altimeter.altitude
            if alt_status == 1:
                calib_list.append(alt)
        self.alt_offset = np.median(calib_list[1:]) - (self.tof.get_distance() * 0.001)
        # print(f'CALIB: {len(calib_list)} {self.alt_offset}')
        

    def read(self):
        t_0 = time.time()
        # _ = self.altimeter.temperature
        tof_height = self.tof.get_distance() * 0.001  # In metres
        alt_status = self.altimeter.read()
        alt_height =self.altimeter.altitude - self.alt_offset
        # Terms of positional choice
        print(f'Tof: {tof_height:.3f} Alt:{alt_status}:{alt_height:.3f} {time.time() - t_0:.3f}')
        if tof_height < 1.1:
            self.height = tof_height
            self.status = 1
        else:
            self.height = alt_height
            self.status = alt_status
        

if __name__ == '__main__':
    H = ZAXIS()
    while True:
        H.read()
