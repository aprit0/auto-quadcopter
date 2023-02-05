import time
import numpy as np
import board
import busio
from devices.sensor_lib.mpl_0 import MPL3115A2
# from mpl_0 import MPL3115A2
# from devices.sensor_lib.VL53_0 import *
import VL53L0X


class ZAXIS:
    MAX_TOF = 1.1
    def __init__(self, ALT=True):
        self.ALT = ALT
        self.tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
        self.tof.open()
        self.tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.HIGH_SPEED)
        self.height = None
        self.last_height = None
        self.d_height = None
        self.last_time = None
        self.status = 0
        self.alt_offset = None
        if self.ALT:
            self.altimeter = MPL3115A2()
            self.calibrate_alt()
        self.read()

    def calibrate_alt(self):
        calib_list = []
        while len(calib_list) < 3:
            alt_status = self.altimeter.read()
            alt = self.altimeter.altitude
            if alt_status == 1:
                calib_list.append(alt)
        self.alt_offset = np.median(calib_list[1:]) - (self.tof.get_distance() * 0.001)
        # print(f'CALIB: {len(calib_list)} {self.alt_offset}')
        

    def read(self):
        self.last_height = self.height
        # _ = self.altimeter.temperature
        tof_height = self.tof.get_distance() * 0.001  # In metres
        if self.ALT:
            alt_status = self.altimeter.read()
            alt_height =self.altimeter.altitude - self.alt_offset
        # print(f'Tof: {tof_height:.3f} Alt:{alt_status}:{alt_height:.3f} {time.time() - t_0:.3f}')
        if tof_height < self.MAX_TOF or not self.ALT:
            self.height = tof_height
            self.status = 1
        else:
            self.height = alt_height
            self.status = alt_status

        if self.last_height is not None:
            dt = time.time() - self.last_time
            self.d_height = (self.height - self.last_height / dt)
        else:
            pass
        self.last_time = time.time()
        
        

if __name__ == '__main__':
    H = ZAXIS()
    while True:
        H.read()
