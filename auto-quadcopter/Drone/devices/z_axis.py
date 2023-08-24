import time
import numpy as np
import board
import busio
try:
    from devices.sensor_lib.mpl_0 import MPL3115A2
    from devices.sensor_lib.gy_us42v2 import GY_US42
except:
    from sensor_lib.mpl_0 import MPL3115A2
    from sensor_lib.gy_us42v2 import GY_US42


class ZAXIS:
    MAX_DIST = 4
    def __init__(self, ALT=True):
        self.ALT = ALT
        self.dist = GY_US42()
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
        self.alt_offset = np.median(calib_list[1:]) - (self.dist.z)
        # print(f'CALIB: {len(calib_list)} {self.alt_offset}')
        

    def read(self):
        t_0 = time.time()
        self.dist.run() 
        self.last_height = self.height
        # _ = self.altimeter.temperature
        dist_z = self.dist.z 
        # print(f'dist: {dist_z:.3f} Alt:{alt_status}:{alt_height:.3f} {time.time() - t_0:.3f}')
        if dist_z < self.MAX_DIST or not self.ALT:
            # Use 
            self.height = dist_z
            self.status = self.dist.status
        else:
            alt_status = self.altimeter.read()
            alt_height = self.altimeter.altitude - self.alt_offset
            self.height = alt_height
            self.status = alt_status

        if self.last_height is not None:
            dt = time.time() - self.last_time
            dh = self.height - self.last_height
            self.d_height = dh / dt if dh != 0.0 else 0.0
        self.last_time = time.time()
        
        

if __name__ == '__main__':
    H = ZAXIS()
    while True:
        H.read()
        time.sleep(0.05)
        print('dist: ', H.height, H.d_height)
