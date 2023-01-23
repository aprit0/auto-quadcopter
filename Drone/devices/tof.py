import board
import busio
import adafruit_vl53l0x

class VL53:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.tof = adafruit_vl53l0x.VL53L0X(i2c)
        # self.tof.measurement_timing_budget = 20000 
    
    def read(self):
        self.height = self.tof.range * 0.001  # In metres
        return self.height