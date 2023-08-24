from VL53_1 import VL53L0X
import time

import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
print([hex(i) for i in i2c.scan()])
vl53 = VL53L0X(i2c)

vl53.measurement_timing_budget = 200000