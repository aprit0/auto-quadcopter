import busio
import board
import time
from adafruit_bus_device.i2c_device import I2CDevice

i2c = busio.I2C(board.SCL, board.SDA)
dev = I2CDevice(i2c, 0x70)
t_1 = time.time()
count = 0
fails = 0
t_0 = time.time()
while True:
    dist = 720
    if time.time() - t_0 > 0.07:
        result = bytearray(2)
        dev.readinto(result)
        dist = int.from_bytes(result, 'big')
        print(dist)
        t_0 = time.time()
        dev.write(bytes([0x51]))
    
    if dist == 720:
        fails += 1
    count += 1
    # if time.time() - t_1 > 0.05:
    print(time.time() - t_1, dist, fails/count)
    t_1 = time.time()
