import board
import busio
import adafruit_vl53l0x
import time

i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)


# Main loop will read the range and print it every second.
while True:
    t_0 = time.time()
    dist = vl53.range
    print(f"Range: {dist}mm {time.time() - t_0:.2f}")
