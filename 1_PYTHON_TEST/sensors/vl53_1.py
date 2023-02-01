# https://github.com/Gadgetoid/VL53L0X-python/blob/master/python/VL53L0X.py
import time
import VL53L0X

# Create a VL53L0X object
tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
# I2C Address can change before tof.open()
# tof.change_address(0x32)
tof.open()
# Start ranging
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.HIGH_SPEED)

timing = tof.get_timing()
if timing < 20000:
    timing = 20000
print("Timing %d ms" % (timing/1000))

t_0 = time.time()
for count in range(1, 101):
    distance = tof.get_distance()
    if distance > 0:
        print("%d mm, %d cm, %d %f" % (distance, (distance/10), count, time.time() - t_0))
        t_0 = time.time()

    # time.sleep(timing/1000000.00)

tof.stop_ranging()
tof.close()
