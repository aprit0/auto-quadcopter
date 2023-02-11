import time
import smbus2 as smbus

DISTANCE = 0x01

bus = smbus.SMBus(1)
print('bus started')
# bus.write_byte(0x40,3) #"3" is the data sent to Raspberry Pi Pico
t_0 = time.time()
while True:
    if time.time() - t_0 > 0.1:
        result = bus.read_i2c_block_data(0x40,DISTANCE, 1)
        t_0 = time.time()
        print(result)
