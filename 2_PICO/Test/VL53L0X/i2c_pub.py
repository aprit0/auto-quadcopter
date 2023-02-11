import time
import machine
from machine import Pin, I2C
from i2c_slave import i2c_slave
from VL53 import VL53L0X

def map_i2c(value, old_min, old_max, new_min=0, new_max=255):
    value = old_min if value < old_min else old_max if value > old_max else value
    OldRange = (old_min - old_max)  
    NewRange = (new_min - new_max)  
    new_value = (((value - old_min) * NewRange) / OldRange) + new_min
    return new_value

if __name__ == '__main__':
    s_i2c = i2c_slave(0,sda=0,scl=1,slaveAddress=0x40)
    i2c = I2C(id=1, sda=Pin(2), scl=Pin(3))

    tof = VL53L0X(i2c)

    register = 0x0
    print('start pub', register)

    while True:
        if s_i2c.any():
            register = s_i2c.get()
            print('get', register)
        if register == 0x1:
            dist = tof.ping()-50
            out = int(map_i2c(dist, 0, 2000))
            print('out',out, dist)
            s_i2c.put(out)
            register = 0x0
        


