# ADAPTED BY AIDAN PRITCHARD FOR ONESHOT CAPABILITIES
# Credit: http://www.henrylahr.com/?p=99, https://www.instructables.com/Raspberry-Pi-MPL3115A2-Precision-Altimeter-Sensor--1/
import smbus
import time

class MPL3115A2:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        time.sleep(1)
        self.bus.write_byte_data(0x60, 0x26, 0xB9)
        self.bus.write_byte_data(0x60, 0x13, 0x07)
        self.last_reading = None
        self.altitude = None
        self.status = 0
    
    def read(self):
        self.bus.write_byte_data(0x60, 0x26, 0b10110011) # BB, AB
        self.bus.write_byte_data(0x60, 0x26, 0b10110001) # B9, A9
        data = self.bus.read_i2c_block_data(0x60, 0x00, 6)
        tHeight = ((data[1] * 65536) + (data[2] * 256) + (data[3] & 0xF0)) / 16
        temp = ((data[4] * 256) + (data[5] & 0xF0)) / 16
        self.altitude = tHeight / 16.0
        cTemp = temp / 16.0
        self.status = 0
        if self.last_reading is not None:
            self.status = 0 if self.last_reading == self.altitude else 1
        self.last_reading = self.altitude
        return self.status


            

if __name__ == "__main__":
    altimeter = MPL3115A2()
    alt = altimeter.read()
    print(altimeter.altitude)