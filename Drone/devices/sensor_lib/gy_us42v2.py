import board
import busio
import time
from adafruit_bus_device.i2c_device import I2CDevice


class GY_US42:
    MAX_LOOP_SPEED = 0.08
    GET_DIST = 0x51
    def __init__(self, addr=0x70):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.dev = I2CDevice(i2c, addr)
        self.z = None
        self.dz = None
        self.last_z = None
        self.status = 0

        self.ping = lambda self: self.dev.write(bytes([self.GET_DIST]))
        self.last_read = time.time()
        self.ping(self)
        time.sleep(self.MAX_LOOP_SPEED)
        self.run()

    def run(self):
        # Returns a status boolean for new value
        dt = time.time() - self.last_read
        if dt > self.MAX_LOOP_SPEED:
            result = bytearray(2)
            self.dev.readinto(result)
            dist = int.from_bytes(result, 'big') * 0.01
            print(dist)
            if self.z:
                if self.last_z and self.dz:
                    # Use previous value to check if current value is reasonable
                    s = self.dz * dt + self.z
                    stdev = 0.3
                    # print('CHECKS: ', dist==s, dist, s)
                    dist = dist if s * (1 + stdev) > dist > s * (1 - stdev) else s
                self.last_z = self.z
                self.z = dist
                self.dz = (self.z - self.last_z) / dt
            else:
                self.z = dist
            self.last_read = time.time()
            self.ping(self)
            self.status = 1
        else:
            self.status = 0
        return self.status
            
            
if __name__ == '__main__':
    Z = GY_US42()
    while True:
        out = Z.run()
        # print(f'Z: {Z.z} dZ: {Z.dz}')
        # if not out:
            # time.sleep(0.01)



