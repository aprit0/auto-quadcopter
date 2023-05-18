import smbus
import time
import numpy as np

# cmd: [fl, fr, bl, br] 
class NANO():
    REG_SERVO = [0x10, 0x11, 0x12, 0x13]
    NUM_SERVO = 4

    def __init__(self, address: int=0x40, servos: int=4):
        self.ADDR: int  = address
        self.bus = smbus.SMBus(1)
        self.to_bytes = lambda x: (x & 0xFFFFFFFF).to_bytes(4, 'little')

    def write_servo(self, cmd: list) -> bool:
        self.write(self.REG_SERVO, cmd)
        return True                

    def write(self, reg: list, cmd: list):
        for i in range(len(reg)):
            y1, y2, y3, y4 = self.to_bytes(cmd[i])
            self.bus.write_i2c_block_data(self.ADDR, reg[i], [y1, y2, y3, y4])
        ack = self.bus.read_byte(self.ADDR)
        assert ack == 24

