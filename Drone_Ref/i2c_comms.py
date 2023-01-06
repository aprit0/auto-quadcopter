from smbus2 import SMBus
import time
import random
import struct


class I2C_MSG():
    '''
    init_drone
        - Init I2C bus, checks connection and that all is well
    
    send_CMDS (0)
        - Set Target Height, Roll, Pitch, Yaw

    send_MODES (1)
        - Set booleans ARM, ...

    get_state (2)
        - Receive Height, Roll, Pitch, Yaw
    '''
    def __init__(self):
        self.addr_nano = 0x04

        # Begin
        self.init_drone()
    

    def init_drone(self):
        self.bus = SMBus(1)

    def send_block(cmd, data):
        # Len of 4, values between 0-255
        t_0 = time.time()
        self.bus.write_i2c_block_data(self.addr_nano, cmd, data)
        print(f'loop: {time.time() - t_0}')

    def read_block(addr, num_floats=4):
        data = bus.read_i2c_block_data(self.addr_nano, addr, num_floats*4)
        floats = []
        for i in range(num_floats):
            floats.append(data[i*4:i*4 + 4], struct.unpack('f', bytes(data[i*4:i*4 + 4] )))
        return floats
        

