from smbus2 import SMBus
import time
import random
import struct
# for RPI version 1, use “bus = smbus.SMBus(0)”

# This is the address we setup in the Arduino Program

def send_CMD(address):
    val = 4*[random.randint(128, 255)]
    t_0 = time.time()
    bus.write_i2c_block_data(address, 0, val)
    print(f'loop: {time.time() - t_0}')
    return val

def get_pose(address):
    num_floats = 4
    t_0 = time.time()
    data = bus.read_i2c_block_data(address, 2, num_floats*4)
    print(data, bytes(data))
    floats = []
    for i in range(num_floats):
        print(data[i*4:i*4 + 4], struct.unpack('f', bytes(data[i*4:i*4 + 4] )))
    print(f'loop: {time.time() - t_0}')
    return 0
    # return data

if __name__ == "__main__":
    address = 0x04
    bus = SMBus(1)
    while True:
        var = input('0(Send) or 1(Get)')
        if var == '0':
            out = send_CMD(address)
        else:
            out = get_pose(address)
        print('Out', out)

