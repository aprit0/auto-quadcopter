import smbus
import time
import struct

int_to_four_bytes = struct.Struct('<I').pack

#setup SMBus
bus = smbus.SMBus(1)
ADD = 0x40
x = 3000
while True:
    y1, y2, y3, y4 = (x & 0xFFFFFFFF).to_bytes(4, 'little')
    print(y1, y2, y3, y4)
    bus.write_i2c_block_data(ADD, 0x00, [y1, y2, y3, y4])
    out = bus.read_byte(ADD)
    print(out)
    time.sleep(1)
    print('.')