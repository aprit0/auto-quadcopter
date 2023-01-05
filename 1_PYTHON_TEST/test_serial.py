import serial
import time
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser.reset_input_buffer()
    recv = [0, 0]
    while True:
        for i in range(1000, 2000):
            out = [str(i)] * 6
            t_0 = time.time()
            send_string = ','.join(out)
            send_string += "\n"
            ser.write(send_string.encode('utf-8'))
            t_1 = time.time()
            line = ser.readline().decode('utf-8').rstrip()
            print(line, time.time() - t_1, t_1 - t_0)
            # time.sleep(1)