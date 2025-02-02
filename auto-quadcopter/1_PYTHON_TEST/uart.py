import time
import serial
if __name__ == "__main__":
        ser = serial.Serial(
                port='/dev/ttyUSB0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
                # port='/dev/ttyACM0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
                baudrate = 115200,
                bytesize=8, parity='N', stopbits=1, timeout=1, xonxoff=False, rtscts=False
        )
        print(ser)
        t_0 = 0

        while 1:
                # if time.time() - t_0 > 2:
                        # ser.write(bytes("<Y,A:1>", "ascii"))
                        # t_0 = time.time()
                #         print(".")
                # ser.rts = False
                # ser.dtr = False
                # ser.setDTR(1)
                # bytesToRead = ser.inWaiting()
                # data=ser.read(bytesToRead)
                data = ser.readline()

                print(data)
                # if data:# or bytesToRead:
                #         print(data, bytesToRead)
                # print(ser.read())#("e"))
                # print(ser.readline())
