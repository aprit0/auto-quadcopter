import serial
print(serial.__file__) 
import time

serialPort = serial.Serial(
    port="/dev/ttyACM0", baudrate=115200, bytesize=8, timeout=0, stopbits=serial.STOPBITS_ONE
)
serialString = ""  # Used to hold data coming over UART
t_0 = time.time()
s = ""
t_1 = time.time()
while True:
    if time.time() - t_0 > 1:
        t_1 = time.time() if s == "" else t_1
        # serialPort.write(b'<100>')
        t_0 = time.time()
        # print(t_0 - t_1)
    # time.sleep(1)
    # Read data out of the buffer until a carraige return / new line is found
    serialString = serialPort.read(serialPort.in_waiting)

    # Print the contents of the serial data
    # try:
    if serialString:
        out = serialString.decode("Ascii")
        # if "<" in out and ">" in out:
            # s = out
        if out:
            print(out)
        # print("Read:",s, 1/(time.time() - t_1))
            # s = ""

        # else:
        #     s += out
    # except:
        # pass
    # input()
# input()
    