import serial
write = True
# Set up the serial port (adjust for your system)
ser = serial.Serial('/dev/ttyUSB0', 115200)
if write:
    f = open("/home/pi/ros2_ws/src/auto-quadcopter/auto-quadcopter/1_PYTHON_TEST/OpticalFlow/readings.txt", "w")
    f.write("dt,us_height,us_velocity,pressure_height,pressure_velocity,tof_height,tof_velocity,dx,dy" + "\n")
while True:
    # Read a line from the serial port
    data = ser.readline().decode('utf-8').strip()  # Read, decode, and remove any extra whitespace
    
    # Split the data by commas into a list
    values = data.split(',')
    if write:
        f.write(data + "\n")
    print(values)
    # =IFS(F2<2,F2,B2<4,B2,TRUE,D2)