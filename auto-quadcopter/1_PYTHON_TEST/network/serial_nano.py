import serial

# Set up the serial port (adjust for your system)
ser = serial.Serial('/dev/ttyUSB0', 115200)
print("start")
while True:
    # Read a line from the serial port
    data = ser.readline().decode('utf-8').strip()  # Read, decode, and remove any extra whitespace
    
    # Split the data by commas into a list
    values = data.split(',')
    print(values)
    # Convert each value to an integer
    # value1 = int(values[0])
    # value2 = int(values[1])
    # value3 = int(values[2])
    # value4 = int(values[3])
    # value5 = int(values[4])
    # value6 = int(values[5])
    # value7 = int(values[6])
    # message_id = int(values[7])  # The last value is the unique message ID

    # Print the received values
    # print(f"Received values: {value1}, {value2}, {value3}, {value4}, {value5}, {value6}, {value7}")
