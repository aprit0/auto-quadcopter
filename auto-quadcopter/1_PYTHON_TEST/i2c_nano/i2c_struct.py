import smbus
import struct
import time

# I2C address of the Arduino (change this to match your Arduino's address)
ARDUINO_ADDRESS = 0x08

# Create an SMBus instance
bus = smbus.SMBus(1)  # Use 1 for Raspberry Pi 2 & 3, use 0 for earlier versions

# Define the struct format
# 'f' for float, 'h' for int16_t, 'I' for unsigned long
struct_format = 'ffffhhI'

def read_sensor_data():
    try:
        # Request data from Arduino
        data = bus.read_i2c_block_data(ARDUINO_ADDRESS, 0, struct.calcsize(struct_format))
        
        # Unpack the received data
        unpacked_data = struct.unpack(struct_format, bytes(data))
        
        # Create a dictionary with named fields
        sensor_data = {
            'height': unpacked_data[0],
            'lastHeight': unpacked_data[1],
            'velocity': unpacked_data[2],
            'dx': unpacked_data[3],
            'dy': unpacked_data[4],
            'lastTime': unpacked_data[5]
        }
        
        return sensor_data
    except Exception as e:
        print(f"Error reading sensor data: {e}")
        return None

# Main loop
while True:
    sensor_data = read_sensor_data()
    if sensor_data:
        print("Sensor Data:")
        print(f"Height: {sensor_data['height']:.2f}")
        print(f"Last Height: {sensor_data['lastHeight']:.2f}")
        print(f"Velocity: {sensor_data['velocity']:.2f}")
        print(f"dx: {sensor_data['dx']}")
        print(f"dy: {sensor_data['dy']}")
        print(f"Last Time: {sensor_data['lastTime']}")
        print("--------------------")
    time.sleep(1)  # Wait for 1 second before reading again
