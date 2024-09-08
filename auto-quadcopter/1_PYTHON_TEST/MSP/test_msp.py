import serial
import struct
import time

class MSPFlightController:
    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port, 115200, timeout=2)

    def send_command(self, command):
        self.ser.write(command)

    def receive_data(self):
        data = self.ser.read(512)
        return data

    def get_attitude(self):
        # Send the ATTITUDE command
        self.send_command(b'$M<\x00\x00\x00\x00')

        # Wait for the response
        time.sleep(0.1)

        # Read the response
        data = self.receive_data()
        print(data)
        # Parse the response
        if len(data) >= 10:
            roll = struct.unpack('<h', data[6:8])[0] / 10.0
            pitch = struct.unpack('<h', data[8:10])[0] / 10.0
            yaw = struct.unpack('<h', data[10:12])[0] / 10.0
            return roll, pitch, yaw
        else:
            return None, None, None

def main():
    fc = MSPFlightController('/dev/ttyUSB0')

    while True:
        roll, pitch, yaw = fc.get_attitude()
        if roll is not None:
            print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
        else:
            print("No data received")

        time.sleep(0.1)

if __name__ == '__main__':
    main()