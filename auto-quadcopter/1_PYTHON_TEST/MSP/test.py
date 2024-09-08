from pymultiwii_local import pyMultiWii
import threading

class MultiWiiCLI:
    def __init__(self, port):
        self.mw = pyMultiWii(port)
        self.reader_thread = threading.Thread(target=self.read_messages)
        self.reader_thread.daemon = True
        self.reader_thread.start()

    def read_messages(self):
        while True:
            data = self.mw.get_data()
            print("Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(data['roll'], data['pitch'], data['yaw']))

    def arm(self):
        self.mw.arm()

    def disarm(self):
        self.mw.disarm()

    def get_pid_values(self):
        return self.mw.get_pid()

    def set_pid_values(self, pid_values):
        self.mw.set_pid(pid_values)

    def set_angles(self, roll, pitch, yaw):
        self.mw.set_attitude(roll, pitch, yaw)

def main():
    cli = MultiWiiCLI('COM5')  # Replace with your serial port

    while True:
        print("1. Arm")
        print("2. Disarm")
        print("3. Get PID values")
        print("4. Set PID values")
        print("5. Set roll, pitch, yaw angles")
        print("6. Quit")

        choice = input("Enter your choice: ")

        if choice == '1':
            cli.arm()

        elif choice == '2':
            cli.disarm()

        elif choice == '3':
            pid_values = cli.get_pid_values()
            print(f"PID values: {pid_values}")

        elif choice == '4':
            pid_values = input("Enter PID values (comma separated): ")
            pid_values = [int(x) for x in pid_values.split(',')]
            cli.set_pid_values(pid_values)

        elif choice == '5':
            roll = input("Enter roll angle: ")
            pitch = input("Enter pitch angle: ")
            yaw = input("Enter yaw angle: ")
            cli.set_angles(float(roll), float(pitch), float(yaw))

        elif choice == '6':
            break

if __name__ == '__main__':
    main()