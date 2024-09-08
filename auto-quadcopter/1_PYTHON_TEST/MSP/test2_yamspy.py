from yamspy import MSPy
import time, threading

class MSPFlightController():
    CMDS_init = {
    'roll':     1500,
    'pitch':    1500,
    'throttle': 900,
    'yaw':      1500,
    'aux1':     1000, # DISARMED (1000) / ARMED (1800)
    'aux2':     1000, # ANGLE (1000) / HORIZON (1500) / FLIP (1800)
    'aux3':     1000, # FAILSAFE (1800)
    'aux4':     1000  # HEADFREE (1800)
    }
    PID_init = [
        [42, 85, 35], [46, 90, 38], [30, 90, 0], [50, 50, 75], [40, 0, 0]
        ]
    def __init__(self, port="/dev/ttyUSB0"):
        self.port = port
        self.board = None
        self.shutdown = False
        self.CMDS = self.CMDS_init.copy()
        self.PIDS = self.PID_init.copy()
        
        self.cli_thread = threading.Thread(target=self.cli)
        self.cli_thread.daemon = True
        self.cli_thread.start()
    
    def arm(self):
        self.CMDS["aux1"] = 1800
        print(self.CMDS)

    def disarm(self):
        self.CMDS["aux1"] = 1000
        print(self.CMDS)

    def get_pid_values(self):
        
        if self.board.send_RAW_msg(MSPy.MSPCodes['MSP_PID'], data=[]):
            # 2. Response msg from the flight controller is received
            dataHandler = self.board.receive_msg()
            # 3. The msg is parsed
            self.board.process_recv_data(dataHandler)
            return self.board.PIDs
        return None

    def set_pid_values(self, pid_values):
        # Roll, Pitch, Yaw, Angle, Mag
        if len(pid_values) == 3:
            self.PIDS = self.PID_init.copy()
            self.PIDS[:2] = [[i[j] + pid_values[j] for j in range(len(pid_values))] for i in self.PIDS[:2]]
            #  [[42, 85, 35], [46, 90, 38], [30, 90, 0], [50, 50, 75], [40, 0, 0]]
            print(self.PIDS)
            if self.board.send_RAW_msg(MSPy.MSPCodes['MSP_PID'], data=self.PIDS):
                # 2. Response msg from the flight controller is received
                dataHandler = self.board.receive_msg()
                # 3. The msg is parsed
                self.board.process_recv_data(dataHandler)
                return self.board.PIDs
        return None

    def set_angles(self, roll, pitch, yaw):
        clip_xy = lambda x: max(10, min(-10, x))
        angle_to_command = lambda x, old_min, old_max, new_min, new_max: new_min + ((x - old_min) * (new_max - new_min) / (old_max - old_min))
        roll_pwm = angle_to_command(clip_xy(roll), -10, 10, 1000, 2000)
        pitch_pwm = angle_to_command(clip_xy(pitch), -10, 10, 1000, 2000)
        yaw_pwm = angle_to_command(clip_z(yaw), 0, 360, 1000, 2000)
        self.CMDS["roll"] = roll_pwm
        self.CMDS["pitch"] = pitch_pwm
        self.CMDS["yaw"] = yaw_pwm
        print(self.CMDS)

    def clip_z(self, z):
        while not 360 > abs(z) > 0:
            if z > 360:
                z -= 360
            elif z < 0:
                z += 360
        return z
    
    def get_angles(self):
        # Kinematics:, "YAW" ["ROLL RIGHT DOWN +", "PITCH FRONT DOWN +", "YAW CLOCKWISE + [0,360)""]
        print(self.board.SENSOR_DATA['kinematics'])

    def loop(self):
        while not self.shutdown:
            with MSPy(device=self.port, loglevel='WARNING', baudrate=115200) as self.board:
                if self.board == 1:
                    print("board state: ", self.board)
                    time.sleep(0.1)
                    continue
                while self.board and not self.shutdown:
                    t_0 = time.time()
                    self.board.fast_msp_rc_cmd([self.CMDS[i] for i in self.CMDS])
                    self.board.fast_read_attitude()

                    loop_time = time.time() - t_0
                    # print("time: ", loop_time, 1/loop_time)
                    time.sleep(0.02)
    
    def cli(self):
        while True:
            print("1. Arm")
            print("2. Disarm")
            print("3. Get PID values")
            print("4. Set PID values #N/A")
            print("5. Set roll, pitch, yaw angles")
            print("6. Get roll, pitch, yaw angles")
            print("7. Quit")

            choice = input("Enter your choice: ")

            if choice == '1':
                self.arm()

            elif choice == '2':
                self.disarm()

            elif choice == '3':
                pid_values = self.get_pid_values()
                print(f"PID values: {pid_values}")

            elif choice == '4':
                pid_values = input("Enter PID values (comma separated): ")
                pid_values = [int(x) for x in pid_values.split(',')]
                print(pid_values)
                self.set_pid_values(pid_values)

            elif choice == '5':
                roll = input("Enter roll angle: ")
                pitch = input("Enter pitch angle: ")
                yaw = input("Enter yaw angle: ")
                self.set_angles(float(roll), float(pitch), float(yaw))
            
            elif choice == '6':
                self.get_angles()

            elif choice == '7':
                self.shutdown = True
                break
            


if __name__ == "__main__":
    MSP = MSPFlightController()
    MSP.loop()
    print("done")