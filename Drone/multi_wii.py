from Comms.pymultiwii import MultiWii

class MW:
    def __init__(self):
        self.board = MultiWii("/dev/ttyUSB0")

        self.motor = {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'elapsed': 0, 'timestamp': 0}
        self.rcChannels = {'roll': 0, 'pitch': 0, 'yaw': 0, 'throttle': 0, 'elapsed': 0, 'timestamp': 0}
        self.altitude = {'estalt': 0, 'vario': 0, 'elapsed': 0, 'timestamp': 0}

    def update_mw(self, data=[1500]*8):
        self.board.getData(MultiWii.ATTITUDE)
        self.motor = self.board.motor
        self.rcChannels = self.board.rcChannels
        self.attitude = self.board.attitude
        print(self.board.attitude)
        self.board.sendCMD(16, MultiWii.SET_RAW_RC, data[:3], 'i')
        print(self.board.attitude)
if __name__ == "__main__":
    FC = MW()
    while True:
        for i in range(1000,2000):
            print(i)
            data = [1000]*8
            data[0] = i
            FC.update_mw(data)
            print(FC.rcChannels)