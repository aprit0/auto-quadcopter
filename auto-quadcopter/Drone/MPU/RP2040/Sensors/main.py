import board
import busio 
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER
from adafruit_bno08x.i2c import BNO08X_I2C
# import adafruit_mpl3115a2
from mpl3115a2 import MPL3115A2
import asyncio
from time import monotonic

# class Altitude(MPL3115A2):
#     def __init__(self, i2c):
#         self.old_altitude = 0
#         super().__init__(i2c)

#     def calibrate(self, n_trys=5):
#         ave_pressure = 0
#         for i in range(n_trys):
#             ave_pressure += self.blocking_read(self.pressure)
#         ave_pressure = int(ave_pressure / n_trys)
#         self.sealevel_pressure = ave_pressure
#         return self.read_altitude(blocking=True), ave_pressure

#     def blocking_read(self, func, n_trys=0.5):
#         t_0 = monotonic()
#         while monotonic() - t_0 < n_trys:
#             out = func()
#             if out:
#                 break
#         if out:
#             return out
#         else:
#             raise Exception(f"{self.__class__.__name__}: Blocking Read failed {i}/{n_trys-1}: out: {out}")\
    
#     def read_altitude(self, blocking=False):
#         if blocking:
#             out = self.blocking_read(self.altitude)
#         else:
#             out = self.altitude()
#         if out:
#             self.old_altitude = out
#             return out, True
#         else:
#             return self.old_altitude, False
            

class State:
    def __init__(self, verbose=True):
        self.verbose = verbose
        self.pose = [0, 0, 0] 
        self.vel = [0, 0, 0] 
        self.acc = [0, 0, 0] 
    
    def println(self, msg):
        if self.verbose:
            print(msg)

class Sensors(State):
    def __init__(self) -> None:
        super().__init__()
        self.i2c = busio.I2C(board.GP3, board.GP2)
        # self.alt = Altitude(self.i2c)
        self.bno = BNO08X_I2C(self.i2c)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        # self.println(self.alt.calibrate())

    def get_acceleration(self):
        return self.bno.acceleration
    
    # def get_altitude(self):
    #     return self.alt.read_altitude()[0]
    


class Drone(Sensors):
    def __init__(self):
        super().__init__()
        self.println(self.__dict__)

def loop():
    robot = Drone()

    while True:
        t_0 = monotonic()
        accel_x, accel_y, accel_z = robot.get_acceleration()
        robot.acc = [accel_x, accel_y, accel_z] 
        print("Run", robot.pose, robot.acc)
        print(1/ (monotonic() - t_0))


if __name__ == "__main__":
    loop()


# async def read_slow(robot):
#     while True:
#         robot.pose[-1] = robot.get_altitude()
#         print("Slow", monotonic())
#         await asyncio.sleep(0.01)

# async def read_fast(robot):
#     while True:
#         accel_x, accel_y, accel_z = robot.get_acceleration()
#         robot.acc = [accel_x, accel_y, accel_z] 
#         print("Fast", monotonic())
#         await asyncio.sleep(0.01)

# async def run(robot):
#     while True:
#         print("Run", monotonic(), robot.pose, robot.acc)
#         await asyncio.sleep(0.01)

# async def main():
#     robot = Drone()

#     buttons_task = asyncio.create_task(
#         read_slow(robot)
#     )
#     animation_task = asyncio.create_task(read_fast(robot))
#     run_task = asyncio.create_task(run(robot))
#     # This will run forever, because no tasks ever finish.
#     await asyncio.gather(buttons_task, animation_task, run_task)


# asyncio.run(main())
