import pwmio, board
from adafruit_motor import servo
import time


class BLDC:
    LIMITS = [900.0, 2000.0]
    def __init__(self, pin="D5") -> None:
        pwm = pwmio.PWMOut(getattr(board, pin), frequency=50)
        self.motor = servo.Servo(pwm, min_pulse = min(self.LIMITS), max_pulse = max(self.LIMITS))
        self.clip = lambda x: min(self.LIMITS) if x < min(self.LIMITS) else max(self.LIMITS) if x > max(self.LIMITS) else x
        self.map = lambda x: (x - min(self.LIMITS)) / (max(self.LIMITS) - min(self.LIMITS))
    
    def write_millis(self, pwm_millis):
        pwm_millis = self.clip(pwm_millis)
        angle = self.map(pwm_millis) * 180
        self.motor.angle = angle

class MOTORS:
    ARM_SPEED = 1050
    def __init__(self) -> None:
        # GP18,19,20,21
        self.fl = BLDC("GP19") 
        self.fr = BLDC("GP20") 
        self.br = BLDC("GP21")
        self.bl = BLDC("GP18") 
        self.disarm()

    def set_speed(self, fl, fr, bl, br, ARM=False):
        self.fl.write_millis(fl if ARM else min(self.fl.LIMITS))
        self.fr.write_millis(fr if ARM else min(self.fr.LIMITS))
        self.bl.write_millis(bl if ARM else min(self.bl.LIMITS))
        self.br.write_millis(br if ARM else min(self.br.LIMITS))

    def arm(self):
        self.set_speed(*[self.ARM_SPEED]*4, ARM=True)

    def disarm(self):
        self.set_speed(*[self.ARM_SPEED]*4, ARM=False)

    def test(self):
        motor_mapping = ["fl", "fr", "bl", "br"]
        self.arm()
        time.sleep(1)
        for i in range(4):
            cmd = [1000]*4
            cmd[i] = 1200
            input(f'Test motor {i}:{motor_mapping[i]}?')
            self.set_speed(*cmd, ARM=True)
            time.sleep(1.5)
            self.disarm()
        self.disarm()


if __name__ == '__main__':
    drone = MOTORS()
    while True:
        x = int(input('Motor began \n0: Disarm \n1: Test \n2: Drive')[0])
        print("GOT: ", x)
        if not x:
            drone.disarm()
        elif x == 1:
            print('Begin Test')
            drone.test()
        elif x == 2:
            x = int(input('Motor speed: '))
            drone.arm()
            cmd = [x]*4
            drone.set_speed(*cmd, ARM=True)