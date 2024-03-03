import pwmio, board
from adafruit_motor import servo


class BLDC:
    LIMITS = [900.0, 2000.0]
    def __init__(self, pin="D5") -> None:
        pwm = pwmio.PWMOut(getattr(board, pin), frequency=50)
        self.motor = servo.Servo(pwm, min_pulse = 900, max_pulse = 2100)
        self.clip = lambda x: min(self.LIMITS) if x < min(self.LIMITS) else max(self.LIMITS) if x > max(self.LIMITS) else x
        self.map = lambda x: (x - min(self.LIMITS)) / (max(self.LIMITS) - min(self.LIMITS))
    
    def write_millis(self, pwm_millis):
        pwm_millis = self.clip(pwm_millis)
        angle = self.map(pwm_millis) * 180
        self.motor.angle = angle

class MOTORS:
    def __init__(self) -> None:
        self.fl = BLDC("GP21")
        self.fr = BLDC("GP20")
        self.br = BLDC("GP19")
        self.bl = BLDC("GP18")

    def set_speed(self, fl, fr, bl, br, ARM):
        self.fl.write_millis(fl if ARM else min(self.fl.LIMITS))
        self.fr.write_millis(fr if ARM else min(self.fr.LIMITS))
        self.bl.write_millis(bl if ARM else min(self.bl.LIMITS))
        self.br.write_millis(br if ARM else min(self.br.LIMITS))
