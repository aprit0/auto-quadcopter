
from gpiozero import Servo, Device
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()


servo = Servo(26, pin_factory = factory)

while True:
    servo.min()
    sleep(2)
    servo.mid()
    sleep(2)
    servo.max()
    sleep(2)

