from gpiozero import PWMOutputDevice as PWM
import time
motor = PWM(26, frequency=500)

while True:
    for i in range(0, 100):
        print(i/100)
        motor.value = i / 100
        time.sleep(0.01)
    time.sleep(0.1)