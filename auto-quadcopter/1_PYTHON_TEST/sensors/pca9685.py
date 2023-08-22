import random
from adafruit_servokit import ServoKit
# https://www.aranacorp.com/en/using-a-pca9685-module-with-raspberry-pi/


if __name__ == '__main__':
    channels = 16
    pca = ServoKit(channels=channels)
    # Init
    for i in range(channels):
        pca.servo[i].set_pulse_width_range(1000, 2000)
    
    # Run
    for i in range(channels):
        val = random.randint(0,180)
        pca.servo[i].angle = val
