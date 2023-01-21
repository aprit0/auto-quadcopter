import board
import busio
import adafruit_vl53l0x
import time
import cv2




class OpticalFlow:
    def __init__(self, camera_idx=0):
        # Distance sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.tof = adafruit_vl53l0x.VL53L0X(i2c)
        self.angle = None
        self.distance = None
        
        # Camera
        self.camera = cv2.VideoCapture(camera_idx)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)


    def get_pose(self):
        if self.angle != None and self.distance != None:
            pass    
    

    def get_distance(self):
        self.distance = self.tof.range * 0.001

    def set_angle(self, angle):
        self.angle = angle