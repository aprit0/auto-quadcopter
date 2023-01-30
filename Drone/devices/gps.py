import time
import serial              
import pymap3d as pm
import numpy as np

class BN0:
    header = 'GGA'
    def __init__(self):
        self.ser = serial.Serial ('/dev/ttyS0', 115200)

        #params
        self.raw_origin = None # lat: deg, long: deg, height: m
        self.local_origin = [0, 0, 0]
        self.raw_pose = None
        self.local_pose = None

        self.calib()

    def calib(self):
        cal = [[], [], []]
        while len(cal[0]) < 5:
            sts, out = self.get_pose()
            if sts:
                cal[0].append(out[0])
                cal[1].append(out[1])
                cal[2].append(out[2])
        self.raw_origin = [np.median(i) for i in cal]

    def read(self):
        sts, out = self.get_pose()
        if sts:
            self.raw_pose = out
            self.local_pose = pm.geodetic2enu(*self.raw_pose, *self.raw_origin)
        return sts

    
    def get_pose(self):
        # self.ser.flushInput()
        GPGGA_data_available = 0
        t_0 = time.time()
        # while GPGGA_data_available <= 0 or time.time() - t_0 > 0.125:
        received_data = (str)(self.ser.readline()) #read NMEA string received
        GPGGA_data_available = received_data.find(self.header)   #check for NMEA GGA string                
        # print(time.time() - t_0)
        # print(received_data)
        if GPGGA_data_available > 0:
            GPGGA_buffer = received_data.split('$GNGGA,',1)[1]  #store data coming after “$GPGGA,” string
            NMEA_buff = (GPGGA_buffer.split(','))
            nmea_time = []
            nmea_latitude = []
            nmea_longitude = []
            nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
            nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
            nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
            nmea_height = (float)(NMEA_buff[8])
            lat = (float)(nmea_latitude)
            lat = self.convert_to_degrees(lat)
            long = (float)(nmea_longitude)
            long = self.convert_to_degrees(long)
            return 1, [lat, long, nmea_height]
        else:
            return 0, []

    @staticmethod
    def convert_to_degrees(raw_value):
        decimal_value = raw_value/100.00
        degrees = int(decimal_value)
        mm_mmmm = (decimal_value - int(decimal_value))/0.6
        position = degrees + mm_mmmm
        return position

if __name__ == '__main__':
    gps= BN0()
    while True:
        gps.read()
        print(gps.local_pose)