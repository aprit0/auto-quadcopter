import time
import serial              
import pymap3d as pm
import numpy as np

class BN0:
    def __init__(self):
        self.ser = serial.Serial ('/dev/ttyS0', 115200)

        #params
        self.raw_origin = None # lat: deg, long: deg
        self.height_origin = None
        self.height = 1# None
        self.speed = None 
        self.local_origin = [0, 0, 0]
        self.raw_pose = None
        self.local_pose = None
        self.sat_status = 0 
        self.status = 0

    def calib(self):
        cal = [[], [], []]
        print('Begin GPS calibration')
        while len(cal[0]) < 5:
            sts, out = self.get_pose()
            if sts:
                cal[0].append(out[0])
                cal[1].append(out[1])
        self.raw_origin = [np.median(i) for i in cal]

    def read(self):
        self.status, out = self.get_pose()
        if type(self.raw_origin) == type(None) and self.status:
            self.calib()
        if self.raw_origin and self.status:
            self.raw_pose = out + [self.height]
            # self.local_pose = pm.geodetic2enu(*self.raw_pose, *self.raw_origin)
        return self.status

    
    def get_pose(self):
        # self.ser.flushInput()
        t_0 = time.time()
        received_data = str(self.ser.readline()) #read NMEA string received
        GPGGA_data_available = "GGA" in received_data   #check for NMEA GGA string                
        GNRMC_data_available = "RMC" in received_data   #check for NMEA GGA string                
        NMEA_buff = received_data.split(',')
        if GPGGA_data_available and NMEA_buff[6] != '0':
            gpgga_out = self.get_gngga(NMEA_buff)
            self.height = gpgga_out[2]
            return 1, gpgga_out[:2]
        elif GNRMC_data_available and NMEA_buff[2] == 'A':
            gnrmc_out = self.get_gnrmc(NMEA_buff)
            self.speed = gnrmc_out[2]
            return 1, gnrmc_out[:2]
        else:
            return 0, []

    def get_gngga(self, buff: list):
        [
            _,
            time, 
            lattitude, 
            lat_hem, 
            longitude, 
            long_hem, 
            status, 
            sat_num,
            _, # hdop_precision
            height,
            *_] = buff
        lat_sign = -1 if buff[2] == 'S' else 1
        long_sign = -1 if buff[4] == 'E' else 1
        lat = self.convert_to_degrees(lattitude) * lat_sign
        long = self.convert_to_degrees(longitude) * long_sign
        return [lat, long, float(height)]

    def get_gnrmc(self, buff: list):
        [
            _,
            time, 
            status, 
            lattitude, 
            lat_hem, 
            longitude, 
            long_hem, 
            speed,
            heading,
            _, # utc date 
            declination,
            declination_direction,
            _] = buff
        lat_sign = -1 if buff[2] == 'S' else 1
        long_sign = -1 if buff[4] == 'E' else 1
        lat = self.convert_to_degrees(lattitude) * lat_sign
        long = self.convert_to_degrees(longitude) * long_sign
        return [lat, long, float(speed)]




    @staticmethod
    def convert_to_degrees(raw: str):
        raw_value = (float)(raw)
        decimal_value = raw_value/100.00
        degrees = int(decimal_value)
        mm_mmmm = (decimal_value - int(decimal_value))/0.6
        position = degrees + mm_mmmm
        return position

if __name__ == '__main__':
    gps= BN0()
    while True:
        sts = gps.read()
        print('Pose: ', sts, gps.local_pose)