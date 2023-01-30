import serial              
from time import sleep
import sys


def convert_to_degrees(raw_value):
    print(raw_value)
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = f'{position:.4f}'
    return position

def gps():
        received_data = (str)(ser.readline()) #read NMEA string received
        GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                
        if (GPGGA_data_available>0):
            # print(received_data)
            GPGGA_buffer = received_data.split('$GNGGA,',1)[1]  #store data coming after “$GPGGA,” string
            NMEA_buff = (GPGGA_buffer.split(','))
            nmea_time = []
            nmea_latitude = []
            nmea_longitude = []
            nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
            nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
            nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
            nmea_height = NMEA_buff[8]
            print('NMEA Time: ', nmea_time,'\n')
            lat = (float)(nmea_latitude)
            lat = convert_to_degrees(lat)
            longi = (float)(nmea_longitude)
            longi = convert_to_degrees(longi)
            print ('NMEA Latitude:', lat,'NMEA Longitude:', longi, 'NMEA Height', nmea_height, '\n')           
import time
if __name__ == '__main__':
    ser = serial.Serial ('/dev/ttyS0', 115200)
    gpgga_info = '$GNGGA,'
    GPGGA_buffer = 0
    NMEA_buff = 0
    while True:
        t_0 = time.time()
        # try:
        gps()
        # print('loop', time.time() - t_0)
        # except Exception as e:
            # print('failed', e)
