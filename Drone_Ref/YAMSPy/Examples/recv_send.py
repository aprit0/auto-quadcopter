"""read_altitude.py: Reboot your flight controller (Betaflight)

Copyright (C) 2020 Ricardo de Azambuja

This file is part of YAMSPy.

YAMSPy is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

YAMSPy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with YAMSPy.  If not, see <https://www.gnu.org/licenses/>.

Acknowledgement:
This work was possible thanks to the financial support from IVADO.ca (postdoctoral scholarship 2019/2020).

Disclaimer (adapted from Wikipedia):
None of the authors, contributors, supervisors, administrators, employers, friends, family, vandals, or anyone else 
connected (or not) with this project, in any way whatsoever, can be made responsible for your use of the information (code) 
contained or linked from here.
"""
from yamspy import MSPy
import time


#
# On Linux, your serial port will probably be something like
# /dev/ttyACM0 or /dev/ttyS0 or the same names with numbers different from 0
#
# On Windows, I would expect it to be 
# COM1 or COM2 or COM3...
#
# This library uses pyserial, so if you have more questions try to check its docs:
# https://pyserial.readthedocs.io/en/latest/shortintro.html
#
#
serial_port = "/dev/ttyUSB0"

CMDS_init = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 900,
        'yaw':      1500,
        'aux1':     1000, # DISARMED (1000) / ARMED (1800)
        'aux2':     1000, # ANGLE (1000) / HORIZON (1500) / FLIP (1800)
        'aux3':     1000, # FAILSAFE (1800)
        'aux4':     1000  # HEADFREE (1800)
        }
CMDS = [1500, 1500, 900, 1500, 1800, 1800, 1800, 1800]

with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200) as board:
    # Read info from the FC
    # Please, pay attention to the way it works:
    # 1. Message is sent: MSP_ALTITUDE without any payload (data=[])
    command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO',
                            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']
    for msg in command_list:
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
    while True:
        t_0 = time.time()
        board.fast_read_attitude()
        print(board.SENSOR_DATA['kinematics'])
        t_1 = time.time()
        CMDS = [1800]*2 + [1800] + [1000] + CMDS[4:]
        print(CMDS)
        #board.send_RAW_RC(CMDS)
        #print(t_1 - t_0, time.time() - t_1)
        board.send_RAW_RC(CMDS)
        print(board.bit_check(board.CONFIG['mode'],0))

# For some msgs there are available specialized methods to read them faster:
# fast_read_altitude
# fast_read_imu
# fast_read_attitude
# fast_read_analog
# fast_msp_rc_cmd
#
# Notice they all start with "fast_" ;)
