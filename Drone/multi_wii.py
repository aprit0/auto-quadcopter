from YAMSPy.yamspy import MSPy
import time


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
CMDS = [1500, 1500, 900, 1500, 1000, 1000, 1000, 1000]

with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200) as board:
    while True:
        for i in range(1000, 2000, 1):
            board.fast_read_attitude()
            CMDS[2] = i
            board.send_RAW_RC(CMDS)
            print(i, board.SENSOR_DATA['kinematics'])

# For some msgs there are available specialized methods to read them faster:
# fast_read_altitude
# fast_read_imu
# fast_read_attitude
# fast_read_analog
# fast_msp_rc_cmd
#
# Notice they all start with "fast_" ;)
