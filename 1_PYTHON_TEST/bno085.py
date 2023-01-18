import time
from math import atan2, sqrt, pi
from board import SCL, SDA
from busio import I2C
from adafruit_bno08x import (
    BNO_REPORT_STEP_COUNTER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = I2C(SCL, SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_STEP_COUNTER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)

# quat_real, quat_i, quat_j, quat_k


def find_heading(dqw, dqx, dqy, dqz):
    norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz)
    dqw = dqw / norm
    dqx = dqx / norm
    dqy = dqy / norm
    dqz = dqz / norm

    ysqr = dqy * dqy

    t3 = +2.0 * (dqw * dqz + dqx * dqy)
    t4 = +1.0 - 2.0 * (ysqr + dqz * dqz)
    yaw_raw = atan2(t3, t4)
    yaw = yaw_raw * 180.0 / pi
    if yaw > 0:
        yaw = 360 - yaw
    else:
        yaw = abs(yaw)
    return yaw  # heading in 360 clockwise

t_0 = time.time()
fails = 0
success = 0
try:
    while True:

        # the geomagnetic sensor is unstable
        # Heading is calculated using geomagnetic vector
        try:
            quat_i, quat_j, quat_k, quat_real = bno.quaternion
            heading = find_heading(quat_real, quat_i, quat_j, quat_k)
            geo_quat_i, geo_quat_j, geo_quat_k, geo_quat_real = bno.geomagnetic_quaternion
            heading_geo = find_heading(geo_quat_real, geo_quat_i, geo_quat_j, geo_quat_k)
            success += 1
            print(heading, heading_geo)
        except KeyError as e:
            fails+= 1
        except RemoteIO as e:
            fails += 1
except Exception as e:
    print(e)
    print(fails, success, time.time() - t_0) 
