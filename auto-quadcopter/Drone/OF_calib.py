import time
import numpy as np
import pandas as pd

from devices.optical_flow import FLOW
from devices.i2c_pico import PICO


def main():
    OF=FLOW()
    tof = PICO()
    odom = {i:[] for i in ['x', 'y', 'z', 'dx', 'dy', 'dz', 'dt']}

    t_start = time.time()
    t_last = t_start
    try:
        while time.time() - t_start < 30:
            if time.time() - t_last > 0.05:
                OF.read()
                tof.read()
                dt = time.time() - t_last
                odom['x'].append(OF.pose[0])
                odom['y'].append(OF.pose[1])
                odom['dx'].append(OF.twist[0])
                odom['dy'].append(OF.twist[1])

                odom['z'].append(tof.height)
                odom['dz'].append(tof.d_height)

                odom['dt'].append(dt)

                # print([odom[key][-1] for key in odom.keys()])
                print(tof.height)
                t_last = time.time()
    except Exception as e:
        print(e)

    df = pd.DataFrame(odom)
    df.to_csv('Outputs/OF_calib.csv')
    print('DF Complete')


if __name__ == '__main__':
    main()