import time
import numpy as np
import pandas as pd

from devices.optical_flow import FLOW
# from devices.tof import ZAXIS


def main():
    OF=FLOW()
    # tof = ZAXIS(ALT=False)
    odom = {i:[] for i in ['x', 'y', 'z', 'dx', 'dy', 'dz', 'dt']}

    t_start = time.time()
    t_last = t_start
    # while time.time() - t_start < 2:
    while True:
        OF.read()
        # tof.read()
        dt = time.time() - t_last
        odom['x'].append(OF.pose[0])
        odom['y'].append(OF.pose[1])
        odom['dx'].append(OF.twist[0])
        odom['dy'].append(OF.twist[1])

        odom['z'].append(0)#tof.height)
        odom['dz'].append(0)# tof.d_height)

        odom['dt'].append(dt)

        print([odom[key][-1] for key in odom.keys()])
        t_last = time.time()
        time.sleep(0.01)
    
    df = pd.DataFrame(odom)
    df.to_csv('Outputs/OF_calib.csv')


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)