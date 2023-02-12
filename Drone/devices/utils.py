import numpy as np

def quat_2_euler(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)

    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z

def map(value, old_min, old_max, new_min=0, new_max=255):
    value = old_min if value < old_min else old_max if value > old_max else value
    OldRange = (old_min - old_max)  
    NewRange = (new_min - new_max)  
    new_value = (((value - old_min) * NewRange) / OldRange) + new_min
    return new_value
