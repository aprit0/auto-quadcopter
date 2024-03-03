import math as m

def quat_2_euler(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = m.degrees(m.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)

    t2 = -1.0 if t2 < -1 else 1 if t2 > 1.0 else t2
    Y = m.degrees(m.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = m.degrees(m.atan2(t3, t4))

    return X, Y, Z