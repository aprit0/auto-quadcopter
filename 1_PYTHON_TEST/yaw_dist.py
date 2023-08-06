import math as m
import numpy as np

def main():
    coords = [
        [60, 80, 20],
        [-40, 20, 60],
        [120, -170, 70],
        [-30, -60, 30]
    ]
    for idx, i in enumerate(coords):
        [x, y, d] = i
        y_new = y - x
        d_0 = y_new if abs(y_new) < 180 else y_new + (-1 * np.sign(y_new) * 360) 
            
        assert d == abs(d_0), f"{idx}, d:{d}, d_0{d_0} {[x, y]}"
    print('valid')


if __name__ == "__main__":
    main()