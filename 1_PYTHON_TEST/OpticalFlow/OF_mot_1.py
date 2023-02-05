import math

import cv2 as cv
import numpy as np
import time

def _get_velocity(flow, sum_velocity_pixels, dimsize_pixels,
                  distance_meters, timestep_seconds):
    count = (flow.shape[0] * flow.shape[1])

    average_velocity_pixels_per_second = (sum_velocity_pixels /
                                          count / timestep_seconds)
    perspective_angle = 0

    return (_velocity_meters_per_second(
        average_velocity_pixels_per_second,
        dimsize_pixels, distance_meters)
            if perspective_angle and distance_meters
            else average_velocity_pixels_per_second)


def _velocity_meters_per_second(velocity_pixels_per_second,
                                dimsize_pixels, distance_meters):
    perspective_angle = 0
    distance_pixels = ((dimsize_pixels / 2) /
                       math.tan(perspective_angle / 2))

    pixels_per_meter = distance_pixels / distance_meters

    return velocity_pixels_per_second / pixels_per_meter

def main():
    mult = 3
    dim = (64 * mult, 48 * mult)
    cap = cv.VideoCapture(0, cv.CAP_DSHOW)
    ret, new = cap.read()
    first_frame = cv.resize(new, dim, interpolation=cv.INTER_AREA)
    prev_gray = cv.cvtColor(first_frame, cv.COLOR_BGR2GRAY)

    mask = np.zeros_like(first_frame)

    # Sets image saturation to maximum
    mask[..., 1] = 255
    t_0 = time.time()
    x, y = 0, 0
    while (cap.isOpened()):
        ret, new = cap.read()
        frame = cv.resize(new, dim, interpolation=cv.INTER_AREA)
        cv.imshow("input", frame)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        flow = cv.calcOpticalFlowFarneback(prev_gray, gray,
                                           None,
                                           0.5, 3, 15, 3, 5, 1.2, 0)
        # magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])  # magnitude, angle = (480, 640)

        net = np.sum(flow, axis=(0,1))
        xsum = net[1]
        ysum = net[0]
        FPS = time.time() - t_0
        dist = 0.3
        xvel = _get_velocity(flow, xsum, flow.shape[1], dist,FPS)
        yvel = _get_velocity(flow, ysum, flow.shape[0], dist,FPS)
        x += xvel if abs(xvel) > 0.01 else 0
        y += yvel if abs(yvel) > 0.01 else 0
        print(x, y, FPS)
        t_0 = time.time()

        # mask[..., 0] = angle * 180 / np.pi / 2
        # mask[..., 2] = cv.normalize(magnitude, None, 0, 255, cv.NORM_MINMAX)
        # rgb = cv.cvtColor(mask, cv.COLOR_HSV2BGR)
        # cv.imshow("dense optical flow", rgb)
        prev_gray = gray
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
