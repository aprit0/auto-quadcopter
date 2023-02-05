import math

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import mode
from sklearn.cluster import KMeans, MiniBatchKMeans
import time
# https://medium.com/@ikunyankin/camera-motion-estimation-using-optical-flow-ce441d7ffec

def estimate_motion(angles, translation):
    '''
    Input:
        - angles - numpy array - array of angles of optical flow lines to the x-axis
        - translation - numpy array - array of length values for optical flow lines
    Output:
        - ang_mode - float - mode of angles of trajectories. can be used to determine the direction of movement
        - transl_mode - float - mode of translation values
        - ratio - float - shows how different values of translation are across a pair of frames. allows to
        conclude about the type of movement
        - steady - bool - show if there is almost no movement on the video at the moment
    '''
    # Get indices of nonzero opical flow values. We'll use just them
    nonzero = np.where(translation > 0)

    # Whether non-zero value is close to zero or not. Should be set as a thershold
    steady = np.mean(translation) < 0.5

    translation = translation[nonzero]
    transl_mode = mode(translation, keepdims=True)[0][0]

    angles = angles[nonzero]
    ang_mode = mode(angles, keepdims=True)[0][0]

    # cutt off twenty percent of the sorted list from both sides to get rid off outliers
    ten_percent = len(translation) // 10
    translations = sorted(translation)
    translations = translations[ten_percent: len(translations) - ten_percent]
    # cluster optical flow values and find out how different these cluster are
    # big difference (i.e. big ratio value) corresponds to panning, otherwise - trucking
    inliers = [tuple([inlier]) for inlier in translations]
    n_cpu = 2
    k_means = MiniBatchKMeans(n_clusters=3, random_state=0, batch_size=n_cpu*256, max_iter=10).fit(inliers)
    centers = sorted(k_means.cluster_centers_)
    ratio = centers[0] / centers[-1]
    return ang_mode, transl_mode, ratio, steady


def main():
    mult = 3
    dim = (64 * mult, 48 * mult)
    cap = cv.VideoCapture(0)
    ret, new = cap.read()
    first_frame = cv.resize(new, dim, interpolation=cv.INTER_AREA)
    prev_gray = cv.cvtColor(first_frame, cv.COLOR_BGR2GRAY)

    mask = np.zeros_like(first_frame)

    # Sets image saturation to maximum
    mask[..., 1] = 255
    t_0 = time.time()
    while (cap.isOpened()):
        ret, new = cap.read()
        frame = cv.resize(new, dim, interpolation=cv.INTER_AREA)
        # cv.imshow("input", frame)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        flow = cv.calcOpticalFlowFarneback(prev_gray, gray,
                                           None,
                                           0.5, 3, 15, 3, 5, 1.2, 0)
        magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1])  # magnitude, angle = (480, 640)
        magnitude
        ang_mode, transl_mode, ratio, steady = estimate_motion(angle, magnitude)
        print(math.degrees(ang_mode), transl_mode, ratio, steady)
        FPS = time.time() - t_0
        pixels_p_s = transl_mode / FPS
        perspective_angle = 0
        distance_pixels = ((dimsize_pixels / 2) /
                           math.tan(self.perspective_angle / 2))



        mask[..., 0] = angle * 180 / np.pi / 2
        mask[..., 2] = cv.normalize(magnitude, None, 0, 255, cv.NORM_MINMAX)
        rgb = cv.cvtColor(mask, cv.COLOR_HSV2BGR)
        cv.imshow("dense optical flow", rgb)
        prev_gray = gray
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
