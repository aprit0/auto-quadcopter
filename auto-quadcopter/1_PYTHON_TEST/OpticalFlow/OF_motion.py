import os
import time
import math
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import mode
from sklearn.cluster import KMeans



# function that analyses optical flow information
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
    k_means = KMeans(n_clusters=3, random_state=0).fit(inliers)
    centers = sorted(k_means.cluster_centers_)
    ratio = centers[0] / centers[-1]

    return ang_mode, transl_mode, ratio, steady




def main():
    cap = cv.VideoCapture(0)
    ret, first_frame = cap.read()
    prev_gray = cv.cvtColor(first_frame, cv.COLOR_BGR2GRAY)
    mask = np.zeros_like(first_frame)
    # Sets image saturation to maximum
    mask[..., 1] = 255
    while True:
        ret, frame = cap.read()
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        flow = cv.calcOpticalFlowFarneback(prev_gray, gray,
                                           None,
                                           0.5, 3, 15, 3, 5, 1.2, 0)
        # Computes the magnitude and angle of the 2D vectors
        magnitude, angle = cv.cartToPolar(flow[..., 0], flow[..., 1]) # magnitude, angle = (480, 640)
        # ang_mode, transl_mode, ratio, steady = estimate_motion(angle, magnitude)
        # print(ang_mode, transl_mode, ratio, steady)
        mask[..., 0] = angle * 180 / np.pi / 2
        mask[..., 2] = cv.normalize(magnitude, None, 0, 255, cv.NORM_MINMAX)
        rgb = cv.cvtColor(mask, cv.COLOR_HSV2BGR)
        cv.imshow("dense optical flow", rgb)



if __name__ == '__main__':
    main()
