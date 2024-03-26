import numpy as np
import math
from scipy.optimize import least_squares
# import csv
# import matplotlib.pyplot as plt
# import scipy as sp
# import time
# from scipy.signal import butter, lfilter, freqz, filtfilt
# from multiprocessing import Process, Queue

AnchorCount = 5  # 3 anchors + 2 tags

# Example locations - replace with your actual measurements
locations = np.array([[0.0000, 0.0000],   # Anchor A
                      [5.0000, 0.0000],   # Anchor B
                      [2.5000, 4.3301],   # Anchor C
                      [1.0000, 1.0000],   # Tag T1
                      [3.0000, 1.0000]])  # Tag T2

# Example measured ranges - replace with your actual measurements
measuredRanges = np.array([distance_A_B, distance_A_C, distance_A_T1, distance_A_T2,
                           distance_B_C, distance_B_T1, distance_B_T2,
                           distance_C_T1, distance_C_T2,
                           distance_T1_T2])  # Replace these with actual values

# # array of antenna to antenna range measurements in order e.g.
# # 1 to 2, 1 to 3, 1 to 4, 1 to 5, 1 to 6
# # 2 to 3, 2 to 4, 2 to 5, 2 to 6 ...
# measuredRanges = np.array([119.1093, 124.9640, 76.1164, 88.7342, 82.8843, 93.7682,
#                            96.9913, 102.3725, 117.9472, 141.0032, 150.4121,
#                            118.3195, 103.9067, 150.8950, 142.1869,
#                            97.0731, 95.4066, 112.9805,
#                            112.0142, 95.0258,
#                            97.1988])  # 6-7


def pythagorean(x, y):
    return math.sqrt(x * x + y * y)


def range_between_anchors(b1, b2):
    return pythagorean(b1[0] - b2[0], b1[1] - b2[1])


def calc_range_errors(delays):
    errors = 0
    measurement_count = 0

    for Anchor1 in range(0, AnchorCount - 1):
        for Anchor2 in range(Anchor1 + 1, AnchorCount):
            if measuredRanges[measurement_count] > 0:
                expected_value = range_between_anchors(locations[Anchor1], locations[Anchor2]) + delays[Anchor1] + delays[Anchor2]
                errors += (expected_value - measuredRanges[measurement_count])**2
            measurement_count += 1

    return errors


def do_least_sqr():
    initial = np.zeros(AnchorCount)
    results = least_squares(calc_range_errors, initial, jac='3-point', ftol=0.001)
    print("Delays are:", results.x[0:AnchorCount])


if __name__ == '__main__':
    do_least_sqr()
    print("Calibration Done")