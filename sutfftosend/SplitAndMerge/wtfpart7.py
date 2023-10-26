import numpy as np
import math as m
import matplotlib.pyplot as plt
from LidarRead import CSV_Read_Lidar_data


def Polar2Cartesian(r, alpha):
    return np.transpose(np.array([np.cos(alpha) * r, np.sin(alpha) * r]))


def Cartesian2Polar(x, y):
    r = np.sqrt(x ** 2 + y ** 2)
    phi = np.arctan2(y, x)
    return r, phi


def GetPolar(X, Y):
    # center the data
    X = X - np.mean(X)
    Y = Y - np.mean(Y)
    # fit line through the first and last point (X and Y contains 2 points, start and end of the line)
    k, n = np.polyfit(X, Y, 1)
    alpha = m.atan(-1 / k)  # in radians
    ro = n / (m.sin(alpha) - k * m.cos(alpha))
    return ro, alpha


def CheckPolar(ro, alpha):
    if ro < 0:
        alpha = alpha + m.pi
        if alpha > m.pi:
            alpha = alpha - 2 * m.pi
        ro = -ro
    return ro, alpha


def getDistance(P, Ps, Pe):  # point to line distance, where the line is given with points Ps and Pe
    if np.all(np.equal(Ps, Pe)):
        return np.linalg.norm(P - Ps)
    return np.divide(np.abs(np.linalg.norm(np.cross(Pe - Ps, Ps - P))), np.linalg.norm(Pe - Ps))


def GetMostDistant(P):
    dmax = 0
    index = -1
    for i in range(1, P.shape[0]):
        d = getDistance(P[i, :], P[0, :], P[-1, :])
        if (d > dmax):
            index = i
            dmax = d
    return dmax, index


def SplitAndMerge(P, threshold):
    d, ind = GetMostDistant(P)
    if (d > threshold):
        P1 = SplitAndMerge(P[:ind + 1, :], threshold)  # split and merge left array
        P2 = SplitAndMerge(P[ind:, :], threshold)  # split and merge right array
        # there are 2 "d" points, so exlude 1 (for example from 1st array)
        points = np.vstack((P1[:-1, :], P2))
    else:
        points = np.vstack((P[0, :], P[-1, :]))
    return points

def covarience_line_fitting(data, sigma_angle=0, sigma_dist=.005):
    # rho is distance
    # alpha is angle
    # sigma_angle = sigma_angle * np.ones(len(input_alpha))
    # sigma_dist = sigma_dist * np.ones(len(input_rho))
    sigma_angle = sigma_angle * np.ones(len(data))
    sigma_dist = sigma_dist * np.ones(len(data))

    dist = data[0]  # whatever positions stores the distances from 0,0
    angle = data[1]  # whatever positions stores the angles with the x axis
    # dist = input_rho  # whatever positions stores the distances from 0,0
    # angle = input_alpha  # whatever positions stores the angles with the x axis
    x = np.array((dist * np.cos(angle)))
    y = np.array((dist * np.sin(angle)))

    n = len(x)
    x_bar = sum(x) / n
    y_bar = sum(y) / n

    S_x2 = sum((x - x_bar) ** 2)
    S_y2 = sum((y - y_bar) ** 2)
    S_xy = sum((x - x_bar) * (y - y_bar))

    # line paramters based on inputs data
    alpha = 0.5 * math.atan2(-2 * S_xy, S_y2 - S_x2)
    rho = x_bar * math.cos(alpha) + y_bar * math.sin(alpha)

    C_l = np.zeros(2)  # for initiailziation
    for i in range(0, n - 1):
        # The covariance of the measurement
        C_m = np.array([[sigma_angle[i], 0],
                        [0, sigma_dist[i]]])

        A = np.zeros((2, 2))

        # The jacobian of the line fit with respect to x and y
        A[1, 0] = ((y_bar - y[i]) * (S_y2 - S_x2) + 2 * S_xy * (x_bar - x[i])) / ((S_y2 - S_x2)**2 + 4 * S_xy**2)

        A[1, 1] = ((x_bar - x[i]) * (S_y2 - S_x2) - 2 * S_xy * (y_bar - y[i])) / ((S_y2 - S_x2)**2 + 4 * S_xy**2)

        A[0, 0] = math.cos(alpha) / n - x_bar * math.sin(alpha) * A[1, 0] + y_bar * math.cos(alpha) * A[1, 0]
        A[0, 1] = math.sin(alpha) / n - x_bar * math.sin(alpha) * A[1, 1] + y_bar * math.cos(alpha) * A[1, 1]

        # Jacobian of function converting dist and angle to x and y

        B = np.array([[math.cos(angle[i]), -dist[i] * math.sin(angle[i])], [math.sin(angle[i]), -dist[i] * math.cos(angle[i])]])
        J = A @ B
        C_l = C_l + J * C_m * J.T

    return rho, alpha, C_l