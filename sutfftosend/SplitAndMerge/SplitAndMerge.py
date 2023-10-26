import numpy as np
from LidarRead import CSV_Read_Lidar_data
import matplotlib.pyplot as plt
import math


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
    alpha = math.atan(-1 / k)  # in radians
    ro = n / (math.sin(alpha) - k * math.cos(alpha))
    return ro, alpha


def CheckPolar(ro, alpha):
    if ro < 0:
        alpha = alpha + math.pi
        if alpha > math.pi:
            alpha = alpha - 2 * math.pi
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


# def point_to_line_distance(point, line_points):
#     x0, y0 = point
#     x1, y1 = line_points[0]
#     x2, y2 = line_points[1]
#
#     m = (y2 - y1) / (x2 - x1)
#     b = y1 - m * x1
#
#     distance = abs(m * x0 - y0 + b) / ((m**2 + 1)**0.5)
#     return distance


def points_within_radius(mainpoint, points, r):
    result = []
    for point in points:
        if math.dist(mainpoint, point) <= r:
            result.append(point)
    return result


def gap_detection(lines, points, threshold):
    good_lines = []

    for i in range(len(lines)):
        # get point 1 and point 2 of the line
        point_1 = lines[i][0]
        point_2 = lines[i][1]

        # get the distance of the line, then take a certain percentage of it (remember its based off both sides)
        line_dist = math.dist(point_2, point_1)
        r = line_dist / 2 * 0.10
        # print(r)

        # check all the points to see if they fall in theshold, store if they do
        points_in_thresh = []

        for j in range(len(points)):
            # distance = point_to_line_distance(points[j], lines[i])
            distance = getDistance(points[j], lines[i][0], lines[i][1])
            if distance <= (threshold * 1):
                # if distance < r:
                points_in_thresh.append(points[j])

        if len(points_in_thresh) <= 5 and line_dist <= 0.1:
            good_lines.append(lines[i])
            continue

        # check to see what % of points are between the threshold of the first and last point (might need my own threshold)
        p1_points = points_within_radius(point_1, points_in_thresh, r)
        p2_points = points_within_radius(point_2, points_in_thresh, r)
        # print(len(p1_points))
        # print(len(p2_points))
        # print(len(points_in_thresh))

        percent_in_radius = (len(p1_points) + len(p2_points)) / (len(points_in_thresh))
        # print(percent_in_radius)

        if percent_in_radius <= 0.15:
            # print("good line")
            good_lines.append(lines[i])
        # else:
        #     print("bad line")
        # plt.show()
        # print("\n")
    return good_lines


def SplitAndMerge(P, threshold):
    d, ind = GetMostDistant(P)
    if d > threshold:
        P1 = SplitAndMerge(P[:ind + 1, :], threshold)  # split and merge left array
        P2 = SplitAndMerge(P[ind:, :], threshold)  # split and merge right array
        # there are 2 "d" points, so exlude 1 (for example from 1st array)
        points = np.vstack((P1[:-1, :], P2))
    else:
        points = np.vstack((P[0, :], P[-1, :]))
    return points

def flatten(lst):
    result = []
    for item in lst:
        if isinstance(item, list):
            result.extend(flatten(item))
        else:
            result.append(item)
    return result

'~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'

# threshold = 0.3
#
# Header_info, Translation_info, Lidar_info = CSV_Read_Lidar_data('Hallway_Lidar_data_dinosars2.csv')  # 70
#
# data = Polar2Cartesian(Lidar_info[0], Lidar_info['radians'])
#
# P = np.array([data[:, 0][np.isfinite(data[:, 0])], data[:, 1][np.isfinite(data[:, 1])]]).T
#
# points = SplitAndMerge(P, threshold)
#
#
#
# lines = []
# for i in range(len(points) - 1):
#     lines.append([points[i], points[i + 1]])
#     # plt.plot([points[i][0], points[i+1][0]], [points[i][1], points[i+1][1]], '-o')
# # final_lines = lines
# final_lines = gap_detection(lines, P, threshold)
#
# #flatten it to get the shitty points
# flat_list = flatten(final_lines)
# flat_list.append(flat_list[0])
# flat_list = np.array(flat_list)
#
# for i in range(points.shape[0]-1):
#     ro, alpha = GetPolar(points[i:i + 2, 0], points[i:i + 2, 1])
#     ro, alpha = CheckPolar(ro, alpha)
#
# plt.figure()
# plt.title('og')
# plt.scatter(P[:, 0], P[:, 1], c='black')
# plt.plot(points[:,0], points[:,1])
#
# plt.figure()
# plt.title('with gap detection')
# plt.scatter(P[:, 0], P[:, 1], c='black')
# plt.plot(flat_list[:,0], flat_list[:,1], '-o')
#
# plt.figure()
# plt.title('actual Lines')
# plt.scatter(P[:, 0], P[:, 1], c='black')
# for i in range(len(final_lines)):
#     tmp = np.array(final_lines[i])
#     plt.plot(tmp[:, 0], tmp[:, 1], '-o')
# # print(len(lines))
# # print(len(final_lines))
# plt.scatter(0, 0, c='red')  # replace this with the origin point
# plt.show()


def Algorithm_split_and_merge(Lidar_info, threshold=0.3, plot=False):
    data = Polar2Cartesian(Lidar_info[0], Lidar_info['radians'])

    P = np.array([data[:, 0][np.isfinite(data[:, 0])], data[:, 1][np.isfinite(data[:, 1])]]).T

    points = SplitAndMerge(P, threshold)

    lines = []
    for i in range(len(points) - 1):
        lines.append([points[i], points[i + 1]])
        # plt.plot([points[i][0], points[i+1][0]], [points[i][1], points[i+1][1]], '-o')
    # final_lines = lines
    final_lines = gap_detection(lines, P, threshold)

    # flatten it to get the shitty points
    flat_list = flatten(final_lines)
    flat_list.append(flat_list[0])
    flat_list = np.array(flat_list)

    swag_money = []
    for i in range(points.shape[0] - 1):
        ro, alpha = GetPolar(points[i:i + 2, 0], points[i:i + 2, 1])
        ro, alpha = CheckPolar(ro, alpha)
        swag_money.append([ro, alpha])

    if plot==True:
        plt.figure()
        plt.title('og')
        plt.scatter(P[:, 0], P[:, 1], c='black')
        plt.plot(points[:, 0], points[:, 1])

        plt.figure()
        plt.title('with gap detection')
        plt.scatter(P[:, 0], P[:, 1], c='black')
        plt.plot(flat_list[:, 0], flat_list[:, 1], '-o')

        plt.figure()
        plt.title('actual Lines')
        plt.scatter(P[:, 0], P[:, 1], c='black')
        for i in range(len(final_lines)):
            tmp = np.array(final_lines[i])
            plt.plot(tmp[:, 0], tmp[:, 1], '-o')
        # print(len(lines))
        # print(len(final_lines))
        plt.scatter(0, 0, c='red')  # replace this with the origin point
        plt.show()

    return swag_money