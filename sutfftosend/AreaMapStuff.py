import numpy as np
import math
import csv
import matplotlib.pyplot as plt
import pandas as pd
from SplitAndMerge import Algorithm_split_and_merge

def CSV_Read_Lidar_data(data_path):
    rows = []
    with open(data_path, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            rows.append(row)

    # Lidar Dataframe
    # X values is the scan number, Y values are the scan pos
    Lidar_info = pd.DataFrame(np.array(rows[2::3]).T)
    Lidar_info.insert(0, "radians", np.array(list(np.multiply(float(Header_info['angle_increment']), np.arange(1, 1081))))) #insert the radians per scan
    Lidar_info.insert(0, "degrees", Lidar_info['radians'] * (180/3.14)) #insert the radians per scan
    Lidar_info = Lidar_info.astype(float)

    return Header_info, Translation_info, Lidar_info

def covarience_line_fitting(points_in_line, line_alpha_rho, sigma_angle=0, sigma_dist=.005):
    sigma_angle = sigma_angle * np.ones(len(points_in_line))
    sigma_dist = sigma_dist * np.ones(len(points_in_line))

    data = np.array(points_in_line)

    #INPUIT IS X AND Y POINTS WITHIN A LINE
    dist = line_alpha_rho[1]  # whatever positions stores the distances from 0,0
    angle = line_alpha_rho[0]  # whatever positions stores the angles with the x axis
    
    x = data[:,0]
    y = data[:,1]

    n = len(x)
    x_bar = sum(x) / n
    y_bar = sum(y) / n

    S_x2 = sum((x - x_bar) ** 2)
    S_y2 = sum((y - y_bar) ** 2)
    S_xy = sum((x - x_bar) * (y - y_bar))

    # line paramters based on inputs data
    alpha = 0.5 * math.atan2(-2 * S_xy, S_y2 - S_x2)
    rho = x_bar * math.cos(alpha) + y_bar * math.sin(alpha)

    C_l = np.zeros(2)
    for i in range(0, n - 1):
        # The covariance of the measurement
        C_m = np.array([[sigma_angle[i], 0],
                        [0, sigma_dist[i]]])
        A = np.zeros((2, 2))

        # The jacobian of the line fit with respect to x and y
        A[1, 0] = ((y_bar - y[i]) * (S_y2 - S_x2) + 2 * S_xy * (x_bar - x[i])) / ((S_y2 - S_x2) ** 2 + 4 * S_xy ** 2)

        A[1, 1] = ((x_bar - x[i]) * (S_y2 - S_x2) - 2 * S_xy * (y_bar - y[i])) / ((S_y2 - S_x2) ** 2 + 4 * S_xy **2)

        A[0, 0] = math.cos(alpha) / n - x_bar * math.sin(alpha) * A[1, 0] + y_bar * math.cos(alpha) * A[1, 0]
        A[0, 1] = math.sin(alpha) / n - x_bar * math.sin(alpha) * A[1, 1] + y_bar * math.cos(alpha) * A[1, 1]

        # Jacobian of function converting dist and angle to x and y

        B = np.array([[math.cos(angle), -dist * math.sin(angle)],
                      [math.sin(angle), -dist * math.cos(angle)]])
        J = A @ B
        C_l = C_l + J * C_m * J.T

    return rho, alpha, C_l

# path to csv data
data_path = 'sutfftosend/DownstairsGTdata.csv'
Header_info, Translation_info, Lidar_info = CSV_Read_Lidar_data(data_path)

Lines, points_in_line, line_alpha_rho = Algorithm_split_and_merge(Lidar_info, plot=True)


# line_info = []
# for i in range(len(points_in_line)):
#     rho, alpha, C_l = covarience_line_fitting(points_in_line[i], line_alpha_rho[i])
#     line_info.append([rho, alpha, C_l])

print('test')




