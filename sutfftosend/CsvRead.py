import csv
from pprint import pprint
import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd

# def pol2cart(rho, phi):
#     x = rho * np.cos(phi)
#     y = rho * np.sin(phi)
#     return(x, y)


# rows = []
# with open('src/CC_HallwayScan/CC_HallwayScan/Hallway_Lidar_data_dinosars2.csv', 'r') as file:
#     csvreader = csv.reader(file)
#     header = next(csvreader)
#     for row in csvreader:
#         rows.append(row)



# # Create a Series for the header
# header_names = ['angle_min', 'angle_max', 'angle_increment', 'time_increment', 'scan_time', 'range_min', 'range_max']
# Header_info = pd.Series(header, header_names)

# # Translation Dataframe
# translation_names = ['trans_x', 'trans_y', 'trans_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w']
# Translation_info = pd.DataFrame(np.array(rows[1::3]).T, translation_names)

# # Lidar Dataframe
# # X values is the scan number, Y values are the scan pos

# Lidar_info = pd.DataFrame(np.array(rows[2::3]).T)
# Lidar_info.insert(0, "radians", np.array(list(np.multiply(float(Header_info['angle_increment']), np.arange(1, 1081))))) #insert the radians per scan


# sensor_angle = sensor_angle * (180 / 3.14) 
# 0 = 0
# 90 = 269
# 180 = 540
# 270 = 810

# ' for plotting '
# array = np.arange(1, 1081)

# plt.figure()
# x_full = []
# y_full = []
# indx = 0
# for i in range(len(rows[2::3])):
#     x, y = pol2cart(list(np.array(rows[2::3][i]).astype(float)), list(np.multiply(float(sensor_angle), array)))
#     y = y + .5*indx
#     x = list(x)
#     y = list(y)
   
#     good_x = [z for z in x if z != 'inf']
#     good_y = [z for z in y if z != 'inf']
#     x_full.append(good_x)
#     y_full.append(good_y)
#     indx = indx+1
# plt.scatter(x_full, y_full, s=3)
# plt.savefig("turtlebot4_ws/src/CC_HallwayScan/CC_HallwayScan/plot.png")


# THIS IS A FUNCITON YOU CAN CALL FROM OTHER PLACES TO GET THE DATA 
# Read Lidar
def CSV_Read_Lidar_data(data_path):
    rows = []
    with open(data_path, 'r') as file:
        csvreader = csv.reader(file)
        header = next(csvreader)
        for row in csvreader:
            rows.append(row)

    # Create a Series for the header
    header_names = ['angle_min', 'angle_max', 'angle_increment', 'time_increment', 'scan_time', 'range_min', 'range_max']
    Header_info = pd.Series(header, header_names)
    Header_info = Header_info.astype(float)

    # Translation Dataframe
    translation_names = ['trans_x', 'trans_y', 'trans_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w']
    Translation_info = pd.DataFrame(np.array(rows[1::3]).T, translation_names)
    Translation_info = Translation_info.astype(float)

    # Lidar Dataframe
    # X values is the scan number, Y values are the scan pos
    Lidar_info = pd.DataFrame(np.array(rows[2::3]).T)
    Lidar_info.insert(0, "radians", np.array(list(np.multiply(float(Header_info['angle_increment']), np.arange(1, 1081))))) #insert the radians per scan
    Lidar_info = Lidar_info.astype(float)

    return Header_info, Translation_info, Lidar_info