import csv
from pprint import pprint
import numpy as np
import matplotlib.pyplot as plt
import math

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)


rows = []
with open('/home/ubuntu/turtlebot4_ws/src/CC_LidarCollect/CC_LidarCollect/CCLidar_data.csv', 'r') as file:
    csvreader = csv.reader(file)
    header = next(csvreader)
    for row in csvreader:
        rows.append(row)

sensor_angle = float(header[2]) # this is the lidar angel
ranges = rows[1]

sensor_angle = sensor_angle * (180 / 3.14) 
# 0 = 0
# 90 = 269
# 180 = 540
# 270 = 810

array = np.arange(1, 1081)
degree_key = list(np.multiply(float(sensor_angle), array))

Sensor_data = rows[2::3] #get the good sensor data
Sensor_data = list(np.array(Sensor_data).astype(float))
Sensor_data = np.array(Sensor_data)

deg0 = Sensor_data[:, 0]
deg90 = Sensor_data[:, 269]
deg180 = Sensor_data[:, 540]
deg270 = Sensor_data[:, 810]

std0 = np.std(deg0)
std90 = np.std(deg90)
std180 = np.std(deg180)
std270 = np.std(deg270)

' for plotting '
array = np.arange(1, 1081)


x, y = pol2cart(list(np.array(ranges).astype(float)), list(np.multiply(float(sensor_angle), array)))

x = list(x)
y = list(y)

good_x = [z for z in x if z != 'inf']
good_y = [z for z in y if z != 'inf']

plt.figure()
plt.scatter(good_x, good_y)
plt.savefig("turtlebot4_ws/src/CC_LidarCollect/CC_LidarCollect/plot.png")