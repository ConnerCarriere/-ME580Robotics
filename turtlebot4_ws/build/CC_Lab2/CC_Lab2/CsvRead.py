import csv
from pprint import pprint


rows = []
with open('/home/ubuntu/turtlebot4_ws/src/CC_Lab2/CC_Lab2/CCLidar_data.csv', 'r') as file:
    csvreader = csv.reader(file)
    header = next(csvreader)
    for row in csvreader:
        rows.append(row)
sensor_angle = header[2] # this is the lidar angel
sensor_min = header[0]
sensor_max = header[1]
sensor_range_min = header[5]
sensor_range_max = header[6]

# print(header)
# range_data = []
# for i in range(len(rows)):
#     range_data.append(rows[i][0])
print(f"sensor rotates {sensor_angle} rads every scan")
pprint(rows) # this is the lidar ranges
print(len(rows[1]))

# degree_rot = rad2deg(sensor_angle)
# print(f"total rotation per scan is {degree_rot} degrees")
# print(f"degree * len(rows[1]){degree_rot * len(rows[1])}")