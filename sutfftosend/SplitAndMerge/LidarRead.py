rows = []
with open('/home/ubuntu/turtlebot4_ws/src/CC_LidarCollect/CC_LidarCollect/CCLidar_data.csv', 'r') as file:
    csvreader = csv.reader(file)
    header = next(csvreader)
    for row in csvreader:
        rows.append(row)