import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import csv 


def Lidar_read(data_name, threshold):

    # EXTRACT CSV AND TURN VALUES INTO FLOATS, REMOVE SHITTY VALUES
    df = pd.read_csv(data_name)
    good_data = list(df)[::6]
    data = [float(i) for i in good_data]

    # do some data processing
    # Get rid of anything that is super close
    good_data = [x for x in data if x >= threshold]

    # GET THE ANGLE AND SEPARATE INTO X AND Y VALUES
    theta = np.transpose(np.pi / 180 * np.linspace(120, -120, len(good_data)))
    x = np.multiply(good_data, np.cos(theta))
    y = np.multiply(good_data, np.sin(theta))

    # GET ALL THE DATA IN ONE SPOT
    #[x, y], r, alpha
    data = [np.array([x, y]).transpose(), good_data, theta]

    return data

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
    Lidar_info.insert(0, 'degrees', Lidar_info['radians'] * (180/3.14))
    Lidar_info = Lidar_info.astype(float)

    return Header_info, Translation_info, Lidar_info

def polar2cart(Lidar_info):
    # GET THE ANGLE AND SEPARATE INTO X AND Y VALUES
    theta = np.transpose(np.pi / 180 * np.linspace(120, -120, len(Lidar_info[0])))
    x = np.multiply(Lidar_info[0], np.cos(theta))
    y = np.multiply(Lidar_info[0], np.sin(theta))

    plt.figure()
    plt.scatter(x, y)
    # GET ALL THE DATA IN ONE SPOT
    #[x, y], r, alpha
    data = [np.array([x, y]).transpose(), Lidar_info.iloc, theta]

    return data

