
# # Kalman Filter 


# include libraries
import numpy as np
import math
import csv
import matplotlib.pyplot as plt
import pandas as pd


# # Needed Functions


def CSV_Read_Lidar_data(data_path):
    rows = []
    with open(data_path, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            rows.append(row)

    dataframe = pd.DataFrame([rows[0], rows[1], rows[2], rows[3]], ['Rho', 'Alpha', 'X', 'Y'])

    return dataframe

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


# Bunch of algorithms for split and merge

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

def points_within_radius(mainpoint, points, r):
    result = []
    for point in points:
        if math.dist(mainpoint, point) <= r:
            result.append(point)
    return result

def gap_detection(lines, points, threshold):
    good_lines = []
    points_in_thresh_total = []
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
        
        if len(points_in_thresh) <= 5 and line_dist <= 0.3:
            good_lines.append(lines[i])
            points_in_thresh_total.append(points_in_thresh)
            continue

        # check to see what % of points are between the threshold of the first and last point (might need my own threshold)
        p1_points = points_within_radius(point_1, points_in_thresh, r)
        p2_points = points_within_radius(point_2, points_in_thresh, r)
        # print(len(p1_points))
        # print(len(p2_points))
        # print(len(points_in_thresh))

        percent_in_radius = (len(p1_points) + len(p2_points)) / (len(points_in_thresh))
        # print(percent_in_radius)

        if percent_in_radius <= 0.40:
            # print("good line")
            good_lines.append(lines[i])
            points_in_thresh_total.append(points_in_thresh)
        # else:
        #     print("bad line")
        # plt.show()
        # print("\n")
        
    return good_lines, points_in_thresh_total

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

def Algorithm_split_and_merge(inputdataframe, threshold=0.3, plot=False):

    P = np.array([list(inputdataframe['X']), list(inputdataframe['Y'])]).T

    points = SplitAndMerge(P, threshold)

    lines = []
    for i in range(len(points) - 1):
        lines.append([points[i], points[i + 1]])
        # plt.plot([points[i][0], points[i+1][0]], [points[i][1], points[i+1][1]], '-o')
    # final_lines = lines
    final_lines, points_in_line = gap_detection(lines, P, threshold)

    # flatten it to get the shitty points
    flat_list = flatten(final_lines)
    flat_list.append(flat_list[0])
    flat_list = np.array(flat_list)

    #convert from xy back to alpha rho
    alpha_rho = []
    for i in range(len(final_lines)):
        alpha, rho = GetPolar([final_lines[i][0][0], final_lines[i][1][0]], [final_lines[i][0][1], final_lines[i][1][1]])
        alpha_rho.append([alpha, rho])

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

    return final_lines, points_in_line, alpha_rho

def matching(z_hat_t,z_t,R_t,H_j,P_hat_t,g):

    matches = []
    v_t_matches = []
    sigmas_matches = []
    H_matches = []
    #initializedng vt

    v_t = np.zeros((2,1,len(z_t),len(z_hat_t)))
    sigma_itt = np.zeros((2,2,len(z_t),len(z_hat_t)))
    #This could be vectorized or whatever but i think itll be okay
    for i in range(len(z_t)):
        for j in range(len(z_hat_t)):
            v_t[:,:,i,j] = z_t[:,:,i] - z_hat_t[:,:,j]
            
            sigma_itt[:,:,i,j] = H_j[:,:,j] @ P_hat_t @ H_j[:,:,j].T + R_t[i]
    # Mahalanobis distance
    for i in range(len(z_t)):
        for j in range(len(z_hat_t)):
            v_ = v_t[:,:,i,j]
            sigma_ = sigma_itt[:,:,i,j]
            # print(v_)
            # print(v_.T)
            mah_dist = v_.T @ sigma_ @ v_
            # print(mah_dist)
            if mah_dist <= g**2:
                matches.append([i,j])
                v_t_matches.append(v_t[:,:,i,j])
                sigmas_matches.append(sigma_itt[:,:,i,j])
                H_matches.append(H_j[:,:,j])
                
    return matches, v_t_matches, sigmas_matches,H_matches

def pos_estimation(H_t, x_hat, v_t,P_t_hat,sigmas):
    # print(H_t)
    # print(v_t)
    # print(sigmas)
    for i in range(len(H_t)):
        K_t = P_t_hat @ H_t[i].T @ np.linalg.pinv(sigmas[i])
        P_t = P_t_hat - K_t @ sigmas[i] @ K_t.T
        x_t = x_hat + K_t @ v_t[i]
        x_hat = x_t

        # print(x_t)

    if range(len(H_t)) == 0:
        x_t = x_hat
        P_t = P_t_hat
    return x_t, P_t

def measurement_prediction(x_hat, gt_map_df):
    """
    The measurements needs to take the MAP data and move the lines into the Robts frame (from world) THEN match lines together or something
    x_hat = 
    map = gt map
    data = observed map data
    """
    N = int(gt_map_df.shape[1])
    z_hat_t = np.zeros([2, 1, N]) # The pridiction of what the Lines detected should be, from map
    alpha_map = gt_map_df.loc['Alpha'].astype(float)
    rho_map = gt_map_df.loc['rhos'].astype(float)

    z_hat_t[0,0,:] = alpha_map[:] - x_hat[2] # removing the robots orientation in the world to rotate the line angles into frame
    z_hat_t[1,0,:] = rho_map[:]-x_hat[0]*np.cos(alpha_map[:])+x_hat[1]*np.sin(alpha_map[:]) # translation portion for the lines

    H_j = np.zeros([2,3,N])

    for k in range(N):
        H_j[:,:,k] = np.array([[0,  0,  -1],  
                    [-np.cos(alpha_map[k]), -np.sin(alpha_map[k]), 0]])  #it might not be able to handle this notation. Could easily be a loop

    return z_hat_t,H_j

def position_prediction(pos_t_minus1, delta_sl, delta_sr, b,P_t_minus1):

    delta_sl
    delta_sr
    theta_t_minus1 = pos_t_minus1[2]
  
    x_hat = np.empty((3,1))

    # This is previous postion + estimate of future position
    x_hat = pos_t_minus1 + np.array([[(delta_sr+delta_sl) / 2 * math.cos(theta_t_minus1 + (delta_sr-delta_sl) / 2 / b)],
                                    [(delta_sr+delta_sl) / 2 * math.sin(theta_t_minus1 + (delta_sr-delta_sl) / 2 / b)],
                                    [(delta_sr-delta_sl) / b]])
    
    delta_s = (delta_sr+delta_sl)/2
    delta_theta = (delta_sr-delta_sl) / b

    k_r =  .001
    k_l =  .001

    Q_t  = np.array([[k_r * abs(delta_sr), 0],
                    [0, k_l * abs(delta_sl) ]])
    F_deltarl = np.array([  [.5*math.cos(theta_t_minus1+delta_theta/2)-delta_s/2/b*math.sin(theta_t_minus1+delta_theta/2),.5*math.cos(theta_t_minus1+delta_theta/2)+delta_s/2/b*math.sin(theta_t_minus1+delta_theta/2)],
                            [.5*math.sin(theta_t_minus1+delta_theta/2)-delta_s/2/b*math.cos(theta_t_minus1+delta_theta/2),.5*math.sin(theta_t_minus1+delta_theta/2)+delta_s/2/b*math.cos(theta_t_minus1+delta_theta/2)],
                            [1/b,-1/b]])

    F_k_minus_1 = np.array([[1, 0, -delta_s*math.sin(theta_t_minus1+delta_theta/2)],
                            [0, 1,  delta_s*math.cos(theta_t_minus1+delta_theta/2)],
                            [0, 0, 1]])
    
    #TODO PUT IN THE FU MATRICXZ
    # @ is matrix multiplication
    P_hat_t = F_k_minus_1 @ P_t_minus1 @ F_k_minus_1.T + F_deltarl@ Q_t @ F_deltarl.T
    return x_hat, P_hat_t


# # Initializing map
# 

# Ground Truth Map Generation


# Load datapath and put into dataframe
# path to csv data
data_path = 'sutfftosend/Hallway_Lidar_data_dinosars2_GT.csv'
gt_map_df = CSV_Read_Lidar_data(data_path)
gt_map_df = gt_map_df.astype(float)

# Delete any column that has an inf in the rho spot
inf_cols = gt_map_df.loc['Rho'][np.isfinite(gt_map_df.loc['Rho'])]
gt_map_df = gt_map_df[inf_cols.index].transpose().reset_index(drop=True)


# # Plot the ground truth data
# plt.figure()
# plt.scatter(gt_map_df['X'], gt_map_df['Y'], s=1)
# plt.show()


# apply the split and merge algorithm
Lines, points_in_line, line_alpha_rho = Algorithm_split_and_merge(gt_map_df.astype(float),threshold=0.1, plot=False)

# Do covarience line fitting, save data to lists
alphas = []
rhos = []
covars = []
for i in range(len(points_in_line)):
    rho, alpha, C_l = covarience_line_fitting(points_in_line[i], line_alpha_rho[i])
    # line_info.append([alpha, rho, C_l])
    alphas.append(alpha)
    rhos.append(rho)
    covars.append(C_l)

# Create a dataframe with the good info
ground_truth_df = pd.DataFrame([alphas, rhos, covars, Lines, points_in_line], ['Alpha','rhos' ,'Covariance', 'Lines (endpoints)', ' Points within line'])
ground_truth_df


# 1. Robot Position Prediction
# Some values that are global

# Parameters for the robot 
# TODO this will be pulled from ros later
x_vel = 0.75
y_vel = 0.0
z_vel = 0.0



b = .235 # distance between robots wheels (meters)
wheel_radius = 0.072 # radius of the actual wheel (meters)

"""
These are the uncertainties in the are error constants representing the 
nondeterministic parameters of the motor drive and the wheel-floor interaction. Probably can be found in DOC or just tune for it
"""

# Pull these from ros, page 337, displacewmnt of left and right wheel
# ut = [delta_sl, delta_sr].T
#TODO these are placeholder values
delta_sl = 1.0
delta_sr = 1.0


# pos_t_minus1 is the [x_t-1, y_t-1, theta_t-1] position of the robot x_t-1
# the robot drives forward with the control input ut (above) to a position vector xt
# Both are world frames
#TODO get a value for this
pos_t_minus1 = np.array([[0],
                         [0],
                         [0]])
# initialize covarience matrix TODO intiialize this beter
P_t_minus1 = np.array([[.01,0,0],
                       [0,.01,0],
                       [0,0,.01]])


x_hat, P_hat_t = position_prediction(pos_t_minus1, delta_sl, delta_sr, b,P_t_minus1)
# print(x_hat)
# print(P_hat_t)


# # 2. Observation


# Load datapath and put into dataframe
# path to csv data
data_path = 'sutfftosend/Hallway_Lidar_data_dinosars2.csv'
scan_df = CSV_Read_Lidar_data(data_path)
scan_df = scan_df.astype(float)
print(scan_df)

for things in range(4):
    

    # Delete any column that has an inf in the rho spot
    inf_cols = scan_df.loc['Rho'][np.isfinite(scan_df.loc['Rho'])]
    scan_df = scan_df[inf_cols.index].transpose().reset_index(drop=True)
    scan_df


    # Run the split and merge and get covar
    # apply the split and merge algorithm
    Lines, points_in_line, line_alpha_rho = Algorithm_split_and_merge(scan_df.astype(float),threshold=0.1, plot=False)

    # Do covarience line fitting, save data to lists
    alphas = []
    rhos = []
    covars = []
    for i in range(len(points_in_line)):
        rho, alpha, C_l = covarience_line_fitting(points_in_line[i], line_alpha_rho[i])
        # line_info.append([alpha, rho, C_l])
        alphas.append(alpha)
        rhos.append(rho)
        covars.append(C_l)

    # Create a dataframe with the good info
    all_scan_df = pd.DataFrame([alphas, rhos, covars, Lines, points_in_line], ['Alpha','rhos' ,'Covariance', 'Lines (endpoints)', ' Points within line'])
    all_scan_df


    # alpha and rho from the lidar data
    z_t = np.array([[all_scan_df.loc['Alpha'].astype(float)],
                [all_scan_df.loc['rhos'].astype(float)]])

    # Covariane matrix for eachlinec calculated
    R_t = np.array(all_scan_df.loc['Covariance'])

    print(z_t.shape)
    print(R_t.shape)


    # # 3. Measuremnt Prediction


    z_hat_t, H_j = measurement_prediction(x_hat, ground_truth_df)


    # # 4. Matching


    #TODO what is g
    g = .3 # mess with this a bit

    matches, v_t_matches, sigmas_matches, H_j = matching(z_hat_t, z_t, R_t, H_j, P_hat_t, g)
    H_j
    sigmas_matches


    # # 5. Estimation


    x_t, P_t = pos_estimation(H_j, x_hat, v_t_matches, P_hat_t, sigmas_matches)

    P_t_minus1 = P_t
    pos_t_minus1 = x_t
    print(x_t)
    print(x_hat)



