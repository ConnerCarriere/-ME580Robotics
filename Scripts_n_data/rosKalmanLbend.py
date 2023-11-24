# include libraries
import numpy as np
import math
import csv
import matplotlib.pyplot as plt
import pandas as pd
import pickle
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)


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
                      [math.sin(angle), dist * math.cos(angle)]])
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
        # plt.figure()
        # plt.title('og')
        # plt.scatter(P[:, 0], P[:, 1], c='black')
        # plt.plot(points[:, 0], points[:, 1])

        # plt.figure()
        # plt.title('with gap detection')
        # plt.scatter(P[:, 0], P[:, 1], c='black')
        # plt.plot(flat_list[:, 0], flat_list[:, 1], '-o')

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

def matching(z_hat_t,z_t,R_t,H_j,P_hat_t,g,helper = True,world_first = False):

    matches = []
    v_t_matches = []
    sigmas_matches = []
    H_matches = []
    #initializedng vt
    #max((pittypoot.shape))
    v_t = np.zeros((2,1,max(z_t.shape),max(z_hat_t.shape)))
    sigma_itt = np.zeros((2,2,max(z_t.shape),max(z_hat_t.shape)))
    #This could be vectorized or whatever but i think itll be okay
    for i in range(max(z_t.shape)):
        for j in range(max(z_hat_t.shape)):
           
            v_t[:,0,i,j] = z_t[:,0,i] - z_hat_t[:,0,j]
            
            sigma_itt[:,:,i,j] = H_j[:,:,j] @ P_hat_t @ H_j[:,:,j].T + R_t[i]


    # Mahalanobis distance
    if world_first:
        for j in range(max(z_hat_t.shape)):
        
            mah_dist_best = g**2
            save_latch = False
            for i in range(max(z_t.shape)):
            # for j in range(12):
                v_ = v_t[:,:,i,j]
                sigma_ = sigma_itt[:,:,i,j]
                # print(v_)
                # print(v_.T)
                mah_dist = v_.T @ sigma_ @ v_
                
                if mah_dist <= g**2:
                    
                    if mah_dist <= mah_dist_best:
                        ibest = i
                        jbest = j
                        save_latch = True
                        mah_dist_best = mah_dist
                        
            if save_latch == True:
                print('bestmatch',ibest,jbest,'mah_dist_best',mah_dist_best)




                if helper == True:
                    # Additional outlier rejection
                    if v_t[0,0,ibest,jbest] < .10 and v_t[1,0,ibest,jbest] < .10:
                        matches.append([ibest,jbest])
                        v_t_matches.append(v_t[:,:,ibest,jbest])
                        sigmas_matches.append(sigma_itt[:,:,ibest,jbest])
                        H_matches.append(H_j[:,:,jbest])
                else:
                    matches.append([ibest,jbest])
                    v_t_matches.append(v_t[:,:,ibest,jbest])
                    sigmas_matches.append(sigma_itt[:,:,ibest,jbest])
                    H_matches.append(H_j[:,:,jbest])
    else:
        
        for i in range(max(z_t.shape)):
            mah_dist_best = g**2
            save_latch = False
            for j in range(max(z_hat_t.shape)):
            # for j in range(12):
                v_ = v_t[:,:,i,j]
                sigma_ = sigma_itt[:,:,i,j]
                # print(v_)
                # print(v_.T)
                mah_dist = v_.T @ sigma_ @ v_
                
                if mah_dist <= g**2:
                    
                    if mah_dist <= mah_dist_best:
                        ibest = i
                        jbest = j
                        save_latch = True
                        mah_dist_best = mah_dist
                        
            if save_latch == True:
                # print('bestmatch',ibest,jbest,'mah_dist_best',mah_dist_best)




                if helper == True:
                    # Additional outlier rejection
                    if v_t[0,0,ibest,jbest] < .10 and v_t[1,0,ibest,jbest] < .10:
                        matches.append([ibest,jbest])
                        v_t_matches.append(v_t[:,:,ibest,jbest])
                        sigmas_matches.append(sigma_itt[:,:,ibest,jbest])
                        H_matches.append(H_j[:,:,jbest])
                else:
                    matches.append([ibest,jbest])
                    v_t_matches.append(v_t[:,:,ibest,jbest])
                    sigmas_matches.append(sigma_itt[:,:,ibest,jbest])
                    H_matches.append(H_j[:,:,jbest])
    # print(matches)

                
    return matches, v_t_matches, sigmas_matches,H_matches

def pos_estimation(H_t, x_hat, v_t,P_t_hat,sigmas):
 
    #for number of matched lines
    for i in range(len(H_t)):
        K_t = P_t_hat @ H_t[i].T @ np.linalg.pinv(sigmas[i])
        # print(f'H_t = {H_t}')
        P_t = P_t_hat - K_t @ sigmas[i] @ K_t.T

        x_t = x_hat + K_t @ v_t[i]
        x_hat = x_t

    if len(H_t) == 0:
        x_t = x_hat
        P_t = P_t_hat
        return x_t, P_t

    
    return x_t, P_t

def pos_estimation_v2(H_t, x_hat, v_t,P_t_hat,sigma):
 
    #for number of matched lines
    
    K_t = P_t_hat @ H_t.T @ np.linalg.pinv(sigma)
    # print(f'H_t = {H_t}')
    P_t = P_t_hat - K_t @ sigma @ K_t.T

    x_t = x_hat + K_t @ v_t
    x_hat = x_t

   
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

def position_prediction_odom(pos_t_minus1, delta_sl, delta_sr, b,P_t_minus1):

    delta_sl
    delta_sr

    theta_t_minus1 = pos_t_minus1[2]
  
    x_hat = np.empty((3,1))

    

    # This is previous postion + estimate of future position
    x_hat = np.add(pos_t_minus1, np.array([[(delta_sr+delta_sl) / 2 * (math.cos(theta_t_minus1 + (delta_sr-delta_sl) / 2 / b))],
                                    [(delta_sr+delta_sl) / 2 * (math.sin(theta_t_minus1 + (delta_sr-delta_sl) / 2 / b))],
                                    [(delta_sr-delta_sl) / b]]))
    
    delta_s = (delta_sr+delta_sl)/2
    delta_theta = (delta_sr-delta_sl) / b

    k_r =  .001
    k_l =  .001

    Q_t  = np.array([[k_r * abs(delta_sr), 0],
                    [0, k_l * abs(delta_sl) ]])
    F_deltarl = np.array([  [.5*math.cos(theta_t_minus1+delta_theta/2)-delta_s/2/b*math.sin(theta_t_minus1+delta_theta/2),.5*math.cos(theta_t_minus1+delta_theta/2)+delta_s/2/b*math.sin(theta_t_minus1+delta_theta/2)],
                            [.5*math.sin(theta_t_minus1+delta_theta/2)+delta_s/2/b*math.cos(theta_t_minus1+delta_theta/2),.5*math.sin(theta_t_minus1+delta_theta/2)-delta_s/2/b*math.cos(theta_t_minus1+delta_theta/2)],
                            [1/b,-1/b]])

    F_k_minus_1 = np.array([[1, 0, -delta_s*math.sin(theta_t_minus1+delta_theta/2)],
                            [0, 1,  delta_s*math.cos(theta_t_minus1+delta_theta/2)],
                            [0, 0, 1]])
    
    #TODO PUT IN THE FU MATRICXZ
    # @ is matrix multiplication
    P_t_minus1_2 = P_t_minus1 * np.identity(3)
    P_hat_t = F_k_minus_1 @ P_t_minus1_2 @ F_k_minus_1.T + F_deltarl@ Q_t @ F_deltarl.T
    # pos_t_minus1 = x_hat
    return x_hat, P_hat_t
def position_prediction(pos_t_minus1, delta_x, delta_y,delta_theta, b,P_t_minus1):


    

    theta_t_minus1 = pos_t_minus1[2]
  
    x_hat = np.empty((3,1))

    if math.cos(theta_t_minus1+delta_theta/2) != 0:
        delta_s = delta_x/math.cos(theta_t_minus1+delta_theta/2) # Hand calc, divide by zero is possible but idk who cares
    else:
        delta_s = delta_y/math.sin(theta_t_minus1+delta_theta/2)

    delta_sl = .5*(-b*delta_theta + 2*delta_s)
    delta_sr = .5*(b*delta_theta + 2*delta_s)

    # This is previous postion + estimate of future position
    x_hat = np.add(pos_t_minus1, np.array([[delta_x],
                                    [delta_y],
                                    [delta_theta]]))
    
    delta_s = (delta_sr+delta_sl)/2
   

    k_r =  .001
    k_l =  .001

    Q_t  = np.array([[k_r * abs(delta_sr), 0],
                    [0, k_l * abs(delta_sl) ]])
    F_deltarl = np.array([  [.5*math.cos(theta_t_minus1+delta_theta/2)-delta_s/2/b*math.sin(theta_t_minus1+delta_theta/2),.5*math.cos(theta_t_minus1+delta_theta/2)+delta_s/2/b*math.sin(theta_t_minus1+delta_theta/2)],
                            [.5*math.sin(theta_t_minus1+delta_theta/2)+delta_s/2/b*math.cos(theta_t_minus1+delta_theta/2),.5*math.sin(theta_t_minus1+delta_theta/2)-delta_s/2/b*math.cos(theta_t_minus1+delta_theta/2)],
                            [1/b,-1/b]])

    F_k_minus_1 = np.array([[1, 0, -delta_s*math.sin(theta_t_minus1+delta_theta/2)],
                            [0, 1,  delta_s*math.cos(theta_t_minus1+delta_theta/2)],
                            [0, 0, 1]])
    
    #TODO PUT IN THE FU MATRICXZ
    # @ is matrix multiplication
    P_t_minus1_2 = P_t_minus1 * np.identity(3)
    P_hat_t = F_k_minus_1 @ P_t_minus1_2 @ F_k_minus_1.T + F_deltarl@ Q_t @ F_deltarl.T
    # pos_t_minus1 = x_hat
    return x_hat, P_hat_t
class KalmanFilter:

    def __init__(self, g = 0.005, b = 0.235):
        self.g = g
        self.b = b # distance between robots wheels (meters)
        self.P_t = np.array([[10, 0, 0], [0, 10, 0], [0, 0, .9]])
        self.all_xt = [] #used for plotting
        self.all_xhat = [] #used for plotting

    def initialize(self, ground_truth_df):
        self.pos_t_minus1 = np.array([[0],[0],[0]])     
        self.ground_truth_df = ground_truth_df
        self.pos_t = self.pos_t_minus1
        self.P_t_minus1 = np.array([[10, 0, 0],
                                    [0, 10, 0],                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
                                    [0, 0, .9]])
        
    

    def kalman_observe(self, scan_df):
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

        # print(covars)
        # Create a dataframe with the good info
        all_scan_df = pd.DataFrame([alphas, rhos, covars, Lines, points_in_line], ['Alpha','rhos' ,'Covariance', 'Lines (endpoints)', ' Points within line'])

        # alpha and rho from the lidar data
        self.z_t = np.array([[all_scan_df.loc['Alpha'].astype(float)],
                    [all_scan_df.loc['rhos'].astype(float)]])
        
        # Covariane matrix for eachlinec calculated
        self.R_t = np.array(all_scan_df.loc['Covariance'])
    #TODO this telometry needs a better data storing method, oh well- honestly shouldnt matter when using real time anyway
    def kalman_observe_tel(self, scan_number, robot_scan):
        inputdataframe = robot_scan[0]
        P = np.array([list(inputdataframe['0']), list(inputdataframe['1']),list(inputdataframe['5'])]).T
        # print(P)
        self.delta_x = -P[scan_number,0] + P[0,0]
        self.delta_y = -P[scan_number,1] + P[0,1]
        self.delta_theta = -P[scan_number,2] + P[0,2]
        print(self.delta_x,self.delta_y,self.delta_theta)
    #if giving deltas in terms of left and right wheels
    def kalman_update_odom(self, delta_sl, delta_sr):
        self.P_t_minus1 = self.P_t
        self.pos_t_minus1 = self.pos_t

        self.x_hat, self.P_hat_t = position_prediction_odom(self.pos_t_minus1, delta_sl, delta_sr, self.b, self.P_t_minus1)
        # print(self.x_hat)
        self.z_hat_t, self.H_j = measurement_prediction(self.x_hat, self.ground_truth_df)
        matches, v_t_matches, sigmas_matches, self.H_j = matching(self.z_hat_t, self.z_t, self.R_t, self.H_j, self.P_hat_t, self.g,helper=False, world_first=False)

        self.all_xhat.append(self.x_hat) #for plotting  

        for i in range(len(self.H_j)):
            self.x_t, self.P_t = pos_estimation_v2(self.H_j[i], self.x_hat, v_t_matches[i], self.P_hat_t, sigmas_matches[i])
            # print(x_t-x_hat)
            self.x_hat = self.x_t
            self.P_hat_t = self.P_t

        if len(self.H_j) < 1:
            self.P_t = self.P_hat_t
            self.x_t = self.x_hat
            print('badmatches')
        self.P_t = self.P_t
        self.pos_t = self.x_t

        # print(self.pos_t)

        
        self.all_xt.append(self.x_t) # for plotting
    #if giving deltas in terms of x, y and theta updates
    def kalman_update(self, delta_x, delta_y, delta_theta):
        self.P_t_minus1 = self.P_t
        self.pos_t_minus1 = self.pos_t

        self.x_hat, self.P_hat_t = position_prediction(self.pos_t_minus1, delta_x, delta_y,delta_theta, self.b, self.P_t_minus1)
        # print(self.x_hat)
        self.z_hat_t, self.H_j = measurement_prediction(self.x_hat, self.ground_truth_df)
        matches, v_t_matches, sigmas_matches, self.H_j = matching(self.z_hat_t, self.z_t, self.R_t, self.H_j, self.P_hat_t, self.g,helper=False, world_first=False)

        self.all_xhat.append(self.x_hat) #for plotting  

        for i in range(len(self.H_j)):
            self.x_t, self.P_t = pos_estimation_v2(self.H_j[i], self.x_hat, v_t_matches[i], self.P_hat_t, sigmas_matches[i])
            # print(x_t-x_hat)
            self.x_hat = self.x_t
            self.P_hat_t = self.P_t

        if len(self.H_j) < 1:
            self.P_t = self.P_hat_t
            self.x_t = self.x_hat
            print('badmatches')
        self.P_t = self.P_t
        self.pos_t = self.x_t

        # print(self.pos_t)

        
        self.all_xt.append(self.x_t) # for plotting
    def kalman_plot(self):
        plt.figure()
        plt.xlim(-1,3)
        plt.ylim(-1.5,1.5)
        color = []
        for i in range(len(self.all_xt)):
            
            color.append(i/len(self.all_xt))
        color = np.array(color)

        plt.scatter(np.array(self.all_xhat)[:,0], np.array(self.all_xhat)[:,1], c='red',s = 150)
        plt.scatter(np.array(self.all_xt)[:,0], np.array(self.all_xt)[:,1], c=color   ,s=100)
        plt.show()


# def main():
'Load in ground truth data ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
file_path = 'Scripts_n_data/Data_Readers_Writers/Data/'
file_name = 'HallwayNewMovepoint1meter_GT_LineData'
# file_name = 'Downstairsdata_GT_LineData'

data_path = file_path + file_name + '.pkl'

ground_truth_df = pd.read_pickle(data_path)
ground_truth_df = ground_truth_df.T


gt_endpoints = ground_truth_df.loc['Lines_(endpoints)']


# If you wanty to plot it GT
# fig = plt.figure()
# plt.title('map')
# # ax = fig.add_subplot()
# for i in range(max(ground_truth_df.shape)):
#     # print(np.array(gt_endpoints[i]))
#     apl = ground_truth_df.loc['Alpha'].astype(float)[i]
#     rh = ground_truth_df.loc['rhos'].astype(float)[i]
#     pts_2_plt = np.array(gt_endpoints[i])

#     plt.plot(pts_2_plt[:, 0], pts_2_plt[:, 1],'-')
#     plt.text((pts_2_plt[0, 0]+pts_2_plt[1, 0])/2,(pts_2_plt[0, 1]+pts_2_plt[1, 1])/2,'r = '+str(round(rh,3)) +'\n'+ ' a '+str(round(apl,3))+'\n'+ 'j'+str(i))
# plt.scatter(0, 0, c='red')  # replace this with the origin point
# plt.show()

'Call the kalman ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
Kalman = KalmanFilter(g= 0.01)
Kalman.initialize(ground_truth_df)

'Load the Obsercation data ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
# Load datapath and put into dataframe
# path to csv data
# data_path = 'Scripts_n_data/Data_Readers_Writers/Data/HallwayNewMovepoint1meter_scan_data_organized.csv'
data_path = 'Scripts_n_data/Data_Readers_Writers/Data/RISLHallwayWithTurn_scan_data_organized.csv'

# data_path = 'Data_Readers_Writers/Data/DownstairsScanV2_scan_data_organized.csv'

tmp_df = pd.read_csv(data_path)

#read and turn to list
lowrange = 0
highrange = 1070

scan_df = []
for i in range(int(tmp_df.shape[0]/1070)):
    lowrange = 0 + 1070 * i
    highrange = lowrange + 1070
    scan_df.append(tmp_df.iloc[lowrange:highrange, :])

# Delete any column that has an inf in the rho spot
for i in range(len(scan_df)):
    inf_cols = scan_df[i]['Rho'][np.isfinite(scan_df[i]['Rho'])]
    # print(inf_cols.index)
    scan_df[i] = scan_df[i].T[inf_cols.index].T.reset_index(drop=True)

'Loop the kalman ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'


delta_x = []
delta_y = []
delta_theta = []

#Manually input positions
for i in range(30):
    delta_x.append(.1)
    delta_y.append(0)
    delta_theta.append(0)

delta_x.append(0)
delta_y.append(0)
delta_theta.append(-math.pi/2)
for i in range(10):
    delta_x.append(0)
    delta_y.append(-.1)
    delta_theta.append(0)   


for robot_scans in range(38):
    #TODO pull in this from ROS
  
    # print(scan_df[robot_scans])

    Kalman.kalman_observe(scan_df[robot_scans])

    # Kalman.kalman_observe_tel(robot_scans,scan_df) # The scanner is still f'd

    Kalman.kalman_update(delta_x[robot_scans], delta_y[robot_scans],delta_theta[robot_scans])
    print(Kalman.pos_t)

Kalman.kalman_plot()
# plt.show()
