import numpy as np
import math
import csv
from pprint import pprint
import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd
from CsvRead import CSV_Read_Lidar_data

# path to csv data
data_path = 'Hallway_Lidar_data_Bathroom.csv'
Header_info, Translation_info, Lidar_info = CSV_Read_Lidar_data(data_path)

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
k_r =  .001
k_l =  .001

Q_t  = np.array([[1.0,   0],
                [  0, 1.0]])
'1. Robot Position Prediction'



'2. Observation'
# alpha and rho from the lidar data
alpha_t = 'line parameter 1'
rho_t = 'line parameter 2'

z_t = np.empty((2,1))
z_t = np.array([alpha_t],
               [rho_t])

# Covariane matrix for eachlinec calculated
# TODO get values  ~~~> call obersvation funtion to update
sigma_aa = 1 
sigma_ar = 0
sigma_ra = 0
sigma_rr = 1

R_t = np.empty((2,2))
R_t = np.array([sigma_aa, sigma_ar ],
               [sigma_ra, sigma_rr ])


F_k_minus_1 = np.array([[1.0,  0],
                        [ 0, 1.0]])


'3. Measurement Prediction'


'4. Matching'


'5. Estimation'


"""
Begining the first portions of the math for the Kalman filter implementations
"""

"""
getting predicted positions p_prime - book 337
"""





def position_prediction(pos_t_minus1,delta_sl,delta_sr):

    delta_sl
    delta_sr
    theta_t_minus1 = pos_t_minus1[2,1]
     
    x_hat = np.empty((3,1))

    # This is previous postion + estimate of future position
    x_hat = pos_t_minus1 + np.array([(delta_sr+delta_sl)/2*math.cos(theta_t_minus1+(delta_sr-delta_sl)/2/b)],
                                    [(delta_sr+delta_sl)/2*math.sin(theta_t_minus1+(delta_sr-delta_sl)/2/b)],
                                    [(delta_sr-delta_sl)/b])
    
    # covarieance of the previouse robot state
    P_t_minus1 = np.array([k_r*abs(delta_sr), 0]
                           [0,k_l*abs(delta_sl) ])
    

    P_hat_t = F_k_minus_1 @ P_t_minus1 @ F_k_minus_1.T +   Q_t 
    return x_hat, P_hat_t

"""
the obsorvation step should spin the lidar, calculate and return the lines in the robot's frame (at timestep t)

"""
def observation(data):

    N = len(data)
    z_t = np.zeros(2,1,N)  # z_t^i = [alpha_t^i, r_t^i]^T for 0<i<N lines
    R_t = np.zeros(2,2,N)

    z_t[0,1,:] = data.alphas 
    z_t[1,1,:] = data.rhos

    return z_t,R_t


"""
The measurements needs to take the MAP data and move the lines into the Robts frame (from world) THEN match lines together or something
"""
def measurement_prediction(x_hat,map):

    N = len(map)
    z_hat_t = np.zeros(2,1,N) # The pridiction of what the Lines detected should be 
    alpha_map = data.alphas
    rho_map = data.rhos
    z_hat_t[0,1,:] = alpha_map[:] - x_hat[2] # removing the robots orientation in the world to rotate the line angles into frame
    z_hat_t[1,1,:] = rho_map[:]-x_hat[0]*np.cos(alpha_map[:])+x_hat[1]*np.sin(alpha_map[:]) # translation portion for the lines

    return 0



"""
find our estimated covarience of the fitted lines.
We assume the varience of angle and distance are calculated before hand and constant for each collects point.

This code is a converstion of Jerimiah's 

params: 
data = 

"""

def covarience_line_fitting(data,sigma_angle = 0, sigma_dist = .005):
    sigma_angle = sigma_angle *np.ones(len(data))
    sigma_dist = sigma_dist* np.ones(len(data))

    dist = data[0] #whatever positions stores the distances from 0,0
    angle = data[1] #whatever positions stores the angles with the x axis
    x = dist*np.cos(angle)
    y = dist*np.sin(angle)

    n = len(x)
    x_bar = sum(x)/n
    y_bar = sum(y)/n

    S_x2 =sum( (x-x_bar)**2)
    S_y2 =sum( (y-y_bar)**2)
    S_xy =sum( (x-x_bar)*(y-y_bar))
    
    # line paramters based on inputs data
    alpha = 0.5*math.atan2(-2*S_xy, S_y2-S_x2)
    rho = x_bar*math.cos(alpha) + y_bar*math.sin(alpha)
    for i in range(0,n-1):

        #The covariance of the measurement
        C_m = np.array([[ sigma_angle(i), 0]
                       [0, sigma_dist(i)]]); 
        A = np.zeros((2,2))

        # The jacobian of the line fit with respect to x and y
        A[1,0] = ((y_bar-y(i))*(S_y2-S_x2)+2*S_xy*(x_bar-x(i)))/((S_y2-S_x2)^2 + 4*S_xy^2)

        A[1,1] = ((x_bar-x(i))*(S_y2-S_x2)-2*S_xy*(y_bar-y(i)))/((S_y2-S_x2)^2+4*S_xy^2)

        A[0,0] = math.cos(alpha)/n-x_bar*math.sin(alpha)*A(1,0)+y_bar*math.cos(alpha)*A(1,0)
        A[0,1] = math.sin(alpha)/n-x_bar*math.sin(alpha)*A(1,1)+y_bar*math.cos(alpha)*A(1,1)

        # Jacobian of function converting dist and angle to x and y

        B = np.array([[math.cos(angle[i]),-dist[i]*math.sin(angle[i])]
                      [math.sin(angle[i]),-dist[i]*math.cos(angle[i])]])
        J = A @ B
        C_l = C_l+J*C_m*J.T

    return rho, alpha, C_l

