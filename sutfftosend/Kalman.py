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
# alpha and rho extracted from the lidar data
alpha_t = 'line parameter 1'
rho_t = 'line parameter 2'

z_t = np.empty((2,1))
z_t = np.array([alpha_t],
               [rho_t])


# extract lines and their uncertenties

# Covariane matrix for eachlinec calculated
# TODO get values
sigma_aa = 1 
sigma_ar = 1
sigma_ra = 1
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
    
    P_hat_t = F_k_minus_1 @ P_t_minus1 @ np.transpose(F_k_minus_1) + F_k_minus_1 @ Q_t @ np.transpose(F_k_minus_1)
    return x_hat, P_hat_t

