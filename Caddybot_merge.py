import math
import nav_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import time
import rclpy
import csv
from pprint import pprint
import pickle
import numpy as np
import PIL.Image as Image
import cv2
from rclpy.node import Node
import os
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.msg import WheelVels
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from irobot_create_msgs.action import RotateAngle
from geometry_msgs.msg import Twist

from tensorflow import keras
from CaddyBot_NN.Deploy_Keras_fcn import deploy_keras_fcn
from CaddyBot_NN.bbtomovement import bb_to_movement

import pandas as pd

class CamCapture(Node):
    def __init__(self):
        super().__init__('camera_capture')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 liveliness=LivelinessPolicy.AUTOMATIC,
                                 durability=DurabilityPolicy.VOLATILE,
                                 depth = 10)
        
        self.subscription = self.create_subscription(sensor_msgs.msg.Image,
                                                     '/oakd/rgb/preview/image_raw',
                                                     self.listener_callback,
                                                     qos_profile=qos_profile)   

        
        self.subscription

    def listener_callback(self, msg):
        # print('listener callback')
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.camera = msg

    def capture_save_image(self):
        cam_data = [[self.camera.height],
                    [self.camera.width],
                    [self.camera.encoding],
                    [self.camera.is_bigendian],
                    [self.camera.step],
                    [self.camera.data]]
        return cam_data

class MoveClient(Node):

    def __init__(self):
        super().__init__('turn_client')
        self._action_client = ActionClient(self, DriveDistance, '/drive_distance')

    def send_goal(self, dist, max_speed=1.0):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = dist
        goal_msg.max_translation_speed = max_speed

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

class TurnClient(Node):

    def __init__(self):
        super().__init__('turn_client')
        self._action_client = ActionClient(self, RotateAngle, '/rotate_angle')
    
    def send_goal(self, angle, max_rotation_speed = 0.7):
        
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

class MaxWheelVel(Node):

    def __init__(self):
        super().__init__('max_wheel_vel')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 liveliness=LivelinessPolicy.AUTOMATIC,
                                 durability=DurabilityPolicy.VOLATILE,
                                 depth = 10)
        self.subscription = self.create_subscription(WheelVels, '/wheel_vels', self.listener_callback, qos_profile)
        self.subscription
        self.max_vel = -1

    def listener_callback(self, msg):
        # print(msg.velocity_left, msg.velocity_right)
        self.max_vel = max(abs(msg.velocity_left), abs(msg.velocity_right))

class status():
    def __init__(self,name):
        self.name = name
        self.tics = 0
        self.has_human = False
    def reset(self):
        self.tics = 0

class ReadingLaser(Node):
    def __init__(self):
        super().__init__('reading_laser')

        self.subscription = self.create_subscription(sensor_msgs.msg.LaserScan,
                'scan',
                self.listener_callback,
                qos_profile=8)

    def listener_callback(self, msg):
        # self.get_logger().info('Got scan') 
        self.scan = msg

    def write_header(self):
        print([self.scan.angle_min, 
                         self.scan.angle_max,
                         self.scan.angle_increment,
                         self.scan.time_increment,
                         self.scan.scan_time,
                         self.scan.range_min,
                         self.scan.range_max])
        return(self.scan.angle_increment)
    def write_results(self, writer):
        writer.writerow(["RANGES"])
        writer.writerow(self.scan.ranges)
    def get_data(self):
        pass

class MovePublisher(Node):
    def __init__(self):
        super().__init__('move_publisher')
        self.publisher_ = self.create_publisher( Twist, '/cmd_vel', 10)
        # timer_period = 0.5
        #self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        msg = Twist()
        msg.linear.x = 0.15
        msg.angular.z = 0.0

        self.publisher_.publish(msg)

class TurnPublisher(Node):
    def __init__(self):
        super().__init__('move_publisher')
        self.publisher_ = self.create_publisher( Twist, '/cmd_vel', 10)
        # timer_period = 0.5
        #self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self, direction):

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = direction * 0.3

        self.publisher_.publish(msg)

def wait_for_action(max_wheel_vel):
    # Wait for the motion to start
    while max_wheel_vel.max_vel < 0.1:
        rclpy.spin_once(max_wheel_vel, timeout_sec=0.5)

        if max_wheel_vel.max_vel == 0:
            break

    num_zero = 3
    while 1:
        rclpy.spin_once(max_wheel_vel, timeout_sec=0.6)
        # Update the num_zero
        if max_wheel_vel.max_vel < 0.1:
            num_zero -= 1
        else:
            num_zero = 3
        # if we got three zeros in a row, stop the loop
        if num_zero < 1:
            break

        time.sleep(0.2)

def robot_turn(turn_client, angle):
    'Turn happens in radians'
    '+ value for ccw, - for cw'

    future = turn_client.send_goal(angle)
    rclpy.spin_until_future_complete(turn_client, future) # spin until complete

def robot_move(move_client, distance):
    # print(f"moving {distance} meters")
    'use this to move a certain distance in meters'
    future = move_client.send_goal(distance) # create future goal with required distance
    rclpy.spin_until_future_complete(move_client, future) # spin until complete

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x, y)
    
def headless(laser_reader,alphas,fov = 30):
    
    alpha_df = pd.DataFrame()
    rho_df = pd.DataFrame()
    
    lower = 270 - fov*2
    upper = 270 + fov*2

    rclpy.spin_once(laser_reader,timeout_sec=1)
    ap = alphas[lower:upper]
    rh = laser_reader.scan.ranges[lower:upper]
    # print(ap)
    min_location = rh.index(min(rh))
    
   
    
    
    return rh[min_location], ap[min_location]
def main():
    
    
    
    use_camera = False
    rclpy.init()
    camera = CamCapture()
    move_client = MoveClient()
    turn_client = TurnClient()
    turn_pub = TurnPublisher()
    move_pub = MovePublisher()
    laser_reader = ReadingLaser()
    max_wheel_vel = MaxWheelVel()

    print('Hi from ians_movement_detector!')
            
    for i in range(5):
        rclpy.spin_once(laser_reader,timeout_sec=1)
        
    angleinc = laser_reader.scan.angle_increment

    alphas = angleinc * np.arange(0,1080) - math.pi/2

    # Load in the keras model
    print('Loading Model')  
    model = keras.models.load_model('/home/ubuntu/turtlebot4_ws/src/Caddybot_merge/Caddybot_merge/Turtlenet_658_HA.keras')
    print('Model Loaded')
    
    index_left = 0
    index_right = 0
    index_angle = 0.1745 / 4 #2.5 degrees
    print('Hi from Caddybot_merge.')
    
    scans_per_set = 30
    
    delay = 5
    move_check = scans_per_set - delay
    robot_status = status('Robert')    

    while True:
        # headless(laser_reader,alphas,fov = 30)

        if use_camera == True:

            if robot_status.tics < 10:
                print('status,',robot_status.tics)
                'Capture an image'
                time.sleep(0.01)
                rclpy.spin_once(camera)
                cam_data = camera.capture_save_image()
                print('Captured image')

                'Process image'
                # Reformat image
                cam_data = np.reshape(cam_data[5][0], (250, 250, 3)).T

                # Open CV uses BGR PIL uses OpenCV
                image_np_RGB = np.ascontiguousarray(cam_data.transpose(2,1,0))
                # print(image_np_RGB.shape)
                image_np_BGR = np.flip(image_np_RGB, -1)
                # print(image_np_BGR.shape)

                # format so the prediction step can read it
                image_np_BGR = image_np_BGR.reshape((1,) + image_np_BGR.shape)
            
                image = image_np_BGR



                ' Predict '
                predictions = deploy_keras_fcn(model, image)

                ' Make movement based off prediction '
                movement_key_value = bb_to_movement(predictions)
                # print(movement_key_value)

                # movement_key_value = move_queue[0] #max(tmpdict, key=tmpdict.get)
                if movement_key_value == 1: # turn left (CCW)
                    # print('Turn Left')
                    turn_pub.timer_callback(1)  
                    robot_status.reset()
            
                
                if movement_key_value == 2: # move forward
                    # print('Move Forward')
                    # move_pub.timer_callback()
                    
                    distance1, angl1 = headless(laser_reader,alphas,fov = 30)
                    
                    if distance1 > .20:
                        print('need to move up')

                        robot_move(move_client, distance1-.20)
                        wait_for_action(max_wheel_vel) # wait for it to stop 
                        print(distance1,angl1)
                    else:
                        print('detected but in range')
                        print(distance1,angl1)

                    
                    robot_status.reset()
                
                if movement_key_value == 3: # turn right (CW)
                    # print('Turn Right')
                    turn_pub.timer_callback(-1)
                    robot_status.reset()

                if movement_key_value == 0:
                    print("No legs")
                    robot_status.tics = robot_status.tics + 1
            
            else:
                #Settings
                alpha_df = pd.DataFrame()
                rho_df = pd.DataFrame()
                angleinc = laser_reader.scan.angle_increment

                alphas = angleinc * np.arange(0,1080) - math.pi/2

                for i in range(scans_per_set):

                    rclpy.spin_once(laser_reader,timeout_sec=1)
                    alpha_df[i] = alphas
                    rho_df[i] = laser_reader.scan.ranges


                for j in range(move_check):
                                
                    difference = rho_df[i-j-delay].astype(float) - rho_df[i].astype(float)
                    indxi = np.array(list(difference[abs(difference.astype(float)) <= 0.1].index))  
                    indxii = np.array(list(difference[abs(difference.astype(float)) >= 12.0].index))
                    if j == 0:
                        indx = np.concatenate((indxi,indxii))
                    else:
                        indx = np.concatenate((indx,indxi,indxii))


                cartesian_df = pd.DataFrame(pol2cart(rho_df[i].astype(float), alpha_df[i].astype(float)), ['X', 'Y'])
                # print(cartesian_df.shape)
                indxiii = np.array(list(rho_df[i][abs(rho_df[i].astype(float)) >= 12.0].index))
                indx = np.concatenate((indx,indxiii))

                row_now = rho_df[i] 
                alpha_now = alpha_df[i]
                row_now.drop(indx)
                alpha_now.drop(indx)
                cartesian_df = cartesian_df.drop(indx, axis=1).T
                # plt.scatter(cartesian_df['X'], cartesian_df['Y'], s=5,color =colors[i])
                x = np.array(list(cartesian_df['X'].astype(float)))
                y = np.array(list(cartesian_df['Y'].astype(float)))
                

                # movement detected 
                if len(x) > 2:
                    centrodx = x.sum()/len(x)
                    centrody = y.sum()/len(y)
                    print('movement at ',centrodx,centrody)

                    
                    theta = math.atan2(centrody, centrodx)
                    distance = math.sqrt(centrodx ** 2 + centrody ** 2) - .33
                    robot_turn(turn_client, theta) # turn the robot 90 degrees
                    wait_for_action(max_wheel_vel) # wait for it to stop 
                    # robot_move(move_client, distance)
                    # wait_for_action(max_wheel_vel) # wait for it to stop 
                    robot_status.reset()

                else:
                    print('no movement detected')

        elif use_camera == False:
            
            while robot_status.has_human:
                distance69, angle69 = headless(laser_reader,alphas,fov = 45)
                
                
                if .70> distance69> 0.4:
                    while  .70> distance69> 0.4:
                        # move and keep track 
                        print( 'right spot my bob')

                        distance69, angle69 = headless(laser_reader,alphas,fov = 30)
                        turn_pub.timer_callback(angle69/abs(angle69)) # this sends it the sign of the direction to turn
                        move_pub.timer_callback()
                        time.sleep(0.05)

                        
                        
                    
                if distance69 >= .70:
                    robot_status.has_human = False 
                    print( 'too far ')

                    
                if distance69 <= 0.4:
                    print( 'too close')
                    robot_status.tics = robot_status.tics + 1
                    if robot_status.tics > 10:
                        robot_status.reset()
                        robot_status.has_human = False 

                    
                

            
            
            else:
                    #Settings
                    alpha_df = pd.DataFrame()
                    rho_df = pd.DataFrame()
                    angleinc = laser_reader.scan.angle_increment

                    alphas = angleinc * np.arange(0,1080) - math.pi/2

                    for i in range(scans_per_set):

                        rclpy.spin_once(laser_reader,timeout_sec=1)
                        alpha_df[i] = alphas
                        rho_df[i] = laser_reader.scan.ranges


                    for j in range(move_check):
                                    
                        difference = rho_df[i-j-delay].astype(float) - rho_df[i].astype(float)
                        indxi = np.array(list(difference[abs(difference.astype(float)) <= 0.1].index))  
                        indxii = np.array(list(difference[abs(difference.astype(float)) >= 12.0].index))
                        if j == 0:
                            indx = np.concatenate((indxi,indxii))
                        else:
                            indx = np.concatenate((indx,indxi,indxii))


                    cartesian_df = pd.DataFrame(pol2cart(rho_df[i].astype(float), alpha_df[i].astype(float)), ['X', 'Y'])
                    # print(cartesian_df.shape)
                    indxiii = np.array(list(rho_df[i][abs(rho_df[i].astype(float)) >= 12.0].index))
                    indx = np.concatenate((indx,indxiii))

                    row_now = rho_df[i] 
                    alpha_now = alpha_df[i]
                    row_now.drop(indx)
                    alpha_now.drop(indx)
                    cartesian_df = cartesian_df.drop(indx, axis=1).T
                    # plt.scatter(cartesian_df['X'], cartesian_df['Y'], s=5,color =colors[i])
                    x = np.array(list(cartesian_df['X'].astype(float)))
                    y = np.array(list(cartesian_df['Y'].astype(float)))
                    

                    # movement detected 
                    if len(x) > 2:
                        centrodx = x.sum()/len(x)
                        centrody = y.sum()/len(y)
                        print('movement at ',centrodx,centrody)

                        
                        theta = math.atan2(centrody, centrodx)
                        distance = math.sqrt(centrodx ** 2 + centrody ** 2) - .33
                        robot_turn(turn_client, theta) # turn the robot 90 degrees
                        wait_for_action(max_wheel_vel) # wait for it to stop 
                        robot_move(move_client, distance)
                        wait_for_action(max_wheel_vel) # wait for it to stop 
                        robot_status.reset()
                        robot_status.has_human = True

                    else:
                        print('no movement detected')
        
                
        
    camera.destroy_node()
    move_client.destroy_node()
    turn_client.destroy_node()
    move_pub.destroy_node()
    max_wheel_vel.destroy_node()
        


if __name__ == '__main__':
    for i in range(10):
        print("Electric Boogaloo")
    main()



#IAN FRENCH TOAST BOT 
#ROBS BANNANA PASTE BOT
#CONNERS IMAGE CAPTURE (NOBOT)
#john for moral support