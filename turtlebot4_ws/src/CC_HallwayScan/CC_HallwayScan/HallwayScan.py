#code altered by conner carriere
import math
import nav_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import time
import rclpy
import csv
from pprint import pprint

from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from irobot_create_msgs.action import RotateAngle # added by conner carriere

from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.msg import WheelVels
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy

#To view rvis 'ros2 launch turtlebot4_viz view_model.launch.py'

class Create3_button(Node):

    def __init__(self):
        super().__init__('HallwayScan')
        self.button_pressed = False
        # Subscribe to the /interface_buttons topic
        self.interface_buttons_subscriber = self.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self.interface_buttons_callback,
            10)

    def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
        # Button 1 is pressed
        if create3_buttons_msg.button_2.is_pressed:
            self.get_logger().info('Button 2 Pressed!')
            self.button_1_function()

    # Perform a function when Button 1 is pressed
    def button_1_function(self):
        # Create a ROS 2 message
        self.button_pressed = True
    def reset(self):
        self.button_pressed = False
        
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
    
class MoveClient(Node):

    def __init__(self):
        super().__init__('turn_client')
        self._action_client = ActionClient(self, DriveDistance, '/drive_distance')

    def send_goal(self, dist, max_speed=0.75):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = dist
        goal_msg.max_translation_speed = max_speed

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

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(geometry_msgs.msg.Pose,
            'odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

class ReadingLaser(Node):
    def __init__(self):
        super().__init__('reading_laser')

        self.subscription = self.create_subscription(sensor_msgs.msg.LaserScan,
                'scan',
                self.listener_callback,
                qos_profile=10)

    def listener_callback(self, msg):
        self.get_logger().info('Got scan') 
        self.scan = msg

    def write_header(self, writer):
        writer.writerow([self.scan.angle_min, 
                         self.scan.angle_max,
                         self.scan.angle_increment,
                         self.scan.time_increment,
                         self.scan.scan_time,
                         self.scan.range_min,
                         self.scan.range_max])

    def write_results(self, writer):
        writer.writerow(["RANGES"])
        writer.writerow(self.scan.ranges)

class TfListener(Node):
    def __init__(self):
        super().__init__('reading_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.got_reading = False

    def clear_reading(self):
        self.got_reading = False


    def timer_callback(self):

        from_frame_rel = 'odom'
        to_frame_rel = 'base_link'
        
        try:
            self.T = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time(), #now
                        timeout = rclpy.duration.Duration(seconds=0.5))
                        
        except TransformException as ex:
            print('.')
            #self.get_logger().info(
            #        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        x = self.T.transform.translation.x
        self.get_logger().info(f'Got pose and scan with x being {x}') 
        self.got_reading = True


    def write_results(self, writer):
        writer.writerow(["Transform"])
        writer.writerow([self.T.transform.translation.x, 
                         self.T.transform.translation.y, 
                         self.T.transform.translation.z,
                         self.T.transform.rotation.x, 
                         self.T.transform.rotation.y, 
                         self.T.transform.rotation.z, 
                         self.T.transform.rotation.w])

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

def check_robot_stopped(max_wheel_vel):
    stopcount = 0
    
    future = (max_wheel_vel.max_vel == 0)

    while stopcount != 3: # this checks to see if there are 3 0 velocities,
        rclpy.spin_until_future_complete(max_wheel_vel, future) # spin until complete
        stopcount += 1

def robot_turn(turn_client, angle):
    # print(f"turning {angle} degrees")
    future = turn_client.send_goal(angle)
    rclpy.spin_until_future_complete(turn_client, future) # spin until complete

def robot_move(move_client, distance):
    # print(f"moving {distance} meters")
    'use this to move a certain distance in meters'
    future = move_client.send_goal(distance) # create future goal with required distance
    rclpy.spin_until_future_complete(move_client, future) # spin until complete
    
def robot_capture_lidar(reading_laser_pose, writer):
    print("Capturing Lidar Data")
    'call this between to capture data'
    # get a reading after we stop
    # Keep reading the sensor until we get a new one
    rclpy.spin_once(reading_laser_pose)
    reading_laser_pose.write_results(writer)
    # Take a step and restart the procss
    
def transfrom_capture(tf_listener, writer):
            #Seems the buffer holds past values so lets read a few
            for jj in range(1,10):
                tf_listener.clear_reading()
                while not tf_listener.got_reading:
                    rclpy.spin_once(tf_listener)

            #write the pose to the data file      
            tf_listener.write_results(writer)

def main():
    rclpy.init()
    
    max_wheel_vel = MaxWheelVel()
    move_client = MoveClient()
    tf_listener =  TfListener()
    laser_reader = ReadingLaser()
    move_client = MoveClient()
    turn_client = TurnClient()

    'This code is to scan a room with the lidar'
    'Save move a bit, save lidar data ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
    # code altered by conner carriere
    # Open a data file
    travel_distance = 0.10 # in meters

    with open('/home/ubuntu/turtlebot4_ws/src/CC_HallwayScan/CC_HallwayScan/ConnersScans/RISLPopmachines.csv', 'w', newline='') as file:
        writer = csv.writer(file)

        'create header'
        # Get a reading from the laser and use it to write the header to the file
        # Keep reading the sensor until we get a new one
        rclpy.spin_once(laser_reader)
        laser_reader.write_header(writer)
        # readingOdomData.write_header(writer)

        transfrom_capture(tf_listener, writer)
        robot_capture_lidar(laser_reader, writer)

        'main part of the code'
        for i in range(20): #move 3 meters forward
            
            wait_for_action(max_wheel_vel) # wait for it to stop 
            print('in loop ')

            # Move the robot x distance
            robot_move(move_client, travel_distance)
            wait_for_action(max_wheel_vel) # wait for it to stop 

            writer.writerow([f"Moved forward {travel_distance} distance, scanning"])
            print(f"Moved forward {travel_distance} distance, scanning")

            transfrom_capture(tf_listener, writer)
            robot_capture_lidar(laser_reader, writer)
            
        # robot_turn(turn_client, -1.57) # turn the robot 90 degrees
        
        # for i in range(20): #move 3 meters forward
            
        #     wait_for_action(max_wheel_vel) # wait for it to stop 
        #     print('ind loop ')

        #     # Move the robot x distance
        #     robot_move(move_client, travel_distance)
        #     wait_for_action(max_wheel_vel) # wait for it to stop 

        #     writer.writerow([f"Moved forward {travel_distance} distance, scanning"])
        #     print(f"Moved forward {travel_distance} distance, scanning")

        #     transfrom_capture(tf_listener, writer)
        #     robot_capture_lidar(laser_reader, writer)
    

    print("Code is done")
    file.close()
    max_wheel_vel.destroy_node()
    move_client.destroy_node()
    
    laser_reader.destroy_node()

    rclpy.shutdown()
    #code altered by conner carriere
