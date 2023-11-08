#code altered by conner carriere
import math
import sensor_msgs.msg
import time
import rclpy
import csv
from pprint import pprint

from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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

class ReadingLaserPose(Node):
    def __init__(self):
        super().__init__('reading_laser')

        self.subscription = self.create_subscription(sensor_msgs.msg.LaserScan,
                                                     'scan',
                                                     self.listener_callback,
                                                     qos_profile=10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.got_reading = False

    def clear_reading(self):
        self.got_reading = False

    def listener_callback(self, msg):

        from_frame_rel = 'odom'
        to_frame_rel = 'base_link'
        when = msg.header.stamp
        when.sec -= 1
        try:
            self.T = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                # msg.header.stamp,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        self.get_logger().info('Got pose and scan')
        self.scan = msg
        self.got_reading = True

    def write_header(self, writer):
        writer.writerow([self.scan.angle_min,
                         self.scan.angle_max,
                         self.scan.angle_increment,
                         self.scan.time_increment,
                         self.scan.scan_time,
                         self.scan.range_min,
                         self.scan.range_max])

    def write_results(self, writer):
        writer.writerow([self.T.transform.translation.x,
                         self.T.transform.translation.y,
                         self.T.transform.translation.z,
                         self.T.transform.rotation.x,
                         self.T.transform.rotation.y,
                         self.T.transform.rotation.z,
                         self.T.transform.rotation.w])
        writer.writerow(self.scan.ranges)

def wait_for_action(max_wheel_vel):
    # Wait for the motion to start
    while max_wheel_vel.max_vel < 0.1:
        rclpy.spin_once(max_wheel_vel, timeout_sec=0.2)

        if max_wheel_vel.max_vel == 0:
            break

    num_zero = 3
    while 1:
        rclpy.spin_once(max_wheel_vel, timeout_sec=0.2)
        # Update the num_zero
        if max_wheel_vel.max_vel < 0.1:
            num_zero -= 1
        else:
            num_zero = 3
        # if we got three zeros in a row, stop the loop
        if num_zero < 1:
            break

        time.sleep(0.1)

def check_robot_stopped(max_wheel_vel):
    stopcount = 0
    
    future = (max_wheel_vel.max_vel == 0)

    while stopcount != 3: # this checks to see if there are 3 0 velocities,
        rclpy.spin_until_future_complete(max_wheel_vel, future) # spin until complete
        stopcount += 1

def robot_turn(turn_client, angle):
    print(f"turning {angle} degrees")
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
    reading_laser_pose.clear_reading()
    # Keep reading the sensor until we get a new one
    while not (reading_laser_pose.got_reading):
        rclpy.spin_once(reading_laser_pose)
    reading_laser_pose.write_results(writer)
    # Take a step and restart the procss

def main():
    rclpy.init()

    Button = Create3_button()
    max_wheel_vel = MaxWheelVel()
    reading_laser_pose = ReadingLaserPose()
    move_client = MoveClient()

    'This code is to scan a room with the lidar'
    'Save move a bit, save lidar data ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
    # code altered by conner carriere
    # Open a data file
    travel_distance = 0.5 # in meters

    with open('/home/ubuntu/turtlebot4_ws/src/CC_HallwayScan/CC_HallwayScan/data.csv', 'w', newline='') as file:
        writer = csv.writer(file)

        'create header'
        # Get a reading from the laser and use it to write the header to the file
        reading_laser_pose.clear_reading() # clear the old
        # Keep reading the sensor until we get a new one
        while not (reading_laser_pose.got_reading):
            rclpy.spin_once(reading_laser_pose)
        reading_laser_pose.write_header(writer)

        button_pressed = 0
        'main part of the code'
        while 1:
            # print(Button.button_pressed)
            # rclpy.spin_once(Button, timeout_sec=5)

            # # check if the button is pressed
            # if Button.button_pressed == True:
            #     print("Button Pressed, Stopping")
            #     button_pressed = 1
            #     break
                
            # Move the robot x distance
            robot_move(move_client, travel_distance)
            wait_for_action(max_wheel_vel) # wait for it to stop 
            # check_robot_stopped(max_wheel_vel) # wait for robot to stop
            writer.writerow([f"Moved forward {travel_distance} distance, scanning"])
            print(f"Moved forward {travel_distance} distance, scanning")

            # take a lidar scan
            robot_capture_lidar(reading_laser_pose, writer)
            print("\n")
            # repeat
   
    print("Code is done")
    file.close()

    rclpy.shutdown()
    #code altered by conner carriere
