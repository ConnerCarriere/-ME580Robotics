#code altered by conner carriere
import math
import sensor_msgs.msg
import time
import rclpy
import csv
from pprint import pprint

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle # added by conner carriere
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import WheelVels
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy

'''
class ReadingLaser(Node):
    def __init__(self):
        super().__init__('reading_laser')

        self.subscription = self.create_subscription(sensor_msgs.msg.LaserScan,
                                                     'scan',
                                                     self.listener_callback,
                                                     qos_profile=10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard "%f"' % msg.ranges[100])
'''

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
        # try:
        #     self.T = self.tf_buffer.lookup_transform(
        #         to_frame_rel,
        #         from_frame_rel,
        #         # msg.header.stamp,
        #         rclpy.time.Time(),
        #         timeout=rclpy.duration.Duration(seconds=1.0))

        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        #     return
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

    reading_laser_pose = ReadingLaserPose()
 
    # code altered by conner carriere
    # Open a data file
    with open('/home/ubuntu/turtlebot4_ws/src/CC_LidarCollect/CC_LidarCollect/CCLidar_data.csv', 'w', newline='') as file:
        writer = csv.writer(file)

        'create header'
        # Get a reading from the laser and use it to write the header to the file
        reading_laser_pose.clear_reading() # clear the old
        # Keep reading the sensor until we get a new one
        while not (reading_laser_pose.got_reading):
            rclpy.spin_once(reading_laser_pose)
        reading_laser_pose.write_header(writer)

        'Capture one rotation of lidar data'
        for i in range(0, 100):
            print(i)
            writer.writerow([f"Collecting scan {i}"])
            robot_capture_lidar(reading_laser_pose, writer)
            time.sleep(0.1)

    file.close()

    rclpy.shutdown()
    #code altered by conner carriere