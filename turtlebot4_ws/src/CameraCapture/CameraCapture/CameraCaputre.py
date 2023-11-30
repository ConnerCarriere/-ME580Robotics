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
import pickle

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
        print('listener callback')
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


def main():
    rclpy.init()
    camera = CamCapture()

    for i in range(40):
        time.sleep(2)
        print(i)
        rclpy.spin_once(camera)
        cam_data = camera.capture_save_image()

        with open(f'/home/ubuntu/turtlebot4_ws/src/CameraCapture/CameraCapture/Pickle/nolegs{i}.pickle', 'wb') as handle:
            pickle.dump(cam_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
        

    camera.destroy_node()
