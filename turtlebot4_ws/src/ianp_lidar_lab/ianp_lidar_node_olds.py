#code belongs to Ian Picklo or something idk im not a licensor
import math
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
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

# Button class for using create 3 node 
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
        self.max_vel = max(abs(msg.velocity_left), abs(msg.velocity_right))

# class ReadingLaser_alldata(Node):

#     def __init__(self):
#         super().__init__('reading_laser')

#         self.subscription= self.create_subscription(
#             sensor_msgs.msg.LaserScan,
#             'scan',
#             self.listener_callback,
#             # qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
#             #                      liveliness=LivelinessPolicy.AUTOMATIC,
#             #                      durability=DurabilityPolicy.VOLATILE,
#             #                      depth = 10)
#             qos_profile = 10
#         )


#     def listener_callback(self, msg):
#         self.get_logger().info('I heard : Range[0] "%f" angle_min: "%f" angle_max: "%f" range_min: "%f" range_max: "%f" scan_time: "%f" timeincrement: "%f" angle increment: "%f"' %(msg.ranges[0],msg.angle_min,msg.angle_max, msg.range_min,msg.range_max,msg.scan_time,msg.time_increment,msg.angle_increment))

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 liveliness=LivelinessPolicy.AUTOMATIC,
                                 durability=DurabilityPolicy.VOLATILE,
                                 depth = 10)
        self.subscription = self.create_subscription(nav_msgs.msg.Odometry,
                                                                    'odom',
                                                                    self.listener_callback,
                                                                    qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Pos_y: "%s"' % msg.pose.pose.position.y)
        self.get_logger().info('Pos_x: "%s"' % msg.pose.pose.position.x)

# class ReadingLaser_due_x(Node):

#     def __init__(self):
#         super().__init__('reading_laser')

#         self.subscription= self.create_subscription(
#             sensor_msgs.msg.LaserScan,
#             'scan',
#             self.listener_callback,
#             # qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
#             #                      liveliness=LivelinessPolicy.AUTOMATIC,
#             #                      durability=DurabilityPolicy.VOLATILE,
#             #                      depth = 10)
#             qos_profile = 10
#         )


#     def listener_callback(self, msg):
#         self.get_logger().info('I heard : Range[0] "%f" angle_min: "%f" angle_max: "%f" range_min: "%f" range_max: "%f" scan_time: "%f" timeincrement: "%f" angle increment: "%f"' %(msg.ranges[0],msg.angle_min,msg.angle_max, msg.range_min,msg.range_max,msg.scan_time,msg.time_increment,msg.angle_increment))
 

def main():
    print('Hi from ianp_lidar_lab.')
    rclpy.init()
    reading_laser = MinimalSubscriber()                  
    reading_laser.get_logger().info("Hello friend!")
    rclpy.spin(reading_laser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
