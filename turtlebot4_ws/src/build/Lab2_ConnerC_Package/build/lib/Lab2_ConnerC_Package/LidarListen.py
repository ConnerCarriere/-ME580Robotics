import math
import sensor_msgs.msg
import time
import rclpy
import csv

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
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import WheelVels
from irobot_create_msgs.msg import DockStatus 
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy


class UndockActionClient(Node):
    '''
    A simple example of an action client that will cause the iRobot Create3
    to dock if it is currently undocked. Subclass of Node.
    '''

    def __init__(self):
        super().__init__('undock_action_client')
        self._action_client = ActionClient(self, Undock, 'undock')

    def send_goal(self):
        goal_msg = Undock.Goal()
        print(goal_msg)

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)


class MoveClient(Node):

    def __init__(self):
        super().__init__('turn_client')
        self._action_client = ActionClient(self, DriveDistance, '/drive_distance')
    
    def send_goal(self, dist, max_speed = 0.5):
        
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = dist
        goal_msg.max_translation_speed = max_speed

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)


class ReadingLaser(Node):
    def __init__(self):
        super().__init__('reading_laser')

        self.subscription = self.create_subscription(sensor_msgs.msg.LaserScan,
                'scan',
                self.listener_callback,
                qos_profile=10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard "%f"' %msg.ranges[100])


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
        self.max_vel = max( abs(msg.velocity_left), abs(msg.velocity_right))


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
        when =  msg.header.stamp
        when.sec -= 1
        try:
            self.T = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        #msg.header.stamp,
                        rclpy.time.Time(),
                        timeout = rclpy.duration.Duration(seconds=1.0))
                        
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

'''
Listen to the dock status
'''
class IsDocked(Node):

    def __init__(self):
        super().__init__('dock_status')

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 liveliness=LivelinessPolicy.AUTOMATIC,
                                 durability=DurabilityPolicy.VOLATILE,
                                 depth = 1)

        self.subscription = self.create_subscription(DockStatus, '/dock_status', 
                                                     self.listener_callback, qos_profile)
        self.dock_status = DockStatus()
        self.dock_status.is_docked = False

    def listener_callback(self, msg):
        self.dock_status = msg
        self.get_logger().info('Dock status is {msg.is_docked}')

def wait_for_action(max_wheel_vel):

    #Wait for the motion to start
    while max_wheel_vel.max_vel < 0.1:
         rclpy.spin_once(max_wheel_vel, timeout_sec = 1.0)


    num_zero = 3
    while 1:
        rclpy.spin_once(max_wheel_vel,timeout_sec = 1.0)
        #Update the num_zero
        if max_wheel_vel.max_vel < 0.1:
            num_zero -=1
        else:
            num_zero = 3
        #if we got three zeros in a row, stop the loop
        if num_zero < 1:
            break

        # time.sleep(0.1)


def main():
    rclpy.init()
    
    dock_status = IsDocked()
    undock = UndockActionClient()
    max_wheel_vel = MaxWheelVel()
    reading_laser_pose =  ReadingLaserPose()
    move_client = MoveClient()

    #Make sure we have dock_status
    for ii in range(1, 5):
        rclpy.spin_once(dock_status)

    #If the robot is docked, we need to undock it to continue
    if dock_status.dock_status.is_docked:
        print('Undocking the robot')

        #Undock the robot
        future = undock.send_goal()
        rclpy.spin_until_future_complete(undock, future)
        #move from the dock
        wait_for_action(max_wheel_vel)
        #Turn arround
        wait_for_action(max_wheel_vel)

    #Open a data file
    with open('/Lab2_ConnerC_Package/lidar_dataCC.csv', 'w', newline='') as file:
        writer = csv.writer(file)

        #Get a reading from the laser and use it to write the header to the file
        reading_laser_pose.clear_reading()
        #Keep reading the sensor until we get a new one
        while not(reading_laser_pose.got_reading):
            rclpy.spin_once(reading_laser_pose)
        reading_laser_pose.write_header(writer)
        
        #get a reading after we stop
        reading_laser_pose.clear_reading()

        #Keep reading the sensor until we get a new one
        while not(reading_laser_pose.got_reading):
            rclpy.spin_once(reading_laser_pose)

        reading_laser_pose.write_results(writer)

        #Take a step and restart the procss
        future = move_client.send_goal(0.2)
        rclpy.spin_until_future_complete(move_client, future)
        wait_for_action(max_wheel_vel)

        file.close()


    
    rclpy.shutdown()