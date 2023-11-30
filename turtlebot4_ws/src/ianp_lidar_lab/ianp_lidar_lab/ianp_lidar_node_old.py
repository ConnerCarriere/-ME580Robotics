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
from irobot_create_msgs.action import RotateAngle

from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.msg import WheelVels
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist

# Button class for using create 3 node 
# class Create3_button(Node):

#     def __init__(self):
#         super().__init__('HallwayScan')
#         self.button_pressed = False
#         # Subscribe to the /interface_buttons topic
#         self.interface_buttons_subscriber = self.create_subscription(
#             InterfaceButtons,
#             '/interface_buttons',
#             self.interface_buttons_callback,
#             10)

#     def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
#         # Button 1 is pressed
#         if create3_buttons_msg.button_2.is_pressed:
#             self.get_logger().info('Button 2 Pressed!')
#             self.button_1_function()

#     # Perform a function when Button 1 is pressed
#     def button_1_function(self):
#         # Create a ROS 2 message
#         self.button_pressed = True
#     def reset(self):
#         self.button_pressed = False

class MovePublisher(Node):
    def __init__(self):
        super().__init__('move_publisher')
        self.publisher_ = self.create_publisher( Twist, '/cmd_vel',10)
        timer_period = 0.5
        #self.timer = self.create_timer(timer_period, self.timer_callback)




    def timer_callback(self):

        msg = Twist()
        msg.linear.x = 0.75
        msg.angular.z = 0.0

        self.publisher_.publish(msg)

class TurnClient(Node):

    def __init__(self):
        super().__init__('turn_client2')
        self._action_client = ActionClient(self, RotateAngle, '/rotate_angle')
    
    def send_goal(self, angle, max_rotation_speed = 1.0):
        
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

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

class ReadingLaser_single(Node):

    def __init__(self):
        super().__init__('reading_laser')
        self.got_reading = False
        self.subscription= self.create_subscription(
            sensor_msgs.msg.LaserScan,
            'scan',
            self.listener_callback,
            # qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
            #                      liveliness=LivelinessPolicy.AUTOMATIC,
            #                      durability=DurabilityPolicy.VOLATILE,
            #                      depth = 10)
            qos_profile = 10
        )
    def clear_reading(self):
        self.got_reading = False

    def listener_callback(self, msg):
        self.get_logger().info('I heard : Range[0] "%f" angle_min: "%f" angle_max: "%f" range_min: "%f" range_max: "%f" scan_time: "%f" timeincrement: "%f" angle increment: "%f"' %(msg.ranges[0],msg.angle_min,msg.angle_max, msg.range_min,msg.range_max,msg.scan_time,msg.time_increment,msg.angle_increment))
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
        
        writer.writerow(["RANGES"])
        writer.writerow(self.scan.ranges[0])
        pass
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
        # writer.writerow([self.T.transform.translation.x,
        #                  self.T.transform.translation.y,
        #                  self.T.transform.translation.z,
        #                  self.T.transform.rotation.x,
        #                  self.T.transform.rotation.y,
        #                  self.T.transform.rotation.z,
        #                  self.T.transform.rotation.w])
        # writer.writerow(["RANGES"])
        writer.writerow([self.scan.ranges[0]])
        pass


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

def robot_move(move_client, distance):
    # print(f"moving {distance} meters")
    'use this to move a certain distance in meters'
    future = move_client.send_goal(distance) # create future goal with required distance
    rclpy.spin_until_future_complete(move_client, future) # spin until complete

def robot_turn(turn_client, angle):
    print(f"turning {angle} degrees")
    future = turn_client.send_goal(angle)
    rclpy.spin_until_future_complete(turn_client, future) # spin until complete
    
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
    
def check_robot_stopped(max_wheel_vel):
    stopcount = 0
    
    future = (max_wheel_vel.max_vel == 0)

    while stopcount != 3: # this checks to see if there are 3 0 velocities,
        rclpy.spin_until_future_complete(max_wheel_vel, future) # spin until complete
        stopcount += 1

def wait_for_robot_to_stop(max_wheel_vel):

    num_zero = 3
    while 1:
        rclpy.spin_once(max_wheel_vel,timeout_sec = 1)
        #Update the num_zero
        if abs(max_wheel_vel.max_vel) < 0.1:
            num_zero -=1
        else:
            num_zero = 3
        #if we got three zeros in a row, stop the loop
        if num_zero < 1:
            break

        time.sleep(0.5)
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
    
    turn_client = TurnClient()
    max_wheel_vel = MaxWheelVel()
    move_client = MoveClient()
    reading_laser_pose = ReadingLaserPose()

    

    
    with open('/home/ubuntu/turtlebot4_ws/src/ianp_lidar_lab/ianp_lidar_lab/objects_trash .csv', 'w', newline='') as file:
        writer = csv.writer(file)
        # reading_laser_pose = ReadingLaser_single() 
        #for the circle collect
        number_of_turns = 8
        angle = 2*3.14/number_of_turns
        travel_distance = 0.2 # in meters

        while not (reading_laser_pose.got_reading):
            rclpy.spin_once(reading_laser_pose)
            print('stufk')
        reading_laser_pose.write_header(writer)
        print('before thing')

        for k in range (3):
            print('you have 20 secponds')
            time.sleep(20)
            for i in range(100):

                                
                reading_laser_pose.get_logger().info("Hello friend!")
                # rclpy.spin(reading_laser_pose)
                robot_capture_lidar(reading_laser_pose, writer)
                time.sleep(.1)
                print('spun')
        # print(range(math.floor(2*3.14/angle)))
        # for i in range(math.floor(2*3.14/angle - 1)):
        #     print(i)
        #     robot_move(move_client,travel_distance)
        #     wait_for_robot_to_stop(max_wheel_vel) # wait for it to stop 
        #     time.sleep(0.1)
        #     # turn_client.send_goal(angle*i)
        #     robot_turn(turn_client,angle)
        #     wait_for_robot_to_stop(max_wheel_vel) # wait for it to stop 
        #     time.sleep(0.1)
        #     writer.writerow([f"Collecting scan {i}"])
        #     robot_capture_lidar(reading_laser_pose, writer)
        #     print('2')


        # turn_angle = -(5 * (math.pi / 180))
        # for i in range(0,10-1):
        #     writer.writerow([f"Move forward data collection {i}"])
        #     robot_move(move_client, 0.04)
        #     wait_for_robot_to_stop(max_wheel_vel) # wait for it to stop 

        #     robot_capture_lidar(reading_laser_pose, writer)

        # for i in range(0, int(90/5)-1):
        #     writer.writerow([f"rotate data collection {i}"])
        #     print("put turn angle thing here")
        #     robot_turn(turn_client, turn_angle)
        #     wait_for_robot_to_stop(max_wheel_vel) # wait for it to stop 

        #     robot_capture_lidar(reading_laser_pose, writer)

        # for i in range(0,10-1):
        #     writer.writerow([f"Move after 90 data collection {i}"])
        #     robot_move(move_client, 0.04)
        #     wait_for_robot_to_stop(max_wheel_vel) # wait for it to stop 

        #     robot_capture_lidar(reading_laser_pose, writer)
            
        


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turn_client.destroy_node()
    max_wheel_vel.destroy_node()
    move_client.destroy_node()
    
    reading_laser_pose.destroy_node()
    rclpy.shutdown()
    
    
    
    
    

if __name__ == '__main__':
    main()
