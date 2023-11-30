import sys
import rclpy
import random
import time
import math

from rclpy import *
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import *#QoSProfile
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import WheelVels 

from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from geometry_msgs.msg import Twist

class MovePublisher(Node):
    def __init__(self):
        super().__init__('move_publisher')
        self.publisher_ = self.create_publisher( Twist, '/cmd_vel',10)
        # timer_period = 0.5
        #self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.0

        self.publisher_.publish(msg)


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
        

'''
Listen for the a bumper event
'''
class Bump(Node):

    def __init__(self):
        super().__init__('bump')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 liveliness=LivelinessPolicy.AUTOMATIC,
                                 durability=DurabilityPolicy.VOLATILE,
                                 depth = 10)
        
        self.subscription = self.create_subscription(HazardDetectionVector, '/hazard_detection', self.listener_callback, qos_profile)
        self.bump_event = HazardDetectionVector()
        self.bump = 0 
        self.subscription

    def listener_callback(self, msg):
        for event in msg.detections:
            # print('listenercb')
            if event.type == 1:
                self.bump = 1
                print('bump')
                
    def reset(self):
        self.bump = 0


'''
Listen to the wheel velocity topic and record the maximum absolute velocity
'''
class MaxWheelVel(Node):

    def __init__(self):
        super().__init__('max_wheel_vel')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 liveliness=LivelinessPolicy.AUTOMATIC,
                                 durability=DurabilityPolicy.VOLATILE,
                                 depth = 1)
        self.subscription = self.create_subscription(WheelVels, '/wheel_vels', self.listener_callback, 10)
        self.subscription
        self.max_vel = -1

    def listener_callback(self, msg):
        self.max_vel = max( abs(msg.velocity_left), abs(msg.velocity_right))
        #self.get_logger().info('I got a wheel velocity message {0}'.format(self.max_vel))


'''
This function checks to see if the robot is stopped. There are random zeros
returned. This is likely a race condition. I just rigged this to fix the problem
'''
def wait_for_robot_to_stop(max_wheel_vel):

    num_zero = 3
    while 1:
        rclpy.spin_once(max_wheel_vel,timeout_sec = 1)
        #Update the num_zero
        if max_wheel_vel.max_vel < 0.1:
            num_zero -=1
        else:
            num_zero = 3
        #if we got three zeros in a row, stop the loop
        if num_zero < 1:
            break

        # time.sleep(0.1)

def main(args=None):
    
    rclpy.init(args=args)

    turn_client = TurnClient()
    move_pub = MovePublisher()
    max_wheel_vel = MaxWheelVel()
    bump = Bump()
    

    print("Waiting for startup")
    time.sleep(2)
    #future = turn_client.send_goal(1.57)
    #rclpy.spin_until_future_complete(turn_client, future)

    while 1:
        #Move forward until we hit someting
        while bump.bump == 0:
            rclpy.spin_once(bump, timeout_sec = .1)
            rclpy.spin_once(max_wheel_vel, timeout_sec = .1)
            move_pub.timer_callback()
            
        
        print('Bumped Something')
        
        bump.reset()

        #Let's sleep a little to let the reflex do its work
        # bump_gone = 5
        # while bump_gone > 0:
        #     print("shit")
        #     rclpy.spin_once(bump, timeout_sec = 8)
        #     if bump.bump == 0:
        #         bump_gone -=1
        #     else:
        #         bump_gone = 5
            # time.sleep(0.5)

        
        #tell everyone bump is done
        print('Bump Gone') 


        #Turn a random amount
        angle = (random.randrange(90,181)*(-1)**random.randint(0,1))/180.0*3.14
        print('The angle of rotation is %f.' % angle)

        print('Waiting for turn')
        #Wait for turn to start
        turn_client.send_goal(angle)


        print('waiting for turn to stop')
        wait_for_robot_to_stop(max_wheel_vel)

    print('Done')
        

    move_pub.destroy_node()
    turn_client.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
ros2 run turtlebot4_python_tutorials turtlebot4_wander_node
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true world:=maze
ros2 topic echo /wheel_vels
'''