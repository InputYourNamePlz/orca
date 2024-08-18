

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import math
import numpy as np


##########################################################
from orca_interfaces.msg import OrcaPose
##########################################################
from geometry_msgs.msg import Twist
##########################################################


##########################################################
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
##########################################################


class MotionController(Node):

    follower_realtime_x:0.0
    follower_realtime_y:0.0

    current_x:0.0
    current_y:0.0
    current_yaw:0.0

    yaw_difference:0.0
    counter=0


    def __init__(self):
        super().__init__('motion_controller')

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.follower_point_subscriber = self.create_subscription(
            OrcaPose,
            '/follower',
            self.follower_point_callback,
            10
        )
        self.current_pose_subscriber = self.create_subscription(
            OrcaPose,
            '/current_pose',
            self.current_pose_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)






    def follower_point_callback(self, follower_point_msg):
        self.follower_realtime_x = follower_point_msg.x
        self.follower_realtime_y = follower_point_msg.y




    def current_pose_callback(self, current_pose_msg):
        self.current_x=current_pose_msg.x
        self.current_y=current_pose_msg.y
        self.current_yaw=current_pose_msg.yaw




    def timer_callback(self):
        if (self.counter!=0):
            self.counter+=1
            if (self.counter==1):
                self.counter=0


        '''

        follower_yaw = math.atan2(self.follower_realtime_y-self.current_y, self.follower_realtime_x-self.current_x)
        self.yaw_difference = follower_yaw - self.current_yaw

        if self.yaw_difference>np.pi:
            self.yaw_difference-=2*np.pi            
        elif self.yaw_difference<-np.pi:
            self.yaw_difference+=2*np.pi    
        '''


        twist_msg = Twist()
        twist_msg.linear.x = self.follower_realtime_x/2.0
        twist_msg.angular.z = self.follower_realtime_y/2.0
        #twist_msg.linear.x=np.sqrt( (self.follower_realtime_x-self.current_x)**2 + (self.follower_realtime_y-self.current_y)**2  ) / 2.5
        #twist_msg.angular.z= -(self.yaw_difference)/1.0

        self.get_logger().info('linear : %.2f, angular : %.2f'%(twist_msg.linear.x,twist_msg.angular.z))
        #print('linear : %.2f, angular : %.2f'%(twist_msg.linear.x,twist_msg.angular.z))
        #print(follower_yaw, self.current_yaw)
        #print(self.follower_realtime_y-self.current_y, self.follower_realtime_x-self.current_x)
        
        #print(self.yaw_difference)
        self.twist_publisher.publish(twist_msg)
        self.counter+=1










def main(args=None):
    rclpy.init(args=args)
    motion_controller = MotionController()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
