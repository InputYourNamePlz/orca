##########################################################
# 이 노드는, 어떤 환경에서 동작하든지 (GPS/SLAM/SIMULATION)
# 위치 정보를 x좌표 값, y좌표 값, yaw 값 으로 변환해서
# 다루기 쉽게 만들어주는 노드입니다! :D
# 새로운 데이터를 받아들여야 할 때, 이 노드만 수정하면 됩니당
##########################################################


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import math
import numpy as np


##########################################################
from orca_interfaces.msg import OrcaPose
# orca_interfaces.msg , OrcaPose => 2차원 Pose 데이터. 내부는
#
# x(float32)
# y(float32)
# yaw(float32)
#
##########################################################


##########################################################
# 아래에는 Subscribe 할 msg들을 추가하여 사용.


from geometry_msgs.msg import PointStamped 
# PointStamped 메시지 내부
#
# header => NEVERMIND
# point => x, y, z

from sensor_msgs.msg import Imu
# Imu 메시지 내부
#
# header => NEVERMIND
# orientation => x, y, z, w (quaternion)
# angular_velocity => x, y, z (vector3)
# linear_acceleration => x, y, z(vector3)


##########################################################


##########################################################
# tf 표시 
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
##########################################################






class PosePublisher(Node):

    x=0.0
    y=0.0
    yaw=0.0

    def __init__(self):
        super().__init__('pose_publisher')
        #qos_profile = QoSProfile(depth=10)
        #qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        #qos_profile.durability = QoSDurabilityPolicy.VOLATILE


        ##########################################################
        # orca_pose로 publish
        self.publisher = self.create_publisher(OrcaPose, 'current_pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        ##########################################################
        self.tf_publihser = self.create_publisher(TFMessage, 'tf', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        ##########################################################


        ##########################################################
        # Webots / GPS & IMU

        # webots 에서 gps 정보 subscribe
        self.webots_gps_subscriber = self.create_subscription(
            PointStamped,
            '/TurtleBot3Burger/gps',
            self.webots_gps_callback,
            10
        )
        # webots 에서 imu 정보 subscribe
        self.webots_imu_subscriber = self.create_subscription(
            Imu,
            'imu',
            self.webots_imu_callback,
            10
        )
        ##########################################################



    ##########################################################
    # Publisher Callback
    def timer_callback(self):
        orca_pose = OrcaPose()
        orca_pose.x = self.x
        orca_pose.y = self.y
        orca_pose.yaw = self.yaw
        self.publisher.publish(orca_pose)
        self.get_logger().info('x : %f , y : %f , yaw : %f' % (self.x , self.y, self.yaw) )



        odom_to_base_link_tf = TransformStamped()
        odom_to_base_link_tf.header.frame_id = 'odom'
        odom_to_base_link_tf.child_frame_id = 'base_link'
        odom_to_base_link_tf.header.stamp = self.get_clock().now().to_msg()
        odom_to_base_link_tf.transform.translation.x=self.x
        odom_to_base_link_tf.transform.translation.y=self.y
        odom_to_base_link_tf.transform.translation.z=0.0
        odom_to_base_link_tf.transform.rotation.x=0.0
        odom_to_base_link_tf.transform.rotation.y=0.0
        odom_to_base_link_tf.transform.rotation.z=np.sin(self.yaw/2.0)
        odom_to_base_link_tf.transform.rotation.w=np.cos(self.yaw/2.0)

        base_link_to_lidar_tf = TransformStamped()
        base_link_to_lidar_tf.header.frame_id = 'base_link'
        base_link_to_lidar_tf.child_frame_id = 'LDS-01'
        base_link_to_lidar_tf.header.stamp = self.get_clock().now().to_msg()
        base_link_to_lidar_tf.transform.translation.x=0.0
        base_link_to_lidar_tf.transform.translation.y=0.0
        base_link_to_lidar_tf.transform.translation.z=0.3
        base_link_to_lidar_tf.transform.rotation.x=0.0
        base_link_to_lidar_tf.transform.rotation.y=0.0
        base_link_to_lidar_tf.transform.rotation.z=0.0
        base_link_to_lidar_tf.transform.rotation.w=1.0

        #tf_msg = TFMessage()
        #tf_msg.transforms+=[odom_to_base_link_tf,base_link_to_lidar_tf]
        #self.tf_publihser.publish(tf_msg)
        self.tf_broadcaster.sendTransform(odom_to_base_link_tf)
        self.tf_broadcaster.sendTransform(base_link_to_lidar_tf)
    ##########################################################



    ##########################################################
    # Webots GPS/IMU Callback
    def webots_gps_callback(self, gps_data):
        self.x = gps_data.point.x
        self.y = gps_data.point.y

    def webots_imu_callback(self, imu_data):
        x = imu_data.orientation.x
        y = imu_data.orientation.y
        z = imu_data.orientation.z
        w = imu_data.orientation.w
        rotation_matrix = np.array([[1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
                                    [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
                                    [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]])
        self.yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    ##########################################################






def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
