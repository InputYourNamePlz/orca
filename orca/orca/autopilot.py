##########################################################
# AUTOPILOT NODE 다. 졸립다..ㅠㅠ
##########################################################

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

#import math
import numpy as np
#import matplotlib.pyplot as plt
#import quaternion
import time
from scipy.spatial import cKDTree


from orca_interfaces.msg import OrcaPose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class Autopilot(Node):

    lidar_ranges=np.array([])
    lidar_angle_min=0.0
    lidar_angle_max=0.0
    lidar_angle_increment=0.0
    points_xy=np.array([])

    waypoint_x=0.0
    waypoint_y=0.0
    current_x=0.0
    current_y=0.0
    current_yaw=0.0
    arrival_radius=0.0

    waypoint_relative_x=0.0
    waypoint_relative_y=0.0

    jump_distance = 0.4
    tree_node_count = 10
    spreading_angle = 0.628
    max_iteration = 1000
    obstacle_avoidance_radius = 0.25
    
    turn_panelty_k = 3.5 #2.5일때 ㄱㅊ았음
    heading_to_wp_panelty_k = 1.00

    tree=np.empty([])
    route_node_count=0



    def __init__(self):
        super().__init__('autopilot')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/projected_scan',
            self.lidar_callback,
            qos_profile
            )
        
        self.waypoint_pose_subscription = self.create_subscription(
            OrcaPose,
            '/waypoint_pose',
            self.waypoint_pose_callback,
            10
            )
        
        self.current_pose_subscription = self.create_subscription(
            OrcaPose,
            '/current_pose',
            self.current_pose_callback,
            10
            )
        
        self.arrival_radius_subscription = self.create_subscription(
            Float32,
            '/arrival_radius',
            self.arrival_radius_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.marker_publisher = self.create_publisher(Marker, 'route_marker', 10)
        self.marker2_publisher = self.create_publisher(Marker, 'destination_straight_marker', 10)
        self.marker3_publisher = self.create_publisher(Marker, 'follower_marker', 10)
        self.follower_publisher = self.create_publisher(OrcaPose, 'follower', 10)
        




    ##########################################################
    
    def waypoint_pose_callback(self, waypoint_pose_msg):
        self.waypoint_x=waypoint_pose_msg.x
        self.waypoint_y=waypoint_pose_msg.y

    def current_pose_callback(self, current_pose_msg):
        self.current_x=current_pose_msg.x
        self.current_y=current_pose_msg.y
        self.current_yaw=current_pose_msg.yaw

    def arrival_radius_callback(self, arrival_radius_msg):
        self.arrival_radius = arrival_radius_msg.data-0.05

    def lidar_callback(self, lidar_msg):
        #self.get_logger().info("Lidar Data")
        self.lidar_ranges = np.array(lidar_msg.ranges)
        self.lidar_angle_max = lidar_msg.angle_max
        self.lidar_angle_min = lidar_msg.angle_min
        self.lidar_angle_increment = lidar_msg.angle_increment

    ##########################################################



    def timer_callback(self):

        start = time.time()

        # 벡터 연산을 통해 빠르게 각 Point에 대한 Radian 값 저장
        if (len(self.lidar_ranges)==0):
            self.get_logger().info("No Lidar Data")
            return
    

        #if (self.lidar_angle_increment<0):
        #    points_radian=self.lidar_angle_max - self.lidar_angle_increment * np.arange(len(self.lidar_ranges))
        #else:
        #    points_radian=self.lidar_angle_min + self.lidar_angle_increment * np.arange(len(self.lidar_ranges))
        points_radian=self.lidar_angle_min + self.lidar_angle_increment * np.arange(len(self.lidar_ranges))
        
        
        # 0보다 크고 obstacle_radius보다 작은 점들만 선택
        mask = (self.lidar_ranges > 0) & (self.lidar_ranges < 40)
        filtered_distances = self.lidar_ranges[mask]
        filtered_angles = points_radian[mask]
        
        # 극좌표를 직교좌표로 변환
        x = filtered_distances * np.cos(filtered_angles)
        y = filtered_distances * np.sin(filtered_angles)

        self.points_xy=np.column_stack((x,y))

        self.kdtree = cKDTree(self.points_xy)




        # Waypoint의 상대적 위치 계산
        self.waypoint_relative_x, self.waypoint_relative_y = self.relative_coord_calc()


        self.tree = np.zeros((self.max_iteration,2))
        self.tree[0:2,]=[[0,0],[self.jump_distance/2,0]]

        self.tree_search()
        route_tree=np.array(self.tree[0:self.route_node_count+1])
        #print(self.route_node_count)

        #self.get_logger().info(f'{route_tree}')


        if(len(route_tree)>5):
            follower=np.array(route_tree[4])
        elif(len(route_tree)<=2):
            follower=np.array([0.0,0.0])
        else:
            follower=np.array(route_tree[-1])

        print(follower)


        if (len(self.tree)!=0):
            #print(len(available_tree[2]))
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.02  # 선의 두께

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            if(len(route_tree)>4): route_tree=np.vstack(([0,0],route_tree[4:]))

            for x, y in route_tree:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0  # 2D 좌표평면에 표시하기 때문에 z는 0
                marker.points.append(point)
            self.marker_publisher.publish(marker)

            marker.scale.x = 0.01
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            p1, p2=Point(),Point()
            p1.x, p1.y, p1.z = 0.0,0.0,0.0
            p2.x, p2.y, p2.z = route_tree[-1,0], route_tree[-1,1],0.0
            marker.points=[p1,p2]
            self.marker2_publisher.publish(marker)

            marker.type = Marker.SPHERE
            marker.scale.x, marker.scale.y, marker.scale.z = 0.2,0.2,0.2
            marker.color.r, marker.color.g, marker.color.b = 0.0,1.0,0.0
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = follower[0], follower[1], 0.0 
            self.marker3_publisher.publish(marker)

            follower_pose=OrcaPose()
            #follower_pose.x = self.current_x + follower[0] * np.cos(self.current_yaw) - follower[1] * np.sin(self.current_yaw)
            #follower_pose.y = self.current_y + follower[0] * np.sin(self.current_yaw) - follower[1] * np.cos(self.current_yaw)
            follower_pose.x, follower_pose.y = follower[0],follower[1]
            follower_pose.yaw = 0.0
            self.follower_publisher.publish(follower_pose)

        else:
            #print("Can't find route")
            self.get_logger().info("Can't find route")




        end=time.time()


        #print('x : %.3f , y : %.3f, lidarLength : %d, time : %.1fms' % (self.waypoint_relative_x, self.waypoint_relative_y, len(self.lidar_ranges), (end-start)*1000.0 ) )
        #print('x : %.3f , y : %.3f, node count : %d, time : %.1fms' % (self.waypoint_relative_x, self.waypoint_relative_y, len(route_tree),(end-start)*1000.0 ) )
        self.get_logger().info('x: %.3f, y: %.3f, node count: %d, time: %.1fms' % (self.waypoint_relative_x, self.waypoint_relative_y, len(route_tree),(end-start)*1000.0 ) )
        #time.sleep(5)


        #quat = quaternion.from_euler_angles(0,0,np.arctan2(self.waypoint_relative_y, self.waypoint_relative_x))







    def next_node_create(self, prev_node, current_node):
        next_node_array=np.full( (self.tree_node_count,2),[current_node[0],current_node[1]] )

        if (self.tree_node_count%2==1): # 노드 홀수개 만들 때
            radians = self.spreading_angle * np.arange( -(self.tree_node_count-1)/2 , (self.tree_node_count-1)/2+1 )
        else: # 노드 짝수개 만들 때
            radians = (self.spreading_angle/2) * np.arange( -(self.tree_node_count-1), (self.tree_node_count-1)+2, 2)

        
        yaws = np.full( (self.tree_node_count), np.arctan2(current_node[1]-prev_node[1], current_node[0]-prev_node[0]) ) - radians



        next_node_array[:,0]= next_node_array[:,0] + self.jump_distance * np.cos(yaws)
        next_node_array[:,1]= next_node_array[:,1] + self.jump_distance * np.sin(yaws)

        return next_node_array


    def tree_search(self):
        index = 1
        while index<15:
            self.route_node_count=index

            # 마지막 노드가 원 안에 도착했는지 확인. 확인했으면 걍 둘려보냄
            if np.sqrt( (self.tree[index , 0]-self.waypoint_relative_x)**2 + (self.tree[index , 1]-self.waypoint_relative_y)**2 ) < self.arrival_radius:
                print(index)
                return
            if (index>self.max_iteration-2):
                return

            # 앞에 놓을 노드들의 후보를 받아옴
            next_nodes = self.next_node_create(self.tree[index-1],self.tree[index])
            # 후보들의 점수표
            
            next_nodes_score=np.zeros((self.tree_node_count))

            

            # 후보 오디션을 시ㅣㅣ이이이ㅣ자아ㅏ아아아악
            for idx, next_node in enumerate(next_nodes):

                # lidar 점이랑 가까운게 있으면 컽.
                #distances = np.sqrt( (self.points_xy[:,0] - next_node[0])**2 + (self.points_xy[:,1] - next_node[1])**2 )
                #points_count_within_radius = distances[distances<self.obstacle_avoidance_radius]
                distance, _ = self.kdtree.query(next_node, k=1)

                if (distance<=self.obstacle_avoidance_radius):
                    next_nodes_score[idx]=-np.inf
                    continue

                prev_vector = self.tree[index,] - self.tree[index-1,]
                current_vector = next_node - self.tree[index,]
                wp_vector = [self.waypoint_relative_x, self.waypoint_relative_y] - next_node

                heading_difference = np.arccos(  np.dot(prev_vector, current_vector) / (np.linalg.norm(prev_vector)*np.linalg.norm(current_vector))  )
                waypoint_heading_difference = np.arccos(  np.dot(current_vector,wp_vector) / (np.linalg.norm(wp_vector)*np.linalg.norm(current_vector))  )
                # 점수 써넣기
                next_nodes_score[idx] = -heading_difference * self.turn_panelty_k - waypoint_heading_difference * self.heading_to_wp_panelty_k

            #print(next_nodes_score)
            if ( np.any(next_nodes_score>(-10000)) != True ):
                print("no available next nodes")
                #self.tree_isAvailable[i] = False
                return
            optimal_next_node = next_nodes[np.argmax(next_nodes_score)]



            self.tree[index+1,]=optimal_next_node
            index+=1










    def relative_coord_calc(self):
        
        relative_x = self.waypoint_x - self.current_x
        relative_y = self.waypoint_y - self.current_y
        distance = np.sqrt(relative_x**2 + relative_y**2)
        waypoint_yaw = np.arctan2(relative_y, relative_x)
        relative_x_coord = distance * np.cos(waypoint_yaw - self.current_yaw)
        relative_y_coord = distance * np.sin(waypoint_yaw - self.current_yaw)
        return relative_x_coord, relative_y_coord












def main(args=None):
    rclpy.init(args=args)
    autopilot = Autopilot()
    rclpy.spin(autopilot)
    autopilot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()