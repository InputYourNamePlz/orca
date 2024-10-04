##########################################################
# 이 노드는, Waypoint에 도착했는지를 검사하고, 설정된 시간을 체크하여
# 알맞은 Waypoint의 정보를 Publish 해주는 노드입니다! :D
##########################################################

import rclpy
from rclpy.node import Node

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
import rclpy.node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32, Header, Int32
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
##########################################################



# Waypoint data를 여기에 넣으면 됩니다!
waypoint_dict = [
    {'order': 1, 'x' : 31.0, 'y' : 0.0, 'stay_time' : 3.0, 'yolo_switch_active': True, 'radius': 2.0},
    {'order': 2, 'x' : 31.0, 'y' : 5.0, 'stay_time' : 3.0, 'yolo_switch_active': False, 'radius': 2.0},
    {'order': 3, 'x' : 2.0, 'y' : 5.0, 'stay_time' : 3.0, 'yolo_switch_active': False, 'radius': 2.0},
    {'order': 4, 'x' : 2.0, 'y' : 0.0, 'stay_time' : 3.0, 'yolo_switch_active': False, 'radius': 2.0},
]

yolo_waypoint_dict = {
    1:[
        {'order': 2, 'x' : 0.0, 'y' : 0.0, 'stay_time' : 3.0, 'yolo_switch_active': False, 'radius': 2.0},
        {'order': 3, 'x' : 0.0, 'y' : 0.0, 'stay_time' : 3.0, 'yolo_switch_active': False, 'radius': 2.0},
    ],
    2:[
        {'order': 2, 'x' : 0.0, 'y' : 0.0, 'stay_time' : 3.0, 'yolo_switch_active': False, 'radius': 2.0},
        {'order': 3, 'x' : 0.0, 'y' : 0.0, 'stay_time' : 3.0, 'yolo_switch_active': False, 'radius': 2.0},
    ],
    3:[
        {'order': 2, 'x' : 0.0, 'y' : 0.0, 'stay_time' : 3.0, 'yolo_switch_active': False, 'radius': 2.0},
        {'order': 3, 'x' : 0.0, 'y' : 0.0, 'stay_time' : 3.0, 'yolo_switch_active': False, 'radius': 2.0},
    ],
}

# 얼마나 가까워져야 도착했다고 판단할 지 
arrival_check_radius = 1.2



class WaypointPublisher(Node):

    #global arrival_check_radius

    global waypoint_dict
    global yolo_waypoint_dict

    waypoint_data=None
    waypoint_counter=1
    current_stay_time=0.0

    current_x=0.0
    current_y=0.0
    current_yaw=0.0
    
    waypoint_x=0.0
    waypoint_y=0.0
    waypoint_stay_time=0.0
    waypoint_yolo_active=False
    arrival_check_radius=0.1
    

    
    def __init__(self):
        super().__init__('waypoint_publisher')

        self.publisher = self.create_publisher(OrcaPose, 'waypoint_pose', 10)
        self.arrival_radius_publisher = self.create_publisher(Float32, 'arrival_radius', 10)
        self.yolo_switch_publisher = self.create_publisher(Header, 'yolo_switch', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)


        #self.marker_timer = self.create_timer(1.0, self.marker_timer_callback)
        self.waypoint_marker_publisher = self.create_publisher(Marker, 'waypoint_marker', 10)
        self.waypoint_gauge_marker_publisher = self.create_publisher(Marker, 'waypoint_gauge_marker', 10)

        self.pose_subscriber = self.create_subscription(
            OrcaPose,
            'current_pose',
            self.pose_callback,
            10
        )
        self.yolo_lane_subscriber = self.create_subscription(
            Int32,
            'yolo_lane',
            self.yolo_lane_callback,
            10
        )

        # 현재 Waypoint 정보 꺼내오기
        self.get_waypoint_data()




    ##########################################################
    # 현재 위치 subscribe
    def pose_callback(self, pose):
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_yaw = pose.yaw
    ##########################################################
        

    ##########################################################
    def timer_callback(self):

        # 현재 Waypoint와의 거리 구하기
        distance = math.sqrt( (self.waypoint_x - self.current_x)**2 + (self.waypoint_y - self.current_y)**2 )
        
        # 거리가 check radius 보다 작으면 카운터 올리기
        if (self.arrival_check_radius > distance):
            self.current_stay_time+=0.1
        else:
            self.current_stay_time=0.0

        if (self.current_stay_time > self.waypoint_stay_time):
            if (self.waypoint_counter+1!=len(waypoint_dict)):
                self.waypoint_counter+=1
                self.current_stay_time=0.0

                self.get_waypoint_data()
                self.marker_refresh()


        waypoint_msg = OrcaPose()
        waypoint_msg.x = self.waypoint_x
        waypoint_msg.y = self.waypoint_y
        waypoint_msg.yaw = 0.0
        self.publisher.publish(waypoint_msg)


        if(self.yolo_switch_publisher):
            yolo_switch_msg = Header()
            self.yolo_switch_publisher.publish(yolo_switch_msg)


        arrival_check_radius_msg = Float32()
        arrival_check_radius_msg.data = self.arrival_check_radius
        self.arrival_radius_publisher.publish(arrival_check_radius_msg)

        self.get_logger().info('Waypoint : %d, time : %.1f, dist : %.2f' % (self.waypoint_counter, self.current_stay_time, distance))


        self.marker_refresh()



    ##########################################################


    def get_waypoint_data(self):
        #print(waypoint_dict[1]['order'])
        #self.waypoint_data = (wp for wp in waypoint_dict if wp['order'] == self.waypoint_counter)
        for wp in waypoint_dict:
            if wp['order']==self.waypoint_counter:
                self.waypoint_data=wp
        print(self.waypoint_data)
        self.waypoint_x = self.waypoint_data['x']
        self.waypoint_y = self.waypoint_data['y']
        self.waypoint_stay_time = self.waypoint_data['stay_time']
        self.waypoint_yolo_active = self.waypoint_data['yolo_switch_active']
        self.arrival_check_radius = self.waypoint_data['radius']
        self.get_logger().info(f'{self.arrival_check_radius}')
        
        
    ##########################################################

    def yolo_lane_callback(self, msg):
        lane = msg.data
        if lane not in yolo_waypoint_dict:
            self.get_logger().info(f"Error: {lane} is not a valid lane")
            return
        
        for yolo_waypoint in yolo_waypoint_dict[lane]:
            for i, waypoint in enumerate(waypoint_dict):
                if waypoint['order'] == yolo_waypoint['order']:
                    waypoint_dict[i] = yolo_waypoint
        self.get_logger().info(f"YOLO Lane: {lane}")
            
                    

    ##########################################################
        

    def marker_refresh(self):
        '''
        marker_msg=Marker()
        marker_msg.header.frame_id="odom"
        marker_msg.header.stamp=self.get_clock().now().to_msg()
        marker_msg.ns="waypoint"
        marker_msg.action = Marker.DELETE
        self.marker_publisher.publish(marker_msg)
        
        Waypoint_Markers = MarkerArray()


        marker_msg=Marker()
        marker_msg.header.frame_id="odom"
        marker_msg.header.stamp=self.get_clock().now().to_msg()
        marker_msg.ns="waypoint"
        marker_msg.type=Marker.SPHERE
        marker_msg.action=Marker.MODIFY
        marker_msg.pose.position.x=self.waypoint_x
        marker_msg.pose.position.y=self.waypoint_y
        marker_msg.pose.position.z=0.0
        marker_msg.scale.x=0.5
        marker_msg.scale.y=0.5
        marker_msg.scale.z=0.5
        marker_msg.color.r=1.0
        marker_msg.color.g=1.0
        marker_msg.color.b=0.0
        marker_msg.color.a=0.0
        self.waypoint_marker_publisher.publish(marker_msg)
        '''

        num_points=30
        radius=self.arrival_check_radius
        points=[]
        for i in range(num_points+1):
            angle = 2.0*math.pi/num_points*i
            x=self.waypoint_x+radius*math.cos(angle)
            y=self.waypoint_y+radius*math.sin(angle)
            points.append(Point(x=x,y=y,z=0.0))
        
        circle_marker_msg=Marker()
        circle_marker_msg.header.frame_id="map"
        circle_marker_msg.header.stamp=self.get_clock().now().to_msg()
        circle_marker_msg.ns="waypoint"
        circle_marker_msg.type=Marker.LINE_STRIP
        circle_marker_msg.action=Marker.MODIFY
        circle_marker_msg.points=points
        circle_marker_msg.scale.x=0.1
        circle_marker_msg.color.r=1.0
        circle_marker_msg.color.g=1.0
        circle_marker_msg.color.b=0.0
        circle_marker_msg.color.a=1.0
        self.waypoint_marker_publisher.publish(circle_marker_msg)


        gauge_num_points=int(num_points*(self.current_stay_time/self.waypoint_stay_time))
        radius=self.arrival_check_radius/2.0
        gauge_points=[]
        for i in range(gauge_num_points+0):
            angle = 2.0*math.pi/num_points*i
            x=self.waypoint_x+radius*math.cos(angle)
            y=self.waypoint_y+radius*math.sin(angle)
            gauge_points.append(Point(x=x,y=y,z=0.0))
        
        circle_gauge_marker_msg=Marker()
        circle_gauge_marker_msg.header.frame_id="map"
        circle_gauge_marker_msg.header.stamp=self.get_clock().now().to_msg()
        circle_gauge_marker_msg.ns="waypoint"
        circle_gauge_marker_msg.type=Marker.LINE_STRIP
        circle_gauge_marker_msg.action=Marker.MODIFY
        circle_gauge_marker_msg.points=gauge_points
        circle_gauge_marker_msg.scale.x=0.2
        circle_gauge_marker_msg.color.r=1.0
        circle_gauge_marker_msg.color.g=1.0
        circle_gauge_marker_msg.color.b=0.1
        circle_gauge_marker_msg.color.a=1.0
        self.waypoint_gauge_marker_publisher.publish(circle_gauge_marker_msg)

        #Waypoint_Markers.markers+=[marker_msg,circle_marker_msg]
        #self.waypoint_marker_publisher.publish(Waypoint_Markers)




    ##########################################################





def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
