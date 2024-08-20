###################
# ROS libs
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from cv_bridge import CvBridge
###################

###################
# ROS msg libs
from std_msgs.msg import String, Int32, Header, Float32MultiArray
from sensor_msgs.msg import Image
from orca_interfaces.msg import OrcaPose
###################

###################
# Other libs
import numpy as np
import cv2
from ultralytics import YOLO
import torch
###################




class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        self.object_publisher = self.create_publisher(String, '/detected_obj_col', 10)
        self.lane_publisher = self.create_publisher(Int32, 'yolo_lane', 10)
        #self.coordinate_publisher = self.create_publisher(Float32MultiArray, '/object_coordinate', 10)

        self.img_subscriber = self.create_subscription(Image, '/image_raw', self.img_callback, 10)
        self.pose_subscriber = self.create_subscription(OrcaPose, '/current_pose', self.pose_callback, 10)
        self.yolo_switch_subscriber = self.create_subscription(Header, '/yolo_switch', self.yolo_switch_callback, 10)
        
        self.bridge = CvBridge()
        self.model = YOLO('/home/nayoung/obj_detect_ws/src/object_detection_pkg/best.pt')  # Path to model
        
        # Target shape and color
        self.target_shape = 'circle'  # Modify as needed
        self.target_color = 'red'     # Modify as needed

        # Color range dictionary
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'red2': ([170, 100, 100], [180, 255, 255]),
            'orange': ([10, 150, 150], [20, 255, 255]),
            'yellow': ([25, 150, 70], [35, 255, 255]),
            'green': ([40, 100, 50], [90, 255, 255]),
            'blue': ([90, 50, 70], [128, 255, 255]),
            'black': ([0, 0, 0], [180, 255, 30])
        }
        
        # 카메라 매트릭스 및 왜곡 계수 설정 (예시 값)
        self.camera_matrix = np.array([
            [800, 0, 320],  # fx, cx
            [0, 800, 240],  # fy, cy
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))  # 왜곡 계수 (보정된 상태로 가정)

        self.segment = np.array([30,-5, 30,-15])  # 선분, 점1(x,y), 점2(x,y)

        self.x = None
        self.y = None
        self.yaw = None

        self.yolo_switch_time = None # when it's pressed, yolo runs for 1 sec



    #####################################################
    #####################################################
    # Callback

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.yaw

    def yolo_switch_callback(self, msg):
        self.yolo_switch_time = Time.from_msg(msg.stamp)
        
    def img_callback(self, msg):

        # if switch is NOT pressed
        if(self.yolo_switch_time==None):
            return
        
        # if 1 SECOND elapsed after switch pressed
        time_diff = self.get_clock().now() - self.yolo_switch_time
        if(time_diff > Duration(seconds=1)):
            return
        
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_and_publish(cv_image)
        #self.display_image(cv_image)

    #####################################################
    #####################################################
    



    #####################################################
    #####################################################
    # Image Detection

    def detect_color(self, image, bbox):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        for color_name, (lower, upper) in self.color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)

            x, y, w, h = bbox
            mask_roi = mask[y:y+h, x:x+w]
            color_detected = cv2.bitwise_and(mask_roi, mask_roi, mask=mask_roi)

            if cv2.countNonZero(color_detected) > 0:
                return color_name

        return "Unknown"

    def detect_and_publish(self, image):
        results = self.model(image)
        detected_objects = []
        detected_coords = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                xywh = box.xywh.type(torch.int)
                x, y, w, h = xywh[0]
                conf = box.conf[0]
                cls = box.cls[0]
                detected_shape = self.model.names[int(cls)]
                detected_color = self.detect_color(image, (x, y, w, h))

                object_info = f'{detected_shape} ({detected_color})'
                detected_objects.append(object_info)
                detected_coords.append([float(x), float(y), float(w), float(h)])

        # Publish only the target shape and color
        for idx, obj_info in enumerate(detected_objects):
            detected_shape, detected_color = obj_info.split(' (')
            detected_color = detected_color.rstrip(')')
            if detected_shape == self.target_shape and detected_color == self.target_color:
                

                
                #####################################################
                # Added.     Angle Calculation && Lane Decision

                distorted_point = np.array([detected_objects[idx,0], detected_objects[idx,1]], dtype=np.float32)
                undistorted_point = cv2.undistortPoints(distorted_point, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)

                angle = self.calculate_angle(undistorted_point[0,0])
                lane = self.check_intersection(self.x, self.y, self.yaw, angle, self.segment)

                lane_msg = Int32()
                lane_msg.data = lane
                self.lane_publisher.publish(lane_msg)
                
                #####################################################

                
    
    def display_image(self, image):
        cv2.imshow('YOLO Real-Time Detection with Color', image)
        cv2.waitKey(1)

    #####################################################
    #####################################################




                
    #####################################################
    #####################################################
    # 'Relative angle' of 'detected shape'

    def calculate_angle(self, image_point):
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        
        # 2D 좌표를 3D 방향 벡터로 변환
        x = (image_point[0] - cx) / fx
        y = (image_point[1] - cy) / fy
        direction_vector = np.array([x, y, 1.0])
        direction_vector /= np.linalg.norm(direction_vector)  # 정규화

        # 카메라의 정면 방향 (z축을 향하는 벡터)
        camera_direction = np.array([0, 0, 1.0])

        # 두 벡터 사이의 각도 계산 (라디안 단위)
        angle_rad = np.arccos(np.clip(np.dot(direction_vector, camera_direction), -1.0, 1.0))
        
        # 라디안을 도 단위로 변환
        angle_deg = np.degrees(angle_rad)
        
        return angle_deg
    
    #####################################################
    #####################################################
    


    #####################################################
    #####################################################
    # Lane Decision

    def check_intersection(self, x, y, yaw, line_angle, segment):
        # 시선 방향으로 직선 그리기
        angle = np.radians(yaw + line_angle)
        end_x, end_y = self.rotate_point(1000, 0, angle)
        line = (x, y, x + end_x, y + end_y)
        
        # 선분을 3등분
        x1, y1, x2, y2 = segment
        third_x, third_y = (x2 - x1) / 3, (y2 - y1) / 3
        
        segments = [
            (x1, y1, x1 + third_x, y1 + third_y),
            (x1 + third_x, y1 + third_y, x1 + 2*third_x, y1 + 2*third_y),
            (x1 + 2*third_x, y1 + 2*third_y, x2, y2)
        ]
        
        # 각 부분과의 교차점 검사
        for i, seg in enumerate(segments, 1):
            intersection = self.line_intersection(line, seg)
            if intersection:
                return i
        
        return -1
    

    def rotate_point(self, x, y, angle):
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        return x * cos_a - y * sin_a, x * sin_a + y * cos_a


    def line_intersection(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        
        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if den == 0:  # 평행한 경우
            return None
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
        if 0 <= t <= 1:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            return x, y
        return None
    
    #####################################################
    #####################################################
    
        
        


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
