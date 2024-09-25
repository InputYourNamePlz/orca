###################
# ROS libs
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
#from cv_bridge import CvBridge
###################

###################
# ROS msg libs
from std_msgs.msg import String, Int32, Header, Float32MultiArray
#from sensor_msgs.msg import Image
from orca_interfaces.msg import OrcaPose
###################

###################
# Other libs
import numpy as np
import cv2
from ultralytics import YOLO
import torch
import math
import threading
import time
###################




class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        self.object_publisher = self.create_publisher(String, '/detected_obj_col', 10)
        self.lane_publisher = self.create_publisher(Int32, 'yolo_lane', 10)
        #self.coordinate_publisher = self.create_publisher(Float32MultiArray, '/object_coordinate', 10)

        #self.img_subscriber = self.create_subscription(Image, '/image_raw', self.img_callback, 10)
        self.pose_subscriber = self.create_subscription(OrcaPose, '/current_pose', self.pose_callback, 10)
        self.yolo_switch_subscriber = self.create_subscription(Header, '/yolo_switch', self.yolo_switch_callback, 10)
        
        #self.bridge = CvBridge()
        self.model = YOLO('/home/orinnx/Desktop/yolo_models/best.pt')  # Path to model
        
        # Target shape and color
        self.target_shape = 'circle'  # Modify as needed
        self.target_color = 'red2' #'red' or 'red2'     # Modify as needed

        # Color range dictionary
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'red2': ([170, 100, 100], [180, 255, 255]),
            'orange': ([11, 170, 150], [24, 255, 255]),  
            'yellow': ([25, 100, 100], [40, 255, 255]),  
            'green': ([41, 100, 50], [90, 255, 255]),   
            'blue': ([90, 50, 70], [128, 255, 255]),
            'black': ([0, 0, 0], [180, 255, 30])
            }

        
        # 카메라 매트릭스 및 왜곡 계수 설정 
        self.camera_matrix = np.array([
            [1419.566, 0, 1017.386],  # fx, cx
            [0, 1425.386, 551.884],  # fy, cy
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))  # 왜곡 계수 (보정된 상태로 가정)


        self.coordinate_x = 1.5
        self.coordinate_y1 = 1.5 # 오른쪽 끝
        self.coordinate_y2 = -1.5 # 왼쪽 끝

        self.x = None
        self.y = None
        self.yaw = None

        self.yolo_switch_time = None # when it's pressed, yolo runs for 1 sec

        self.cap = cv2.VideoCapture('/dev/video0')
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.thread = threading.Thread(target=self.cam_read_thread)
        self.thread.daemon = True
        self.thread.start()

        self.frame = None
        self.isFrameGot = False



    #####################################################
    #####################################################
    # Callback

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.yaw

    def yolo_switch_callback(self, msg):       
        if self.yaw == None: return
        if self.isFrameGot == False: return
        
        

            
        # 프레임을 그레이스케일로 변환
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # 가우시안 블러 적용하여 노이즈 제거
        blurred = cv2.GaussianBlur(gray, (21, 21), 0)

        # Lambertian 모델을 이용해 조명 성분 추정
        illumination = cv2.divide(gray, blurred, scale=255)

        # 조명 성분을 컬러 프레임에 맞게 3채널로 확장
        illumination_color = cv2.merge([illumination] * 3)

        # 조명 성분을 사용해 보정된 이미지 생성
        corrected_frame = cv2.multiply(self.frame.astype(np.float32), illumination_color.astype(np.float32) / 255.0)
        corrected_frame = np.clip(corrected_frame, 0, 255).astype(np.uint8)

        
        #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_and_publish(corrected_frame)
        self.display_image(corrected_frame)

    #####################################################
    #####################################################
    
    def cam_read_thread(self):
        while rclpy.ok():
            ret, self.frame = self.cap.read()
            if ret==True: self.isFrameGot=True
            time.sleep(0.01)



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
            print(idx, obj_info)
            detected_shape, detected_color = obj_info.split(' (')
            detected_color = detected_color.rstrip(')')
            if detected_shape == self.target_shape and detected_color == self.target_color:
                

                #####################################################
                # Added.     Angle Calculation && Lane Decision
                print("debug///////////////////////")
                distorted_point = np.array([detected_coords[idx][0], detected_coords[idx][1]], dtype=np.float32)
                undistorted_point = cv2.undistortPoints(distorted_point, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
                angle = self.calculate_angle(undistorted_point[0][0][0],undistorted_point[0][0][1])
                print(angle)
                
                # 레인 안 나오면 안 내보내기
                
                lane = self.lane_decision(angle)
                lane_msg = Int32()
                lane_msg.data = lane
                self.lane_publisher.publish(lane_msg)
                # 실험용
                print (f'lane<<<<<<<<<<<<<<<<<<<<<<<<<<{lane}')
                
                



                #####################################################

                
    
    def display_image(self, image):
        cv2.imshow('YOLO Real-Time Detection with Color', image)
        cv2.waitKey(1)

    #####################################################
    #####################################################




                
    #####################################################
    #####################################################
    # 'Relative angle' of 'detected shape'

    def calculate_angle(self, x, y, width=640, height=360, h_fov=78, v_fov=39.7):
        # 이미지 중심점 좌표
        cx = width / 2
        cy = height / 2
        
        # 카메라 화각의 절반 값
        h_fov_rad = math.radians(h_fov / 2)
        v_fov_rad = math.radians(v_fov / 2)
        
        # 화면의 픽셀 단위 각도
        h_angle_per_pixel = h_fov / width
        v_angle_per_pixel = v_fov / height
        
        # 점과 중심점 사이의 픽셀 거리
        dx = x - cx
        dy = cy - y
        
        # 수평 각도 (화면 중심을 기준으로)
        h_angle = dx * h_angle_per_pixel
        
        # 수직 각도 (화면 중심을 기준으로)
        v_angle = dy * v_angle_per_pixel
        
        # 각도를 구하기 위해 삼각법 사용
        angle = math.sqrt(h_angle**2 + v_angle**2)

        if dx < 0:
            return -angle
        else:
            return angle
    
    #####################################################
    #####################################################
    


    #####################################################
    #####################################################
    # Lane Decision

    def lane_decision(self, angle) :
        # 그림판의 위치
        dx = self.x - self.coordinate_x 
        k = (self.coordinate_y1 - self.coordinate_y2)/3

        intersect_length = dx * math.tan(np.deg2rad(angle - self.yaw)) + self.y

        print(intersect_length, self.x, k)

        if self.coordinate_y1 - k < intersect_length < self.coordinate_y1 : 
            return 1
        elif self.coordinate_y2 + k < intersect_length < self.coordinate_y1 - k : 
            return 2
        elif self.coordinate_y2 < intersect_length < self.coordinate_y2 + k : 
            return 3
        else : 
            return -1
    
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
