import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import rclpy.time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import numpy as np
from scipy.signal import correlate2d, correlate, find_peaks
from scipy.spatial import cKDTree
import time
from numba import jit, prange



class OrcaScanMatcher(Node):

    def __init__(self):
        super().__init__('correlative_scan_matcher')
        
        qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
        )
        
        # 구독자 및 발행자 설정
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/projected_scan', self.lidar_callback, qos_profile)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'curr_point_cloud', 10)
        self.pointcloud_publisher2 = self.create_publisher(PointCloud2, 'prev_point_cloud', 10)

        self.imu_data=None
        self.prev_scan=None
        self.prev_yaw=None

        self.initial_yaw=None
        
        self.odom_x=0.0
        self.odom_y=0.0

        self.is_lidar_data_imported=False
        self.is_imu_data_imported=False

        self.iter=0

        


    def imu_callback(self, msg):
        self.is_imu_data_imported=True
        self.imu_data = msg

        '''
        tf=TransformStamped()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        
        tf.header.stamp = rclpy.time.Time().to_msg()
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = msg.orientation.x
        tf.transform.rotation.y = msg.orientation.y
        tf.transform.rotation.z = msg.orientation.z
        tf.transform.rotation.w = msg.orientation.w
        

        self.tf_broadcaster.sendTransform(tf)
        '''


    def lidar_callback(self, msg):
        if(self.is_imu_data_imported==False):
            return
        elif(self.is_lidar_data_imported==False):
            self.is_lidar_data_imported=True
            self.prev_scan=self.lidar_to_xy(msg)
            self.prev_yaw=self.imu_to_yaw(msg, self.imu_data)
            self.initial_yaw=self.prev_yaw
            return
        
        curr_imu_data=self.imu_data
        curr_scan=self.lidar_to_xy(msg)
        curr_scan_origin=np.copy(curr_scan)
        curr_yaw=self.imu_to_yaw(msg, curr_imu_data)
        prev_yaw=self.prev_yaw
        prev_scan=self.prev_scan
        
        start_time=time.time()
        ###################################
        # rotating scan with IMU data

        yaw_diff = np.deg2rad(curr_yaw - self.prev_yaw) 
        curr_scan=self.rotate_points(curr_scan, yaw_diff)

        ###################################
        ###################################
        # rotating scan with Polar Scan Matching

        yaw_diff = -np.deg2rad(self.polar_scan_matching(curr_scan, prev_scan))
        curr_scan=self.rotate_points(curr_scan,yaw_diff)

        ###################################


        aligned, (delta_x, delta_y), iterations = self.point_to_line_icp(curr_scan, prev_scan)
        end_time=time.time()


        self.odom_x = self.odom_x + (delta_x*np.cos(np.deg2rad(prev_yaw)) - delta_y*np.sin(np.deg2rad(prev_yaw)))
        self.odom_y = self.odom_y + (delta_x*np.sin(np.deg2rad(prev_yaw)) + delta_y*np.sin(np.deg2rad(prev_yaw)))
        
        #if(np.sqrt(delta_x**2+delta_y**2)>0.05):
        #print(np.sqrt(delta_x**2+delta_y**2))
        print("--------------------------------------------")
        print(f"ICP converged after {iterations} iterations")
        print(f"delta: x={delta_x:.4f}, y={delta_y:.4f}")
        print(f"{(end_time-start_time)*1000} ms elapsed")
        print("--------------------------------------------")
        print(f"odom: {self.odom_x} , {self.odom_y}")
        print("--------------------------------------------")
        self.iter+=iterations
        print(self.iter)


        #print(curr_yaw, self.prev_yaw, yaw_diff)
        #print(aligned, distances, iterations)


        ###################################
        # curr_pc publish
        points_3d=np.hstack([curr_scan.astype(np.float32), np.zeros((curr_scan.shape[0], 1), dtype=np.float32)])

        pc_msg=PointCloud2()
        pc_msg.header.frame_id="laser_frame"
        
        pc_msg.height = 1
        pc_msg.width = curr_scan.shape[0]
        pc_msg.point_step = 12  # 4 bytes each for x, y, z
        pc_msg.row_step = pc_msg.point_step * curr_scan.shape[0]

        pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.is_dense=True
        pc_msg.data=points_3d.tobytes()
        self.pointcloud_publisher.publish(pc_msg)
        ###################################
        ###################################
        # prev_pc publish
        prev_points_3d=np.hstack([prev_scan.astype(np.float32), np.zeros((prev_scan.shape[0], 1), dtype=np.float32)])

        pc_msg=PointCloud2()
        pc_msg.header.frame_id="laser_frame"
        
        pc_msg.height = 1
        pc_msg.width = prev_scan.shape[0]
        pc_msg.point_step = 12  # 4 bytes each for x, y, z
        pc_msg.row_step = pc_msg.point_step * prev_scan.shape[0]

        pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.is_dense=True
        pc_msg.data=prev_points_3d.tobytes()
        self.pointcloud_publisher2.publish(pc_msg)
        ###################################
        ###################################
        # tf publish
        
        tf=TransformStamped()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        
        tf.header.stamp = msg.header.stamp
        tf.transform.translation.x = self.odom_x
        tf.transform.translation.y = self.odom_y
        tf.transform.translation.z = 0.0
        cy = np.cos(np.deg2rad(curr_yaw) * 0.5)
        sy = np.sin(np.deg2rad(curr_yaw) * 0.5)
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = -sy
        tf.transform.rotation.w = cy
        

        self.tf_broadcaster.sendTransform(tf)
        
        ###################################



        self.prev_scan=curr_scan_origin
        self.prev_yaw=curr_yaw
        


    def lidar_to_xy(self,msg):

        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        angles = angles[mask]
        ranges = ranges[mask]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        points = np.column_stack((x, y))

        return points

    def imu_to_yaw(self,lidar_msg,imu_msg):

        x=imu_msg.orientation.x
        y=imu_msg.orientation.y
        z=imu_msg.orientation.z
        w=imu_msg.orientation.w
        quat_yaw=np.rad2deg(np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))

        dt=-np.round((lidar_msg.header.stamp.sec+lidar_msg.header.stamp.nanosec*1e-9) - (imu_msg.header.stamp.sec+imu_msg.header.stamp.nanosec*1e-9),5)
        gyro_yaw_change = imu_msg.angular_velocity.z * (dt) # delay of lidar
        
        yaw=quat_yaw+gyro_yaw_change

        return yaw
    
    def rotate_points(self, scan, yaw_diff):

        rotation_matrix = np.array([
            [np.cos(yaw_diff), -np.sin(yaw_diff)],
            [np.sin(yaw_diff), np.cos(yaw_diff)]
        ])

        return np.dot(scan, rotation_matrix.T)
    








    ################################################################
    # Polar Scan Matching

    def cartesian_to_polar(self, scan):
        x=scan[:,0]
        y=scan[:,1]
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)

        return r, theta
        
    def polar_scan_matching(self, scan1, scan2, angle_resolution=1):

        # Cartesian to polar 변환
        r1, theta1 = self.cartesian_to_polar(scan1)
        r2, theta2 = self.cartesian_to_polar(scan2)
        
        # 각도를 도 단위로 변환
        theta1 = np.degrees(theta1)
        theta2 = np.degrees(theta2)
        
        # 각도 히스토그램 생성
        bins = np.arange(0, 360 + angle_resolution, angle_resolution)
        hist1, _ = np.histogram(theta1, bins=bins)
        hist2, _ = np.histogram(theta2, bins=bins)
        
        # 상관관계 계산
        correlation = correlate(hist1, hist2, mode='full')
        
        # 최대 상관관계를 갖는 인덱스 찾기
        max_corr_index = np.argmax(correlation)
        
        # 회전 각도 계산
        rotation_angle = (max_corr_index - (len(hist1) - 1)) * angle_resolution
        
        return rotation_angle % 360







    ################################################################
    # Point-to-Line ICP

    @staticmethod
    @jit(nopython=True, parallel=True)
    def point_to_line_distances(points, line_starts, line_ends):
        distances = np.empty(len(points))
        for i in prange(len(points)):
            point = points[i]
            line_start = line_starts[i]
            line_end = line_ends[i]
            line_vec = line_end - line_start
            point_vec = point - line_start
            line_len = np.linalg.norm(line_vec)
            line_unitvec = line_vec / line_len
            t = np.dot(line_unitvec, point_vec) / line_len
            t = max(0, min(1, t))
            nearest = line_start + t * line_vec
            distances[i] = np.linalg.norm(point - nearest)
        return distances

    def best_fit_transform(self, A, B):
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        t = centroid_B - np.dot(R, centroid_A)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        return R, t, yaw

    def point_to_line_icp(self, source, target, max_iterations=30, tolerance=0.0000001):
        prev_error = np.inf
        cumulative_R = np.eye(2)
        cumulative_t = np.zeros(2)
        cumulative_yaw = 0

        target_tree = cKDTree(target)

        for i in range(max_iterations):
            distances, indices = target_tree.query(source, k=2)
            
            line_starts = target[indices[:, 0]]
            line_ends = target[indices[:, 1]]
            
            line_distances = self.point_to_line_distances(source, line_starts, line_ends)
            
            R, t, yaw = self.best_fit_transform(source, target[indices[:, 0]])
            
            cumulative_R = np.dot(R, cumulative_R)
            cumulative_t = np.dot(R, cumulative_t) + t
            cumulative_yaw += yaw
            
            source = np.dot(source, R.T) + t
            
            mean_error = np.mean(line_distances)
            if np.abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error
        
        x_change, y_change = cumulative_t
        yaw_change = cumulative_yaw


        return source, (x_change, y_change), i

    ################################################################


        
    

def main(args=None):
    rclpy.init(args=args)
    orca_scan_matcher = OrcaScanMatcher()
    rclpy.spin(orca_scan_matcher)
    orca_scan_matcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()