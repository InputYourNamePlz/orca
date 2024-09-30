import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class LidarImuProcessor(Node):
    z_lower_threshold = -0.15  # 예: 10cm 이하의 점은 무시
    z_upper_threshold = 0.15  # 예: 10cm 이하의 점은 무시

    def __init__(self):
        super().__init__('lidar_imu_processor')
        
        qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
        )
        
        # 구독자 설정
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        
        # 발행자 설정
        self.original_scan_pub = self.create_publisher(LaserScan, 'original_scan', 10)
        self.projected_scan_pub = self.create_publisher(LaserScan, 'projected_scan', 10)
        
        # TF 브로드캐스터 설정
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.laser_data = None
        self.imu_data = None
        

    def laser_callback(self, msg):
        self.laser_data = msg
        self.process_data()

    def imu_callback(self, msg):
        # 쿼터니언을 오일러 각도로 변환
        r = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.imu_data = r.as_euler('xyz', degrees=True)[:2]  # roll과 pitch만 사용
        
    def process_data(self):
        if self.laser_data is None or self.imu_data is None:
            return

        # 원본 스캔 데이터 발행 (imu_link 프레임)
        original_scan = LaserScan()
        original_scan.header = self.laser_data.header
        original_scan.header.frame_id = "imu_link"
        original_scan.angle_min = self.laser_data.angle_min#-np.pi
        original_scan.angle_max = self.laser_data.angle_max#-np.pi
        original_scan.angle_increment = self.laser_data.angle_increment
        original_scan.time_increment = self.laser_data.time_increment
        original_scan.scan_time = self.laser_data.scan_time
        original_scan.range_min = self.laser_data.range_min
        original_scan.range_max = self.laser_data.range_max
        original_scan.ranges = self.laser_data.ranges
        original_scan.intensities = self.laser_data.intensities
        self.original_scan_pub.publish(original_scan)

        # LiDAR 데이터를 3D 점으로 변환
        angles = np.arange(self.laser_data.angle_min, self.laser_data.angle_min+self.laser_data.angle_increment*len(self.laser_data.ranges), self.laser_data.angle_increment)
        #angles = np.arange(self.laser_data.angle_min, self.laser_data.angle_max, self.laser_data.angle_increment)
        #angles = np.arange(self.laser_data.angle_min, self.laser_data.angle_max+self.laser_data.angle_increment, self.laser_data.angle_increment)

        points = np.array([np.cos(angles) * self.laser_data.ranges,
                           np.sin(angles) * self.laser_data.ranges,
                           np.zeros_like(angles)]).T

        # IMU 데이터를 이용해 점들을 회전
        r = R.from_euler('xy', self.imu_data, degrees=True)
        rotated_points = r.apply(points)

        # 투영된 스캔 데이터 생성 (base_link 프레임)
        projected_scan = LaserScan()
        projected_scan.header = self.laser_data.header
        projected_scan.header.frame_id = "base_link"
        projected_scan.angle_min = self.laser_data.angle_min#-np.pi
        projected_scan.angle_max = self.laser_data.angle_max#-np.pi
        projected_scan.angle_increment = self.laser_data.angle_increment
        projected_scan.time_increment = self.laser_data.time_increment
        projected_scan.scan_time = self.laser_data.scan_time
        projected_scan.range_min = self.laser_data.range_min
        projected_scan.range_max = self.laser_data.range_max
        
        '''
        # 투영된 점들의 거리 계산
        projected_ranges = np.sqrt(rotated_points[:, 0]**2 + rotated_points[:, 1]**2)
        projected_scan.ranges = projected_ranges.tolist()
        
        # 원본 intensities 유지 (있는 경우)
        if self.laser_data.intensities:
            projected_scan.intensities = self.laser_data.intensities
        '''
        
        valid_indices = (rotated_points[:, 2] >= self.z_lower_threshold) & (rotated_points[:, 2] <= self.z_upper_threshold)

        # 투영된 점들의 거리 계산 (Z축 필터링 적용)
        projected_ranges = np.sqrt(rotated_points[:, 0]**2 + rotated_points[:, 1]**2)
        filtered_ranges = np.where(valid_indices, projected_ranges, float('inf'))
        projected_scan.ranges = filtered_ranges.tolist()
        
        # 원본 intensities 유지 (있는 경우, Z축 필터링 적용)
        if self.laser_data.intensities:
            filtered_intensities = np.where(valid_indices, self.laser_data.intensities, 0)
            projected_scan.intensities = filtered_intensities.tolist()

        self.projected_scan_pub.publish(projected_scan)

        # imu_link에서 base_link로의 변환 발행
        self.broadcast_transform()

        print("PUBLISHING: original/projected scan, base-imu tf")

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        
        # IMU 데이터로부터 회전 설정
        r = R.from_euler('xy', self.imu_data, degrees=True)
        quat = r.as_quat()
        
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        # 여기서는 간단히 위치를 0으로 설정합니다. 실제로는 IMU와 LiDAR의 상대적 위치를 고려해야 할 수 있습니다.
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LidarImuProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
