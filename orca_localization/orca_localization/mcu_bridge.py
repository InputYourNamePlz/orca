import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import serial
import numpy as np
from scipy.signal import butter, lfilter


class MCUBridge(Node):

    def __init__(self):
        super().__init__('mcu_bridge')
        self.serial_port = serial.Serial('/dev/ttyESP32', 115200, timeout=1)  # 시리얼포트는 tty 고정해서 쓰면 좋아용
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz, 필요에 따라 조정

        self.imu_subscriber = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.twist_subscriber = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)


        self.roll = None
        self.pitch = None
        self.x_angular_vel = None
        self.y_angular_vel = None

        self.desired_linear_vel = None
        self.desired_angular_vel = None

        self.filter_order = 2
        self.cutoff_freq = 5.0  # Hz
        self.sample_rate = 100.0  # Hz
        self.b, self.a = butter(self.filter_order, self.cutoff_freq / (0.5 * self.sample_rate), btype='low')

        self.alpha = 0.98
        self.integral = np.array([0.0,0.0])
        


    def imu_callback(self, msg):
        x,y,z,w = msg.orientation
        self.roll = np.arctan2(2 * (w * x + y * z),1 - 2 * (x * x + y * y))
        self.pitch = np.arcsin(2 * (w * y - z * x))
        self.x_angular_vel, self.y_angular_vel = msg.angular_velocity

    def twist_callback(self, msg):
        self.desired_linear_vel = msg.linear.x
        self.desired_angular_vel = msg.angular.z

        
    def timer_callback(self):
        
        thruster, rudder = self.processTwist()
        servo1, servo2, servo3, servo4 = self.processPlatform()

        data = f',{servo1},{servo2},{servo3},{servo4},{thruster},{rudder},'

        self.serial_port.write(f"{data}".encode())
        self.get_logger().info(f'Data to MCU: {data}')

    def processTwist(self):
        thruster = 1500 + self.desired_linear_vel*400
        rudder = 90 - self.desired_angular_vel*90
        return thruster, rudder
    
    def processPlatform(self):
        servo1, servo2, servo3, servo4 = 0,0,0,0
        
        return servo1,servo2,servo3,servo4


    

    
    def __del__(self):
        self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    mcu_bridge = MCUBridge()
    rclpy.spin(mcu_bridge)
    mcu_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
