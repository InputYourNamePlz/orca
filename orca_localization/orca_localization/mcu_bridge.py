import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import serial
import numpy as np
from scipy.signal import butter, lfilter
import threading
import time
import csv


a = 9.45
b = 7.8664
c = 10
d = 1.6
e = 6.31



class MCUBridge(Node):

    global a,b,c,d,e

    def __init__(self):
        super().__init__('mcu_bridge')
        self.serial_port = serial.Serial('/dev/ttyESP32', 115200, timeout=1)  # 시리얼포트는 tty 고정해서 쓰면 좋아용
        self.timer = self.create_timer(0.1, self.timer_callback)  # 100Hz, 필요에 따라 조정

        self.imu_subscriber = self.create_subscription(Imu, 'imu', self.imu_callback, 100)
        self.twist_subscriber = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)


        self.thread = threading.Thread(target=self.serial_read_thread)
        self.thread.daemon = True
        self.thread.start()

        self.imu_msg=None
        self.roll = 0.0
        self.pitch = 0.0
        self.x_angular_vel = 0.0
        self.y_angular_vel = 0.0
        self.z_angular_vel = 0.0

        self.desired_linear_vel = 0.0
        self.desired_angular_vel = None

        self.gimbal_desired_theta=0
        self.gimbal_integral=0


        self.alpha=[0.0,0.0,0.0,0.0]
        
        
        self.prev_error=0.0
        self.integral=0.0

        self.last_time=time.time()

        self.filename="/home/orinnx/kaboat.csv"
        self.file = open(self.filename, 'a', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)

        self.pwm=1500



    def serial_read_thread(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    self.pwm=int(line)
                    #self.get_logger().info(line)
                    

    def imu_callback(self, msg):
        #print(f'{(time.time()-self.last_time)*1000} ms')
        #self.last_time = time.time()        
        self.imu_msg=msg
        #print(msg.header.stamp)
        

    def twist_callback(self, msg):
        self.desired_linear_vel = msg.linear.x
        self.desired_angular_vel = msg.angular.z
        #self.get_logger().info(f'{self.desired_linear_vel},{self.desired_angular_vel}')

        
    def timer_callback(self):
        #print(f'{(time.time()-self.last_time)*1000} ms')
        #self.last_time = time.time()
        if(self.imu_msg==None): return
        elif(self.desired_angular_vel==None): return
        msg=self.imu_msg
        x,y,z,w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        self.roll = np.arctan2(2 * (w * x + y * z),1 - 2 * (x * x + y * y))
        self.pitch = np.rad2deg(np.arcsin(2 * (w * y - z * x)))
        self.x_angular_vel, self.y_angular_vel, self.z_angular_vel = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        
        #self.writer.writerow([int(self.pwm), round(float(self.pitch), 3)])
        #self.file.flush()  # 즉시 디스크에 쓰기

        #print(self.z_angular_vel)
        
        
        thruster, rudder = self.processTwist()
        servo1, servo2, servo3, servo4 = self.processPlatform()

        data = f',{servo1:.0f},{servo2:.0f},{servo3:.0f},{servo4:.0f},{thruster:.0f},{rudder:.0f},\n'
        #print(rudder)
        print(data)
        self.serial_port.write(data.encode())
        #self.get_logger().info(f'Data to MCU: {data}')
        #time.sleep(0.05)

    def processTwist(self):
        #thruster = 1500 + self.desired_linear_vel*300
        
        if (self.desired_linear_vel>0):thruster=1610
        else:thruster=1500
        #if(thruster>1500): thruster=1650
        #if(thruster<1450): thruster=1450
        #rudder = 90 - self.desired_angular_vel*500


        p_term = (self.desired_angular_vel)*300
        d_term = self.z_angular_vel*4.3        


        #print(self.desired_angular_vel)
        
        rudder = 90 - (p_term - d_term)

        if(self.desired_angular_vel>=0.4): rudder = 30
        elif(self.desired_angular_vel<=-0.4): rudder = 150

        #print(rudder)

        if self.desired_linear_vel==0 and self.desired_angular_vel==0:
            rudder=90
            thruster=1500
        #print(f'{p_term:.1f}, {d_term:.1f},{rudder:.1f}')
        #print(self.z_angular_vel)
        #print(self.z_angular_vel)
        
        #self.get_logger().info(f'{rudder:.1f}, {p_term*100:.1f}, {d_term:.1f}')
        
        #if(self.desired_angular_vel>0): rudder=30
        #elif(self.desired_angular_vel<0): rudder=150

        if(rudder>150): rudder=150
        elif(rudder<30): rudder=30
        
        #return 1500, rudder
        return thruster, rudder
    
    def processPlatform(self):
        # gimbal front-back inverse
        '''
        roll=-self.roll
        pitch=-self.pitch
        roll_angular_v=-self.x_angular_vel
        pitch_angular_v=-self.y_angular_vel

        p=np.rad2deg(pitch)*0.25
        i=0#self.gimbal_integral+ pitch*4/10
        d=pitch_angular_v/30
        '''


        x=self.pwm
        
        #self.gimbal_desired_theta+=(p+i+d)
        #self.gimbal_desired_theta = -self.pitch
        self.gimbal_desired_theta=(0.0001505*x**2 - 0.4446*x + 319.8)+7
        #self.gimbal_desired_theta += (p+i+d)
        #print("self.gimbal_desired_theta : ", self.gimbal_desired_theta)
        #print("p : ", p)
        if self.gimbal_desired_theta>22.44: self.gimbal_desired_theta=22.44
        if self.gimbal_desired_theta<-1.5: self.gimbal_desired_theta=-1.5
        servo_angle=self.gimbal_calculation(self.gimbal_desired_theta)
        
        #print(f'{servo_angle:.1f}')
        #print(f'{np.rad2deg(self.pitch):.1f}')
        #print(-self.pitch)
        print(f'{self.gimbal_desired_theta:.1f}')

        
        servo1=180-servo_angle
        servo2=servo_angle
        #print(f'{p:.1f}, {i:.1f}, {d:.1f}, {pitch_angular_v:.1f}, ')
        if servo1>180:
            servo1=180
            servo2=0
        elif servo1<0:
            servo1=0
            servo2=180
        elif servo2>180:
            servo1=0
        elif servo2<0:
            servo2=0
            servo1=180
            servo2=180

        if np.isnan(servo_angle):
            servo1=90
            servo2=90
        
        #print(servo_angle)
        #print(type(servo_angle))

        return servo1,servo2,90,90
        

        #return self.alpha[0],self.alpha[1],self.alpha[2],self.alpha[3]
        
        

        #return 90, front_servo, 90, rear_servo
        
        return 90,90,90,90
    
    def gimbal_calculation(self, theta):

        theta=theta-7.12
        theta=np.deg2rad(theta)

        # servo-to-stabilizer joint-length squared
        alpha_squared = np.square(b*np.cos(theta)-e)+np.square(a+b*np.sin(theta))
        #print(np.sqrt(alpha_squared))

        up = alpha_squared+np.square(d)-np.square(c)
        down = 2*d*np.sqrt(alpha_squared)
        #print(up, down)
        

        right_triangle = np.arcsin((b*np.cos(theta)-e)/alpha_squared)

        servo_angle_radian = np.arccos(up/down)+right_triangle
        
        return np.rad2deg(servo_angle_radian)

    

    
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
