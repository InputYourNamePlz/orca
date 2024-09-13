import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import serial
import numpy as np
from scipy.signal import butter, lfilter
import threading
import time


# 전역 변수 설정, 길이 단위 : cm
P = 2.5 #Plate Center - Moter dist
B = 3.5#Base Center - Moter dist
T = [0,0,10.5]
a = 1.8 #radius
#s = 9.7 #rod length
beta = [0,np.pi/2,np.pi,np.pi*3/2]



kp=0.7
ki=0.5
kd=0.50
dt=0.05



class MCUBridge(Node):

    global P, B, T, a, beta #, s
    global kp, ki, kd, dt

    def __init__(self):
        super().__init__('mcu_bridge')
        self.serial_port = serial.Serial('/dev/ttyESP32', 115200, timeout=1)  # 시리얼포트는 tty 고정해서 쓰면 좋아용
        self.timer = self.create_timer(0.1, self.timer_callback)  # 100Hz, 필요에 따라 조정

        self.imu_subscriber = self.create_subscription(Imu, 'imu', self.imu_callback, 100)
        self.twist_subscriber = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)


        #self.thread = threading.Thread(target=self.serial_read_thread)
        #self.thread.daemon = True
        #self.thread.start()

        self.imu_msg=None
        self.roll = 0.0
        self.pitch = 0.0
        self.x_angular_vel = 0.0
        self.y_angular_vel = 0.0
        self.z_angular_vel = 0.0

        self.desired_linear_vel = 0.0
        self.desired_angular_vel = None


        self.alpha=[0.0,0.0,0.0,0.0]
        
        
        self.prev_error=0.0
        self.integral=0.0

        self.last_time=time.time()



    def serial_read_thread(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    self.get_logger().info(line)


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
        self.pitch = np.arcsin(2 * (w * y - z * x))
        self.x_angular_vel, self.y_angular_vel, self.z_angular_vel = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
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
        
        if (self.desired_linear_vel>0):thruster=1580
        else:thruster=1500
        #if(thruster>1500): thruster=1650
        #if(thruster<1450): thruster=1450
        #rudder = 90 - self.desired_angular_vel*500


        p_term = (self.desired_angular_vel)*100
        d_term = self.z_angular_vel*1.5

        


        print(self.desired_angular_vel)
        
        rudder = 90 - (p_term - d_term)

        if(self.desired_angular_vel>=0.4): rudder = 30
        elif(self.desired_angular_vel<=-0.4): rudder = 150

        print(rudder)

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
        
        
        phi=-(self.roll)
        theta=(self.pitch)
        print(f'roll: {np.rad2deg(phi):.0f}, pitch: {np.rad2deg(theta):.0f}')
        R_roll = np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]])
        R_pitch = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
        R_B = R_roll@R_pitch

        for i in range(4):
            # 하드웨어상으로 p_i b_i 맞춰주기
            p_i = np.array([P*np.cos(np.pi/2*i),P*np.sin(np.pi/2*i),0])
            b_i = np.array([B*np.cos(np.pi/2*i),B*np.sin(np.pi/2*i),0])
            q_i = (T + p_i)@np.transpose(R_B)
            l_i = (T + p_i)@np.transpose(R_B)-b_i
            
            l = np.linalg.norm(l_i)
            s = (l*l-a*a)**(1/2)
            L=l*l-(s*s-a*a)
            M=2*a*(q_i[2]-b_i[2])
            N=2*a*(np.cos(beta[i])*(q_i[0]-b_i[0])+np.sin(beta[i])*(q_i[1]-b_i[1]))
            
            self.alpha[i] = 90+np.rad2deg(np.arcsin(L/((M**2+N**2)**(1/2))) - np.arctan(N/M))/3
        
        #self.get_logger().info(f'{self.alpha[0]:.0f} {self.alpha[1]:.0f} {self.alpha[2]:.0f} {self.alpha[3]:.0f}')
        #print(f'{self.roll:1f}, {self.pitch:.1f}')

        return self.alpha[0],self.alpha[1],self.alpha[2],self.alpha[3]
        
        

        #return 90, front_servo, 90, rear_servo
        
        return 90,90,90,90


    

    
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
