import time
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16MultiArray, Header 
from sensor_msgs.msg import CompressedImage

from devices.bno085 import INERTIAL
from devices.motors import ESC
from devices.tof import ZAXIS
from devices.camera import Video
from devices.gps import BN0

'''
External connection node
- Passes the FC messages to the motors and proveds i2c sensors information
Subscribes:
- drone/CMD | Int16MultiArray - [fl, fr, bl, br]
- drone/ARM | Bool

Publishes:
- drone/Odom | Odometry
- drone/Img0 | CompressedImage
'''


class DeviceNode(Node):
    def __init__(self):
        super().__init__('device_node')
        # Subscribers
        self.sub_cmd = self.create_subscription(Int16MultiArray,'drone/CMD',self.cmd_callback,10)
        self.sub_cmd  
        self.sub_arm = self.create_subscription(Bool,'drone/ARM',self.arm_callback,10)
        self.sub_arm  
        # Publishers
        # Publish Odom at a higher rate to complement IMU
        self.pub_odom = self.create_publisher(Odometry, 'drone/Odom', 10)
        timer_odom = 0.005  # seconds
        self.timer_odom = self.create_timer(timer_odom, self.odom_callback)

        timer_height = 0.05  # seconds
        self.timer_height = self.create_timer(timer_height, self.height_callback)

        self.pub_img0 = self.create_publisher(CompressedImage, 'drone/Img0', 10)
        timer_img0 = 0.05  # seconds
        self.timer_img0 = self.create_timer(timer_img0, self.img0_callback)

        timer_print= 0.1  # seconds
        self.timer_print = self.create_timer(timer_print, self.print_callback)
        
        timer_gps= 0.1  # seconds
        self.timer_gps = self.create_timer(timer_gps, self.gps_callback)

        self.mpu = INERTIAL()
        self.motors = ESC()
        self.tof = ZAXIS()
        self.cam0 = Video()
        self.gps = BN0()
        self.gps_dt = time.time()

        self.pose = [0., 0., 0.]
        self.imu_dt = None

    def print_callback(self):
        # print(f'{self.motors.armed}')
        # print(f'fl,fr:{self.motors.last_cmd[:2]} \nbl,br:{self.motors.last_cmd[2:]}')
        pass

    def gps_callback(self):
        sts = self.gps.read()
        if sts:
            self.pose[0] = self.gps.local_pose[0]
            self.pose[1] = self.gps.local_pose[1]

    def img0_callback(self):
        self.cam0.read()
        image_np = self.cam0.processed
        msg = CompressedImage()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        msg.format = "jpeg"
        msg.data = np.array(cv.imencode('.jpg', image_np)[1]).tostring()
        self.pub_img0.publish(msg)

    def height_callback(self):
        self.tof.read()
        # self.pose[2] = self.tof.height if self.tof.status else self.pose[2]
        

    def odom_callback(self):
        print('loop_time: ', time.time() - self.gps_dt)
        self.gps_dt = time.time()
        try:
            self.mpu.read()
        except Exception as e:
            print(f'--------------{e}')
            time.sleep(2)

        if self.imu_dt is not None:
            dt = time.time() - self.imu_dt
            self.pose[2] += -self.mpu.linear_vel[2]
            print(self.pose[-1], self.mpu.linear_accel[2] * dt, dt)
        self.imu_dt = time.time()

        msg = Odometry()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        msg.pose.pose.position.x = self.pose[0]
        msg.pose.pose.position.y = self.pose[1]
        msg.pose.pose.position.z = self.pose[2]
        msg.pose.pose.orientation.x =self.mpu.quat[0]
        msg.pose.pose.orientation.y =self.mpu.quat[1]
        msg.pose.pose.orientation.z =self.mpu.quat[2]
        msg.pose.pose.orientation.w =self.mpu.quat[3]
        msg.twist.twist.linear.x =self.mpu.linear_vel[0]
        msg.twist.twist.linear.y =self.mpu.linear_vel[1]
        msg.twist.twist.linear.z =self.mpu.linear_vel[2]
        msg.twist.twist.angular.x =self.mpu.angular_vel[0]
        msg.twist.twist.angular.y =self.mpu.angular_vel[1]
        msg.twist.twist.angular.z =self.mpu.angular_vel[2]
        self.pub_odom.publish(msg)
    
    def cmd_callback(self, msg):
        cmd = msg.data
        # print('CMD: ', cmd)
        try:
            self.motors.drive(cmd)
        except Exception as e:
            print(f'--------------{e}')
            time.sleep(2)
    
    def arm_callback(self, msg):
        arm = msg.data
        if arm and not self.motors.armed:
            self.motors.arm()
        elif not arm and self.motors.armed:
            self.motors.disarm()
            
        # print(f'Motor: {arm} == {self.motors.armed}')




def main(args=None):
    rclpy.init(args=args)
    device_node = DeviceNode()
    rclpy.spin(device_node)
    device_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
