import time
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16MultiArray, Header 
from sensor_msgs.msg import CompressedImage, NavSatFix

from devices.bno085 import INERTIAL
from devices.motors import ESC
from devices.z_axis import ZAXIS
from devices.camera import Video
from devices.gps import BN0
from devices.optical_flow import FLOW

'''
External connection node
- Passes the FC messages to the motors and proveds i2c sensors information
Subscribes:
- drone/CMD | Int16MultiArray - [fl, fr, bl, br]
- drone/ARM | Bool

Publishes:
- drone/odom | Odometry
- drone/img0 | CompressedImage
'''


class DeviceNode(Node):
    def __init__(self):
        super().__init__('device_node')
        self.gps_init = int(input("Use GPS? (0|1)") or 1)
        # Subscribers
        self.sub_cmd = self.create_subscription(Int16MultiArray,'drone/CMD',self.cmd_callback,10)
        self.sub_cmd  
        self.sub_arm = self.create_subscription(Bool,'drone/ARM',self.arm_callback,10)
        self.sub_arm  

        # Publishers
        # Publish Odom at a higher rate to complement IMU
        self.pub_odom = self.create_publisher(Odometry, 'drone/odom', 10)
        timer_odom = 0.005  # seconds
        self.timer_odom = self.create_timer(timer_odom, self.odom_callback)

        timer_height = 0.08  # seconds
        self.timer_height = self.create_timer(timer_height, self.height_callback)
        
        timer_flow = 0.01  # seconds
        self.timer_flow = self.create_timer(timer_flow, self.flow_callback)

        # self.pub_img0 = self.create_publisher(CompressedImage, 'drone/img0', 10)
        # timer_img0 = 0.05  # seconds
        # self.timer_img0 = self.create_timer(timer_img0, self.img0_callback)

        timer_print= 0.1  # seconds
        self.timer_print = self.create_timer(timer_print, self.print_callback)
        

        self.mpu = INERTIAL()
        self.motors = ESC()
        self.height = ZAXIS()
        # self.cam0 = Video()
        if self.gps_init:
            self.pub_gps = self.create_publisher(NavSatFix, 'drone/GPS', 10)
            timer_gps= 0.1  # seconds
            self.timer_gps = self.create_timer(timer_gps, self.gps_callback)
            self.gps_init = 1
            self.gps = BN0()
        
        self.of = FLOW()
        self.odom_dt = time.time()

        self.pose = [0., 0., 0.] # x, y, z
        self.quat = [0., 0., 0., 0.,] # x, y, z, w
        self.twist = [0., 0., 0., 0., 0., 0.] # linear: x, y, z, angular: x, y, z
        self.imu_dt = None

    def print_callback(self):
        # print(f'{self.motors.armed}')
        # print(f'fl,fr:{self.motors.last_cmd[:2]} \nbl,br:{self.motors.last_cmd[2:]}')
        print(f'Status: height:{self.height.status} | *mpu: {self.mpu.status} | gps: {self.gps.status if self.gps_init else "NULL"} | *of: {self.of.status}')
        pass

    def gps_callback(self):
        sts = self.gps.read()
        if sts:
            # self.pose[0] = self.gps.local_pose[0]
            # self.pose[1] = self.gps.local_pose[1]
            msg = NavSatFix()
            h = Header()
            h.stamp = self.get_clock().now().to_msg()
            msg.header = h
            msg.status.status = self.gps.sat_status
            msg.latitude = self.gps.raw_pose[0]
            msg.longitude = self.gps.raw_pose[1]
            msg.altitude = self.gps.raw_pose[2]
            self.pub_gps.publish(msg)



    # def img0_callback(self):
    #     print('img0_callback')
    #     self.cam0.read()
    #     image_np = self.cam0.processed
    #     msg = CompressedImage()
    #     h = Header()
    #     h.stamp = self.get_clock().now().to_msg()
    #     msg.header = h
    #     msg.format = "jpeg"
    #     msg.data = np.array(cv.imencode('.jpg', image_np)[1]).tostring()
    #     self.pub_img0.publish(msg)

    def height_callback(self):
        print('height_callback')
        self.height.read()
        self.pose[2] = self.height.height

    def flow_callback(self):
        self.of.read()
        self.pose[0:2] = self.of.pose
        self.twist[0:2] = self.of.twist
        print(self.pose, self.twist)
        

    def odom_callback(self):
        print('ODOM loop: ', time.time() - self.odom_dt)
        self.odom_dt = time.time()
        self.mpu.read()
        self.quat = self.mpu.quat
        self.twist[3:] = self.mpu.angular_vel


        msg = Odometry()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h

        msg.pose.pose.position.x = float(self.pose[0])
        msg.pose.pose.position.y = float(self.pose[1])
        msg.pose.pose.position.z = self.pose[2]
        msg.pose.pose.orientation.x = float(self.mpu.quat[0])
        msg.pose.pose.orientation.y = float(self.mpu.quat[1])
        msg.pose.pose.orientation.z = float(self.mpu.quat[2])
        msg.pose.pose.orientation.w = float(self.mpu.quat[3])
        msg.twist.twist.linear.x = float(self.twist[0])
        msg.twist.twist.linear.y = float(self.twist[1])
        msg.twist.twist.linear.z = float(self.twist[2])
        msg.twist.twist.angular.x = float(self.twist[3])
        msg.twist.twist.angular.y = float(self.twist[4])
        msg.twist.twist.angular.z = float(self.twist[5])
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
