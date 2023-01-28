import time
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped 
from std_msgs.msg import Bool, Int16MultiArray, Header, Float32 
from sensor_msgs.msg import CompressedImage

from devices.bno085 import INERTIAL
from devices.motors import ESC
from devices.tof import VL53
from devices.camera import Video

'''
External connection node
- Passes the FC messages to the motors and proveds i2c sensors information
Subscribes:
- drone/CMD | Int16MultiArray - [fl, fr, bl, br]
- drone/ARM | Bool

Publishes:
- drone/Quaternion | QuaternionStamped
- drone/Height | Float32
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
        self.pub_quat = self.create_publisher(QuaternionStamped, 'drone/Quaternion', 10)
        timer_quat = 0.005  # seconds
        self.timer_quat = self.create_timer(timer_quat, self.quat_callback)

        self.pub_height = self.create_publisher(Float32, 'drone/Height', 10)
        timer_height = 0.05  # seconds
        self.timer_height = self.create_timer(timer_height, self.height_callback)

        self.pub_img0 = self.create_publisher(CompressedImage, 'drone/Img0', 10)
        timer_img0 = 0.05  # seconds
        self.timer_img0 = self.create_timer(timer_img0, self.img0_callback)

        timer_print= 0.1  # seconds
        self.timer_print = self.create_timer(timer_print, self.print_callback)

        self.mpu = INERTIAL()
        self.motors = ESC()
        self.tof = VL53()
        self.cam0 = Video()

    def print_callback(self):
        print(f'{self.motors.armed}')
        print(f'fl,fr:{self.motors.last_cmd[:2]} \nbl,br:{self.motors.last_cmd[2:]}')

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
        try:
            height = self.tof.read()
        except Exception as e:
            print(f'--------------{e}')
            time.sleep(2)
        msg = Float32()
        msg.data = height
        self.pub_height.publish(msg)

    def quat_callback(self):
        try:
            self.mpu.read()
        except Exception as e:
            print(f'--------------{e}')
            time.sleep(2)
        msg = QuaternionStamped()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        msg.quaternion.x =self.mpu.quat[0]
        msg.quaternion.y =self.mpu.quat[1]
        msg.quaternion.z =self.mpu.quat[2]
        msg.quaternion.w =self.mpu.quat[3]
        self.pub_quat.publish(msg)
        # print(self.mpu.euler)
    
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
