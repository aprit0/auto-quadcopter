import time
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16MultiArray, Header
from sensor_msgs.msg import NavSatFix, Imu, Image

from devices.bno085 import INERTIAL
from devices.motors import ESC
from devices.z_axis import ZAXIS
from devices.gps import BN0
from devices.optical_flow import FLOW

'''
External connection node
- Passes the FC messages to the motors and proveds i2c sensors information
Subscribes:
- drone/CMD | Int16MultiArray - [fl, fr, bl, br]
- drone/ARM | Bool

Publishes:
- drone/imu_raw | Imu 
- drone/odom_raw | Odometry
- drone/img0 | CompressedImage
'''


class DeviceNode(Node):
    def __init__(self):
        super().__init__('device_node')
        self.gps_init = 1  # int(input("Use GPS? (0|1)") or 1)
        # Subscribers
        self.sub_cmd = self.create_subscription(Int16MultiArray, 'drone/CMD', self.cmd_callback, 10)
        self.sub_cmd
        self.sub_arm = self.create_subscription(Bool, 'base/ARM', self.arm_callback, 10)
        self.sub_arm
        timer_flow = 0.25  # seconds
        self.timer_flow = self.create_timer(timer_flow, self.heartbeat_callback)

        # Publishers
        # Publish Odom at a rate matching components: OF/Z_axis
        self.pub_odom = self.create_publisher(Odometry, 'drone/odom_raw', 10)
        timer_odom = 0.08  # seconds
        self.timer_odom = self.create_timer(timer_odom, self.odom_callback)

        # Publish Imu at a rate matching imu
        self.pub_imu = self.create_publisher(Imu, 'drone/imu_raw', 10)
        timer_imu = 0.005  # seconds
        self.timer_imu = self.create_timer(timer_imu, self.imu_callback)

        timer_print = 0.1  # seconds
        self.timer_print = self.create_timer(timer_print, self.print_callback)

        self.mpu = INERTIAL()
        self.motors = ESC()
        self.height = ZAXIS()
        if self.gps_init:
            self.pub_gps = self.create_publisher(NavSatFix, 'drone/GPS', 10)
            timer_gps = 0.1  # seconds
            self.timer_gps = self.create_timer(timer_gps, self.gps_callback)
            self.gps_init = 1
            self.gps = BN0()

        self.of = FLOW()
        self.arm_heartbeat_limit = 0.5  # second
        self.arm_heartbeat_last = time.time()

        self.pose = [0., 0., 0.]  # x, y, z
        self.quat = [0., 0., 0., 0., ]  # x, y, z, w
        self.twist = [0., 0., 0., 0., 0., 0.]  # linear: x, y, z, angular: x, y, z
        self.imu_dt = None

    def print_callback(self):
        # print(f'{self.motors.armed}')
        # print(f'fl,fr:{self.motors.last_cmd[:2]} \nbl,br:{self.motors.last_cmd[2:]}')
        # print(f'Status: height:{self.height.status} | *mpu: {self.mpu.status} | gps: {self.gps.status if self.gps_init else "NULL"} | *of: {self.of.status}')
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
            msg.header.frame_id = "map"
            msg.status.status = int(self.gps.sat_status)
            msg.latitude = self.gps.raw_pose[0]
            msg.longitude = self.gps.raw_pose[1]
            msg.altitude = self.gps.raw_pose[2]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub_gps.publish(msg)

    def imu_callback(self):
        self.mpu.read()
        self.quat = self.mpu.quat
        self.twist[3:] = self.mpu.angular_vel
        angular_vel = self.mpu.angular_vel
        linear_acc = self.mpu.linear_accel

        msg = Imu()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        msg.header.frame_id = "base_link"
        msg.orientation.x = float(self.mpu.quat[0])
        msg.orientation.y = float(self.mpu.quat[1])
        msg.orientation.z = float(self.mpu.quat[2])
        msg.orientation.w = float(self.mpu.quat[3])
        msg.angular_velocity.x = float(angular_vel[0])
        msg.angular_velocity.y = float(angular_vel[1])
        msg.angular_velocity.z = float(angular_vel[2])
        msg.linear_acceleration.x = float(linear_acc[0])
        msg.linear_acceleration.y = float(linear_acc[1])
        msg.linear_acceleration.z = float(linear_acc[2])
        self.pub_imu.publish(msg)

    def height_callback(self):
        self.height.read()
        self.pose[2] = self.height.height
        self.twist[2] = self.height.d_height

    def flow_callback(self):
        self.of.read()
        self.twist[:2] = [i * self.pose[2] for i in self.of.twist]
        # print('flow', self.of.twist, self.pose[2], self.twist[:3])

    def odom_callback(self):
        self.height_callback()
        self.flow_callback()

        msg = Odometry()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        msg.header.frame_id = "odom"
        msg.pose.pose.position.z = float(self.pose[2])
        msg.twist.twist.linear.x = float(self.twist[0])
        msg.twist.twist.linear.y = float(self.twist[1])
        msg.twist.twist.linear.z = float(self.twist[2])
        msg.pose.pose.orientation.x = float(self.mpu.quat[0])
        msg.pose.pose.orientation.y = float(self.mpu.quat[1])
        msg.pose.pose.orientation.z = float(self.mpu.quat[2])
        msg.pose.pose.orientation.w = float(self.mpu.quat[3])
        self.pub_odom.publish(msg)

    def cmd_callback(self, msg):
        cmd = msg.data
        print('CMD: ', cmd)
        try:
            self.motors.drive(cmd)
        except Exception as e:
            print(f'--------------{e}')
            time.sleep(2)

    def arm_callback(self, msg):
        arm = msg.data
        self.set_arm_state(arm)

    def set_arm_state(self, arm: bool = False):
        self.arm_heartbeat_last = time.time()
        if arm and not self.motors.armed:
            self.motors.arm()
        elif not arm:  # and self.motors.armed:
            self.motors.disarm()

    def heartbeat_callback(self):
        dt = time.time() - self.arm_heartbeat_last
        if dt > self.arm_heartbeat_limit:
            print(f"CONNECTION LOST: {dt}/{self.arm_heartbeat_limit}")
            self.set_arm_state(False)


def main(args=None):
    rclpy.init(args=args)
    device_node = DeviceNode()
    rclpy.spin(device_node)
    device_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
