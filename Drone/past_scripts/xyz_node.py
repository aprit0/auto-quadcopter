
import time
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, PoseStamped
from std_msgs.msg import Header, Float32 
from sensor_msgs.msg import CompressedImage

from control.optical_flow import OpticalFlow


class XYZNode(Node):
    def __init__(self):
        super().__init__('xyz_node')
        self.sub_quat = self.create_subscription(QuaternionStamped, 'drone/Quaternion', self.quat_callback, 10)
        self.sub_quat 
        self.sub_height = self.create_subscription(Float32, 'drone/Height', self.height_callback, 10)
        self.sub_height 
        self.sub_img0 = self.create_subscription(CompressedImage, 'drone/Img0', self.img0_callback, 10)
        self.sub_img0 
        self.pub_pose = self.create_publisher(PoseStamped, 'drone/Pose', 10)
        timer_period = 0.05  # seconds
        # self.timer = self.create_timer(timer_period, self.pose_callback)

        # params
        self.quaternion = [None] * 4 # x, y, z, w
        self.pose = [None] * 3 

        # objects
        self.OF = OpticalFlow()

    def get_pose(self):
        t_0 = time.time()
        self.pose = self.OF.get_pose()
        # print('Total time: ', time.time() - t_0)
        if self.pose is not None:
            return 1
        else:
            return 0
    def img0_callback(self, msg):
        np_arr = np.array(msg.data, np.uint8)
        image_np = cv.imdecode(np_arr, cv.IMREAD_GRAYSCALE)
        self.OF.set_image(image_np)
    
    def height_callback(self, msg):
        self.OF.set_height(msg.data)

    def quat_callback(self, msg):
        self.quaternion[0] = msg.quaternion.x 
        self.quaternion[1] = msg.quaternion.y
        self.quaternion[2] = msg.quaternion.z
        self.quaternion[3] = msg.quaternion.w
        self.OF.set_angle(self.quaternion)

    def pose_callback(self):
        if self.get_pose():
            msg = PoseStamped()
            h = Header()
            h.frame_id = 'map'
            h.stamp = self.get_clock().now().to_msg()
            msg.header = h
            msg.pose.position.x = self.pose[0]
            msg.pose.position.y = self.pose[1]
            msg.pose.position.z = self.pose[2]
            msg.pose.orientation.x = self.quaternion[0]
            msg.pose.orientation.y = self.quaternion[1]
            msg.pose.orientation.z = self.quaternion[2]
            msg.pose.orientation.w = self.quaternion[3]
            self.pub_pose.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    xyz_node = XYZNode()
    rclpy.spin(xyz_node)
    xyz_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
