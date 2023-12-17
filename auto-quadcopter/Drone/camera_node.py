import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16MultiArray, Header
from sensor_msgs.msg import CompressedImage 
from devices.camera_ip import Video


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.pub_img0 = self.create_publisher(CompressedImage, 'drone/img0', 10)
        timer_img0 = 0.05  # seconds
        self.timer_img0 = self.create_timer(timer_img0, self.img0_callback)
        self.cam0 = Video()

    def img0_callback(self):
        image_np = self.cam0.read()
        msg = CompressedImage()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        msg.format = "jpeg"
        msg.data = np.array(cv.imencode('.jpg', image_np)[1]).tostring()
        self.pub_img0.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
