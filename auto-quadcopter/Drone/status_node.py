import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16 

from tools.display import OLED


class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher_ = self.create_publisher(Int16, 'drone/wifi', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.disp = OLED()

    def timer_callback(self):
        self.disp.set_status()
        msg = Int16()
        msg.data = int(self.disp.status_dict['RSSI'].split()[0])
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    status_publisher = StatusPublisher()
    rclpy.spin(status_publisher)
    status_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
