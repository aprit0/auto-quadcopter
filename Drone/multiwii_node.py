import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from multi_wii import MW


class MultiWii(Node):

    def __init__(self):
        super().__init__('multi_wii')
        self.sub_joy = self.create_subscription(Joy, 'base/Joy',
                                                     self.listener_callback,
                                                     10)
        self.sub_joy  # prevent unused variable warning
        self.FC = MW()

    def listener_callback(self, msg):
        axes = []
        buttons = []
        axes = msg.axes
        buttons = msg.buttons
        self.FC.update_state(axes)
        print(self.FC.state)



def main(args=None):
    rclpy.init(args=args)
    multi_wii = MultiWii()
    rclpy.spin(multi_wii)
    multi_wii.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
