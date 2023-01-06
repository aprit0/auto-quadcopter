import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from flight_controller import NANO


class MultiWii(Node):
    def __init__(self):
        super().__init__('multi_wii')
        self.sub_joy = self.create_subscription(Joy, 'base/Joy',
                                                     self.listener_callback,
                                                     10)
        self.sub_joy  # prevent unused variable warning
        # self.timer = self.create_timer(1, self.listener_callback)
        self.FC = NANO()
        print('fin setup')

    def listener_callback(self, msg):
        self.FC.update_CMD(msg.axes)
        self.FC.update_mode(msg.buttons)
        print(self.FC.state)



def main(args=None):
    rclpy.init(args=args)
    multi_wii = MultiWii()
    rclpy.spin(multi_wii)
    multi_wii.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()