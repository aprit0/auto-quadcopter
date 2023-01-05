import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from multi_wii import MW
import traceback



class MultiWii(Node):
    def __init__(self):
        super().__init__('multi_wii')
        self.sub_joy = self.create_subscription(Joy, 'base/Joy',
                                                     self.listener_callback,
                                                     10)
        self.sub_joy  # prevent unused variable warning
        # self.timer = self.create_timer(1, self.listener_callback)
        self.FC = MW()
        print('fin setup')

    def listener_callback(self, msg):
        axes = list(msg.axes)
        buttons = list(msg.buttons)
        self.FC.main(axes + buttons)



def main(args=None):
    rclpy.init(args=args)
    multi_wii = MultiWii()
    try:
        rclpy.spin(multi_wii)
    except KeyboardInterrupt:
        print('Exit Node')
    except Exception as e:
        print(traceback.format_exc())

    multi_wii.FC.exit()
    multi_wii.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
