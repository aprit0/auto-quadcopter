import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from joystick import Joystick
import pygame


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_mode')
        self.pub_joy = self.create_publisher(Joy, 'base/joy', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.Joy = Joystick()

    def timer_callback(self):
        events = pygame.event.get()
        if events and not (len(events) == 1 and events[0].type == pygame.JOYBUTTONUP):
            self.Joy.update_state(events)
            msg = Joy()
            msg.header = Header()
            msg.axes = self.Joy.state['axes']
            msg.buttons = self.Joy.state['buttons']
            self.pub_joy.publish(msg)
            self.get_logger().info('{}'.format(msg))


def main(args=None):
    pygame.init()
    while not pygame.joystick.get_count():
        print('No joystick detected', pygame.joystick.get_count())
    init_joystick = pygame.joystick.Joystick(0)
    init_joystick.init()
    rclpy.init(args=args)
    joy_publisher = JoystickNode()
    rclpy.spin(joy_publisher)
    joy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()