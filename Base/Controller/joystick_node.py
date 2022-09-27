import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from joystick import Joystick
import pygame


class JoystickNode(Node):
    def __init__(self, args):
        super().__init__('joystick_mode')
        self.pub_joy = self.create_publisher(Joy, 'base/joy', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.Joy = Joystick(Keyboard=args['Key'])

    def timer_callback(self):
        events = pygame.event.get()
        if events and not (len(events) == 1 and events[0].type == pygame.JOYBUTTONUP):
            state = self.Joy.update_state(events)
            msg = Joy()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [float(i) for i in state['axes']]
            msg.buttons = [int(i) for i in state['buttons']]
            self.pub_joy.publish(msg)
            self.get_logger().info('{}'.format(msg))


def main(args={'Key': True}):
    pygame.init()
    while not args['Key'] and not pygame.joystick.get_count():
        print('No joystick detected', pygame.joystick.get_count())
    if not args['Key']:
        init_joystick = pygame.joystick.Joystick(0)
        init_joystick.init()
    windowSize = width, height = 8, 6
    screen = pygame.display.set_mode(windowSize)

    rclpy.init()
    joy_publisher = JoystickNode(args)
    rclpy.spin(joy_publisher)
    joy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
