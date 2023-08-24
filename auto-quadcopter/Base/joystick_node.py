import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from joystick import Joystick
import pygame


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        pygame.init()
        self.pub_joy = self.create_publisher(Joy, 'base/Joy', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.Joy = Joystick()
        self.events = []

    def timer_callback(self):
        state = self.Joy.update_state()
        # state['axes'][2] = 1500
        # state['axes'][3] = 1500
        # state['axes'][3] = 1500
        msg = Joy()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.axes = [float(i) for i in state['axes']]
        msg.buttons = [int(i) for i in state['buttons']]
        print(f'{self.Joy.status}: Commands ax:{[round(i, 2) for i in msg.axes]} || bt: {[int(i) for i in msg.buttons]}')
        if self.Joy.status:
            self.pub_joy.publish(msg)
        


def main():
    # os.environ["SDL_VIDEODRIVER"] = "dummy"
    # os.environ['SDL_AUDIODRIVER'] = 'dsp'
    # windowSize = width, height = 8, 6
    #screen = pygame.display.set_mode(windowSize)

    rclpy.init()
    joy_publisher = JoystickNode()
    rclpy.spin(joy_publisher)
    joy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
