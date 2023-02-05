import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from joystick import Joystick
import pygame


class JoystickNode(Node):
    def __init__(self, args):
        super().__init__('joystick_mode')
        self.pub_joy = self.create_publisher(Joy, 'base/Joy', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.Joy = Joystick(Keyboard=args['Key'])
        self.events = []

    def timer_callback(self):
        event_new = pygame.event.get()
        # print(event_new, pygame.JOYDEVICEREMOVED)
        # print(type(event_new))
        # print(event_new.type)
        # if event_new == pygame.JOYDEVICEREMOVED:
        #     print("Phew")
        #     self.Joy.joy.quit()
        # elif event_new == pygame.JOYDEVICEADDED:
        #     print("AAAAAAAAAAAAAAAAAAAAAA")
        #     self.Joy.joy.init()
        # else:
        self.events  = self.events if event_new == [] else event_new
        state = self.Joy.update_state(self.events)
        state['axes'][2] = 1500
        state['axes'][3] = 1500
        state['axes'][4] = 1500
        msg = Joy()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.axes = [float(i) for i in state['axes']]
        msg.buttons = [int(i) for i in state['buttons']]
        print(f'{self.Joy.Key}: Commands ax:{[round(i, 2) for i in msg.axes]} || bt: {[float(i) for i in msg.buttons]}')
        self.pub_joy.publish(msg)
        


import os
def main(args={'Key': False}):
    # os.environ["SDL_VIDEODRIVER"] = "dummy"
    # os.environ['SDL_AUDIODRIVER'] = 'dsp'
    pygame.init()
    windowSize = width, height = 8, 6
    #screen = pygame.display.set_mode(windowSize)

    rclpy.init()
    joy_publisher = JoystickNode(args)
    rclpy.spin(joy_publisher)
    joy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
