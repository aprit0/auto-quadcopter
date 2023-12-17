import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, Vector3 

'''
Aim: Convert joystick messages to control schema 
- Velocity mode: Twist
- Position mode: Pose # FUTURE
Justification: Maintain modularity of controller and limit messages required for transmission

Mapping:
Buttons = [ARM, HOLD, SCHEMA, ...]
ARM:0 = Disarm
HOLD:0 = No altitude hold
SCHEMA:0 = Velocity Control

linear/angular: [-1, 1]

Subscribes:
- base/Joy | Joy

Publishes:
- base/twist | Twist
- base/ARM | Bool
- base/HOLD | Bool
'''
class SchemaNode(Node):
    def __init__(self):
        super().__init__('schema_node')
        self.sub_joy = self.create_subscription(Joy,'base/Joy',self.joy_callback,10)
        self.pub_twist = self.create_publisher(TwistStamped, 'base/twist', 10)
        self.pub_arm = self.create_publisher(Bool, 'base/ARM', 10)
        self.pub_hold = self.create_publisher(Bool, 'base/HOLD', 10)
        self.pub_mode = self.create_publisher(Bool, 'base/MODE', 10)
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.msg_twist = None
        self.msg_arm = None
        self.msg_mode = None
        self.msg_hold = None
        self.timer_callback()

    def timer_callback(self):
        if not self.msg_arm:
            # Controller yet to be connected
            heartbeat = Bool()
            heartbeat.data = False
            self.pub_arm.publish(heartbeat)
        else:
            self.pub_arm.publish(self.msg_arm)
            self.pub_mode.publish(self.msg_mode)
            self.pub_hold.publish(self.msg_hold)
            # if self.msg_mode.data == False:
            self.pub_twist.publish(self.msg_twist)



    def joy_callback(self, msg):
        reverse = lambda x: 1000 if x == 2000 else 2000
        axes = msg.axes
        [arm, hold, schema] = msg.buttons[:3]
        # Velocity Control
        self.msg_twist = TwistStamped()
        self.msg_twist.header = Header()
        self.msg_twist.twist.linear, self.msg_twist.twist.angular = self.vel_control(axes)

        self.msg_arm = Bool()
        self.msg_arm.data = bool(arm - 1000)
        self.msg_hold = Bool()
        self.msg_hold.data = bool(reverse(hold)-1000)
        self.msg_mode = Bool()
        self.msg_mode.data = bool(reverse(schema)-1000)
        print(f"[{self.msg_arm.data}]: Mode:{self.msg_mode.data}, Hold:{self.msg_hold.data}")
    
        
    def vel_control(self, axes):
        # Input: axes = [Throttle, Roll, Pitch, Yaw] 
        # Output: linear, angular = [x, y, z] * 2
        linear, angular = Vector3(), Vector3()
        key_map = lambda x: np.interp(x, [1000, 2000], [-2, 2])
        zero_out = lambda x: x if abs(x) > 1e-7 else 0.
        linear.x = zero_out(key_map(axes[2]))
        linear.y = zero_out(key_map(axes[1]))
        linear.z = zero_out(axes[0])
        angular.z = zero_out(key_map(axes[3]))
        return linear, angular
        
def main():
    rclpy.init()
    schema_publisher = SchemaNode()
    rclpy.spin(schema_publisher)
    schema_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 

        




    
