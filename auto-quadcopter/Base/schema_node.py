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
        self.pub_twist = self.create_publisher(TwistStamped, 'base/twist', 10)
        self.pub_arm = self.create_publisher(Bool, 'base/ARM', 10)
        self.pub_hold = self.create_publisher(Bool, 'base/HOLD', 10)
        self.pub_mode = self.create_publisher(Bool, 'base/MODE', 10)
        self.sub_joy = self.create_subscription(Joy,'base/Joy',self.joy_callback,10)


    def joy_callback(self, msg):
        axes = msg.axes
        [arm, hold, schema] = msg.buttons[:3]
        print(arm,hold,schema)
        if schema == 2000:
            # Velocity Control
            msg_twist = TwistStamped()
            msg_twist.header = Header()
            msg_twist.twist.linear, msg_twist.twist.angular = self.vel_control(axes)
            self.pub_twist.publish(msg_twist)

        msg_arm = Bool()
        msg_arm.data = bool(arm - 1000)
        self.pub_arm.publish(msg_arm)
        msg_hold = Bool()
        msg_hold.data = bool(hold-1000)
        self.pub_hold.publish(msg_hold)
        msg_mode = Bool()
        msg_mode.data = bool(schema-1000)
        self.pub_mode.publish(msg_mode)
        print(f"[{msg_mode.data}]: A:{msg_arm.data}, H:{msg_hold.data}")
        
    def vel_control(self, axes):
        # Input: axes = [Throttle, Roll, Pitch, Yaw] 
        # Output: linear, angular = [x, y, z] * 2
        linear, angular = Vector3(), Vector3()
        key_map = lambda x: np.interp(x, [1000, 2000], [-2, 2])
        linear.x = key_map(axes[2])
        linear.y = key_map(axes[1])
        linear.z = axes[0]
        angular.z = key_map(axes[3])
        return linear, angular
        
def main():
    rclpy.init()
    schema_publisher = SchemaNode()
    rclpy.spin(schema_publisher)
    schema_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 

        




    
