import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16MultiArray, Header, MultiArrayDimension, Int16, Float32MultiArray
from geometry_msgs.msg import TwistStamped, QuaternionStamped 

from Control.grand_central_station import GRANDCENTRALSTATION
'''
Subscribes:
- base/ARM | Bool
- base/HOLD | Bool
- base/twist | Twist
- base/euler | Int16MultiArray
- drone/dev/euler | Int16MultiArray
    - [R, P, Y]
# Future sub to devices as well

Publishes:
- drone/cmd/euler | Int16MultiArray
    - [R, P, Y]
- drone/cmd/throttle | Int16
'''

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node') # type: ignore
        self.sub_twist = self.create_subscription(TwistStamped,'base/twist',self.twist_callback,10)
        self.sub_twist 
        self.sub_angle = self.create_subscription(Float32MultiArray,'base/angle',self.angle_callback,10)
        self.sub_angle
        self.sub_arm = self.create_subscription(Bool,'base/ARM',self.arm_callback,10)
        self.sub_arm 
        self.sub_hold = self.create_subscription(Bool,'base/HOLD',self.hold_callback,10)
        self.sub_hold 
        self.sub_mode = self.create_subscription(Bool,'base/MODE',self.mode_callback,10)
        self.sub_mode
        self.pub_cmd_euler = self.create_publisher(Float32MultiArray,'drone/cmd/euler', 10)
        self.pub_cmd_throttle = self.create_publisher(Int16,'drone/cmd/throttle', 10)

        timer_period = 0.1  # seconds
        self.timer_cmd = self.create_timer(timer_period, self.cmd_callback)

        self.GCS = GRANDCENTRALSTATION()
    
    def angle_callback(self, msg):
        self.GCS.set_command(msg, "Angle")
    
    def twist_callback(self, msg):
        self.GCS.set_command(msg, "Twist")

    def arm_callback(self, msg):
        self.GCS.update_bools('arm', value=bool(msg.data))
    
    def hold_callback(self, msg):
        self.GCS.update_bools('hold', value=bool(msg.data))
    
    def mode_callback(self, msg):
        self.GCS.update_bools('mode', value=bool(msg.data))

    def cmd_callback(self):
        cmd_throttle, cmd_euler = self.GCS.get_command()
        width = 3
        height = 1
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "height"
        msg.layout.dim[1].label = "width"
        msg.layout.dim[0].size = height
        msg.layout.dim[1].size = width
        msg.layout.dim[0].stride = width*height
        msg.layout.dim[1].stride = width
        msg.layout.data_offset = 0
        msg.data = [float(i) for i in cmd_euler]
        print('CMD: ', cmd_throttle, cmd_euler, self.GCS.bools)
        self.pub_cmd_euler.publish(msg)
        msg = Int16()
        msg.data = int(cmd_throttle)
        self.pub_cmd_throttle.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()