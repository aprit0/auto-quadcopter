import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import QuaternionStamped 
from std_msgs.msg import Bool, Int16MultiArray, Header, MultiArrayDimension 

from control.flight_controller import FC

'''
Subscribes:
- base/Joy | Joy
- drone/Quaternion | QuaternionStamped

Publishes:
- drone/CMD | Int16MultiArray
    - [fl, fr, bl, br]
- drone/ARM | Bool
'''


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.sub_quat = self.create_subscription(QuaternionStamped,'drone/Quaternion',self.quat_callback,10)
        self.sub_quat 
        self.sub_joy = self.create_subscription(Joy,'base/Joy',self.joy_callback,10)
        self.sub_joy 
        self.pub_cmd = self.create_publisher(Int16MultiArray,'drone/CMD', 10)
        timer_period = 0.01  # seconds
        self.timer_cmd = self.create_timer(timer_period, self.cmd_callback)
        self.pub_arm = self.create_publisher(Bool,'drone/ARM', 10)
        
        self.Control = FC()

    def quat_callback(self, msg):
        quat = [0, 0, 0, 0]
        quat[0] = msg.quaternion.x 
        quat[1] = msg.quaternion.y
        quat[2] = msg.quaternion.z 
        quat[3] = msg.quaternion.w 
        self.Control.update_pose(quat)
    
    def joy_callback(self, msg):
        self.Control.read_joystick(msg.axes, msg.buttons)
        # print('ARM: ', self.Control.ARM)

    def cmd_callback(self):
        cmd = self.Control.run()
        width = 4
        height = 1
        msg = Int16MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "height"
        msg.layout.dim[1].label = "width"
        msg.layout.dim[0].size = height
        msg.layout.dim[1].size = width
        msg.layout.dim[0].stride = width*height
        msg.layout.dim[1].stride = width
        msg.layout.data_offset = 0
        msg.data = [int(i) for i in cmd]
        # print('CMD: ', cmd)
        self.pub_cmd.publish(msg)

        msg = Bool()
        msg.data = bool(self.Control.ARM)
        self.pub_arm.publish(msg)

    
def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

