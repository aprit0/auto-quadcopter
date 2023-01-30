import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16MultiArray, Header, MultiArrayDimension 

from control.flight_controller import FC
from devices.utils import quat_2_euler

'''
Subscribes:
- base/Joy | Joy
- drone/Odom | Odometry

Publishes:
- drone/CMD | Int16MultiArray
    - [fl, fr, bl, br]
- drone/ARM | Bool
'''


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.sub_odom = self.create_subscription(Odometry,'drone/Odom',self.odom_callback,10)
        self.sub_odom 
        self.sub_joy = self.create_subscription(Joy,'base/Joy',self.joy_callback,10)
        self.sub_joy 
        self.pub_cmd = self.create_publisher(Int16MultiArray,'drone/CMD', 10)
        timer_period = 0.01  # seconds
        self.timer_cmd = self.create_timer(timer_period, self.cmd_callback)
        self.pub_arm = self.create_publisher(Bool,'drone/ARM', 10)
        
        self.Control = FC()
        self.euler = None

    def odom_callback(self, msg):
        q = [0, 0, 0, 0]
        q[0] = msg.pose.pose.orientation.x  
        q[1] = msg.pose.pose.orientation.y 
        q[2] = msg.pose.pose.orientation.z 
        q[3] = msg.pose.pose.orientation.w 
        self.euler = quat_2_euler(q[0], q[1], q[2], q[3])
        self.Control.update_pose(self.euler)
    
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

