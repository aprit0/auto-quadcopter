import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16MultiArray, Header, MultiArrayDimension 
from geometry_msgs.msg import TwistStamped, QuaternionStamped 

from control.flight_controller_interpreter import FCI as FC
from devices.utils import quat_2_euler

'''
Subscribes:
- base/ARM | Bool
- base/HOLD | Bool
- base/twist | Twist
- drone/odom | Odometry

Publishes:
- drone/CMD | Int16MultiArray
    - [fl, fr, bl, br]
- drone/ARM | Bool
'''


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node') # type: ignore
        self.sub_odom = self.create_subscription(Odometry,'drone/odom',self.odom_callback,10)
        self.sub_odom 
        self.sub_twist = self.create_subscription(TwistStamped,'base/twist',self.twist_callback,10)
        self.sub_twist 
        self.sub_euler = self.create_subscription(QuaternionStamped,'base/euler',self.euler_callback,10)
        self.sub_euler 
        self.sub_arm = self.create_subscription(Bool,'base/ARM',self.arm_callback,10)
        self.sub_arm 
        self.sub_hold = self.create_subscription(Bool,'base/HOLD',self.hold_callback,10)
        self.sub_hold 
        self.sub_mode = self.create_subscription(Bool,'base/MODE',self.mode_callback,10)
        self.sub_mode
        self.pub_cmd = self.create_publisher(Int16MultiArray,'drone/CMD', 10)
        timer_period = 0.01  # seconds
        self.timer_cmd = self.create_timer(timer_period, self.cmd_callback)
        
        self.Control = FC()

    def odom_callback(self, msg):
        q = [0, 0, 0, 0]
        pose = msg.pose.pose
        twist = msg.twist.twist
        q[0] = pose.orientation.x  
        q[1] = pose.orientation.y 
        q[2] = pose.orientation.z 
        q[3] = pose.orientation.w 
        euler = list(quat_2_euler(q[0], q[1], q[2], q[3]))
        cartesian = [pose.position.x, pose.position.y, pose.position.z]
        linear = [twist.linear.x, twist.linear.y,twist.linear.z]
        angular = [twist.angular.x, twist.angular.y,twist.angular.z]
        # print('odom: ', cartesian, euler, linear, angular)
        rnd = lambda x: [round(i, 2) for i in x]
        # print(f"odom: {rnd(linear), rnd(angular)}")
        self.Control.update_pose([cartesian, euler], [linear, angular])
    
    def euler_callback(self, msg):
        self.Control.update_euler_setpoints([msg.quaternion.x, 
                                             msg.quaternion.y,
                                             msg.quaternion.z,
                                             msg.quaternion.w,
                                             ])
    def twist_callback(self, msg):
        # print(f'twist callback: {[msg.twist.linear, msg.twist.angular]}')
        self.Control.update_twist_setpoints([msg.twist.linear, msg.twist.angular])

    def arm_callback(self, msg):
        self.Control.update_bools('arm', value=bool(msg.data))
    
    def hold_callback(self, msg):
        self.Control.update_bools('hold', value=bool(msg.data))
    
    def mode_callback(self, msg):
        self.Control.update_bools('mode', value=bool(msg.data))

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

    
def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

