
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16, Int16MultiArray, Float32MultiArray, Header, MultiArrayDimension 
from geometry_msgs.msg import TwistStamped, QuaternionStamped 
import threading

from MSP.communication import PI2MSP

'''
Subscribes:
- base/ARM | Bool
- drone/cmd/euler | Bool
- drone/cmd/throttle | Int16MultiArray

Publishes:
- drone/dev/euler | Float32MultiArray
    - [R, P, Y]
'''

class MSPNode(Node, PI2MSP):
    def __init__(self):
        # super(Node, self).__init__('PICO_node') # type: ignore
        Node.__init__(self, 'MSP_node')
        PI2MSP.__init__(self)
        self.sub_angle = self.create_subscription(Float32MultiArray,'base/angle',self.angle_callback,10)
        self.sub_angle
        self.sub_arm = self.create_subscription(Bool,'base/ARM',self.arm_callback,10)
        self.sub_arm 
        self.pub_euler = self.create_publisher(Float32MultiArray,'drone/dev/euler', 10)

        timer_period = 0.1  # seconds
        self.timer_euler = self.create_timer(timer_period, self.dev_euler_callback)

        self.msg_list = []
        self.cmd_euler = []
        self.cmd_pid = {}
        self.cmd_throttle = 0
        self.ARM = False

        self.thread = threading.Thread(target=self.main)
        self.thread.start()

    
    def arm_callback(self, msg):
        self.ARM = bool(msg.data)
        self.set_arm()

    def angle_callback(self, msg):
        # print("angle", msg.data)
        self.throttle = list(msg.data)[0]
        self.cmd_euler = list(msg.data)[1:]
    
    def dev_euler_callback(self):
        if self.euler_pose:
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
            # print("dev_euler_callback: ", self.euler_pose)
            msg.data = [float(i) for i in self.euler_pose]
            self.pub_euler.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    MSP_node = MSPNode()
    rclpy.spin(MSP_node)
    MSP_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
