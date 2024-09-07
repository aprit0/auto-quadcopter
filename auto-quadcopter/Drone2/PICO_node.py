
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16, Int16MultiArray, Float32MultiArray, Header, MultiArrayDimension 
from geometry_msgs.msg import TwistStamped, QuaternionStamped 

from PICO.communication import PI2PICO

'''
Subscribes:
- base/ARM | Bool
- drone/cmd/euler | Bool
- drone/cmd/throttle | Int16MultiArray

Publishes:
- drone/dev/euler | Float32MultiArray
    - [R, P, Y]
'''

class PICONode(Node, PI2PICO):
    def __init__(self):
        # super(Node, self).__init__('PICO_node') # type: ignore
        Node.__init__(self, 'PICO_node')
        PI2PICO.__init__(self)
        self.sub_euler = self.create_subscription(Float32MultiArray,'drone/cmd/euler',self.euler_callback,10)
        self.sub_euler
        self.sub_throttle = self.create_subscription(Int16,'drone/cmd/throttle',self.throttle_callback,10)
        self.sub_throttle
        self.sub_arm = self.create_subscription(Bool,'base/ARM',self.arm_callback,10)
        self.sub_arm 
        self.sub_pid = self.create_subscription(Float32MultiArray,'base/PID',self.pid_callback,10)
        self.sub_pid 
        self.pub_euler = self.create_publisher(Float32MultiArray,'drone/dev/euler', 10)

        timer_period = 0.1  # seconds
        self.timer_send = self.create_timer(timer_period, self.send_callback)
        timer_period = 0.5  # seconds
        self.timer_cmd = self.create_timer(timer_period, self.status_callback)
        self.timer_euler = self.create_timer(timer_period, self.dev_euler_callback)
        timer_period = 0.01  # seconds
        self.timer_cmd = self.create_timer(timer_period, self.recv_callback)

        self.msg_list = []
        self.cmd_euler = []
        self.cmd_pid = {}
        self.cmd_throttle = 0
        self.ARM = [0, False] # delta state, current arm
    
    def arm_callback(self, msg):
        if bool(msg.data) == bool(self.ARM[1]):
            # Not new data,
            pass
        else:
            self.ARM = [1, bool(msg.data)]
            print("arm: ", self.ARM)
    
    def pid_callback(self, msg):
        self.cmd_pid = {key: value for (key, value) in zip(["P", "I", "D"], list(msg.data))}

    def recv_callback(self):
        # print("reading")
        self.read_serial()
        self.update_state()

    def status_callback(self):
        if self.cmd_pid:
            self.set_pid_setpoints(self.cmd_pid.values())
            print("PID", self.cmd_pid)
            self.cmd_pid = {}
        # self.get_pid_setpoints()

    def send_callback(self):
        self.get_pose()
        # print("getting")
        if self.cmd_euler and self.ARM[1]:
            msg = [self.cmd_throttle] + self.cmd_euler
            self.set_setpoints(msg)
            self.cmd_euler = []
        if self.ARM[0]:
            self.set_arm(self.ARM[1])
            self.ARM[0] = 0
            print("set_arm: ", self.ARM)


    def euler_callback(self, msg):
        self.cmd_euler = list(msg.data)
    
    def throttle_callback(self, msg):
        self.cmd_throttle = int(msg.data)

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
    PICO_node = PICONode()
    rclpy.spin(PICO_node)
    PICO_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
