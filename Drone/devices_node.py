import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped 
from std_msgs.msg import Bool, Int16MultiArray, Header 

from devices.bno085 import INERTIAL
from devices.motors import ESC

'''
External connection node
- Passes the FC messages to the motors and proveds i2c sensors information
Subscribes:
- drone/CMD | Int16MultiArray
    - [fl, fr, bl, br]
- drone/ARM | Bool

Publishes:
- drone/Quaternion | QuaternionStamped
'''


class DeviceNode(Node):
    def __init__(self):
        super().__init__('device_node')
        self.sub_cmd = self.create_subscription(Int16MultiArray,'drone/CMD',self.cmd_callback,10)
        self.sub_cmd  
        self.sub_arm = self.create_subscription(Bool,'drone/ARM',self.arm_callback,10)
        self.sub_arm  
        self.pub_quat = self.create_publisher(QuaternionStamped, 'drone/Quaternion', 10)
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.quat_callback)
        self.timer_print = self.create_timer(0.1, self.print_callback)

        self.mpu = INERTIAL()
        self.motors = ESC()

    def print_callback(self):
        print(f'{self.motors.armed}')
        print(f'fl,fr:{self.motors.last_cmd[:2]} \nbl,br:{self.motors.last_cmd[2:]}')
        pass

    def quat_callback(self):
        self.mpu.read()
        msg = QuaternionStamped()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        msg.header = h
        msg.quaternion.x =self.mpu.quat[0]
        msg.quaternion.y =self.mpu.quat[1]
        msg.quaternion.z =self.mpu.quat[2]
        msg.quaternion.w =self.mpu.quat[3]
        self.pub_quat.publish(msg)
        # print(self.mpu.euler)
    
    def cmd_callback(self, msg):
        cmd = msg.data
        # print('CMD: ', cmd)
        self.motors.drive(cmd)
    
    def arm_callback(self, msg):
        arm = msg.data
        if arm and not self.motors.armed:
            self.motors.arm()
        elif not arm and self.motors.armed:
            self.motors.disarm()
            
        # print(f'Motor: {arm} == {self.motors.armed}')




def main(args=None):
    rclpy.init(args=args)
    device_node = DeviceNode()
    rclpy.spin(device_node)
    device_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
