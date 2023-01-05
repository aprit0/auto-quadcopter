import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped 
from std_msgs.msg import Bool, Int16MultiArray, Header 

from Devices.imu import INERTIAL
from Devices.motors import ESC

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
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.quat_callback)

        self.mpu = INERTIAL()
        self.motors = ESC()

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
        print(self.mpu.euler)
    
    def cmd_callback(self, msg):
        cmd = msg.data
        self.motors.drive(cmd)
    
    def arm_callback(self, msg):
        arm = msg.data
        if arm:
            self.motors.arm()
        else:
            self.motors.disarm()
        print(f'Motor: {arm} == {self.motors.armed}')




def main(args=None):
    rclpy.init(args=args)
    device_node = DeviceNode()
    rclpy.spin(device_node)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()