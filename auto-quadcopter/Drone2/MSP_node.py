
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16, Int16MultiArray, Float32MultiArray, Header, MultiArrayDimension 
from geometry_msgs.msg import TwistStamped, QuaternionStamped 
import threading
import time

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

        timer_period = 0.05  # seconds
        self.timer_euler = self.create_timer(timer_period, self.dev_euler_callback)

        self.msg_list = []
        self.cmd_euler = []
        self.cmd_pid = {}
        self.cmd_throttle = 0
        self.ARM = False

        pid_gains = {
            "x": (1.0, 0.1, 0.05, (-10, 10)),
            "y": (1.0, 0.1, 0.05, (-10, 10)),
            "z": (1.5, 0.2, 0.1, (-5, 5)),
            "roll": (1.2, 0.1, 0.08, (-5, 5)),
            "pitch": (1.2, 0.1, 0.08, (-5, 5)),
            "yaw": (1.0, 0.1, 0.05, (-5, 5)),
            "last_time": time.time()
        }

        self.thread = threading.Thread(target=self.main)
        self.thread.start()

    
    def arm_callback(self, msg):
        self.ARM = bool(msg.data)
        self.set_arm()

    def angle_callback(self, msg):
        # print("angle", msg.data)
        self.throttle = list(msg.data)[0]
        state = (0.1, -0.2, 1.5, 0, 0, 0, 5, -3, 10)  # Sample drone state
        setpoints = (0, 0, 0)  # Desired roll, pitch, yaw
        control_signals = self.autonomous_drone_control(state, setpoints, pid_gains)
        self.cmd_euler = list(msg.data)[1:]
    
    def autonomous_drone_control(self, state, setpoints, pid_gains):
        x, y, z, dx, dy, dz, roll, pitch, yaw = state
        roll_set, pitch_set, yaw_set = setpoints
        
        current_time = time.time()
        dt = pid_gains.get("last_time", current_time) - current_time
        pid_gains["last_time"] = current_time
        
        def compute_pid(error, dt, gains):
            kp, ki, kd, output_limits = gains
            integral = pid_gains.get(f"integral_{error}", 0) + error * dt
            derivative = (error - pid_gains.get(f"prev_error_{error}", 0)) / dt if dt > 0 else 0
            output = kp * error + ki * integral + kd * derivative
            pid_gains[f"integral_{error}"] = integral
            pid_gains[f"prev_error_{error}"] = error
            
            return max(min(output, output_limits[1]), output_limits[0])
        
        control_x = compute_pid(-x, dt, pid_gains["x"])
        control_y = compute_pid(-y, dt, pid_gains["y"])
        control_z = compute_pid(-z, dt, pid_gains["z"])
        roll_output = compute_pid(roll_set - roll, dt, pid_gains["roll"])
        pitch_output = compute_pid(pitch_set - pitch, dt, pid_gains["pitch"])
        yaw_output = compute_pid(yaw_set - yaw, dt, pid_gains["yaw"])
        
        return control_x, control_y, control_z, roll_output, pitch_output, yaw_output
def autonomous_drone_control(state, setpoints, pid_gains):
    x, y, z, dx, dy, dz, roll, pitch, yaw = state
    roll_set, pitch_set, yaw_set = setpoints
    
    current_time = time.time()
    dt = pid_gains.get("last_time", current_time) - current_time
    pid_gains["last_time"] = current_time
    
    def compute_pid(error, dt, gains):
        kp, ki, kd, output_limits = gains
        integral = pid_gains.get(f"integral_{error}", 0) + error * dt
        derivative = (error - pid_gains.get(f"prev_error_{error}", 0)) / dt if dt > 0 else 0
        output = kp * error + ki * integral + kd * derivative
        pid_gains[f"integral_{error}"] = integral
        pid_gains[f"prev_error_{error}"] = error
        
        return max(min(output, output_limits[1]), output_limits[0])
    
    # Compute desired accelerations based on roll and pitch setpoints
    ax = np.sin(np.radians(pitch_set))
    ay = -np.sin(np.radians(roll_set))
    
    # Adjust x, y control based on velocity and acceleration
    control_x = compute_pid(-x - dx + ax, dt, pid_gains["x"])
    control_y = compute_pid(-y - dy + ay, dt, pid_gains["y"])
    control_z = compute_pid(-z - dz, dt, pid_gains["z"])
    
    roll_output = compute_pid(control_y * np.cos(yaw) - control_x * np.sin(yaw), dt, pid_gains["roll"])
    pitch_output = compute_pid(control_x * np.cos(yaw) + control_y * np.sin(yaw), dt, pid_gains["pitch"])
    yaw_output = compute_pid(yaw_set - yaw, dt, pid_gains["yaw"])
    
    return roll_output, pitch_output, yaw_output, control_z

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
