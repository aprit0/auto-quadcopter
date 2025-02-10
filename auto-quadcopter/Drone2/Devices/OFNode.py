import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
import math
import serial
import time

class OpticalFlowVelocityNode(Node):
    def __init__(self):
        super().__init__('optical_flow_velocity_node')

        # Publisher for velocity in x and y directions
        self.velocity_publisher = self.create_publisher(Twist, 'drone/dev/velocity', 10)
        self.z_publisher = self.create_publisher(Float32, 'drone/dev/z', 10)

        # Subscriber for Euler angles (roll, pitch, yaw) in degrees
        self.euler_angles_subscriber = self.create_subscription(
            Float32MultiArray,
            'drone/dev/euler',
            self.euler_angles_callback,
            10
        )
        f = open("/home/pi/ros2_ws/src/auto-quadcopter/auto-quadcopter/1_PYTHON_TEST/OpticalFlow/readings.txt", "w")
        f.close()
        self.write("dt,self.z,dz,self.tof_height},self.us_height},self.roll},{self.pitch,self.dx},{self.dy,{dist_x,{dist_y\n")

        # Timer to periodically publish velocity
        self.timer = self.create_timer(0.01, self.publish_velocity)  # Timer for 10 Hz (0.1s interval)

        # Serial port setup to read data from /dev/ttyUSB0
        self.serial_port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
        self.ser = serial.Serial(self.serial_port, 115200, timeout=1)  # Adjust the baud rate if needed
        time.sleep(2)  # Wait for the serial connection to initialize

        # Example variables for calculation (these should be updated with actual sensor data)
        self.last_data = None
        self.current_data = None
        self.z = 0
        self.dz = 0
        self.dt = 0
        self.dx = 0.0
        self.dy = 0.0
        self.px = 0.0
        self.py = 0.0
        self.us_height = 0  # height from TOF sensor (in meters)
        self.us_velocity = 0  # height from pressure sensor (in meters)
        self.pressure_height = 0  # height from pressure sensor (in meters)
        self.pressure_velocity = 0  # height from TOF sensor (in meters)
        self.tof_height = 0  # height from TOF sensor (in meters)
        self.tof_velocity = 0  # height from pressure sensor (in meters)
        self.roll = 0.0  # roll angle in radians
        self.pitch = 0.0  # pitch angle in radians
        self.yaw = 0.0  # yaw angle in radians
    def write(self, msg):
        f = open("/home/pi/ros2_ws/src/auto-quadcopter/auto-quadcopter/1_PYTHON_TEST/OpticalFlow/readings.txt", "a")
        f.write(msg)
        f.close()

    def euler_angles_callback(self, msg):
        """
        Callback to update roll, pitch, and yaw from the subscribed message.
        The angles are in degrees, so we convert them to radians.
        """
        if len(msg.data) == 3:
            self.roll = math.radians(msg.data[0])  # Convert roll from degrees to radians
            self.pitch = math.radians(msg.data[1])  # Convert pitch from degrees to radians
            self.yaw = math.radians(msg.data[2])  # Convert yaw from degrees to radians
            # self.get_logger().info(f"Received Euler angles: roll={msg.data[0]}°, pitch={msg.data[1]}°, yaw={msg.data[2]}°")

    def read_serial_data(self):
        """
        Read data from the serial port. This function assumes that the data is in a comma-separated
        format, and it can be customized based on your actual serial data format.
        """
        try:
            while self.ser.in_waiting > 0:
                # Read a line from the serial port
                line = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Serial data received: {line}")
                # Parse the data (e.g., optical flow dx, dy, and heights)
                data = [float(i) for i in line.split(',')]
                # dt	us_height	us_velocity	pressure_height	pressure_velocity	tof_height	tof_velocity	dx	dy
                self.last_data = self.current_data
                self.current_data = data
                [self.dt, self.us_height, self.us_velocity, self.pressure_height, self.pressure_velocity, self.tof_height, self.tof_velocity, self.px, self.py] = self.current_data
                _ = self.calculate_velocity_from_optical_flow()
            # else:
                # self.dt = 0
        except Exception as e:
            self.get_logger().warn(f"Failed to read serial data: {str(e)}")

    def calculate_true_height(self, tof_limit=2, us_limit=4):
        # Kinematics model to smooth tof_height
        z_est = self.z + self.dz * self.last_data[0] / 1000
        if self.current_data[5] > tof_limit and self.current_data[1] < us_limit:
            if not (z_est * 1.2 > self.current_data[1] > z_est * 0.8):
                # If the us value is significantly different to the estimated value, and we are in low level flight
                return z_est
        return self.tof_height if self.tof_height < tof_limit else self.us_height if self.us_height < us_limit else self.pressure_height, z_est




    def calculate_velocity_from_optical_flow(self, tof_limit=2, us_limit=4):
        """
        Calculate the velocity in the x and y directions from optical flow and sensor height.
        """
        
        true_height, z_est = self.calculate_true_height()
        self.dz = (true_height - self.z) / self.dt / 1000
        self.z = true_height
        # true_height = self.tof_height if self.tof_height < tof_limit else self.us_height if self.us_height < us_limit else self.pressure_height
        if true_height != self.pressure_height:
            true_height *= math.cos(self.pitch) * math.cos(self.roll)
            
        scaling_factor = (true_height / (35 * 10)) * 2 * math.tan(math.radians(42)/2)
        dist_x = self.px * scaling_factor
        dist_y = self.py * scaling_factor
        self.dx = dist_x / (self.dt / 1000)
        self.dy = dist_y / (self.dt / 1000)

        print(f"{self.dt},{self.z},{self.dz},{self.roll},{self.pitch},{self.dx},{self.dy}")
        self.write(f"{self.dt},{self.z},{self.dz},{self.roll},{self.pitch},{self.dx},{self.dy}\n")

    def publish_velocity(self):
        # Read serial data (optical flow and heights)
        self.read_serial_data()
        # Calculate the velocities from the optical flow and sensor data
        if self.z != None:
            # velocity_x, velocity_y = self.calculate_velocity_from_optical_flow()
            
            # # Create a Twist message to hold the velocity information
            velocity_msg = Twist()
            velocity_msg.linear.x = self.dx
            velocity_msg.linear.y = self.dy
            velocity_msg.linear.z = float(self.dz)
            
            z_msg = Float32()
            z_msg.data = float(self.z)

            # Publish the velocity message
            self.velocity_publisher.publish(velocity_msg)
            self.z_publisher.publish(z_msg)
            self.get_logger().info(f"Published velocity - x: {self.dx:.2f} m/s, y: {self.dx:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
