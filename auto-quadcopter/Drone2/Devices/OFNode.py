import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
import serial
import time

class OpticalFlowVelocityNode(Node):
    def __init__(self):
        super().__init__('optical_flow_velocity_node')

        # Publisher for velocity in x and y directions
        self.velocity_publisher = self.create_publisher(Twist, 'drone/dev/velocity', 10)

        # Subscriber for Euler angles (roll, pitch, yaw) in degrees
        self.euler_angles_subscriber = self.create_subscription(
            Float32MultiArray,
            'drone/dev/euler',
            self.euler_angles_callback,
            10
        )

        # Timer to periodically publish velocity
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Timer for 10 Hz (0.1s interval)

        # Serial port setup to read data from /dev/ttyUSB0
        self.serial_port = '/dev/ttyUSB0'
        self.ser = serial.Serial(self.serial_port, 115200, timeout=1)  # Adjust the baud rate if needed
        time.sleep(2)  # Wait for the serial connection to initialize

        # Example variables for calculation (these should be updated with actual sensor data)
        self.dx = 0.0
        self.dy = 0.0
        self.height_tof = 0  # height from TOF sensor (in meters)
        self.height_pressure = 0  # height from pressure sensor (in meters)
        self.roll = 0.0  # roll angle in radians
        self.pitch = 0.0  # pitch angle in radians
        self.yaw = 0.0  # yaw angle in radians

    def euler_angles_callback(self, msg):
        """
        Callback to update roll, pitch, and yaw from the subscribed message.
        The angles are in degrees, so we convert them to radians.
        """
        if len(msg.data) == 3:
            self.roll = math.radians(msg.data[0])  # Convert roll from degrees to radians
            self.pitch = math.radians(msg.data[1])  # Convert pitch from degrees to radians
            self.yaw = math.radians(msg.data[2])  # Convert yaw from degrees to radians
            self.get_logger().info(f"Received Euler angles: roll={msg.data[0]}°, pitch={msg.data[1]}°, yaw={msg.data[2]}°")

    def read_serial_data(self):
        """
        Read data from the serial port. This function assumes that the data is in a comma-separated
        format, and it can be customized based on your actual serial data format.
        """
        try:
            if self.ser.in_waiting > 0:
                # Read a line from the serial port
                line = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Serial data received: {line}")
                # Parse the data (e.g., optical flow dx, dy, and heights)
                data = line.split(',')
                if len(data) >= 4:
                    self.dx = float(data[0])  # optical flow in x (in pixels)
                    self.dy = float(data[1])  # optical flow in y (in pixels)
                    self.height_tof = float(data[2])  # TOF height (in meters)
                    self.height_pressure = float(data[3])  # Pressure height (in meters)
        except Exception as e:
            self.get_logger().warn(f"Failed to read serial data: {str(e)}")

    def calculate_velocity_from_optical_flow(self, dx, dy, height_tof, height_pressure, roll, pitch, focal_length, sensor_width, sensor_height, image_width, image_height):
        """
        Calculate the velocity in the x and y directions from optical flow and sensor height.
        """
        # 1. Calculate the average height
        height_avg = (height_tof + height_pressure) / 2.0
        
        # 2. Calculate pixel size in meters
        pixel_size_x = (sensor_width / image_width) * (height_avg / focal_length)  # meters per pixel in x direction
        pixel_size_x = (height_avg / focal_length) * math.tan(FoV_angle_rad / 2.0)
        pixel_size_y = pixel_size_x * (sensor_height / sensor_width) * (image_width / image_height)
        pixel_size_y = (sensor_height / image_height) * (height_avg / focal_length)  # meters per pixel in y direction

        # 3. Convert optical flow displacements to real-world velocities in x and y directions
        velocity_x = dx * pixel_size_x
        velocity_y = dy * pixel_size_y
        
        # 4. Adjust the velocity for the roll and pitch
        velocity_x_corrected = velocity_x * math.cos(pitch) - velocity_y * math.sin(roll) * math.sin(pitch)
        velocity_y_corrected = velocity_y * math.cos(roll) + velocity_x * math.sin(roll) * math.sin(pitch)

        return velocity_x_corrected, velocity_y_corrected

    def publish_velocity(self):
        # Read serial data (optical flow and heights)
        self.read_serial_data()
        sensor_width = 3.6
        sensor_height = 3.6
        focal_length = 80/1000
        image_width = 640
        image_height = 480
        # Calculate the velocities from the optical flow and sensor data
        velocity_x, velocity_y = self.calculate_velocity_from_optical_flow(
            self.dx, self.dy, self.height_tof, self.height_pressure, 
            self.roll, self.pitch, sensor_width, sensor_height, focal_length, image_width, image_height  # Example camera parameters
        )
        
        # Create a Twist message to hold the velocity information
        velocity_msg = Twist()
        velocity_msg.linear.x = velocity_x
        velocity_msg.linear.y = velocity_y
        
        # Publish the velocity message
        self.velocity_publisher.publish(velocity_msg)
        self.get_logger().info(f"Published velocity - x: {velocity_x:.2f} m/s, y: {velocity_y:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
