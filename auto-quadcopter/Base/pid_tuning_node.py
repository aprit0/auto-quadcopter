import rclpy
from std_msgs.msg import Float32MultiArray

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('pid_input_publisher')
    publisher = node.create_publisher(Float32MultiArray, 'base/PID', 10)

    p_input = input("Enter the P value: ") or 3.0
    i_input = input("Enter the I value: ") or 0.0
    d_input = input("Enter the D value: ") or 0.0

    try:
        p = float(p_input)
        i = float(i_input)
        d = float(d_input)

        msg = Float32MultiArray()
        msg.data = [p, i, d]

        publisher.publish(msg)
        node.get_logger().info('Published PID values: %s' % msg.data)

    except ValueError:
        node.get_logger().error('Invalid input. Please enter valid float values for P, I, and D.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()

