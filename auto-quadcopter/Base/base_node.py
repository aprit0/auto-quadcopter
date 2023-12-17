import threading
import rclpy
from joystick_node import JoystickNode
from schema_node import SchemaNode
if __name__ == '__main__':
    rclpy.init()
    node = SchemaNode()
    node2 = JoystickNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=1)
    executor.add_node(node)
    executor.add_node(node2)
    # Spin in a separate thread
    try:
        executor.spin()
    except Exception as e:
        print(e)
        node.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()


