import threading
import rclpy
from joystick_node import JoystickNode
from schema_node import SchemaNode
if __name__ == '__main__':
    rclpy.init()
    node = SchemaNode()
    node2 = JoystickNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node2)
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = node.create_rate(2)
    try:
        while rclpy.ok():
            print('Help me body, you are my only hope')
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()


