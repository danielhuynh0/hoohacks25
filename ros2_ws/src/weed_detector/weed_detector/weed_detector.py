import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        # Initialize the node with a name (in this case, 'simple_node')
        super().__init__('simple_node')
        
        # Log a message at the start
        self.get_logger().info('Simple Node has been started.')
        
        # Create a timer to call the timer_callback function every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # This callback will be called every 1 second
        self.get_logger().info('Hello from Simple Node!')

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the SimpleNode class
    node = SimpleNode()
    
    # Keep the node running (spinning) until interrupted
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown and clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()