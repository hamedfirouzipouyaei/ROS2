# Adding interpreter to the shebang line
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class MyNode(Node): # Define a class for the node inheriting from Node
    def __init__(self):
        super().__init__('py_test')

        self.counter = 0  # Initialize a counter

        # Log a message
        self.get_logger().info('Hello, ROS 2! from a class-based node')

        # Add a timer to log a message periodically [1 second interval]
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello ' + str(self.counter))
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library

    # Create a node instance
    # Using the class defined above
    # This is a more structured way to define nodes in ROS 2
    node = MyNode()
   
    # Spin the node to keep it alive
    rclpy.spin(node)

    # Shutdown the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    print("Node has been shut down.")
    