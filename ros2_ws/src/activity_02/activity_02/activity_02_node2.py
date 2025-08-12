import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class node2(Node):
    def __init__(self):
        super().__init__('act_02_node2')
        self.publisher_ = self.create_publisher(Int64, '/number_count', 10)
        self.subscriber_ = self.create_subscription(Int64, '/number', self.listener_callback, 10)
        self.count = 0
        # Publish initial count so topic shows value immediately
        self.publish_count(self.count)

    def listener_callback(self, msg):
        self.count += 1
        self.publish_count(self.count)   

    def publish_count(self, count):
        msg = Int64()
        msg.data = count
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published count: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = node2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
