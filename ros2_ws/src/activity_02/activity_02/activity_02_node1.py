import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class node1(Node):
    def __init__(self):
        super().__init__('act_02_node1')
        self.publisher_ = self.create_publisher(Int64, '/number', 10)
        self.timer = self.create_timer(2.0, self.publish_number)


    def publish_number(self):
        msg = Int64()
        msg.data = 56
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = node1()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()
