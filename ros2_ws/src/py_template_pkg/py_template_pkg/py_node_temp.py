import rclpy
from rclpy.node import Node


class TemplateNode(Node): # you have to change the class name to match your package
    def __init__(self):
        super().__init__('py_template_node')  # change 'py_template_node' to your node name
        self.count_ = 0
        self.get_logger().info('Starting py_template_node...')
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second timer

    # This is an example of a simple function that will be called periodically
    def timer_callback(self):
        self.get_logger().info('Hello ' + str(self.count_))
        self.count_ += 1



def main(args=None):
    rclpy.init(args=args)

    node = TemplateNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()