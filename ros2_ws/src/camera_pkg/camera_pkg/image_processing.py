#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class CameraImageProcessing(Node):
    def __init__(self):
        super().__init__('cam_image_processing')
        self.get_logger().info('Camera Image Processing Node has been started.')
        self.subscription_ = self.create_subscription(String, 'camera_data', self.listener_callback, 10)
        
    def listener_callback(self, msg):
        self.get_logger().info(f'Received camera data: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = CameraImageProcessing()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()