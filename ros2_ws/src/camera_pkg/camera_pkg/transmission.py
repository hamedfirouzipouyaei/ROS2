#!/usr/bin/env python3  
"""
Camera Transmission Node

This module implements a ROS 2 publisher node that simulates camera data transmission.
The node publishes string messages to demonstrate basic publisher functionality in ROS 2.

Author: [Your Name]
Date: July 30, 2025
Package: camera_pkg
"""

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class CameraTransmission(Node):
    """
    A ROS 2 node that simulates camera data transmission.
    
    This class inherits from rclpy.node.Node and creates a publisher that sends
    camera data messages at regular intervals. It demonstrates the basic
    publisher pattern in ROS 2.
    
    Attributes:
        publisher_ (Publisher): ROS 2 publisher for camera data
        timer (Timer): Timer that triggers periodic data publishing
    """
    
    def __init__(self):
        """
        Initialize the Camera Transmission node.
        
        Sets up the node name, creates a publisher for camera data,
        and establishes a timer for periodic publishing.
        """
        # Initialize the parent Node class with node name 'camera_transmission'
        super().__init__('camera_transmission')
        
        # Log node startup message
        self.get_logger().info('Camera Transmission Node has been started.')
        
        # Create a publisher for camera data
        # Parameters:
        # - String: Message type (from example_interfaces)
        # - "camera_data": Topic name where data will be published
        # - 10: Queue size (number of messages to buffer)
        self.publisher_ = self.create_publisher(String, "camera_data", 10)
        
        # Create a timer that calls publish_data() every 0.5 seconds
        # This simulates regular camera frame capture and transmission
        self.timer = self.create_timer(0.5, self.publish_data)

    def publish_data(self):
        """
        Publish camera data to the topic.
        
        This method is called periodically by the timer. It creates a new
        String message, populates it with simulated camera data, publishes
        it to the 'camera_data' topic, and logs the action.
        """
        # Create a new String message
        msg = String()
        
        # Set the message data (in real application, this would be actual camera data)
        msg.data = "Camera data transmission"
        
        # Publish the message to the 'camera_data' topic
        self.publisher_.publish(msg)
        


def main(args=None):
    """
    Main function to run the Camera Transmission node.
    
    This function initializes the ROS 2 Python client library,
    creates and runs the camera transmission node, and handles
    proper shutdown when the program is terminated.
    
    Args:
        args: Command line arguments (optional)
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the CameraTransmission node
    camera_transmission = CameraTransmission()
    
    # Keep the node running and processing callbacks
    # This will continue until Ctrl+C is pressed or the node is shutdown
    rclpy.spin(camera_transmission)
    
    # Clean shutdown of the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    """
    Entry point when the script is run directly.
    
    This allows the script to be executed as a standalone program
    or imported as a module without automatically running main().
    """
    main()