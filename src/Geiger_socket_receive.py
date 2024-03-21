# File: Geiger_socket_receive.py
# Author: Nathan Leadbetter
# Date: 21/03/2024
# Source: OpenAI ChatGPT
# 
# Description:
# This file receives socket data from the PiGi module on a raspberry pi, and
# publishes that data to a ROS2 topic

"""
PUBLISHERS:
  + /count_rate (CountRate) - Radiation count rate in seconds

SUBSCRIBERS:
  + /pose (PoseWithCovarianceStamped) - The position of the robot in the world
"""

import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from bunker_mini.msg import CountRate

SERVER_HOST = '192.168.0.103'
SERVER_PORT = 12345

class PiGiRadiationPublisher(Node):
    def __init__(self):
        super().__init__('pigi_radiation_publisher')
        # Initialize publisher
        self.publisher_ = self.create_publisher(CountRate, '/count_rate', 10)
        # Initialize subscriber
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/pose', 
            self.pose_callback, 
            10)
        # Setup a timer for other periodic tasks
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.client_socket = None

    def pose_callback(self, msg):
        # Process pose data here
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        # For example, just print it
        self.get_logger().info(f"Received Pose: x={self.pose_x}, y={self.pose_y}")

    def timer_callback(self):
        if self.client_socket is None:
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((SERVER_HOST, SERVER_PORT))
            except Exception as e:
                self.get_logger().error(f"Connection failed: {e}")
                return

        try:
            # Assume the socket is ready to receive data
            data = self.client_socket.recv(1024)
            if data:
                # Process received data
                count, timestamp = data.decode().split(',')
                self.get_logger().info(f"Received Count: {count} at {timestamp}")
                # Example of publishing data
                # Convert count to a Float32 message and publish
                count_msg = CountRate()
                count_msg.x_position = float(self.pose_x)
                count_msg.y_position = float(self.pose_y)
                count_msg.count_rate = float(count)
                self.publisher_.publish(count_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to receive data: {e}")

def main():
    rclpy.init()
    pigi_radiation_publisher = PiGiRadiationPublisher()
    rclpy.spin(pigi_radiation_publisher)
    pigi_radiation_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
