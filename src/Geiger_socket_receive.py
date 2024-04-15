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
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from bunker_mini.msg import CountRate # CountRate is a custom ROS2 message defined in bunker_mini/msg/CountRate.msg

# Assign the host (Raspberry Pi) IP address to receive from (can use 0.0.0.0 to listen on all channels)
SERVER_HOST = '192.168.0.103'

# Assign the socket port. This must be the same as the port assigned on the host (raspberry pi)
SERVER_PORT = 12345

# Define the ROS2 node that contains all functionality of the script
class PiGiRadiationPublisher(Node):

    # Define __init__ function to initialize all variables and create publishers and subscribers
    def __init__(self):
        super().__init__('pigi_radiation_publisher')
        # Initialize publisher
        self.publisher_ = self.create_publisher(
            CountRate, # CountRate is a custom ROS2 message defined in bunker_mini/msg/CountRate.msg
            '/count_rate', 
            10)
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

    # Define function to subscribe to the robots position,
    # This allows the radiation reading to be matched up to a co-ordinate on the map
    def pose_callback(self, msg):
        # Process pose data here
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        # Debugging print statement
        #self.get_logger().info(f"Received Pose: x={self.pose_x}, y={self.pose_y}")

    # This timer function gets called every 1 second (timer_period = 1) as long as the node is spinning
    def timer_callback(self):
        # Establish connection to the socket
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
                # Convert count to a CountRate message and publish
                count_msg = CountRate()
                count_msg.x_position = float(self.pose_x)
                count_msg.y_position = float(self.pose_y)
                count_msg.count_rate = float(count)
                file_path = "./src/appfre/msg/Radiation_Data.txt"
                with open(file_path, 'a') as file:
                    # Write the data to a new line in the file
                    file.write(f"{count_msg.x_position}, {count_msg.y_position}, {count_msg.count_rate}\n")
                self.publisher_.publish(count_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to receive data: {e}")

# Initialize node in main function and run script
def main():
    rclpy.init()
    pigi_radiation_publisher = PiGiRadiationPublisher()
    rclpy.spin(pigi_radiation_publisher)
    pigi_radiation_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
