# File: Geiger_receive.py
# Author: Nathan Leadbetter and Ben Cheshire
# Date: 18/04/2024
# Source: OpenAI ChatGPT
# 
# Description:
# This file receives socket data from the PiGi module on a raspberry pi, and
# receives coordinate data from ROS, the publishes onto a text file
# UPDATED AND IMPROVED VERSION

"""
PUBLISHERS:
  + /count_rate (CountRate) - Radiation count rate in seconds

SUBSCRIBERS:
  + /pose (PoseWithCovarianceStamped) - The position of the robot in the world
"""

import datetime
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from appfre.msg import CountRate
# CountRate is a custom ROS2 message defined in appfre/msg/CountRate.msg

current_datetime = datetime.datetime.now()
current_date = current_datetime.date()
current_time = current_datetime.time()

# Define server address and port
SERVER_HOST = '192.168.0.103'  # IP for Appfre-PiGI on Appfre-Network
SERVER_PORT = 12345 #Must be the same defined on server side
FILEPATH = f"/home/appfrebase/appfre_ws/src/appfre/rad_data/Radiation_Data_{current_date}_{current_time}.txt"

# Define the ROS2 node that contains all functionality of the script
class PiGiRadiationPublisher(Node):

	# Define __init__ function to initialize all variables and create publishers and subscribers
    def __init__(self):

        super().__init__('pigi_radiation_publisher')
        # Initialize publisher
        self.publisher_ = self.create_publisher(
            CountRate,  # CountRate is a custom ROS2 message defined in appfre/msg/CountRate.msg
            '/count_rate', 
            10)
        # Initialize subscriber
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/pose', 
            self.pose_callback, 
            10)
        # Setup a timer for other periodic tasks
        self.timer = self.create_timer(1, self.timer_callback)  # 1-second timer
        self.client_socket = None
        self.pose_x = 0
        self.pose_y = 0

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
                self.get_logger().info("Connected to the server")
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

                with open(FILEPATH, 'a') as file:
                	#Write the data to a new line in a new file
                	file.write(f"{count_msg.x_position}, {count_msg.y_position}, {count_msg.count_rate}\n")

                self.publisher_.publish(count_msg)
            else:
                self.client_socket.close()
                self.client_socket = None
                self.get_logger().info("Socket closed. No data received.")
        except Exception as e:
            self.get_logger().error(f"Failed to receive data: {e}")
            self.client_socket.close()
            self.client_socket = None

# Initialize node in main function and run script
def main():
    rclpy.init()
    pigi_radiation_publisher = PiGiRadiationPublisher()
    rclpy.spin(pigi_radiation_publisher)
    pigi_radiation_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
