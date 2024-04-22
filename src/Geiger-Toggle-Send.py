# File: Geiger.py
# Author: Nathan Leadbetter and Aidan Taylor-Lynch
# Date: 22/04/2024
# Source: OpenAI ChatGPT
# 
# Description:
# This file sends socket data from the PiGi module on a raspberry pi,
# with an additional toggle button
#UPDATED WITH NEW SOCKET FUNTIONALITY

#This code is the final combined code
#It combines functionality of the Geiger counter, a toggle switch when the "Enter" key is pressed and sending data through a python socket
#This will be run on the PiGi Pi5
#On the Robot Pi5, a script needs to be run to recieve this data, then publish it to a ROS topic

import socket
import threading
import datetime
import time
from gpiozero import Button

# Define the GPIO pin used for the Geiger counter on a Raspberry Pi, as per PiGI Schematic
GEIGER_PIN = 4
# Initialize a Button object to detect falling edges (radiation events)
geiger_button = Button(GEIGER_PIN, pull_up=False, bounce_time=0.0001)

# Initialize a global counter to keep track of radiation event counts
count = 0
# Flag to control whether counting is enabled
counting_enabled = True

def falling_edge_callback():
    """Callback function triggered by a falling edge on the Geiger counter's GPIO pin."""
    global count, counting_enabled
    if counting_enabled:
        count += 1
        print(f"Falling edge detected! Count: {count}")

# Assign the falling edge callback function to the button release event
geiger_button.when_released = falling_edge_callback

def toggle_counting():
    """Toggle the state of counting (pause/resume)."""
    global counting_enabled
    counting_enabled = not counting_enabled
    state = "resumed" if counting_enabled else "paused"
    print(f"Counting {state}.")

def key_listener():
    """Listen for keyboard input to toggle counting. Runs in a separate thread."""
    while True:
        input()  # Wait for Enter key press to toggle counting
        toggle_counting()

# Start thread for listening to keyboard input
threading.Thread(target=key_listener).start()

# Define server address and port to listen on all network interfaces
HOST = '0.0.0.0'
PORT = 12345

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    print("Server listening on {}:{}".format(HOST, PORT))

    # Keep the server running indefinitely
    while True:
        # Accept new client connections
        client_socket, client_address = server_socket.accept()
        print("Connected to client:", client_address)

        # Use a with statement to ensure the client socket closes properly
        with client_socket:
            while True:
                # Check if counting is enabled
                if counting_enabled:
                    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    data_to_send = f"{count},{timestamp}"
                    print(f"{timestamp} Sending count:", count)
                    
                    # Send the data
                    try:
                        client_socket.sendall(data_to_send.encode())
                    except socket.error as e:
                        print(f"Socket error: {e}")
                        break

                    # Reset the count after sending
                    if count > 0:
                        print("Resetting count.")
                        count = 0

                time.sleep(10)  # Delay between sending data

except Exception as e:
    print(f"Server Error: {e}")
finally:
    server_socket.close()
    print("Server shutting down. Cleaning up connections")
