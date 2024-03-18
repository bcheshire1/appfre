#This code is the final combined code
#It combines functionality of the Geiger counter, a toggle switch when the "Enter" key is pressed and sending data through a python socket
#This will be run on the PiGi Pi5
#On the Robot Pi5, a script needs to be run to recieve this data, then publish it to a ROS topic

import socket
from gpiozero import Button
from signal import pause
import time
import datetime
import threading

# Define GPIO pin for the Geiger counter input
# As specified in PiGI Schematic - Physical Pin 7 - BCM pin GPIO4
GEIGER_PIN = 4

# Setup Button
geiger_button = Button(GEIGER_PIN, pull_up=False, bounce_time=0.0001)  # Use Button with pull_down and bounce_time

# Initialize counter - keeps track of the number of falling edges detected
count = 0

# Flag to control counting
counting_enabled = True

# Function to handle falling edge interrupt
def falling_edge_callback():
    global count, counting_enabled
    if counting_enabled:
        count += 1
        print(f"Falling edge detected! Count: {count}")

# Attach falling edge detection event
geiger_button.when_released = falling_edge_callback

# Function to pause/unpause counting
def toggle_counting():
    global counting_enabled
    counting_enabled = not counting_enabled
    if counting_enabled:
        print("Counting resumed.")
    else:
        print("Counting paused.")

# Thread function to listen for Enter key press
def key_listener():
    global counting_enabled
    while True:
        input()  # Wait for Enter key press
        toggle_counting()

# Start the key listener thread
key_listener_thread = threading.Thread(target=key_listener)
key_listener_thread.start()

# Define server address and port
HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 12345      # Arbitrary port number

# Create a socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    # Bind the socket to the address and port
    server_socket.bind((HOST, PORT))

    # Listen for incoming connections
    server_socket.listen()

    print("Server listening on {}:{}".format(HOST, PORT))

    # Accept a connection
    client_socket, client_address = server_socket.accept()

    with client_socket:
        print("Connected to client:", client_address)

        # Send real-time data to the client
        while True:
            if counting_enabled:
                # Get current timestamp
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
                # Concatenate count and timestamp into a single string
                data_to_send = f"{count},{timestamp}"

                # Print to confirm on both SSH terminals
                print(f"{timestamp} Sending count:", count)
                
                # Send data to the client
                client_socket.sendall(data_to_send.encode())

            # Wait for a short interval
            time.sleep(10)  # Adjust as needed for your desired update rate