import socket
from gpiozero import Button
from signal import pause
import time
import datetime

# Define GPIO pin for the Geiger counter input
# As specified in PiGI Schematic - Physical Pin 7 - BCM pin GPIO4
GEIGER_PIN = 4

# Setup Button
geiger_button = Button(GEIGER_PIN, pull_up=True, bounce_time=0.05)  # Use Button with pull_up and bounce_time

# Initialize count - keeps track of the number of falling edges detected
count = 0

# Function to handle falling edge interrupt
def falling_edge_callback():
    global count
    count += 1
    
    print(f"Falling edge detected! Count: {count}")

# Attach falling edge detection event
geiger_button.when_pressed = falling_edge_callback

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
         
            # Send count to the client
            data_to_send = f"{count},{timestamp}"
            client_socket.sendall(data_to_send.encode())

            # Reset the count every x amount of seconds
            if count > 0:
                print("Resetting count.")
                count = 0

            # Wait for a while before logging again
            # Divide by value to get cps or times by a different value to get cpm
            time.sleep(10)

# Test on RPi, PiGI, and Leybold detector
# Detects falling edges w/ cumulative count
# Speeds up when the lantern close to the window - def detecting radiation
# needs to be tuned

# Publish timestamp and counts to ROS topic
# RUN AS ROOT (SUDO) ON Pi TO ACCESS GPIO
pause()  # Keep the program running