import socket

# Define server address and port
SERVER_HOST = '192.168.0.103'  # IP address of the server Raspberry Pi - Used pigi on APPFRE network as an example
SERVER_PORT = 12345  # Same port as the server

# Create a socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    # Connect to the server
    client_socket.connect((SERVER_HOST, SERVER_PORT))

    # Receive and print real-time data from the server
    while True:
        # Receive data from the server
        data = client_socket.recv(1024)
        
        # If no data received, break the loop
        if not data:
            break
        
        # Split received data into count and timestamp
        count, timestamp = data.decode().split(',')
        
        # Print count and timestamp
        print("Recieved Count", count, "at", timestamp)

#WRITTEN mainly BY CHATGPT
#tested and tweaked by NL