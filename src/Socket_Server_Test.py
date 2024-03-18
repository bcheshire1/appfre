import socket

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
            # Get current timestamp
            timestamp = time.time()
            
            # Increment count
            count += 1
            
            # Concatenate count and timestamp into a single string
            data_to_send = f"{count},{timestamp}"

            #Print to confirm on both SSH terminals
            print(f"{timestamp} Sending count:", count)
            
            # Send data to the client
            client_socket.sendall(data_to_send.encode())

            # Wait for a short interval
            time.sleep(1)  # Adjust as needed for your desired update rate

#WRITTEN BY CHATGPT
#Tested and tweaked by NL