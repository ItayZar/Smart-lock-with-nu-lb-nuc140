import bluetooth

def process_message(message):
    # Perform your desired action based on the received message
    print("Received message:", message)
    # Add your code here to perform the desired action based on the message

def start_server():
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    port = 1  # RFCOMM port used by the HC-05 device

    server_sock.bind(("", port))
    server_sock.listen(1)

    print("Waiting for connections...")
    client_sock, client_info = server_sock.accept()
    print("Accepted connection from", client_info)

    try:
        while True:
            data = client_sock.recv(1024)
            if len(data) == 0:
                break

            message = data.decode("utf-8")
            process_message(message)

    except IOError:
        pass

    client_sock.close()
    server_sock.close()

if __name__ == "__main__":
    start_server()
