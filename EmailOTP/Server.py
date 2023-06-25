import bluetooth
import re

# Bluetooth MAC address of the HC-05 module
hc05_address = '20:17:01:03:41:02'  # Replace with the actual address

# Create a Bluetooth socket
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

# Connect to the HC-05 module
sock.connect((hc05_address, 1))  # Port number 1 for RFCOMM

# Receive and display messages
while True:
    data = sock.recv(1024)  # Adjust the buffer size as per your requirements
    data = data.decode('utf-8', 'ignore')  # Convert bytes to string
    data = data.strip()  # Strip whitespace
    data = re.sub(r'[^\x20-\x7E]', '', data)  # Filter non-printable characters
    if data:
        print("Received message:", data)  # Display the received message

# Close the socket connection when done
sock.close()
