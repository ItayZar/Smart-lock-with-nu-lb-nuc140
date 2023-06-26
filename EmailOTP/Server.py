import bluetooth
import re
import random
from SendEmail import send_email_message,gmail_init

# Bluetooth MAC address of the HC-05 module
hc05_address = '20:17:01:03:41:02'  # Replace with the actual address

# Create a Bluetooth socket
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

# Connect to the HC-05 module
sock.connect((hc05_address, 1))  # Port number 1 for RFCOMM

gmail_init()

def generate_otp():
    otp = ""
    for _ in range(4):
        digit = random.randint(1, 9)
        otp += str(digit)
    return otp

# Receive and display messages
while True:
    data = sock.recv(1024)  # Adjust the buffer size as per your requirements
    data = data.decode('utf-8', 'ignore')  # Convert bytes to string
    data = data.strip()  # Strip whitespace
    data = re.sub(r'[^\x20-\x7E]', '', data)  # Filter non-printable characters
    if data:
        print("Received message:", data)  # Display the received message
        myotp=generate_otp()
        send_email_message(otp=myotp,id=data)  # Send the message via email
        try:
            sock.send(myotp.encode('utf-8'))  # Send the OTP to the Bluetooth device
        except Exception as e:
            print("Error sending data:", str(e))

# Close the socket connection when done
sock.close()
