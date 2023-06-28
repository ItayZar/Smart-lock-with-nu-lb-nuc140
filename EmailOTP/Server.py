import bluetooth
import re
import random
from SendEmail import send_email_message,gmail_init
from config import hc05_address

# Create a Bluetooth socket and connect to the HC-05 module
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((hc05_address, 1))  # Port number 1 for RFCOMM

gmail_init() 

def generate_otp():
    """ Generates OTP of 4 digits between 1 to 9 because the keypad can not take 0 as an input """
    otp = ""
    for _ in range(4):
        digit = random.randint(1, 9)
        otp += str(digit)
    return otp

while True:
    data = sock.recv(1024)  # Buffer adjustment 
    data = data.decode('utf-8', 'ignore')  # Convert bytes to string
    data = data.strip()  # Strip whitespace
    data = re.sub(r'[^\x20-\x7E]', '', data)  # Filter non-printable characters in case the data is sent with "\n" or "\r"
    if data:
        myotp=generate_otp() # Generate OTP for authentication
        send_email_message(otp=myotp,id=data)  # Send the OTP via email to the specific user
        try:
            sock.send(myotp.encode('utf-8'))  # Send the OTP to the Bluetooth device
        except Exception as e:
            print("Error sending data:", str(e)) # In case of error

# Close the socket connection when done
sock.close()
