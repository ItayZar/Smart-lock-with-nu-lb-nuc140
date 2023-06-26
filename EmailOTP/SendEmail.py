import base64
from email.mime.text import MIMEText
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from requests import HTTPError

users={'1':'itayzar10@gmail.com','2':'myprojct2023@gmail.com'}

def gmail_init():
    global service
    SCOPES = ["https://www.googleapis.com/auth/gmail.send"]
    flow = InstalledAppFlow.from_client_secrets_file('credentials.json', SCOPES)
    creds = flow.run_local_server(port=0)
    service = build('gmail', 'v1', credentials=creds)


def send_email_message(otp,id):
    global service
    message = MIMEText(f"Your OTP is:\t {otp}")
    message['to'] = users[id]
    message['subject'] = 'OTP'
    create_message = {'raw': base64.urlsafe_b64encode(message.as_bytes()).decode()}

    try:
        message = service.users().messages().send(userId="me", body=create_message).execute()
        print(f'Sent message to {message} Message Id: {message["id"]}')
    except HTTPError as error:
        print(f'An error occurred: {error}')
        message = None
