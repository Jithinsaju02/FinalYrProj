import cv2
import numpy as np
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.image import MIMEImage
import time
import os

# Constants
COOLDOWN = 60  # Cooldown period in seconds
last_alert_time = 0
ALERT_SENT = False  # Flag to track if an alert has been sent during the current detection event

# Email configuration (REPLACE WITH YOUR CREDENTIALS)
SENDER_EMAIL = "scoutsentry390@gmail.com"
RECEIVER_EMAIL = "labosmits@gmail.com"
EMAIL_PASSWORD = "msll hnbt byty odgg" # Use an App Password!

# MobileNet SSD model paths
prototxt = "deploy.prototxt"
model = "mobilenet_iter_73000.caffemodel"

# Load the MobileNet SSD model
net = cv2.dnn.readNetFromCaffe(prototxt, model)

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Function to send an email with the detected image
def send_email(image_path):
    msg = MIMEMultipart()
    msg['From'] = SENDER_EMAIL
    msg['To'] = RECEIVER_EMAIL
    msg['Subject'] = "Person Detected!"

    body = "A person has been detected in the frame. See the attached image."
    msg.attach(MIMEText(body, 'plain'))

    try:
        with open(image_path, 'rb') as f:
            img_data = f.read()
        image = MIMEImage(img_data, name=os.path.basename(image_path)) # Include filename
        msg.attach(image)

        with smtplib.SMTP_SSL('smtp.gmail.com', 465) as server: # Use SMTP_SSL for Gmail
            server.login(SENDER_EMAIL, EMAIL_PASSWORD)
            server.sendmail(SENDER_EMAIL, RECEIVER_EMAIL, msg.as_string())
        print("Email sent successfully!")
        return True  # Indicate successful email send
    except Exception as e:
        print(f"Failed to send email: {e}")
        return False # Indicate email send failure


# Main loop for person detection
while True:
    ret, frame = cap.read()
    if not ret:
        break

    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    person_detected = False

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.2:
            idx = int(detections[0, 0, i, 1])
            if idx == 15:  # Class ID for "person" in MobileNet SSD
                person_detected = True
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)


    if person_detected:
        current_time = time.time()

        if current_time - last_alert_time > COOLDOWN or not ALERT_SENT: # Check cooldown OR if no alert was sent yet
            # Save the detected frame (use timestamp in filename to avoid overwriting)
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            image_filename = f"detected_person_{timestamp}.jpg"
            image_path = image_filename  # Save in the current directory
            cv2.imwrite(image_path, frame)

            # Send email alert
            email_sent = send_email(image_path)

            if email_sent:
                last_alert_time = current_time
                ALERT_SENT = True # Set the flag after sending an alert

    else: # Reset the alert flag if no person is detected
        ALERT_SENT = False

    # Display the output
    cv2.imshow('People Detection', frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close windows
cap.release()
cv2.destroyAllWindows()
