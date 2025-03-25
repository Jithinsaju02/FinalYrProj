import cv2
import os
import numpy as np
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.image import MIMEImage
import time
import threading

class PeopleDetector:
    def __init__(self):
        # Constants
        self.COOLDOWN = 60  # seconds
        self.last_alert_time = 0
        self.ALERT_SENT = False

        # Email config
        self.SENDER_EMAIL = "scoutsentry390@gmail.com"
        self.RECEIVER_EMAIL = "labosmits@gmail.com"
        self.EMAIL_PASSWORD = "msll hnbt byty odgg"

        # Use absolute path to Haar Cascade classifier
        cascade_path = "/usr/local/share/opencv4/haarcascades/haarcascade_fullbody.xml"
        
        # Alternative paths to try
        alternative_paths = [
            "/usr/share/opencv4/haarcascades/haarcascade_fullbody.xml",
            "/usr/local/share/OpenCV/haarcascades/haarcascade_fullbody.xml",
            os.path.join(os.path.dirname(cv2.__file__), "data", "haarcascades", "haarcascade_fullbody.xml")
        ]

        # Find the first existing path
        for path in [cascade_path] + alternative_paths:
            if os.path.exists(path):
                self.cascade_path = path
                break
        else:
            raise FileNotFoundError("Could not find Haar Cascade classifier file")

        # Load the classifier
        self.person_cascade = cv2.CascadeClassifier(self.cascade_path)

        # Webcam init
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("[ERROR] Could not open webcam")

    def send_email(self, image_path):
        # [Your existing send_email method remains the same]
        msg = MIMEMultipart()
        msg['From'] = self.SENDER_EMAIL
        msg['To'] = self.RECEIVER_EMAIL
        msg['Subject'] = "Person Detected!"

        body = "A person has been detected. See attached image."
        msg.attach(MIMEText(body, 'plain'))

        try:
            with open(image_path, 'rb') as f:
                img_data = f.read()
            image = MIMEImage(img_data, name=os.path.basename(image_path))
            msg.attach(image)

            with smtplib.SMTP_SSL('smtp.gmail.com', 465) as server:
                server.login(self.SENDER_EMAIL, self.EMAIL_PASSWORD)
                server.sendmail(self.SENDER_EMAIL, self.RECEIVER_EMAIL, msg.as_string())
            print("[ALERT] Email sent successfully!")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send email: {e}")
            return False

    def run_detection(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            # Convert to grayscale for Haar Cascade
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect people
            people = self.person_cascade.detectMultiScale(gray, 1.1, 3)

            if len(people) > 0:
                current_time = time.time()
                if current_time - self.last_alert_time > self.COOLDOWN or not self.ALERT_SENT:
                    # Draw rectangles around detected people
                    for (x,y,w,h) in people:
                        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                    
                    # Save and send email
                    timestamp = time.strftime("%Y%m%d-%H%M%S")
                    image_filename = f"detected_person_{timestamp}.jpg"
                    cv2.imwrite(image_filename, frame)

                    email_sent = self.send_email(image_filename)
                    if email_sent:
                        self.last_alert_time = current_time
                        self.ALERT_SENT = True
            else:
                self.ALERT_SENT = False

            # Optional: Uncomment for debugging
            # cv2.imshow('People Detection', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def start_people_detection():
    print("[INFO] Starting people detection")
    detector = PeopleDetector()
    
    # Run detection in a separate thread
    detection_thread = threading.Thread(target=detector.run_detection, daemon=True)
    detection_thread.start()