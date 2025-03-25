import cv2
import numpy as np
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.image import MIMEImage
import time
import os
import threading
import uuid

class PeopleDetector:
    def __init__(self):
        # Constants
        self.COOLDOWN = 30  # 5 minutes between emails
        self.last_alert_time = 0

        # Email config
        self.SENDER_EMAIL = "scoutsentry390@gmail.com"
        self.RECEIVER_EMAIL = "labosmits@gmail.com"
        self.EMAIL_PASSWORD = "msll hnbt byty odgg"

        # Tracking unique people
        self.detected_people = set()

        # Use absolute path to Haar Cascade classifier
        cascade_path = "/usr/local/share/opencv4/haarcascades/haarcascade_fullbody.xml"
        
        # Load the classifier
        self.person_cascade = cv2.CascadeClassifier(cascade_path)

        # Webcam init
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("[ERROR] Could not open webcam")

    def send_email(self, image_path):
        msg = MIMEMultipart()
        msg['From'] = self.SENDER_EMAIL
        msg['To'] = self.RECEIVER_EMAIL
        msg['Subject'] = "New Person Detected!"

        body = "A new person has been detected in the area. See attached image."
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

            # Track new unique people
            new_people = set()

            if len(people) > 0:
                current_time = time.time()
                
                # Identify and track new people
                for (x,y,w,h) in people:
                    # Create a unique identifier based on position and size
                    person_id = (x, y, w, h)
                    
                    if person_id not in self.detected_people:
                        # New person detected
                        new_people.add(person_id)
                        self.detected_people.add(person_id)
                        
                        # Draw rectangle for new people
                        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)

                # Send email if new people detected and cooldown passed
                if new_people and (current_time - self.last_alert_time > self.COOLDOWN):
                    # Save and send email
                    timestamp = time.strftime("%Y%m%d-%H%M%S")
                    image_filename = f"new_person_{timestamp}.jpg"
                    cv2.imwrite(image_filename, frame)

                    email_sent = self.send_email(image_filename)
                    if email_sent:
                        self.last_alert_time = current_time

            # Optional: Periodically clear old detections to prevent memory growth
            if len(self.detected_people) > 100:
                self.detected_people.clear()

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