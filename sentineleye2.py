#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import sqlite3
import requests 
import json
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SentinelEye(Node):
    def __init__(self):
        super().__init__('sentinel_eye')

        # 1. SETUP THE SQL DATABASE (The Memory)
        self.db_path = os.path.expanduser('~/sentinel_memory.db')
        self.conn = sqlite3.connect(self.db_path) 
        self.cursor = self.conn.cursor()
        self.threat1 = self.generate_threat()
        self.get_logger().warn(f" i say {self.threat1}")
        # Create the notebook (Table) if it doesn't exist
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS intruders (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                image_path TEXT,
                threat_level INTEGER DEFAULT 5
            )
        ''')
        self.conn.commit()
        
        # 2. CREATE EVIDENCE FOLDER
        self.evidence_path = os.path.expanduser('~/evidence')
        if not os.path.exists(self.evidence_path):
            os.makedirs(self.evidence_path)

        # 3. SETUP CAMERA SUBSCRIPTION
        # We match the "Reliable" policy of the camera to ensure we get data
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            qos_profile)
        
        self.bridge = CvBridge()
        
        # 4. LOAD THE AI MODEL (Face Detector)
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        
        self.last_action_time = 0
        self.cooldown_seconds = 3.0 # Scream/Log only every 3 seconds
        
        self.get_logger().info('SENTINEL BRAIN ONLINE. DATABASE CONNECTED.')

    def generate_threat(self):
        prompt = " you are a sintinel security system named sentinel . You have detected an intruder . Generate a short , terrifying , robotic warning sentence. Do not explain. keep it short"
        url = "http://localhost:11434/api/generate"
        data = { "model": "tinyllama", "prompt":prompt, "stream": False} #use your desired model
        try:
            response = requests.post(url,json=data)
            response_text = response.json()
            response_final = response_text['response']
            return response_final
        except Exception as e :
             return " system failure"

    def listener_callback(self, msg):
        try:
            # Convert ROS message -> OpenCV Image
            # Note: We expect "mono8" (Greyscale) from the camera now!
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect Faces
            faces = self.face_cascade.detectMultiScale(frame, 1.1, 5)
            
            for (x, y, w, h) in faces:
                # CHECK COOLDOWN (Don't spam the database)
                current_time = time.time()
                if (current_time - self.last_action_time) > self.cooldown_seconds:
                    
                    # 1. GENERATE TIMESTAMP
                    timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    file_time = datetime.now().strftime("%H%M%S")
                    
                    # 2. SAVE THE PHOTO (Evidence)
                    filename = f"{self.evidence_path}/INTRUDER_{file_time}.jpg"
                    cv2.imwrite(filename, frame)
                    
                    # 3. LOG TO DATABASE (The Permanent Record)
                    self.cursor.execute('''
                        INSERT INTO intruders (timestamp, image_path)
                        VALUES (?, ?)
                    ''', (timestamp_str, filename))
                    self.conn.commit() # Save the ink
                    
                    # 4. SCREAM (Audio Warning)
                    # Using '&' to run in background so video doesn't freeze
                    os.system("espeak 'Intruder Detected. You are being recorded.' &")
                    threat = self.generate_threat()
                    self.get_logger().warn(f'>>> INTRUSION LOGGED: ID {self.cursor.lastrowid} at {timestamp_str} and {threat} ')
                    clean_threat= threat.replace("'","")
                    os.system(f"espeak '{clean_threat}' &")


                    self.last_action_time = current_time
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')
        cv2.imshow("SENTINEL HUD", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SentinelEye()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

