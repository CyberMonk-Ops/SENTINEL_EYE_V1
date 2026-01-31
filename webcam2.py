#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# --- THE DRAIN CLASS ---
class ThreadedCamera:
    def __init__(self, src=0):
        self.capture = cv2.VideoCapture(src)
        # Set buffer size to 1 (Just in case the backend supports it)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.status = False
        self.frame = None
        
        # Read one frame to start
        if self.capture.isOpened():
            self.status, self.frame = self.capture.read()
            self.thread.start()

    def update(self):
        # This loop runs constantly in the background
        while True:
            if self.capture.isOpened():
                # We read efficiently and overwrite 'self.frame'
                # This ensures self.frame is always the NEWEST image
                status, frame = self.capture.read()
                if status:
                    self.status = status
                    self.frame = frame
                else:
                    time.sleep(0.01) # Don't burn CPU if cam dies
            else:
                time.sleep(0.01)

    def get_frame(self):
        return self.status, self.frame

# --- THE PUBLISHER CLASS ---
class PhoneCamera(Node):
    def __init__(self):
        super().__init__('phone_camera_publisher')
        
        # OPTIMIZED QoS: Fire and Forget. Don't confirm delivery.
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.publisher_ = self.create_publisher(Image, 'video_frames', qos_profile)
        self.timer = self.create_timer(0.05, self.timer_callback) # Run fast (20Hz)
        
        #option 1 : use laptop webcam default / change to 1,2 if multiple usb cmeras are connected
        self.camera_url = 0
        # use ip camera / phone put your ip and port (shown in ip camera once video stream  has been started)
        # We use the ThreadedCamera class we built above
        #self.camera_source = "http://x.x.x.x:port/video" 
        
        
        self.cam = ThreadedCamera(self.camera_url)
        self.bridge = CvBridge()
        self.get_logger().info('THREADED CAMERA STARTED. LATENCY SHOULD BE ZERO.')

    def timer_callback(self):
        status, frame = self.cam.get_frame()
        
        if status and frame is not None:
            # 1. RESIZE (Speed)
            small_frame = cv2.resize(frame, (640, 480))
            
            # 2. GREYSCALE (Speed)
            #grey_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)

            # 3. PUBLISH
            msg = self.bridge.cv2_to_imgmsg(small_frame, "bgr8")
            self.publisher_.publish(msg)
        else:
            # If no frame, just pass. Don't crash.
            pass

def main(args=None):
    rclpy.init(args=args)
    node = PhoneCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

