#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import os
from flask import Flask, Response, render_template_string, request, jsonify
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# --- GLOBAL VARIABLES ---
output_frame = None
lock = threading.Lock()
app = Flask(__name__)

# --- THE ROS NODE (Eyes) ---
class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self.bridge = CvBridge()
        
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

    def listener_callback(self, msg):
        global output_frame, lock
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            # If image is grayscale, we don't convert. If color, keep it.
            with lock:
                ret, buffer = cv2.imencode('.jpg', cv_image)
                output_frame = buffer.tobytes()
        except Exception as e:
            pass

# --- THE WEB SERVER (Voice & Vision) ---
@app.route('/')
def index():
    return render_template_string("""
        <html>
        <head>
            <title>SENTINEL UPLINK</title>
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <style>
                body { background-color: #000; color: #0f0; font-family: 'Courier New', monospace; text-align: center; }
                h1 { font-size: 20px; border-bottom: 1px solid #0f0; padding-bottom: 10px; }
                #video_feed { border: 2px solid #0f0; width: 95%; max-width: 640px; margin-bottom: 20px; }
                .btn { 
                    background: #000; border: 2px solid #0f0; color: #0f0; 
                    padding: 15px 30px; font-size: 18px; cursor: pointer; margin: 10px; font-weight: bold;
                }
                .btn:active { background: #0f0; color: #000; }
                #status { margin-top: 10px; color: #fff; font-style: italic; }
            </style>
        </head>
        <body>
            <h1>SENTINEL COMMAND LINK</h1>
            
            <img id="video_feed" src="/video_feed">
            <br>
            
            <button class="btn" onclick="startListening()">🎤 SEND VOICE COMMAND</button>
            <button class="btn" style="border-color: red; color: red;" onclick="sendCommand('shutdown')">☠️ KILL SWITCH</button>
            
            <p id="status">Status: SYSTEM ONLINE</p>

            <script>
                function startListening() {
                    // Use the Phone's built-in Speech Recognition
                    var recognition = new (window.SpeechRecognition || window.webkitSpeechRecognition)();
                    recognition.lang = 'en-US';
                    
                    document.getElementById("status").innerText = "LISTENING...";
                    
                    recognition.onresult = function(event) {
                        var command = event.results[0][0].transcript;
                        document.getElementById("status").innerText = "SENDING: " + command;
                        sendCommand(command);
                    };
                    
                    recognition.start();
                }

                function sendCommand(text) {
                    fetch('/command', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify({command: text})
                    })
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById("status").innerText = "SENTINEL: " + data.reply;
                    });
                }
            </script>
        </body>
        </html>
    """)

@app.route('/video_feed')
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/command', methods=['POST'])
def handle_command():
    data = request.json
    text = data.get('command', '').lower()
    print(f">>> VOICE COMMAND RECEIVED: {text}")
    
    reply = "Command Acknowledged."
    
    if "shutdown" in text or "sleep" in text or "kill" in text:
        reply = "SHUTTING DOWN SYSTEMS."
        os.system("espeak 'Shutting down' &")
        # Trigger the kill in a separate thread so we can reply first
        threading.Timer(1.0, lambda: os.system("killall -9 python3")).start()
        
    elif "report" in text:
        reply = "Systems Nominal."
        os.system("espeak 'All systems nominal' &")
        
    elif "who are you" in text:
        reply = "I am Sentinel."
        os.system("espeak 'I am Sentinel' &")

    return jsonify({"reply": reply})

def generate():
    global output_frame, lock
    while True:
        with lock:
            if output_frame is None:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + output_frame + b'\r\n')

def run_ros_node():
    rclpy.init()
    node = DashboardNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    # Start ROS
    t = threading.Thread(target=run_ros_node)
    t.daemon = True
    t.start()
    
    # Start Web Server
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

    
