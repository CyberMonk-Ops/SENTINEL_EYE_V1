# Sentinel Eye V1 👁️

**Autonomous Surveillance Node** powered by **ROS 2** (Jazzy) and **Computer Vision**.
Designed for **Low-Resource Environments** (Intel i3 / No GPU).

## 🚀 Features
- **Zero-Latency Streaming:** Flask-based dashboard optimized for mobile browsers.
- **Face Tracking:** Real-time detection using Haar Cascades.
- **Voice Core:** Ready for integration with local LLMs (TinyLlama) and Espeak.
- **Resource Efficient:** Runs on <15% CPU on dual-core legacy hardware.

## 🛠️ Installation

1. **System Requirements (Linux/WSL):**
   ```bash
   sudo apt update
   sudo apt install espeak ffmpeg portaudio19-dev


2. Python Dependencies:
      ```bash
   pip install -r requirements.txt

⚡ Usage 

1. Launch the System:
      ```bash
   chmod +x sentineleye_launch.sh
   ./sentineleye_launch.sh

2.Access the Dashboard:

   ​Open your browser (Phone or PC)
   ​Navigate to: http://YOUR_PC_IP:5000

 📂 Architecture
webcam2.py: ROS2 Node for threaded camera capture.
sentineleye2.py: Main logic node (Face detection + HUD overlay).
dash_board2.py: Flask server for video streaming.

## ⚠️ Important: Enable Voice on Mobile
Browsers block microphone access on insecure (HTTP) connections by default.
To fix this on Chrome/Brave (Android):

1. Go to `chrome://flags/#unsafely-treat-insecure-origin-as-secure`
2. Enable the flag.
3. Add your PC's IP (e.g., `http://192.168.x.x:5000`) to the text box.
4. Relaunch Chrome.
5. 

<!-- end list -->
