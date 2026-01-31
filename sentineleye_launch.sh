#!/bin/bash

# --- THE CLEANUP TRAP ---
# If you press Ctrl+C, this runs immediately.
cleanup() {
    echo ">>> SHUTTING DOWN SENTINEL..."
    kill 0
}
trap cleanup EXIT

# --- SETUP ENVIRONMENT ---
echo ">>> INITIALIZING ROS2 JAZZY..."
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

# --- START THE ENGINE ---

# 1. The Brain (Ollama)
# We check if it's already running. If not, we start it.
if ! pgrep -x "ollama" > /dev/null
then
    echo ">>> WAKING UP OLLAMA..."
    ollama serve &
    sleep 5 # Give it time to wake up
else
    echo ">>> OLLAMA IS READY."
fi

# 2. The Eyes (Webcam)
echo ">>> STARTING OPTICAL SENSORS (webcam.py)..."
python3 webcam2.py &
PID_CAM=$!
sleep 2

# 3. The Mind (Sentinel Logic)
echo ">>> CONNECTING NEURAL PATHWAYS (sentinel_eye.py)..."
python3 sentineleye2.py &
PID_BRAIN=$!
sleep 2

# 4. The Interface (Dashboard)
echo ">>> ESTABLISHING COMMAND CENTER (dash_board.py)..."
python3 dash_board.py &
PID_DASH=$!

echo "---------------------------------------------------"
echo ">>> SENTINEL SYSTEM ONLINE."
echo ">>> PRESS CTRL+C TO SHUT DOWN ALL SYSTEMS."
echo "---------------------------------------------------"

# Keep the script running so the Trap works
wait

