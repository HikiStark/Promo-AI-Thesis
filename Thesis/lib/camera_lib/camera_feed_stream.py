import zmq
import cv2
import time
import threading
import numpy as np
from omni.isaac.sensor import Camera

# Set up ZMQ publisher
context = zmq.Context()
socket = context.socket(zmq.PUB)

# -------------------------
# 1) Limit backlog on publisher side
socket.setsockopt(zmq.SNDHWM, 1)
# You could alternatively do:
# socket.setsockopt(zmq.CONFLATE, 1)
# But typically conflate is used on the subscriber side.
# -------------------------

socket.bind("tcp://*:5555")  # Listen on port 5555


def publish_camera_frames(camera):
    while True:
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            # If the frame is in float format, convert to uint8
            if rgba.dtype == np.float32:
                rgba = (rgba * 255).astype(np.uint8)
            # Convert RGBA to BGR since OpenCV uses BGR ordering
            bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)

            # Encode the frame as JPEG to compress data before sending
            ret, encoded = cv2.imencode(".jpg", bgr)
            if ret:
                try:
                    # Use non-blocking send; if the subscriber is slow, you may drop frames
                    socket.send(encoded.tobytes(), zmq.NOBLOCK)
                except zmq.Again:
                    print("Warning: Dropped frame due to high load or slow subscriber.")
            else:
                print("Warning: Frame encoding failed.")

        # Step simulation or add your simulation stepping logic here.
        # Sleep to simulate ~30 FPS; adjust as needed.
        time.sleep(0.033)


def start_publisher(camera: Camera):
    # Start the publisher in a daemon thread so it won't block the main thread on exit
    publisher_thread = threading.Thread(target=publish_camera_frames, args=(camera,), daemon=True)
    publisher_thread.start()


if __name__ == "__main__":
    # For testing:
    # You would normally pass your camera object here
    dummy_camera = None
    start_publisher(dummy_camera)
    while True:
        time.sleep(1)
    print("-" * 120 + "\n")
