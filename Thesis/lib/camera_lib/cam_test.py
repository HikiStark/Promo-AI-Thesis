import zmq
import cv2
import numpy as np
from omni.isaac.sensor import Camera  # Ensure you have the correct camera module import
import time

# Set up ZMQ publisher
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")  # Listen on port 5555


def publish_camera_frames(camera: Camera):
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


if __name__ == "__main__":
    # Replace with your actual camera initialization if needed
    camera = Camera()
    try:
        publish_camera_frames(camera)
    except KeyboardInterrupt:
        print("Publisher stopped by user.")
    finally:
        socket.close()
        context.term()
