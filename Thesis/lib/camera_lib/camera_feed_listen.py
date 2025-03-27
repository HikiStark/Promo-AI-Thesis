import sys
import os
import cv2
import zmq
import time
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
lib_dir = os.path.abspath(os.path.join(current_dir, ".."))  # one level up = "lib"
if lib_dir not in sys.path:
    sys.path.append(lib_dir)

from object_detect_lib.detection_feed_main import YOLODetector

# Set up ZMQ subscriber
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics
time.sleep(1)  # Allow time for subscription registration

detector = YOLODetector()


def receive_and_detect():
    while True:
        try:
            msg = socket.recv()
            img_array = np.frombuffer(msg, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if frame is None:
                print("Failed to decode frame")
                continue

            # Run YOLO detection in half precision
            results = detector.detect(frame)
            print("Detections:", results)

            # (Optional) visualize or do further processing
            # ...

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        except KeyboardInterrupt:
            break

    socket.close()
    context.term()
    cv2.destroyAllWindows()


def receive_and_display():
    while True:
        try:
            # Receive a message (blocking)
            msg = socket.recv()
            print("Received a frame")
            # Convert the byte message to a NumPy array
            img_array = np.frombuffer(msg, dtype=np.uint8)
            # Decode the JPEG image into a frame
            bgr = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if bgr is not None:
                cv2.imshow("Camera Feed", bgr)
            else:
                print("Warning: Failed to decode frame.")

            # Check for 'q' key press to exit the loop
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        except KeyboardInterrupt:
            break
    cv2.destroyAllWindows()
    socket.close()
    context.term()


def recieve_and_use_frames():
    while True:
        try:
            # Receive a message (blocking)
            msg = socket.recv()
            print("Received a frame")
            # Convert the byte message to a NumPy array
            img_array = np.frombuffer(msg, dtype=np.uint8)
            # Decode the JPEG image into a frame
            bgr = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if bgr is not None:
                print("Frame received")
            else:
                print("Warning: Failed to decode frame.")
        except KeyboardInterrupt:
            break
    cv2.destroyAllWindows()
    socket.close()
    context.term()


if __name__ == "__main__":
    # receive_and_display()
    receive_and_detect()
