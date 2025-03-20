import cv2
import zmq
import numpy as np
import time

# Set up ZMQ subscriber
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics
time.sleep(1)  # Allow time for subscription registration


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


if __name__ == "__main__":
    receive_and_display()
