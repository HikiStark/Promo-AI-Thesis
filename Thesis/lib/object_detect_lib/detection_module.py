import os
import zmq
import cv2
import time
import numpy as np
import torch
from ultralytics import YOLO
from pose_estimator import estimate_box_position, is_at_target

# Path to your custom YOLO model
CUSTOM_MODEL_PATH = "custom_dataset.pt"


class YOLODetector:
    def __init__(self, model_path, use_half=False):  # Initialize the YOLO model.
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # If you saved your model using torch.save() directly, load like this: self.model = torch.load(model_path, map_location=self.device)
        self.model = YOLO(model_path)
        self.model.eval()
        self.model.to(self.device)
        self.use_half = use_half
        if use_half:
            self.model.half()

    def preprocess(self, frame_bgr):  # Convert BGR image to the correct format (RGB + channels-first).
        # Convert from BGR to RGB
        img_rgb = frame_bgr[:, :, ::-1]
        # Make array contiguous and create a Float/half tensor
        img_rgb = np.ascontiguousarray(img_rgb)
        tensor = torch.from_numpy(img_rgb).to(self.device)
        tensor = tensor.permute(2, 0, 1)  # (H, W, 3) -> (3, H, W)
        if self.use_half:
            tensor = tensor.half()
        else:
            tensor = tensor.float()
        # Add batch dimension
        tensor = tensor.unsqueeze(0)
        return tensor

    def detect(self, frame_bgr):
        """Run inference on a single BGR frame."""
        input_tensor = self.preprocess(frame_bgr)
        with torch.no_grad():
            outputs = self.model(input_tensor)
        # Post-process 'outputs' depending on your model's return format
        return outputs


def visualize_detections(frame, results, detector):
    """Visualize detections on the frame."""
    # Visualize bounding boxes if your model outputs them
    for result in results:
        boxes = result.boxes  # Assuming the model outputs bounding boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integer coordinates
            confidence = box.conf[0]  # Confidence score
            label = box.cls[0]  # Class label index
            label_text = f"{detector.model.names[int(label)]} {confidence:.2f}"  # Class name and confidence

            # Draw the bounding box and label on the frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
            cv2.putText(frame, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Compute the estimated table position using the 640x640 image resolution
            estimated_position = estimate_box_position((x1, y1, x2, y2), (frame.shape[1], frame.shape[0]))
            pos_text = f"Est: ({estimated_position[0]:.2f}, {estimated_position[1]:.2f})"

            # Overlay the estimation text on the frame in red
            cv2.putText(frame, pos_text, (x1, y2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Optionally, draw a red circle at the center of the bounding box
            center_pixel = ((x1 + x2) // 2, (y1 + y2) // 2)
            cv2.circle(frame, center_pixel, radius=3, color=(0, 0, 255), thickness=-1)


def detection_module():
    # Set up ZeroMQ subscriber
    context = zmq.Context()
    socket = context.socket(zmq.SUB)

    # Connect to your camera feed publisher (change the IP/port if necessary)
    socket.connect("tcp://127.0.0.1:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all
    time.sleep(1)  # Allow some time to connect

    # Initialize YOLO Detector
    detector = YOLODetector(
        model_path=CUSTOM_MODEL_PATH,
        use_half=False,
    )

    print("Starting detection loop. Press Ctrl+C to exit.")

    try:
        while True:
            # Receive the frame from the stream
            msg = socket.recv()
            img_array = np.frombuffer(msg, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if frame is None:
                print("[Warning] Failed to decode frame.")
                continue

            # Run detection
            results = detector.detect(frame)
            print("Detection Output:", results)

            # Display the frame with detections
            visualize_detections(frame, results, detector)
            # Display the frame
            cv2.imshow("YOLO Detections", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        cv2.destroyAllWindows()
        socket.close()
        context.term()


if __name__ == "__main__":
    detection_module()
