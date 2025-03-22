import cv2
import numpy as np


class ObjectDetector:
    def __init__(
        self,
        weights_path,
        cfg_path,
        names_path,
        stream_source=0,
        conf_threshold=0.5,
        nms_threshold=0.4,
    ):
        self.stream_source = stream_source
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold

        # Load YOLO network
        self.net = cv2.dnn.readNet(weights_path, cfg_path)
        with open(names_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        # Get names of all layers and determine the output layers
        layer_names = self.net.getLayerNames()
        self.output_layers = [
            layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()
        ]
        self.cap = None

    def open_stream(self):
        self.cap = cv2.VideoCapture(self.stream_source)
        if not self.cap.isOpened():
            raise Exception(
                "Error: Could not open video stream: " + str(self.stream_source)
            )

    def detect(self, frame):
        height, width, _ = frame.shape
        # Preprocess the frame into a blob
        blob = cv2.dnn.blobFromImage(
            frame, scalefactor=1 / 255.0, size=(416, 416), swapRB=True, crop=False
        )
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)
        return outputs, height, width

    def process_detections(self, outputs, height, width):
        boxes = []
        confidences = []
        class_ids = []

        # Iterate through each detection from YOLO
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.conf_threshold:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        return boxes, confidences, class_ids

    def draw_boxes(self, frame, boxes, confidences, class_ids):
        # Apply non-maximum suppression to filter overlapping boxes
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, self.conf_threshold, self.nms_threshold
        )
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    label,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
        return frame

    def run(self):
        self.open_stream()
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            outputs, height, width = self.detect(frame)
            boxes, confidences, class_ids = self.process_detections(
                outputs, height, width
            )
            frame = self.draw_boxes(frame, boxes, confidences, class_ids)

            cv2.imshow("Object Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    detector = ObjectDetector(
        "yolov3.weights",
        "yolov3.cfg",
        "coco.names",
        stream_source="tcp://127.0.0.1:5555",
    )
    detector.run()
