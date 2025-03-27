import cv2
import torch


def listen_and_detect_yolo(
    model_path: str,
    camera_source=0,  # could be an int for local cam, or string 'rtsp://...' for streaming
    confidence_thresh=0.25,
):
    """
    Listens to the camera feed, runs YOLO inference, and returns object detection results.
    Adjust the `camera_source` according to your Isaac Sim camera pipeline.
    """

    model = torch.hub.load("ultralytics/yolov5", "custom", path=model_path, force_reload=True)
    model.conf = confidence_thresh  # confidence threshold, if needed

    # 2. Open the video capture
    cap = cv2.VideoCapture(camera_source)

    if not cap.isOpened():
        print("[ERROR] Could not open camera source:", camera_source)
        return

    print("[INFO] Starting YOLO detection... Press Ctrl+C or close window to stop.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARNING] Frame not received properly. Exiting.")
                break

            # 3. Run YOLO inference
            # YOLOv5 forward pass:
            results = model(frame, size=640)  # you can tweak the size
            # results.xyxy[0] --> bounding boxes in [x1, y1, x2, y2, confidence, class]
            # results.xywh[0] --> [x_center, y_center, width, height, confidence, class]
            # results.pandas().xyxy[0] --> Pandas DataFrame with boxes, confidences, class, name

            # 4. (Optional) Visualize the bounding boxes – for debug
            # This will do the typical YOLOv5 overlay on the frame
            #   annotated_frame = np.squeeze(results.render())  # if you want to see the annotations
            #   cv2.imshow("YOLO Detection", annotated_frame)
            #   if cv2.waitKey(1) & 0xFF == ord('q'):
            #       break

            # 5. Get the detection info you need (e.g. bounding boxes + class names + confidences)
            detections = results.pandas().xyxy[0]

            # For example, you could parse it like:
            detection_list = []
            for idx in range(len(detections)):
                x1 = detections.iloc[idx]["xmin"]
                y1 = detections.iloc[idx]["ymin"]
                x2 = detections.iloc[idx]["xmax"]
                y2 = detections.iloc[idx]["ymax"]
                conf = detections.iloc[idx]["confidence"]
                cls = detections.iloc[idx]["class"]
                name = detections.iloc[idx]["name"]

                # Add to detection_list
                detection_list.append(
                    {
                        "bbox": [x1, y1, x2, y2],
                        "confidence": float(conf),
                        "class_id": int(cls),
                        "class_name": name,
                    }
                )

            # 6. Now you can do *whatever you want* with detection_list
            # e.g. yield it to the calling function, do some logic, store it, etc.
            # Let's just print them for debugging:
            print("Detections:", detection_list)

            # If you want to do real-time decisions, call your logic function here
            # decide_actions(detection_list)

    except KeyboardInterrupt:
        print("[INFO] Stopping camera feed...")

    # 7. Release resources
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Camera feed closed and resources released.")


if __name__ == "__main__":
    listen_and_detect_yolo(
        model_path="best.pt",
        camera_source="tcp://127.0.0.1:5555",  # or 0, or some other pipeline
    )
