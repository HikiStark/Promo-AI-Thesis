import cv2
import numpy as np


cap = cv2.VideoCapture(0)
detector = cv2.QRCodeDetector()

while True:
    ret, frame = cap.read()
    if not ret:
        break
    data, bbox, _ = detector.detectAndDecode(frame)
    
    
    if bbox is not None and len(bbox) > 0:
        # Squeeze the first dimension so that points has shape (4, 2)
        points = np.squeeze(bbox)
        # Now points should be shape (4,2), each row is [x, y]

        # Convert float64 to int if necessary
        points = points.astype(int)

        # Draw lines between consecutive corners
        for i in range(4):
            start_point = (points[i][0], points[i][1])
            end_point = (points[(i + 1) % 4][0], points[(i + 1) % 4][1])
            cv2.line(frame, start_point, end_point, (255, 0, 0), 2)
    
# Handle Multiple QR Codes
    # if bbox is not None and len(bbox) > 0:
    #     for b in bbox:
    #         corners = b.astype(int)  # b should be shape (4, 2)
    #         for i in range(4):
    #             start_point = (corners[i, 0], corners[i, 1])
    #             end_point   = (corners[(i+1) % 4, 0], corners[(i+1) % 4, 1])
    #             cv2.line(frame, start_point, end_point, (255, 0, 0), 2)

    cv2.imshow("QR Code Scanner", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
