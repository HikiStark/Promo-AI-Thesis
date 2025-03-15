import cv2
import numpy as np
import math


def rotationMatrixToEulerAngles(R):
    """
    Converts a rotation matrix to Euler angles.
    Assumes the rotation matrix is in the form: R = Rx(roll)*Ry(pitch)*Rz(yaw).
    Returns the Euler angles in radians.
    """
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# Open video capture
cap = cv2.VideoCapture(0)
detector = cv2.QRCodeDetector()

# Define the real-world size of the QR code (for example, a 10 cm x 10 cm square)
qr_size = 0.1  # in meters

# 3D coordinates for the QR code corners in the world coordinate system.
# Order should correspond to the order of the detected image points.
object_points = np.array(
    [[0, 0, 0], [qr_size, 0, 0], [qr_size, qr_size, 0], [0, qr_size, 0]],
    dtype=np.float32,
)

# Set example camera intrinsic parameters.
# Replace these with your actual calibration data.
fx = 800
fy = 800
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
cx = frame_width / 2
cy = frame_height / 2

camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

# Assuming no lens distortion (or use your distortion coefficients)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect and decode the QR code
    data, bbox, _ = detector.detectAndDecode(frame)

    if bbox is not None and len(bbox) > 0:
        # Squeeze bbox to get a 4x2 array of corner points
        points = np.squeeze(bbox)
        # Convert to integer for drawing
        points_int = points.astype(int)

        # Draw the bounding box on the frame
        for i in range(4):
            start_point = tuple(points_int[i])
            end_point = tuple(points_int[(i + 1) % 4])
            cv2.line(frame, start_point, end_point, (255, 0, 0), 2)

        # Prepare image points (ensure float32)
        image_points = points.astype(np.float32)

        # Pose estimation using solvePnP to get rotation and translation vectors
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, camera_matrix, dist_coeffs
        )
        if success:
            # Convert the rotation vector to a rotation matrix
            R, _ = cv2.Rodrigues(rvec)
            # Convert rotation matrix to Euler angles (roll, pitch, yaw)
            euler_angles = rotationMatrixToEulerAngles(R)
            euler_deg = np.degrees(euler_angles)

            # Display the orientation and translation on the frame
            cv2.putText(
                frame,
                "Pose Estimation Success",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            orientation_text = f"Roll: {euler_deg[0]:.1f}, Pitch: {euler_deg[1]:.1f}, Yaw: {euler_deg[2]:.1f}"
            cv2.putText(
                frame,
                orientation_text,
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            translation_text = (
                f"tvec: [{tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f}]"
            )
            cv2.putText(
                frame,
                translation_text,
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            # Optionally, print the values to the console
            print("Rotation vector:", rvec.ravel())
            print("Translation vector:", tvec.ravel())
            print("Euler angles (deg):", euler_deg)
        else:
            cv2.putText(
                frame,
                "Pose Estimation Failed",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

    cv2.imshow("QR Code Pose Estimation", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
