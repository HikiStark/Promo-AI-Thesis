import cv2
import numpy as np
import math


class QRCodePoseEstimator:
    """
    Class for detecting QR codes in a video stream and estimating their pose.
    This uses OpenCV's QRCodeDetector and solvePnP for pose estimation.
    """

    def __init__(
        self,
        camera_index: int = 0,
        qr_size: float = 0.1,
        fx: float = 800,
        fy: float = 800,
    ) -> None:
        """
        Initialize the QRCodePoseEstimator.

        Args:
            camera_index (int): Index for the video capture device.
            qr_size (float): Real-world size of the QR code in meters (assumed square).
            fx (float): Focal length in x direction (in pixels).
            fy (float): Focal length in y direction (in pixels).
        """
        # Open video capture
        self.cap = cv2.VideoCapture(camera_index)
        self.detector = cv2.QRCodeDetector()
        self.qr_size = qr_size

        # 3D coordinates for the QR code corners in the world coordinate system.
        # Order should correspond to the order of the detected image points.
        self.object_points = np.array(
            [[0, 0, 0], [qr_size, 0, 0], [qr_size, qr_size, 0], [0, qr_size, 0]],
            dtype=np.float32,
        )

        # Set example camera intrinsic parameters.
        # Replace these with your actual calibration data.
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cx = frame_width / 2
        cy = frame_height / 2

        self.camera_matrix = np.array(
            [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32
        )

        # Assuming no lens distortion (or use your distortion coefficients)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    @staticmethod
    def rotationMatrixToEulerAngles(R: np.ndarray) -> np.ndarray:
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

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Process a single frame to detect QR codes and estimate their pose.

        Args:
            frame (np.ndarray): The video frame.

        Returns:
            np.ndarray: The annotated frame.
        """
        # Detect and decode the QR code
        data, bbox, _ = self.detector.detectAndDecode(frame)

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
                self.object_points, image_points, self.camera_matrix, self.dist_coeffs
            )
            if success:
                # Convert the rotation vector to a rotation matrix
                R, _ = cv2.Rodrigues(rvec)
                # Convert rotation matrix to Euler angles (roll, pitch, yaw)
                euler_angles = QRCodePoseEstimator.rotationMatrixToEulerAngles(R)
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
        return frame

    def run(self) -> None:
        """
        Start the video capture loop to continuously process frames.
        Press 'q' to quit.
        """
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            annotated_frame = self.process_frame(frame)
            cv2.imshow("QR Code Pose Estimation", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()


def main() -> None:
    estimator = QRCodePoseEstimator()
    estimator.run()


if __name__ == "__main__":
    main()
