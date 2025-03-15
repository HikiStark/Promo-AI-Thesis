import numpy as np
import cv2

# Define the 3D coordinates of the QR code corners in the real world (e.g., assuming a square of side length L)
L = 0.1  # Example: 10 cm QR code
obj_points = np.array([[0, 0, 0], [L, 0, 0], [L, L, 0], [0, L, 0]], dtype=np.float32)

# Assume bbox is already obtained from the detector and reshaped appropriately:
img_points = np.array([point[0] for point in bbox], dtype=np.float32)

# Camera matrix and distortion coefficients from calibration
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1))  # or use actual distortion coefficients

ret, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

if ret:
    print("Rotation Vector:", rvec)
    print("Translation Vector:", tvec)
