import cv2
import os
import numpy as np


def detect_edges_pic():

    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Construct the full path to the image
    image_path = os.path.join(script_dir, "resources", "overheadangled.png")
    image_path_2 = os.path.join(script_dir, "resources", "topdown.png")

    img = cv2.imread(image_path)
    # Convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define a color range for black (tweak if needed)
    # The upper limit for 'V' might need to be raised/lowered depending on lighting.
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 50])

    # Threshold to isolate black regions
    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    # Morphological closing to fill small holes (like text)
    kernel = np.ones((5, 5), np.uint8)
    mask_closed = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Find external contours in the closed mask
    contours, _ = cv2.findContours(
        mask_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    # Filter and draw contours
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # Adjust this area threshold based on the size of the cubes in the image
        if area < 500:
            continue

        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # For demonstration, just draw a bounding box around the contour
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Calculate the center of the rectangle
        center_x = x + w // 2
        center_y = y + h // 2

        # Draw a red circle (target point) at the center
        cv2.circle(img, (center_x, center_y), 3, (0, 0, 455), -1)

    cv2.imshow("mask_closed", mask_closed)
    cv2.waitKey(0)
    cv2.imshow("detected_cubes", img)


def main():
    detect_edges_pic()
    cv2.waitKey(0)
    cv2.destroyAllWindows()


main()
