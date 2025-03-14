import cv2
import os
import numpy as np

class ImagePaths:
    def __init__(self, script_dir):
        self.overheadangled = os.path.join(script_dir, "resources", "overheadangled.png")
        self.overheadangledthreshold = 50
        self.topdown = os.path.join(script_dir, "resources", "topdown.png")
        self.topdownthreshold = 40 
        self.verysideangle = os.path.join(script_dir, "resources", "verysideangle.png")
        self.verysideanglethreshold = 25
        self.sideangle = os.path.join(script_dir, "resources", "sideangle.png")
        self.sideanglethreshold = 25
        self.topanglefar = os.path.join(script_dir, "resources", "topanglefar.png")
        self.topanglefarthreshold = 30

def detect_edges_pic(image_path, thresholdd):
    img = cv2.imread(image_path)
    # Convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    v = np.median(hsv[:, :, 2])  # Compute the median brightness
    lower_black = np.array([0, 0, max(0, v - 50)])
    upper_black = np.array([180, 255, min(255, v + 50)])

    # Threshold to isolate black regions
    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    # Morphological closing to fill small holes (like text)
    kernel = np.ones((3, 3), np.uint8)  # Reduced from (6,6) to avoid over-expansion
    mask_closed = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Find external contours in the closed mask
    contours, _ = cv2.findContours(
        mask_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    # Filter and draw contours
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # Adjust this area threshold based on the size of the cubes in the image
        if area < 2000:  # Lowered from 4000
            continue

        # Approximate the contour to a polygon
        epsilon = 0.08 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        if len(approx) >= 4 and len(approx) <= 6:
            # For demonstration, just draw a bounding box around the contour
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h
            if 0.8 < aspect_ratio < 1.2:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate the center of the rectangle
                center_x = x + w // 2
                center_y = y + h // 2

                # Draw a red circle (center point) at the center
                cv2.circle(img, (center_x, center_y), 3, (0, 0, 455), -1)

    # Distance transform and watershed-like approach
    dist_transform = cv2.distanceTransform(mask_closed, cv2.DIST_L2, 5)
    _, sure_fg = cv2.threshold(dist_transform, 0.2 * dist_transform.max(), 255, 0)  # Adjusted threshold
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(mask_closed, sure_fg)

    cv2.imshow("mask_closed", mask_closed)
    cv2.waitKey(0)
    cv2.imshow("detected_cubes", img)

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    paths = ImagePaths(script_dir)
    image_paths = [
        paths.overheadangled,
        paths.topdown,
        paths.verysideangle,
        paths.sideangle,
        paths.topanglefar
    ]
    image_threshold = [
        paths.overheadangledthreshold,
        paths.topdownthreshold,
        paths.verysideanglethreshold,
        paths.sideanglethreshold,
        paths.topanglefarthreshold
    ]
    
    for img_path, threshold in zip(image_paths, image_threshold):
        detect_edges_pic(img_path, threshold)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

main()