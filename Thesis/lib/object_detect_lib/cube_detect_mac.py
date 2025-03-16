import cv2
import os
import numpy as np


class ImagePaths:
    """
    Class to manage the paths to the different image resources and their thresholds.
    """

    def __init__(self, script_dir: str) -> None:
        self.overheadangled = os.path.join(
            script_dir, "resources", "overheadangled.png"
        )
        self.overheadangledthreshold = 50
        self.topdown = os.path.join(script_dir, "resources", "topdown.png")
        self.topdownthreshold = 40
        self.verysideangle = os.path.join(script_dir, "resources", "verysideangle.png")
        self.verysideanglethreshold = 25
        self.sideangle = os.path.join(script_dir, "resources", "sideangle.png")
        self.sideanglethreshold = 25
        self.topanglefar = os.path.join(script_dir, "resources", "topanglefar.png")
        self.topanglefarthreshold = 30


class CubeDetector:
    """
    Class to encapsulate cube detection operations using image processing.
    """

    def __init__(self, script_dir: str) -> None:
        self.paths = ImagePaths(script_dir)
        self.image_paths = [
            self.paths.overheadangled,
            self.paths.topdown,
            self.paths.verysideangle,
            self.paths.sideangle,
            self.paths.topanglefar,
        ]
        self.image_thresholds = [
            self.paths.overheadangledthreshold,
            self.paths.topdownthreshold,
            self.paths.verysideanglethreshold,
            self.paths.sideanglethreshold,
            self.paths.topanglefarthreshold,
        ]

    def detect_edges(self, image_path: str, threshold: int) -> None:
        """
        Process the image to detect edges of cubes using thresholding and contour detection.

        Args:
            image_path (str): Path to the image.
            threshold (int): Threshold value for detecting black regions.
        """
        # Read the image
        img = cv2.imread(image_path)
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, threshold])

        # Threshold to isolate black regions
        mask_black = cv2.inRange(hsv, lower_black, upper_black)

        # Morphological closing to fill small holes (like text)
        kernel = np.ones((3, 3), np.uint8)  # Reduced from (6,6) to avoid over-expansion
        mask_closed = cv2.morphologyEx(
            mask_black, cv2.MORPH_CLOSE, kernel, iterations=2
        )

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
        _, sure_fg = cv2.threshold(
            dist_transform, 0.5 * dist_transform.max(), 255, 0
        )  # Reverted threshold for better separation
        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(mask_closed, sure_fg)

        # Display the intermediate and final results
        cv2.imshow("mask_closed", mask_closed)
        cv2.waitKey(0)
        cv2.imshow("detected_cubes", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def process_images(self) -> None:
        """
        Process all images using the configured paths and thresholds.
        """
        for img_path, threshold in zip(self.image_paths, self.image_thresholds):
            self.detect_edges(img_path, threshold)

    def run(self) -> None:
        """
        Execute the cube detection process.
        """
        self.process_images()


def main() -> None:
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Create a CubeDetector instance
    detector = CubeDetector(script_dir)
    # Run the detection process
    detector.run()


if __name__ == "__main__":
    main()
