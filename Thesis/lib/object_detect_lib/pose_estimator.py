import numpy as np
import cv2

# Define table parameters (in meters) and coordinate system.
TABLE_WIDTH = 1.0  # e.g., table is 1 meter wide
TABLE_HEIGHT = 0.5  # e.g., table is 0.5 meter deep
TABLE_ORIGIN = np.array([0.0, 0.0])  # bottom-left corner as origin
TARGET_POSITION = np.array([0.5, 0.25])  # example target position


def pixel_to_table_coordinates(pixel_coords, image_resolution, table_dimensions=(TABLE_WIDTH, TABLE_HEIGHT)):
    """
    Convert pixel coordinates (x, y) from the camera image to table coordinates.
    Args:
      pixel_coords: tuple (x, y) in pixels.
      image_resolution: tuple (width, height) of the image.
      table_dimensions: physical dimensions (width, height) of the table.
    Returns:
      A numpy array [x, y] representing the position on the table in meters.
    """
    image_width, image_height = image_resolution
    table_width, table_height = table_dimensions
    # Normalize pixel coordinates (assuming (0,0) is top-left in the image)
    norm_x = pixel_coords[0] / image_width
    norm_y = pixel_coords[1] / image_height
    # Flip y so that 0 is at the bottom of the table
    table_x = norm_x * table_width
    table_y = (1 - norm_y) * table_height
    return np.array([table_x, table_y])


def estimate_box_position(bbox, image_resolution):
    """
    Given a bounding box (x1, y1, x2, y2) from detection,
    estimate the center position of the box on the table.
    """
    x1, y1, x2, y2 = bbox
    center_pixel = ((x1 + x2) / 2, (y1 + y2) / 2)
    return pixel_to_table_coordinates(center_pixel, image_resolution)


def is_at_target(position, target=TARGET_POSITION, threshold=0.05):
    """
    Check if the estimated position is within a threshold of the target.
    """
    return np.linalg.norm(position - target) < threshold
