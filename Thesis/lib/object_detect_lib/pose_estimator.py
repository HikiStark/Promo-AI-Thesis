import numpy as np
import cv2

# Define table parameters (in meters) and coordinate system.
TABLE_WIDTH = 1.0  # e.g., table is 1 meter wide
TABLE_HEIGHT = 0.5  # e.g., table is 0.5 meter deep
TABLE_ORIGIN = np.array([0.0, 0.0])  # bottom-left corner as origin
# TARGET_POSITION = np.array([0.5, 0.5])  # example target position
TARGET_POSITION = np.array([np.random.uniform(0, TABLE_WIDTH), np.random.uniform(0, TABLE_HEIGHT)])
print("TARGET_POSITION:", TARGET_POSITION)


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


def table_to_pixel_coordinates(table_coords, image_resolution, table_dimensions=(TABLE_WIDTH, TABLE_HEIGHT)):
    """
    Convert table coordinates (x, y) back to pixel coordinates in the image.
    """
    image_width, image_height = image_resolution
    table_width, table_height = table_dimensions
    norm_x = table_coords[0] / table_width
    norm_y = 1 - (table_coords[1] / table_height)  # flip back for pixel space
    pixel_x = int(norm_x * image_width)
    pixel_y = int(norm_y * image_height)
    return (pixel_x, pixel_y)


def get_target_pixel_position(image_resolution, table_dimensions=(TABLE_WIDTH, TABLE_HEIGHT), target=TARGET_POSITION):
    """
    Convert the target table coordinates to pixel coordinates for the image.
    Returns a tuple (pixel_x, pixel_y).
    """
    return table_to_pixel_coordinates(target, image_resolution, table_dimensions)


def show_target_position(frame, image_resolution, table_dimensions=(TABLE_WIDTH, TABLE_HEIGHT), target=TARGET_POSITION):
    """
    Draw a marker (circle) at the target position on the feed.
    Returns the pixel coordinates of the target.
    """
    target_pixel = get_target_pixel_position(image_resolution, table_dimensions, target)
    # Draw a red circle at the target position
    cv2.circle(frame, target_pixel, radius=5, color=(0, 0, 255), thickness=-1)
    return target_pixel


def draw_line_to_target(frame, bbox, image_resolution, table_dimensions=(TABLE_WIDTH, TABLE_HEIGHT), target=TARGET_POSITION):
    """
    Draw a red line from the center of the detected box to the target position on the feed.
    Args:
      frame: the image frame (numpy array)
      bbox: bounding box of the detected box (x1, y1, x2, y2)
      image_resolution: tuple (width, height) of the image (e.g., 640x640)
      table_dimensions: physical dimensions (width, height) of the table
      target: the target position in table coordinates
    Returns:
      A tuple containing:
         - box_center: (pixel_x, pixel_y) center of the bounding box
         - target_pixel: (pixel_x, pixel_y) target position in image coordinates
    """
    x1, y1, x2, y2 = bbox
    box_center = ((x1 + x2) // 2, (y1 + y2) // 2)
    target_pixel = get_target_pixel_position(image_resolution, table_dimensions, target)
    # Draw a red line from the center of the box to the target position
    cv2.line(frame, box_center, target_pixel, color=(0, 0, 255), thickness=2)
    return box_center, target_pixel
