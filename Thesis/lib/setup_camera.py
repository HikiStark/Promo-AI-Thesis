import sys
import numpy as np
from omni.isaac.core import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import Camera

from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Gf


def add_camera_to_scene(
    prim_path="/World/Camera",
    position=(0, 0, 0),
    orientation=(1, 0, 0, 0),
    resolution=(1920, 1080),
    frequency=60,
):
    """
    Adds a camera to the Isaac Sim scene at the specified position and orientation.

    Parameters:
        prim_path (str): The prim path for the camera in the scene.
        position (tuple): A tuple of (x, y, z) coordinates for the camera's position.
        orientation (tuple): A tuple of (w, x, y, z) quaternion values for the camera's orientation.
        resolution (tuple): Resolution of the camera as (width, height).
        frequency (int): Frequency at which the camera captures frames.

    Returns:
        Camera: The created Camera object.
    """
    # Convert position and orientation to numpy arrays
    position_np = np.array(position)
    orientation_np = np.array(orientation)

    # Create the Camera object
    camera = Camera(
        prim_path=prim_path,
        position=position_np,
        orientation=orientation_np,
        resolution=resolution,
        frequency=frequency,
    )

    # Initialize the camera
    camera.initialize()

    return camera