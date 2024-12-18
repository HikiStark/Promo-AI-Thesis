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


# def add_camera_to_scene(name="Camera", position=(0, 0, 0), orientation=(0, 0, 0)):
#     """
#     Adds a camera to the Isaac Sim scene at the specified position and orientation.

#     Parameters:
#         name (str): The name of the camera prim.
#         position (tuple): A tuple of (x, y, z) coordinates for the camera's position.
#         orientation (tuple): A tuple of (x, y, z) Euler angles for the camera's rotation.

#     Returns:
#         str: The path of the created camera prim.
#     """
#     # Create the camera prim
#     stage = get_current_stage()
#     camera_path = f"/World/{name}"
#     create_prim(camera_path, "Camera")

#     # Set position and orientation
#     camera_prim = stage.GetPrimAtPath(camera_path)
#     camera_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*position))
#     camera_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3d(*orientation))

#     return camera_path

# def camera_add():
#     camera = Camera(
#         prim_path="/World/MyCamera",  # The USD path where the camera will be added
#         translation=(0.0, 0.0, 2.0),  # Position of the camera in world coordinates
#         orientation=(1.0, 0.0, 0.0, 0.0),  # Quaternion for camera orientation (w,x,y,z)
#         resolution=(1080, 1080),  # Resolution of the captured images
#     )

#     world.scene.add(camera)


# rgb_image = camera_add.get_rgb_image()

# def camera_add_overhead():
#     camera_orientation = euler_angles_to_quat(
#         [0, -90, 0]
#     )  # [roll, pitch, yaw] in degrees
#     camera = Camera(
#         prim_path="/World/OverheadCamera",
#         translation=(0.0, 0.0, 1.5),
#         orientation=camera_orientation,
#         resolution=(1080, 1080),
#     )
#     world.scene.add(camera)

# Add a Camera pointing downward at the table
# Let's place it at (0,0,1.5) so it's above the table.
# We need to rotate the camera to look straight down.
# If the camera initially looks along the +X axis, a rotation of -90 degrees about the Y-axis
# will align it to look down the -Z axis.
