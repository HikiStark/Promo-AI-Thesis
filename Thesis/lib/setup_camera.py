from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys
import numpy as np
from omni.isaac.core import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import Camera

world = World(stage_units_in_meters=1.0)
stage = world.get_stage()


class camera_add:
    camera = Camera(
        prim_path="/World/MyCamera",  # The USD path where the camera will be added
        translation=(0.0, 0.0, 2.0),  # Position of the camera in world coordinates
        orientation=(1.0, 0.0, 0.0, 0.0),  # Quaternion for camera orientation (w,x,y,z)
        resolution=(1080, 1080),  # Resolution of the captured images
    )

    world.scene.add(camera)


rgb_image = camera_add.get_rgb_image()


class camera_add_overhead:
    camera_orientation = euler_angles_to_quat(
        [0, -90, 0]
    )  # [roll, pitch, yaw] in degrees
    camera = Camera(
        prim_path="/World/OverheadCamera",
        translation=(0.0, 0.0, 1.5),
        orientation=camera_orientation,
        resolution=(1080, 1080),
    )
    world.scene.add(camera)


# Add a Camera pointing downward at the table
# Let's place it at (0,0,1.5) so it's above the table.
# We need to rotate the camera to look straight down.
# If the camera initially looks along the +X axis, a rotation of -90 degrees about the Y-axis
# will align it to look down the -Z axis.
