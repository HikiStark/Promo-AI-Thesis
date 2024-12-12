from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys
import numpy as np
from omni.isaac.core import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.sensor import Camera

world = World(stage_units_in_meters=1.0)
stage = world.get_stage()


class camera_add():
    camera = Camera(
        prim_path="/World/MyCamera",    # The USD path where the camera will be added
        translation=(0.0, 0.0, 2.0),    # Position of the camera in world coordinates
        orientation=(1.0, 0.0, 0.0, 0.0), # Quaternion for camera orientation (w,x,y,z)
        resolution=(1920, 1080)         # Resolution of the captured images
    )

    world.scene.add(camera)

    world.reset()
    world.step(render=True)

    rgb_image = camera.get_rgb_image()