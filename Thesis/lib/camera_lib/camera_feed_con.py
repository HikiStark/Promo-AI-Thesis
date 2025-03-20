from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
from omni.isaac.sensor import Camera  # Ensure you have the correct camera module import
from omni.isaac.core import World
from typing import Optional, Tuple, List

import threading
import zmq
import cv2
import numpy as np
import time
import sys
import logging
import carb
import argparse
import numpy as np
from omni.isaac.nucleus import get_assets_root_path

from pxr import UsdPhysics, PhysxSchema, Usd, Sdf, UsdGeom, Gf

import omni.isaac.core
from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.physics_context import PhysicsContext
from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_at_path

# from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.string import find_unique_string_name

import omni.isaac.core.utils.numpy.rotations as rot_utils

# world = World(stage_units_in_meters=1.0)

# assets_root_path: Optional[str] = get_assets_root_path()
# if assets_root_path is None:
#     carb.log_error("Could not find Isaac Sim assets folder")
#     sys.exit()


# Set up ZMQ publisher
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")  # Listen on port 5555


class add_Camera_test:  # Class to create an overhead camera
    def __init__(self) -> None:
        self.camera_path = "/World/Camera_overhead_1"  # Define the camera's path
        self.camera_position = (0.0, 0.0, 4.5)  # Define camera position
        self.frequency = 30  # Capture frequency in Hz
        self.resolution = (1920, 1080)  # Resolution of the camera
        self.enable_rgb = True  # Enable RGB capture
        self.camera = None  # Camera object

    def compute_orientation(self):
        # Compute the quaternion for the camera's orientation. The commented out section shows an alternative method for computing the orientation.
        eu_ang = np.array([0, 90, 0])
        # Compute quaternion using Euler angles (in degrees)
        orientation_quat = rot_utils.euler_angles_to_quats(eu_ang, degrees=True)
        # print("Quaternion Orientation:", orientation_quat)
        return orientation_quat

    def create_camera(self):
        """
        Create and initialize the overhead camera.
        """
        # Get the current stage (if needed for further processing)
        stage = get_current_stage()

        # Compute orientation
        orientation_quat = self.compute_orientation()

        # Create the camera
        self.camera = Camera(
            prim_path=self.camera_path,
            position=self.camera_position,
            orientation=orientation_quat,
            frequency=self.frequency,  # Capture frequency in Hz
            resolution=self.resolution,  # Resolution of the camera
        )
        # Initialize the camera
        self.camera.initialize()
        self.camera.add_motion_vectors_to_frame()


def publish_camera_frames(camera):
    while True:
        rgba = camera.get_rgba()
        print("rgba:")
        if rgba is not None and rgba.size > 0:
            # If the frame is in float format, convert to uint8
            print("rgba.dtype:")
            if rgba.dtype == np.float32:
                rgba = (rgba * 255).astype(np.uint8)
            # Convert RGBA to BGR since OpenCV uses BGR ordering
            bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)

            # Encode the frame as JPEG to compress data before sending
            ret, encoded = cv2.imencode(".jpg", bgr)
            if ret:
                try:
                    # Use non-blocking send; if the subscriber is slow, you may drop frames
                    print("encoded.tobytes():")
                    socket.send(encoded.tobytes(), zmq.NOBLOCK)
                except zmq.Again:
                    print("Warning: Dropped frame due to high load or slow subscriber.")
            else:
                print("Warning: Frame encoding failed.")
        print("Frame sent")

        # Step simulation or add your simulation stepping logic here.
        # Sleep to simulate ~30 FPS; adjust as needed.
        time.sleep(0.033)


def start_publisher(camera: Camera):
    # Start the publisher in a daemon thread so it won't block the main thread on exit
    publisher_thread = threading.Thread(
        target=publish_camera_frames, args=(camera,), daemon=True
    )
    publisher_thread.start()


if __name__ == "__main__":
    # Print a divider line.
    print("-" * 120 + "\n")
    # world.scene.add_default_ground_plane()
    # camera = add_Camera_test()
    # camera.create_camera()
    # camera_obj = camera.camera
    # print("camera_obj:", camera_obj)

    # Start the publisher thread
    # start_publisher(camera_obj)

    # # Continue with Isaac Sim’s main loop or simulation stepping
    # # while True:
    # #     # Call your simulation step or update function here
    # #     # For example, if Isaac Sim requires a step call, do it here.
    # #     time.sleep(0.033)
    # reset_needed = False
    # while simulation_app.is_running():
    #     # Step the simulation with rendering enabled.
    #     world.step(render=True)

    #     if world.is_stopped() and not reset_needed:
    #         reset_needed = True

    #     if world.is_playing():
    #         if reset_needed:
    #             world.reset()
    #             reset_needed = False

    # # Close the simulation when finished.
    # simulation_app.close()
