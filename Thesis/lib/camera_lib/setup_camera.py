import os
import cv2
import math
import numpy as np
import omni.kit
import omni.kit.viewport.utility
import omni.isaac.core.utils.nucleus as nucleus
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.sensor import Camera
from scipy.spatial.transform import Rotation as R
from lib.setup_import_standart import *
from lib.camera_lib.camera_feed_con import publish_camera_frames, start_publisher

save_dir = "E:/NVIDIA/isaacsim/myscripts/Thesis/lib/camera_lib/cam_out"
os.makedirs(save_dir, exist_ok=True)

class OverheadCamera:  # Class to create an overhead camera
    def __init__(self) -> None:
        self.camera_path = "/World/Camera_overhead_1"  # Define the camera's path
        self.table_prim_path = "/World/Table"  # Define the table's prim path
        self.camera_position = (0.0, 0.0, 4.5)  # Define camera position
        # Camera properties
        self.frequency = 30  # Capture frequency in Hz
        self.resolution = (1920, 1080)  # Resolution of the camera
        self.enable_rgb = True  # Enable RGB capture
        self.camera = None  # Camera object

    def get_table_position(self):  # Get the table's position
        table_prim = get_prim_at_path(self.table_prim_path)
        table_position = table_prim.GetAttribute("xformOp:translate").Get()
        return table_position

    def compute_orientation(self):
        # Compute the quaternion for the camera's orientation. The commented out section shows an alternative method for computing the orientation.
        eu_ang = np.array([0, 90, 0])
        # Compute quaternion using Euler angles (in degrees)
        orientation_quat = rot_utils.euler_angles_to_quats(eu_ang, degrees=True)
        print("Quaternion Orientation:", orientation_quat)
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
            # enable_rgb=self.enable_rgb,  # Enable RGB capture
            # annotation_types=["rgb"],
        )
        # Initialize the camera
        self.camera.initialize()
        self.camera.add_motion_vectors_to_frame()

    def save_camera_frames(self):
        max_frames = 2
        camera = self.camera
        frame_count = 0
        while frame_count < max_frames:
            # Get RGBA data
            rgba = camera.get_rgba()
            if rgba is not None and rgba.size > 0:
                # Convert float32 RGBA [0..1] to uint8 [0..255], if needed
                if rgba.dtype == np.float32:
                    rgba = (rgba * 255).astype(np.uint8)

                # Convert RGBA -> BGR
                bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)

                # Generate a filename and write to disk
                filename = os.path.join(save_dir, f"frame_{frame_count:04d}.png")
                cv2.imwrite(filename, bgr)
                print(f"Saved {filename}")

                frame_count += 1
            else:
                print("No RGBA data retrieved.")
                break
    
    def setup_in_viewport(self):
        """
        Configure an Isaac Sim viewport to view the camera at `camera_prim_path`.
        """
        camera_prim_path = self.camera_path
        # Create a new viewport window or retrieve an existing one
        viewport_window = omni.kit.viewport.utility.create_viewport_window("SensorCam")

        # Get the low-level viewport API
        viewport_api = viewport_window.viewport_api

        # Set the viewport resolution (optional)
        viewport_api.resolution = (960, 540)

        # Assign this viewport to use the given camera prim
        viewport_api.set_active_camera(camera_prim_path)


    def start_publishing(self):
        """
        Start publishing camera frames over ZMQ.
        """
        start_publisher(self.camera)
        # publish_camera_frames(self.camera)



def add_camera_overhead(simulation_app):
    overhead_camera = OverheadCamera()
    overhead_camera.create_camera()
    camera_obj = overhead_camera.camera
    # overhead_camera.save_camera_frames(10)

    return overhead_camera


if __name__ == "__main__":
    add_camera_overhead()
