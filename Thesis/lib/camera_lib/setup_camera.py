import math
import numpy as np
import cv2
import numpy as np
import omni.isaac.core.utils.nucleus as nucleus
from lib.setup_import_standart import *
from omni.isaac.sensor import Camera
from scipy.spatial.transform import Rotation as R
import omni.isaac.core.utils.numpy.rotations as rot_utils

# import lib.camera_lib.cam_test as cam_test

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


def add_camera_overhead(simulation_app):
    overhead_camera = OverheadCamera()
    overhead_camera.create_camera()
    camera_obj = overhead_camera.camera
    while simulation_app.is_running():
        # Step the simulation
        simulation_app.update()
        rgba = camera_obj.get_rgba()
        if rgba is not None:
                # Convert RGBA -> BGR for OpenCV
                # rgba shape is [height, width, 4], type float32 or uint8 (depending on setup).
                # Make sure it is in the correct data type for cv2. For example, if it's float32, scale to [0..255].
                if rgba.dtype == np.float32:
                    rgba = (rgba * 255).astype(np.uint8)

                bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)

                # Display in a window
                cv2.imshow("Camera Feed", bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    # cam_test.read_camera_data(camera_obj)
    return overhead_camera


if __name__ == "__main__":
    add_camera_overhead()
