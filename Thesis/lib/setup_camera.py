from lib.setup_import_standart import *
from omni.isaac.sensor import Camera
import math
from scipy.spatial.transform import Rotation as R
import omni.isaac.core.utils.numpy.rotations as rot_utils


def add_camera_overhead():
    stage = get_current_stage()
    camera_path = "/World/Camera_overhead_1"  # Define the camera's path
    table_prim_path = "/World/Table"

    # Define camera position
    camera_position = (0.75855, 1.6528, 2.44948)

    # Get the table's position
    table_prim = get_prim_at_path(table_prim_path)
    table_position = table_prim.GetAttribute("xformOp:translate").Get()

    # Calculate direction vector from camera to table
    direction_vector = (
        table_position[0] - camera_position[0],
        table_position[1] - camera_position[1],
        table_position[2] - camera_position[2],
    )

    # Normalize the direction vector
    norm = math.sqrt(sum([coord**2 for coord in direction_vector]))
    direction_vector = tuple(coord / norm for coord in direction_vector)

    # Calculate the quaternion for the camera's orientation
    # z_axis = direction_vector
    # up_vector = (0, 0, 1)  # Assuming the up vector is along the z-axis
    # x_axis = (
    #     up_vector[1] * z_axis[2] - up_vector[2] * z_axis[1],
    #     up_vector[2] * z_axis[0] - up_vector[0] * z_axis[2],
    #     up_vector[0] * z_axis[1] - up_vector[1] * z_axis[0],
    # )
    # norm_x = math.sqrt(sum([coord**2 for coord in x_axis]))
    # x_axis = tuple(coord / norm_x for coord in x_axis)
    # y_axis = (
    #     z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
    #     z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
    #     z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0],
    # )

    # x_axis = -49.0
    # y_axis = 17.0
    # z_axis = 166.0
    
    x_axis = 65.01
    y_axis = -46.20
    z_axis = 94.59
    
    quat = [0.37118, 0.57839, 0.13886, 0.71303]

    # eu_ang = np.array([x_axis, y_axis, z_axis])
    eu_ang = np.array([0, 90, 0])

    rotation_matrix = [x_axis, y_axis, z_axis]
    # rotation = R.from_matrix(rotation_matrix)
    
    # orientation_quat = rotation.as_quat()
    # orientation_quat = quat
    
    orientation_quat = rot_utils.euler_angles_to_quats(eu_ang, degrees=True)
    print("Quaternion Orientation:", orientation_quat)


    # Create the camera
    camera = Camera(
        prim_path=camera_path,
        position=(camera_position[0], camera_position[1], camera_position[2]),
        orientation=orientation_quat,
        frequency=30,  # Capture frequency in Hz
        resolution=(1920, 1080),  # Resolution of the camera
    )

    # Initialize the camera
    camera.initialize()
