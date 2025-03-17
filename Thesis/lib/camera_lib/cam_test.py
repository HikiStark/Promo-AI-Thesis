"""
Below is a Python script for retrieving the camera output from an Isaac Sim scene.
Replace /World/Camera_overhead_1 with the correct camera path as needed.
Make sure that the Isaac Sim Python environment is set up.
"""

import omni
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.stage import get_current_stage
# from omni.isaac.core.robots.robot import Robot
from omni.isaac.sensor import Camera

# A sample function to demonstrate how to retrieve camera images

# For this script, ensure you have the Isaac Sim python environment or the kit environment active.
# The usage here might differ slightly depending on your exact environment.


def read_camera_data(camera_object):
    # Initialize a simulation context
    sim_context = SimulationContext()

    # Retrieve the current stage
    stage = get_current_stage()

    # Optionally, you could load your scene or ensure it is already loaded
    # stage.SaveAs("my_scene.usd") # For example

    # Create the camera handle if not already created
    camera_path = camera_object.
    print("Camera Path:", camera_path)
    camera = Camera(prim_path=camera_path, frequency=20)

    # Start the simulation
    sim_context.play()

    # Wait a few frames so that the simulator can generate data
    for _ in range(10):
        sim_context.step()

    # Get RGBA data
    rgba = camera.get_rgb()
    if rgba is not None:
        print("RGBA Data Type:", type(rgba))
        print("RGBA Data Shape:", rgba.shape)
    else:
        print("No RGBA data retrieved.")
    
    print("RGBA Data:", rgba)
    # # Optionally get depth data
    # depth = camera.get_depth()
    # if depth is not None:
    #     print("Depth Data Shape:", depth.shape)
    # else:
    #     print("No depth data retrieved.")

    # # Optionally get point cloud data
    # pointcloud = camera.get_point_cloud()
    # if pointcloud is not None:
    #     print("Point Cloud Data Shape:", pointcloud.shape)
    # else:
    #     print("No point cloud data retrieved.")

    sim_context.stop()


# Example usage
if __name__ == "__main__":
    read_camera_data("/World/Camera_overhead_1")
