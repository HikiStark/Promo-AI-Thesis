from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys
import carb
import argparse
import numpy as np
from omni.isaac.nucleus import get_assets_root_path

from omni.isaac.core import World
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage

from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import SurfaceGripper

from omni.isaac.universal_robots import UR10
from omni.isaac.universal_robots.controllers import StackingController
from omni.isaac.universal_robots.controllers.pick_place_controller import (
    PickPlaceController,
)
from omni.isaac.universal_robots.controllers.rmpflow_controller import RMPFlowController
from omni.isaac.universal_robots.tasks import FollowTarget

# 1. Load the UR10 Model
# Parse arguments for test mode
parser = argparse.ArgumentParser()
parser.add_argument(
    "--test", default=False, action="store_true", help="Run in test mode"
)
args, unknown = parser.parse_known_args()

# Load asset root path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()


# Initialize the world
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# 1.1 Load the UR10 model
asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")

# 1.2 Load the gripper
gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10/ee_link")
gripper = SurfaceGripper(
    end_effector_prim_path="/World/UR10/ee_link", translate=0.1611, direction="x"
)

# 1.3 Initialize the robot with gripper
ur10 = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/UR10",
        name="my_ur10",
        end_effector_prim_path="/World/UR10/ee_link",
        gripper=gripper,
    )
)

# 1.4 Set initial joint states
ur10.set_joints_default_state(
    positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
)

# 1.5 Set default gripper state
ur10.gripper.set_default_state(opened=True)

# 2. Load 3 boxes on the table
my_world.scene.add_default_ground_plane()
my_task2 = FollowTarget(name="follow_target_task", attach_gripper=True)
# my_world.add_task(my_task2) # When activated, robot moves crazy.
my_world.reset()

# 3. Define the tasks
articulation_controller = ur10.get_articulation_controller()

# Pick and place controller
# my_controller = PickPlaceController(
#     name="pick_place_controller", gripper=ur10.gripper, robot_articulation=ur10
# )

# RMPFlow controller for advanced movement
my_controller_RMP = RMPFlowController(
    name="target_follower_controller", robot_articulation=ur10, attach_gripper=True
)

# 4. Move the robot to initial position
# Move robot's gripper on top of the table

# Initialize task parameters
# task_params = my_world.get_task("follow_target_task").get_params()
# target_name = task_params["target_name"]["value"]

# 5-7. Tasks: Detect boxes, estimate grip points, pick and place
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller_RMP.reset()
            reset_needed = False
        observations = my_world.get_observations()

        # Example: Move to a specific target position
        target_position = np.array([0.5, 0.5, 0.3])  # Define specific target position
        target_orientation = np.array([0, 0, 0, 1])  # Define specific target orientation (quaternion)

        actions = my_controller_RMP.forward(
            target_end_effector_position=target_position,
            target_end_effector_orientation=target_orientation,
        )
        # 6. Task 2: Detect boxes and estimate grip points
        # actions = my_controller_RMP.forward(
            # target_end_effector_position=observations[target_name]["position"],
            # target_end_effector_orientation=observations[target_name]["orientation"],
            # target_end_effector_position=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]),
            # target_end_effector_orientation=,

        # )

        # Execute actions
        articulation_controller.apply_action(actions)


# 8. Close the simulation
simulation_app.close()