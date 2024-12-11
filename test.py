from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.universal_robots import UR10
from omni.isaac.universal_robots.controllers import StackingController
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.string import find_unique_string_name
from universal_robots.Stacking_task.task_definitions.stacking import testtask
from universal_robots.Stacking_task.task_definitions.stacking_Basestacking import Stacking as my_tasks

my_world = World(stage_units_in_meters=1.0)
my_world.reset()
# my_task = Stacking()
# my_task = testtask()
# my_world.add_task(my_task)

robot_name = my_tasks.get_params()["robot_name"]["value"]
# robot_name = "ur10"

my_ur10 = my_world.scene.get_object(robot_name)

my_controller = StackingController(
    name="stacking_controller",
    gripper=my_ur10.gripper,
    robot_articulation=my_ur10,
    robot_observation_name=robot_name,
)
articulation_controller = my_ur10.get_articulation_controller()

i = 0
reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        observations = my_world.get_observations()
        actions = my_controller.forward(observations=observations, end_effector_offset=np.array([0.0, 0.0, 0.02]))
        articulation_controller.apply_action(actions)

simulation_app.close()